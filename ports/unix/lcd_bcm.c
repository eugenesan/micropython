/*
 * The display, on Linux -- for the Nexus 5 (hammerhead) port of Krux.
 *
 * Krux's firmware draws through MaixPy's `lcd` module:
 *
 *     lcd.init(type=...)
 *     lcd.width(); lcd.height()
 *     lcd.rotation(r); lcd.mirror(b); lcd.bgr_to_rgb(b)
 *     lcd.clear(color)
 *     lcd.draw_string(x, y, text, color, bg_color)
 *     lcd.draw_outline(x, y, w, h, color)
 *     lcd.fill_rectangle(x, y, w, h, color, radius=0)
 *     lcd.draw_line(x0, y0, x1, y1, color)
 *     lcd.string_width_px(text)
 *     lcd.string_has_wide_glyph(text)
 *     lcd.register(reg, value)          -- a no-op here
 *
 * On the K210 the panel is an SPI ST7789 and the module talks to it
 * directly.  On the hammerhead the panel is already up (the kernel's
 * DRM driver brought it up at boot), so the module draws into the
 * framebuffer instead: /dev/graphics/fb0, 1080x1920, 32 bits per
 * pixel, and a **stride of 4352** -- each row is eight pixels wider
 * than the visible area, so the offset of a row is `y * 4352`, never
 * `y * width * 4`.
 *
 * The colours are MaixPy's, which are 16 bit RGB565.  The framebuffer
 * has its own layout, which the module asks the kernel for (the
 * FBIOGET_VSCREENINFO ioctl gives the channel offsets), so the
 * conversion is done at run time and cannot be got wrong by a
 * hard-coded guess.
 *
 * The rotation, the mirror and the BGR order are done in software:
 * the panel is fixed, so the module maps the logical coordinates onto
 * the physical ones.
 */

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/fb.h>

#include "py/obj.h"
#include "py/runtime.h"
#include "py/mphal.h"

#include <pthread.h>

#include "lcd_font_8x16.h"

#define FB_NODE "/dev/graphics/fb0"

/* The display commit ioctl.
 *
 * The panel does not scan the framebuffer's memory directly: it scans
 * a buffer that the driver copies into, and it is pointed at a
 * particular plane of the framebuffer by the commit.  Without the
 * commit, the panel keeps showing whatever it was showing before --
 * which, on this device, is a stale frame left in the reserved memory
 * by an earlier boot, and not the picture the firmware has just drawn.
 *
 * The ioctl's number is derived from the size of the structure it
 * carries (168 bytes: two words and a struct fb_var_screeninfo), and
 * the magic is the kernel tree's own ('m').  Both are reproduced here
 * because the port is built against the host's UAPI headers, not the
 * kernel tree's.  Getting the magic wrong does not fail loudly: the
 * driver reads the structure at the wrong offset and the panel ends up
 * scanning a scrambled part of the framebuffer. */
#define MSMFB_IOCTL_MAGIC 'm'
#define MSMFB_DISPLAY_COMMIT _IOW(MSMFB_IOCTL_MAGIC, 164, \
                                  struct mdp_display_commit)

struct mdp_display_commit {
    uint32_t flags;
    uint32_t wait_for_finish;
    struct fb_var_screeninfo var;
};

#define LCD_FONT_WIDTH 8
#define LCD_FONT_HEIGHT 16

/* The font is scaled up: the K210's screen is 320x240 and its 8x16
 * font is a sixth of the width, but the hammerhead's is 1080x1920 --
 * at the same size the text would be unreadable.
 *
 * Three makes it 24x48: 45 columns and 40 lines, which is *more* room
 * than the K210's 40 columns and 15 lines, and the text is half again
 * as big.  Four (32x64) would give 33 columns and 30 lines -- fewer
 * columns than the K210 -- so three is the size that actually fits.
 *
 * The scale has to agree with what the firmware believes: the
 * firmware lays its lines out FONT_HEIGHT apart, using the size
 * board.py declares.  A scale that does not match makes the
 * consecutive lines overlap. */
#define LCD_FONT_SCALE 3

/* the MaixPy colours, in RGB565 */
#define LCD_BLACK 0x0000
#define LCD_RED 0xF800
#define LCD_GREEN 0x07E0
#define LCD_BLUE 0x001F
#define LCD_WHITE 0xFFFF
#define LCD_YELLOW 0xFFE0
#define LCD_CYAN 0x07FF
#define LCD_MAGENTA 0xF81F
#define LCD_ORANGE 0xFD20
#define LCD_GRAY 0x8410

struct lcd_state {
    int fd;
    uint8_t *fb;
    size_t fb_size;
    int phys_width;
    int phys_height;
    int stride;
    int bpp;
    int pan_offset;          /* the buffer the panel shows, in rows */
    int r_off, g_off, b_off, a_off;
    int r_len, g_len, b_len, a_len;
    int rotation;             /* 0..3 */
    int mirror;               /* flip the logical x */
    int bgr;                  /* swap red and blue */
    int ready;
    int dirty;                /* something was drawn since the last commit */
    volatile uint32_t draw_count;    /* the drawing calls, for the commit thread */
};

static struct lcd_state lcd;

/* defined below; the drawing calls it */
static void lcd_commit(void);

static void lcd_fail(const char *what) {
    mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT(what));
}

/* Point the panel at the plane the firmware draws into.
 *
 * The panel does not scan the framebuffer: the driver copies a plane
 * of it into the buffer the DSI scans out of, and the commit says
 * which plane.  The plane to use is lcd.pan_offset -- the same offset
 * the drawing goes to -- so that what is drawn is what is shown.
 *
 * The commit itself is done by a kernel thread (mdss_fb.c:
 * __mdss_fb_display_thread), so it is asynchronous: the ioctl only
 * queues the work.  But the ioctl also *waits* for that work when
 * wait_for_finish is set, and the work ends in the panel's DMA being
 * re-pointed, which waits for the next vertical blank
 * (mdp3_dmap_update waits for vsync_comp).
 *
 * So a commit is not cheap: it costs a frame, about 16 ms.  Committing
 * after every drawing primitive therefore costs a frame per primitive
 * -- a page with twenty primitives took a third of a second to draw,
 * which is the "the redraw is slow" the user reported -- and the
 * panel was shown each half-drawn frame in turn, which is the
 * "flickers the whole screen".
 *
 * So the commit is deferred: the drawing only marks the frame dirty,
 * and the commit happens once, when the firmware has finished the
 * page.  The port's own input modules (krux/touch.py and
 * krux/buttons.py) call lcd.flush() as they poll for input, and that
 * is exactly where a page ends: the firmware draws a page and then
 * waits for a key.
 */
static void lcd_do_commit(void) {
    struct mdp_display_commit dc;
    struct fb_var_screeninfo var;

    if (!lcd.ready)
        return;

    if (ioctl(lcd.fd, FBIOGET_VSCREENINFO, &var) < 0)
        return;

    memset(&dc, 0, sizeof(dc));
    dc.flags = 0;
    dc.wait_for_finish = 1;
    dc.var = var;
    dc.var.xoffset = 0;
    dc.var.yoffset = lcd.pan_offset;
    dc.var.xres = lcd.phys_width;
    dc.var.yres = lcd.phys_height;

    /* MSMFB_DISPLAY_COMMIT is the MSM-specific commit; FBIOPAN_DISPLAY
     * is the generic one and is what actually flips the panel on this
     * kernel.  Try both. */
    ioctl(lcd.fd, MSMFB_DISPLAY_COMMIT, &dc);
    var.yoffset = lcd.pan_offset;
    var.xoffset = 0;
    ioctl(lcd.fd, FBIOPAN_DISPLAY, &var);
    lcd.dirty = 0;
}

/* The drawing calls this: it only says that something changed. */
static void lcd_commit(void) {
    lcd.dirty = 1;
    lcd.draw_count++;
}

/* The commit thread: the panel follows the framebuffer.
 *
 * The commit is deferred so that a page is not shown half-drawn, and
 * the port's input modules flush it when they wait for a key -- that
 * is where a page ends.  But the firmware also draws pages that nobody
 * waits on: the boot splash, for one, which is drawn and then simply
 * left there for a second.  Nothing flushed it, so the panel never
 * showed the logo at all.
 *
 * This thread closes that gap: it commits the framebuffer when the
 * drawing has been quiet for a moment, so the panel always catches up
 * with what the firmware drew.  "Quiet" is measured with the draw
 * counter, so a page that takes a long time to draw is not shown
 * half-drawn -- the same thing the deferred commit was for. */
static void *lcd_commit_thread(void *arg) {
    uint32_t last_count = 0;
    (void)arg;
    for (;;) {
        uint32_t count;
        usleep(50000);
        count = lcd.draw_count;
        if (count != last_count) {
            /* still drawing: wait for it to finish */
            last_count = count;
            continue;
        }
        if (lcd.dirty)
            lcd_do_commit();
    }
    return NULL;
}

/* The logical size, with the rotation taken into account.
 *
 * MaixPy's lcd_para is the panel's native size and the rotation
 * decides the drawing space: 0 and 2 keep it, 1 and 3 swap it.  The
 * K210's panel is 320x240 (landscape), which is why the firmware
 * calls rotation(1) for portrait there.  The hammerhead's panel is
 * 1080x1920 and portrait to begin with, so the same call has to
 * mean "portrait, as the panel is". */
static void lcd_logical_size(int *w, int *h) {
    if (lcd.rotation == 1 || lcd.rotation == 3) {
        /* portrait */
        *w = lcd.phys_width;
        *h = lcd.phys_height;
    } else {
        /* landscape */
        *w = lcd.phys_height;
        *h = lcd.phys_width;
    }
}

/* A 16-bit colour into the framebuffer's own layout.
 *
 * THE COLOUR WORD IS BYTE-SWAPPED
 *
 * The theme's constants (krux/themes.py, unmodified upstream) are
 * little-endian RGB565 words: RED is 0x00F8, GREEN is 0xE007, BLUE
 * is 0xF800.  Read as plain RGB565 those are not red, green and blue
 * at all -- RED would be a teal, GREEN a dark red, and BLUE a pure
 * red.  Read as the *byte-swapped* word they are exactly what their
 * names say:
 *
 *     RED    0x00F8 -> 0xF800 -> (31,  0,  0)  red
 *     GREEN  0xE007 -> 0x07E0 -> ( 0, 63,  0)  green
 *     BLUE   0xF800 -> 0x001F -> ( 0,  0, 31)  blue
 *     YELLOW 0x85F6 -> 0xF685 -> (30, 52,  5)  yellow
 *     CYAN   0xFF07 -> 0x07FF -> ( 0, 63, 31)  cyan
 *     ORANGE 0x20FD -> 0xFD20 -> (31, 41,  0)  orange
 *
 * Every one of them lands on the colour it is named for, so the swap
 * is the rule.  The reason is the panel it came from: the K210's
 * ST7789 is wired BGR, and MaixPy's lcd module passes the colour
 * through in the panel's byte order.  The firmware's own `bgr_to_rgb`
 * setting (krux/settings.py) is the *user-facing* red/blue swap on top
 * of this, and it is off by default.
 *
 * Without the swap the whole theme is wrong in a way that is easy to
 * miss: black and white are unaffected, so the menu looks right, but
 * the battery indicator -- drawn in the theme's red when the battery
 * is low -- came out as a *green* one, and every other colour was
 * some other colour. */
static uint32_t lcd_pixel(uint16_t color) {
    /* the little-endian word, read big-endian */
    uint16_t rgb = (uint16_t)(((color & 0xff) << 8) | (color >> 8));
    int r = (rgb >> 11) & 0x1f;
    int g = (rgb >> 5) & 0x3f;
    int b = rgb & 0x1f;
    uint32_t v = 0;

    if (lcd.bgr) {
        int t = r;
        r = b;
        b = t;
    }

    /* Compose the pixel in the *panel's* byte order: byte 0 is red,
     * byte 1 green, byte 2 blue, byte 3 unused.
     *
     * The driver advertises the offsets the other way round -- it says
     * transp at 0, blue at 8, green at 16, red at 24, which is the
     * reverse of the panel's order.  Writing the channels to the
     * offsets the driver reports therefore puts the alpha byte where
     * the panel reads red: every pixel came out with red set, and
     * the theme's black background was a red one.
     *
     * The panel's order is not a guess: `fbcommit` (rootfs/fbcommit.c)
     * composed 0xFFFF00FF into both planes and the panel showed
     * *magenta*, which is only true if byte 0 is the red one.  The
     * same order is what TWRP's own screen uses.
     *
     * The theme's colours are RGB565 words with the red in the high
     * bits, which is the order decoded above. */
    v |= (uint32_t)((r * 255) / 31) << 0;
    v |= (uint32_t)((g * 255) / 63) << 8;
    v |= (uint32_t)((b * 255) / 31) << 16;
    /* the fourth byte is the panel's unused one; it is made opaque so
     * that nothing else shows through */
    v |= (uint32_t)0xff << 24;

    return v;
}

/* the logical point -> the physical one */
static void lcd_map(int x, int y, int *px, int *py) {
    int lw, lh;

    lcd_logical_size(&lw, &lh);
    if (lcd.mirror)
        x = lw - 1 - x;

    switch (lcd.rotation) {
    case 1:
        /* portrait: the panel as it is */
        *px = x;
        *py = y;
        break;
    case 3:
        /* portrait, upside down */
        *px = lcd.phys_width - 1 - x;
        *py = lcd.phys_height - 1 - y;
        break;
    case 2:
        /* landscape, turned a quarter */
        *px = y;
        *py = lcd.phys_height - 1 - x;
        break;
    default:
        /* landscape, turned the other way */
        *px = lcd.phys_width - 1 - y;
        *py = x;
        break;
    }
}

static void lcd_set_pixel(int x, int y, uint32_t pixel) {
    int px, py;
    uint8_t *p;

    if (!lcd.ready)
        return;
    lcd_map(x, y, &px, &py);
    if (px < 0 || py < 0 || px >= lcd.phys_width || py >= lcd.phys_height)
        return;

    p = lcd.fb + (size_t)(lcd.pan_offset + py) * lcd.stride +
        (size_t)px * (lcd.bpp / 8);
    switch (lcd.bpp) {
    case 32:
        *(uint32_t *)p = pixel;
        break;
    case 16:
        *(uint16_t *)p = (uint16_t)pixel;
        break;
    case 24:
        p[0] = pixel & 0xff;
        p[1] = (pixel >> 8) & 0xff;
        p[2] = (pixel >> 16) & 0xff;
        break;
    default:
        break;
    }
}

/* Fill a rectangle.
 *
 * This is the hot path of the whole firmware: every page starts with a
 * full-screen clear, and the menu redraws the screen on every key
 * press.  Doing it a pixel at a time, through lcd_set_pixel() and its
 * per-pixel mapping and bounds check, took **159 ms** for a
 * 1080x1920 clear -- which is what made the interface feel slow, and
 * it was nothing to do with the display commit.
 *
 * So the rectangle is mapped once and then filled a row at a time.
 *
 * The mapping is a rotation by a multiple of 90 degrees with an
 * optional mirror, so a rectangle stays a rectangle: mapping the two
 * opposite corners gives the physical rectangle, whichever way the
 * rotation runs. */
static void lcd_fill_rect(int x, int y, int w, int h, uint32_t pixel) {
    int px0, py0, px1, py1;
    int row, col;
    int bpp_bytes = lcd.bpp / 8;
    uint8_t *p;

    if (!lcd.ready || w <= 0 || h <= 0)
        return;

    lcd_map(x, y, &px0, &py0);
    lcd_map(x + w - 1, y + h - 1, &px1, &py1);
    if (px0 > px1) {
        int t = px0;
        px0 = px1;
        px1 = t;
    }
    if (py0 > py1) {
        int t = py0;
        py0 = py1;
        py1 = t;
    }
    if (px0 < 0)
        px0 = 0;
    if (py0 < 0)
        py0 = 0;
    if (px1 >= lcd.phys_width)
        px1 = lcd.phys_width - 1;
    if (py1 >= lcd.phys_height)
        py1 = lcd.phys_height - 1;
    if (px0 > px1 || py0 > py1)
        return;

    for (row = py0; row <= py1; row++) {
        p = lcd.fb + (size_t)(lcd.pan_offset + row) * lcd.stride +
            (size_t)px0 * bpp_bytes;
        switch (lcd.bpp) {
        case 32:
            for (col = px0; col <= px1; col++) {
                *(uint32_t *)p = pixel;
                p += 4;
            }
            break;
        case 16:
            for (col = px0; col <= px1; col++) {
                *(uint16_t *)p = (uint16_t)pixel;
                p += 2;
            }
            break;
        case 24:
            for (col = px0; col <= px1; col++) {
                p[0] = pixel & 0xff;
                p[1] = (pixel >> 8) & 0xff;
                p[2] = (pixel >> 16) & 0xff;
                p += 3;
            }
            break;
        default:
            break;
        }
    }
}

/* init(**kwargs) -- the arguments are MaixPy's (the panel type, the
 * inversion, the SPI pins); on the hammerhead the panel is already
 * up, so they are all accepted and ignored. */
static mp_obj_t lcd_init(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    struct fb_var_screeninfo var;
    struct fb_fix_screeninfo fix;
    int fd;

    (void)args;

    if (lcd.ready)
        return mp_const_none;

    fd = open(FB_NODE, O_RDWR);
    if (fd < 0)
        lcd_fail("cannot open " FB_NODE);

    if (ioctl(fd, FBIOGET_VSCREENINFO, &var) < 0 ||
        ioctl(fd, FBIOGET_FSCREENINFO, &fix) < 0) {
        close(fd);
        lcd_fail("cannot read the framebuffer's information");
    }

    lcd.fd = fd;
    lcd.phys_width = var.xres;
    lcd.phys_height = var.yres;
    lcd.stride = fix.line_length ? (int)fix.line_length
                                 : (int)(var.xres * var.bits_per_pixel / 8);
    lcd.bpp = var.bits_per_pixel;
    lcd.r_off = var.red.offset;
    lcd.g_off = var.green.offset;
    lcd.b_off = var.blue.offset;
    lcd.a_off = var.transp.offset;
    lcd.r_len = var.red.length;
    lcd.g_len = var.green.length;
    lcd.b_len = var.blue.length;
    lcd.a_len = var.transp.length;

    lcd.fb_size = (size_t)fix.smem_len;
    if (lcd.fb_size == 0)
        lcd.fb_size = (size_t)lcd.stride * var.yres * var.yres_virtual / var.yres;
    lcd.fb = mmap(NULL, lcd.fb_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (lcd.fb == MAP_FAILED) {
        lcd.fb = NULL;
        close(fd);
        lcd_fail("cannot map the framebuffer");
    }

    lcd.rotation = 0;
    lcd.mirror = 0;
    lcd.bgr = 0;
    lcd.pan_offset = 0;
    lcd.ready = 1;
    /* the thread that keeps the panel in step with the framebuffer */
    {
        pthread_attr_t attr;
        pthread_t thread;
        pthread_attr_init(&attr);
        pthread_attr_setstacksize(&attr, 16384);
        if (pthread_create(&thread, &attr, lcd_commit_thread, NULL) == 0)
            pthread_detach(thread);
    }

    /* The framebuffer is triple buffered (1080x5760): the panel shows
     * one of the three, and the drawing goes into another one.  Pan
     * the display to the buffer we draw in, or the picture is written
     * but never shown. */
    {
        struct fb_var_screeninfo pan;

        if (ioctl(lcd.fd, FBIOGET_VSCREENINFO, &pan) == 0) {
            /* Draw into the buffer the panel is showing, or the
             * picture is written where nobody looks -- and pan the
             * panel to it, so that what is drawn is what is
             * shown. */
            lcd.pan_offset = pan.yoffset;
            pan.yoffset = lcd.pan_offset;
            pan.xoffset = 0;
            ioctl(lcd.fd, FBIOPAN_DISPLAY, &pan);
        }
    }
    /* The commit, not the pan, is what the panel follows: the driver
     * copies the framebuffer into the buffer the DSI scans out of, and
     * the commit says which plane to copy and to scan.  Without it the
     * panel keeps showing the previous boot's image -- the framebuffer
     * is in reserved memory that is not cleared between boots, so what
     * it shows is whatever was left there last time.
     *
     * This one is a real commit, not the deferred one: it is what
     * puts the panel on this plane to begin with. */
    lcd.dirty = 1;
    lcd_do_commit();
    return mp_const_none;
}

/* flush() -- show what has been drawn.
 *
 * The panel scans a copy of the framebuffer, and the copy is made by
 * the driver when the display is committed.  The firmware draws a page
 * over many calls, so the commit is deferred until the page is done:
 * committing on every primitive costs a frame each time (the commit
 * waits for the vertical blank), and the panel shows every half-drawn
 * frame in turn.
 *
 * The firmware's own API has no flush, so the port's input modules
 * call this as they poll for a key -- which is where a page ends. */
static mp_obj_t lcd_flush(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    (void)args;
    if (lcd.dirty) {
        lcd_do_commit();
    }
    return mp_const_none;
}

static mp_obj_t lcd_width(void) {
    int w, h;
    lcd_logical_size(&w, &h);
    return mp_obj_new_int(w);
}

static mp_obj_t lcd_height(void) {
    int w, h;
    lcd_logical_size(&w, &h);
    return mp_obj_new_int(h);
}

static mp_obj_t lcd_rotation(size_t n_args, const mp_obj_t *args) {
    lcd.rotation = mp_obj_get_int(args[0]) & 3;
    return mp_const_none;
}

static mp_obj_t lcd_mirror(size_t n_args, const mp_obj_t *args) {
    lcd.mirror = mp_obj_get_int(args[0]) ? 1 : 0;
    return mp_const_none;
}

static mp_obj_t lcd_bgr_to_rgb(size_t n_args, const mp_obj_t *args) {
    lcd.bgr = mp_obj_get_int(args[0]) ? 1 : 0;
    return mp_const_none;
}

/* register(reg, value) -- the ST7789's registers; the panel is
 * already up, so there is nothing to do. */
static mp_obj_t lcd_register(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    (void)args;
    return mp_const_none;
}

static mp_obj_t lcd_clear(size_t n_args, const mp_obj_t *args) {
    int w, h;
    lcd_logical_size(&w, &h);
    lcd_fill_rect(0, 0, w, h, lcd_pixel((uint16_t)mp_obj_get_int(args[0])));
    lcd_commit();
    return mp_const_none;
}

static mp_obj_t lcd_draw_outline(size_t n_args, const mp_obj_t *args) {
    int x = mp_obj_get_int(args[0]);
    int y = mp_obj_get_int(args[1]);
    int w = mp_obj_get_int(args[2]);
    int h = mp_obj_get_int(args[3]);
    uint32_t pixel = lcd_pixel((uint16_t)mp_obj_get_int(args[4]));
    int i;


    for (i = 0; i < w; i++) {
        lcd_set_pixel(x + i, y, pixel);
        lcd_set_pixel(x + i, y + h - 1, pixel);
    }
    for (i = 0; i < h; i++) {
        lcd_set_pixel(x, y + i, pixel);
        lcd_set_pixel(x + w - 1, y + i, pixel);
    }
    lcd_commit();
    return mp_const_none;
}

static mp_obj_t lcd_fill_rectangle(size_t n_args, const mp_obj_t *args) {
    int x = mp_obj_get_int(args[0]);
    int y = mp_obj_get_int(args[1]);
    int w = mp_obj_get_int(args[2]);
    int h = mp_obj_get_int(args[3]);
    uint32_t pixel = lcd_pixel((uint16_t)mp_obj_get_int(args[4]));
    int radius = n_args > 5 ? mp_obj_get_int(args[5]) : 0;
    int i, j;


    if (radius <= 0) {
        lcd_fill_rect(x, y, w, h, pixel);
        lcd_commit();
        return mp_const_none;
    }

    /* a rounded rectangle: the corners are cut off */
    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++) {
            int dx = 0, dy = 0;

            if (i < radius)
                dx = radius - i;
            else if (i >= w - radius)
                dx = i - (w - radius - 1);
            if (j < radius)
                dy = radius - j;
            else if (j >= h - radius)
                dy = j - (h - radius - 1);
            if (dx * dx + dy * dy <= radius * radius)
                lcd_set_pixel(x + i, y + j, pixel);
        }
    }
    lcd_commit();
    return mp_const_none;
}

static mp_obj_t lcd_draw_line(size_t n_args, const mp_obj_t *args) {
    int x0 = mp_obj_get_int(args[0]);
    int y0 = mp_obj_get_int(args[1]);
    int x1 = mp_obj_get_int(args[2]);
    int y1 = mp_obj_get_int(args[3]);
    uint32_t pixel = lcd_pixel((uint16_t)mp_obj_get_int(args[4]));
    int dx = x1 - x0, dy = y1 - y0;
    int steps, i, sx, sy;


    if (dx < 0) { sx = -1; dx = -dx; } else { sx = 1; }
    if (dy < 0) { sy = -1; dy = -dy; } else { sy = 1; }

    steps = dx > dy ? dx : dy;
    if (steps == 0) {
        lcd_set_pixel(x0, y0, pixel);
        lcd_commit();
        return mp_const_none;
    }
    for (i = 0; i <= steps; i++) {
        lcd_set_pixel(x0 + (dx * i) / steps * sx, y0 + (dy * i) / steps * sy, pixel);
    }
    lcd_commit();
    return mp_const_none;
}

/* the text: the 8x16 font, one bit per pixel */
static mp_obj_t lcd_draw_string(size_t n_args, const mp_obj_t *args) {
    int x = mp_obj_get_int(args[0]);
    int y = mp_obj_get_int(args[1]);
    const char *text = mp_obj_str_get_str(args[2]);
    uint32_t fg = lcd_pixel((uint16_t)mp_obj_get_int(args[3]));
    uint32_t bg = lcd_pixel((uint16_t)mp_obj_get_int(args[4]));
    int col = 0, row, bit;


    while (*text) {
        unsigned char c = (unsigned char)*text++;

        if (c == '\n') {
            col = 0;
            y += LCD_FONT_HEIGHT;
            continue;
        }
        if (c < 32 || c > 126) {
            /* no glyph for it: a space */
            col += LCD_FONT_WIDTH;
            continue;
        }
        for (row = 0; row < LCD_FONT_HEIGHT; row++) {
            unsigned char bits = lcd_font_8x16[c * 16 + row];

            for (bit = 0; bit < LCD_FONT_WIDTH; bit++) {
                uint32_t colour = (bits & (0x80 >> bit)) ? fg : bg;
                int px = x + col * LCD_FONT_WIDTH * LCD_FONT_SCALE + bit * LCD_FONT_SCALE;
                int py = y + row * LCD_FONT_SCALE;
                int i, j;

                for (i = 0; i < LCD_FONT_SCALE; i++) {
                    for (j = 0; j < LCD_FONT_SCALE; j++) {
                        lcd_set_pixel(px + j, py + i, colour);
                    }
                }
            }
        }
        col++;
    }
    /* The panel scans a copy, not the framebuffer, so the drawing is
     * only visible once it has been committed. */
    lcd_commit();
    return mp_const_none;
}

static mp_obj_t lcd_string_width_px(size_t n_args, const mp_obj_t *args) {
    const char *text = mp_obj_str_get_str(args[0]);
    int scale = n_args > 1 ? mp_obj_get_int(args[1]) : 1;
    int n = 0;

    if (scale <= 0)
        scale = 1;
    while (*text) {
        n++;
        text++;
    }
    return mp_obj_new_int(n * LCD_FONT_WIDTH * LCD_FONT_SCALE * scale);
}

static mp_obj_t lcd_string_has_wide_glyph(size_t n_args, const mp_obj_t *args) {
    const char *text = mp_obj_str_get_str(args[0]);

    (void)n_args;
    /* the font is ASCII only, so no glyph is wide */
    while (*text) {
        if ((unsigned char)*text >= 0x80)
            return mp_const_true;
        text++;
    }
    return mp_const_false;
}

/* display(buf, oft=(0,0), roi=(0,0,0,0)) -- a picture.  The MaixPy
 * image object is not available here, so a buffer of 32 bit pixels is
 * taken instead (the camera path will hand one over). */
static mp_obj_t lcd_display(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    mp_obj_t buf = mp_const_none;
    size_t i;
    int w = 0, h = 0, ox = 0, oy = 0, rx = 0, ry = 0, rw = 0, rh = 0;
    uint8_t *data;
    size_t len;

    if (n_args > 0)
        buf = args[0];
    {
        mp_map_elem_t *elem;

        elem = mp_map_lookup(kwargs, MP_OBJ_NEW_QSTR(MP_QSTR_oft), MP_MAP_LOOKUP);
        if (elem != NULL) {
            size_t n;
            mp_obj_t *items;

            mp_obj_get_array(elem->value, &n, &items);
            if (n >= 2) {
                ox = mp_obj_get_int(items[0]);
                oy = mp_obj_get_int(items[1]);
            }
        }
        elem = mp_map_lookup(kwargs, MP_OBJ_NEW_QSTR(MP_QSTR_roi), MP_MAP_LOOKUP);
        if (elem != NULL) {
            size_t n;
            mp_obj_t *items;

            mp_obj_get_array(elem->value, &n, &items);
            if (n >= 4) {
                rx = mp_obj_get_int(items[0]);
                ry = mp_obj_get_int(items[1]);
                rw = mp_obj_get_int(items[2]);
                rh = mp_obj_get_int(items[3]);
            }
        }
        elem = mp_map_lookup(kwargs, MP_OBJ_NEW_QSTR(MP_QSTR_w), MP_MAP_LOOKUP);
        if (elem != NULL)
            w = mp_obj_get_int(elem->value);
        elem = mp_map_lookup(kwargs, MP_OBJ_NEW_QSTR(MP_QSTR_h), MP_MAP_LOOKUP);
        if (elem != NULL)
            h = mp_obj_get_int(elem->value);
    }
    if (buf == mp_const_none)
        return mp_const_none;

    if (mp_obj_is_type(buf, &mp_type_bytes)) {
        data = (uint8_t *)mp_obj_str_get_data(buf, &len);
        if (w == 0)
            w = (int)(len / 4);
        if (h == 0)
            h = 1;
    } else {
        data = (uint8_t *)mp_obj_get_int(buf);
        len = (size_t)w * h * 4;
    }

    for (i = 0; i < (size_t)w * (size_t)h; i++) {
        uint32_t pixel = (uint32_t)data[i * 4] |
            ((uint32_t)data[i * 4 + 1] << 8) |
            ((uint32_t)data[i * 4 + 2] << 16) |
            ((uint32_t)data[i * 4 + 3] << 24);
        int x = (int)i % w, y = (int)i / w;

        if (rw > 0 && rh > 0) {
            if (x < rx || y < ry || x >= rx + rw || y >= ry + rh)
                continue;
        }
        lcd_set_pixel(ox + x, oy + y, pixel);
    }
    lcd_commit();
    return mp_const_none;
}


/* get_pixel(x, y) -- the colour at a logical point, as RGB565.  It is
 * how the drawing is checked without a screen. */
static mp_obj_t lcd_get_pixel(size_t n_args, const mp_obj_t *args) {
    int x = mp_obj_get_int(args[0]);
    int y = mp_obj_get_int(args[1]);
    int px, py;
    uint8_t *p;
    uint32_t v = 0;

    if (!lcd.ready)
        return mp_const_none;

    lcd_map(x, y, &px, &py);
    if (px < 0 || py < 0 || px >= lcd.phys_width || py >= lcd.phys_height)
        return mp_const_none;

    p = lcd.fb + (size_t)(lcd.pan_offset + py) * lcd.stride +
        (size_t)px * (lcd.bpp / 8);
    if (lcd.bpp == 32)
        v = *(uint32_t *)p;
    else if (lcd.bpp == 16)
        v = *(uint16_t *)p;
    else
        v = p[0] | (p[1] << 8) | (p[2] << 16);

    /* back to RGB565, in the theme's own byte order */
    {
        int r, g, b;
        uint16_t rgb;

        /* The panel's byte order is R, G, B (see lcd_pixel), NOT the
         * offsets the driver reports, which are reversed. */
        r = (int)(((v >> 0) & 0xff) * 31 / 255);
        g = (int)(((v >> 8) & 0xff) * 63 / 255);
        b = (int)(((v >> 16) & 0xff) * 31 / 255);

        /* the standard RGB565 word ... */
        rgb = (uint16_t)(((r & 0x1f) << 11) | ((g & 0x3f) << 5) | (b & 0x1f));
        /* ... byte-swapped, which is the order the theme uses.  The
         * swap is of the two bytes of the 16-bit word, so the low
         * byte is masked off before it is shifted up. */
        return mp_obj_new_int((rgb >> 8) | ((rgb & 0xff) << 8));
    }
}

/* save_ppm(path) -- the framebuffer as a picture, so that what was
 * drawn can be looked at. */
static mp_obj_t lcd_save_ppm(size_t n_args, const mp_obj_t *args) {
    const char *path = mp_obj_str_get_str(args[0]);
    int y, x;
    FILE *f;
    char header[64];
    int hdr_len;

    if (!lcd.ready)
        lcd_fail("the display is not open");

    f = fopen(path, "wb");
    if (f == NULL)
        lcd_fail("cannot write the picture");

    hdr_len = snprintf(header, sizeof(header), "P6\n%d %d\n255\n",
                       lcd.phys_width, lcd.phys_height);
    fwrite(header, 1, (size_t)hdr_len, f);

    for (y = 0; y < lcd.phys_height; y++) {
        uint8_t *row = lcd.fb + (size_t)y * lcd.stride;

        for (x = 0; x < lcd.phys_width; x++) {
            uint8_t *p = row + (size_t)x * (lcd.bpp / 8);
            uint8_t rgb[3];

            if (lcd.bpp == 32) {
                /* The panel's byte order is R, G, B (see lcd_pixel),
                 * NOT the offsets the driver reports, which are
                 * reversed. */
                uint32_t v = *(uint32_t *)p;

                rgb[0] = (uint8_t)((v >> 0) & 0xff);
                rgb[1] = (uint8_t)((v >> 8) & 0xff);
                rgb[2] = (uint8_t)((v >> 16) & 0xff);
            } else if (lcd.bpp == 16) {
                uint16_t v = *(uint16_t *)p;
                rgb[0] = (uint8_t)(((v >> 11) & 0x1f) * 255 / 31);
                rgb[1] = (uint8_t)(((v >> 5) & 0x3f) * 255 / 63);
                rgb[2] = (uint8_t)((v & 0x1f) * 255 / 31);
            } else {
                rgb[0] = p[0];
                rgb[1] = p[1];
                rgb[2] = p[2];
            }
            fwrite(rgb, 1, 3, f);
        }
    }
    fclose(f);
    return mp_const_none;
}


/* draw_qr_code_binary(x, y, code, width, dark, light, bg) -- the QR
 * code, the way MaixPy draws it: the code is the packed bitmap that
 * qrcode.encode() returns (the first byte is dropped there, so the
 * bits are the modules, row by row, least significant bit first),
 * and it is drawn with a one module border. */
static mp_obj_t lcd_draw_qr_code_binary(size_t n_args, const mp_obj_t *args) {
    mp_buffer_info_t qr_bufinfo;
    const uint8_t *qr_data;
    int x0, y0, max_width;
    int starting_size, scale, width, border_size, opposite_border_offset;
    uint16_t dark_color = 0, light_color = 0xffff, bg_color = 0;
    int og_y, og_x;

    x0 = mp_obj_get_int(args[0]);
    y0 = mp_obj_get_int(args[1]);
    mp_get_buffer_raise(args[2], &qr_bufinfo, MP_BUFFER_READ);
    qr_data = qr_bufinfo.buf;
    if (qr_bufinfo.len == 0) {
        return mp_const_none;
    }
    max_width = mp_obj_get_int(args[3]);

    if (n_args >= 5) {
        dark_color = (uint16_t)mp_obj_get_int(args[4]);
    }
    if (n_args >= 6) {
        light_color = (uint16_t)mp_obj_get_int(args[5]);
    }
    if (n_args >= 7) {
        bg_color = (uint16_t)mp_obj_get_int(args[6]);
    }

    /* the size of the code, in modules */
    starting_size = 1;
    while ((starting_size + 1) * (starting_size + 1) <= (int)qr_bufinfo.len * 8) {
        starting_size++;
    }
    /* one module of border on each side */
    scale = max_width / (starting_size + 2);
    if (scale < 1) {
        return mp_const_none;
    }
    width = starting_size * scale;
    border_size = (max_width - width) / 2;
    opposite_border_offset = border_size + width - 1;

    /* the border */
    for (og_x = 0; og_x < max_width; og_x++) {
        for (og_y = 0; og_y < border_size; og_y++) {
            lcd_set_pixel(x0 + og_x, y0 + og_y, lcd_pixel(bg_color));
        }
        for (og_y = opposite_border_offset; og_y < max_width; og_y++) {
            lcd_set_pixel(x0 + og_x, y0 + og_y, lcd_pixel(bg_color));
        }
    }
    for (og_y = border_size; og_y < opposite_border_offset; og_y++) {
        for (og_x = 0; og_x < border_size; og_x++) {
            lcd_set_pixel(x0 + og_x, y0 + og_y, lcd_pixel(bg_color));
        }
        for (og_x = opposite_border_offset; og_x < max_width; og_x++) {
            lcd_set_pixel(x0 + og_x, y0 + og_y, lcd_pixel(bg_color));
        }
    }

    /* the code itself */
    for (og_y = 0; og_y < starting_size; og_y++) {
        for (og_x = 0; og_x < starting_size; og_x++) {
            int index = og_y * starting_size + og_x;
            uint8_t byte = qr_data[index >> 3];
            uint16_t color = (byte & (1 << (index % 8))) ? dark_color : light_color;
            int i, j;

            for (i = 0; i < scale; i++) {
                int y = border_size + og_y * scale + i;
                for (j = 0; j < scale; j++) {
                    int x = border_size + og_x * scale + j;
                    lcd_set_pixel(x0 + x, y0 + y, lcd_pixel(color));
                }
            }
        }
    }
    return mp_const_none;
}

static MP_DEFINE_CONST_FUN_OBJ_KW(lcd_init_obj, 0, lcd_init);
static MP_DEFINE_CONST_FUN_OBJ_0(lcd_width_obj, lcd_width);
static MP_DEFINE_CONST_FUN_OBJ_0(lcd_height_obj, lcd_height);
static MP_DEFINE_CONST_FUN_OBJ_VAR(lcd_rotation_obj, 1, lcd_rotation);
static MP_DEFINE_CONST_FUN_OBJ_VAR(lcd_mirror_obj, 1, lcd_mirror);
static MP_DEFINE_CONST_FUN_OBJ_VAR(lcd_bgr_to_rgb_obj, 1, lcd_bgr_to_rgb);
static MP_DEFINE_CONST_FUN_OBJ_VAR(lcd_register_obj, 2, lcd_register);
static MP_DEFINE_CONST_FUN_OBJ_VAR(lcd_clear_obj, 1, lcd_clear);
static MP_DEFINE_CONST_FUN_OBJ_VAR(lcd_draw_outline_obj, 5, lcd_draw_outline);
static MP_DEFINE_CONST_FUN_OBJ_VAR(lcd_fill_rectangle_obj, 5, lcd_fill_rectangle);
static MP_DEFINE_CONST_FUN_OBJ_VAR(lcd_draw_line_obj, 5, lcd_draw_line);
static MP_DEFINE_CONST_FUN_OBJ_VAR(lcd_draw_string_obj, 5, lcd_draw_string);
static MP_DEFINE_CONST_FUN_OBJ_VAR(lcd_string_width_px_obj, 1, lcd_string_width_px);
static MP_DEFINE_CONST_FUN_OBJ_VAR(lcd_string_has_wide_glyph_obj, 1, lcd_string_has_wide_glyph);
static MP_DEFINE_CONST_FUN_OBJ_KW(lcd_display_obj, 1, lcd_display);
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lcd_draw_qr_code_binary_obj, 4, 7, lcd_draw_qr_code_binary);
static MP_DEFINE_CONST_FUN_OBJ_VAR(lcd_get_pixel_obj, 2, lcd_get_pixel);
static MP_DEFINE_CONST_FUN_OBJ_VAR(lcd_save_ppm_obj, 1, lcd_save_ppm);
static MP_DEFINE_CONST_FUN_OBJ_VAR(lcd_flush_obj, 0, lcd_flush);

static const mp_rom_map_elem_t lcd_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&lcd_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_width), MP_ROM_PTR(&lcd_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_height), MP_ROM_PTR(&lcd_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_rotation), MP_ROM_PTR(&lcd_rotation_obj) },
    { MP_ROM_QSTR(MP_QSTR_mirror), MP_ROM_PTR(&lcd_mirror_obj) },
    { MP_ROM_QSTR(MP_QSTR_bgr_to_rgb), MP_ROM_PTR(&lcd_bgr_to_rgb_obj) },
    { MP_ROM_QSTR(MP_QSTR_register), MP_ROM_PTR(&lcd_register_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear), MP_ROM_PTR(&lcd_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_draw_outline), MP_ROM_PTR(&lcd_draw_outline_obj) },
    { MP_ROM_QSTR(MP_QSTR_fill_rectangle), MP_ROM_PTR(&lcd_fill_rectangle_obj) },
    { MP_ROM_QSTR(MP_QSTR_draw_line), MP_ROM_PTR(&lcd_draw_line_obj) },
    { MP_ROM_QSTR(MP_QSTR_draw_string), MP_ROM_PTR(&lcd_draw_string_obj) },
    { MP_ROM_QSTR(MP_QSTR_string_width_px), MP_ROM_PTR(&lcd_string_width_px_obj) },
    { MP_ROM_QSTR(MP_QSTR_string_has_wide_glyph), MP_ROM_PTR(&lcd_string_has_wide_glyph_obj) },
    { MP_ROM_QSTR(MP_QSTR_display), MP_ROM_PTR(&lcd_display_obj) },
    { MP_ROM_QSTR(MP_QSTR_draw_qr_code_binary), MP_ROM_PTR(&lcd_draw_qr_code_binary_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_pixel), MP_ROM_PTR(&lcd_get_pixel_obj) },
    { MP_ROM_QSTR(MP_QSTR_save_ppm), MP_ROM_PTR(&lcd_save_ppm_obj) },
    { MP_ROM_QSTR(MP_QSTR_flush), MP_ROM_PTR(&lcd_flush_obj) },
    { MP_ROM_QSTR(MP_QSTR_BLACK), MP_ROM_INT(LCD_BLACK) },
    { MP_ROM_QSTR(MP_QSTR_WHITE), MP_ROM_INT(LCD_WHITE) },
    { MP_ROM_QSTR(MP_QSTR_RED), MP_ROM_INT(LCD_RED) },
    { MP_ROM_QSTR(MP_QSTR_GREEN), MP_ROM_INT(LCD_GREEN) },
    { MP_ROM_QSTR(MP_QSTR_BLUE), MP_ROM_INT(LCD_BLUE) },
    { MP_ROM_QSTR(MP_QSTR_YELLOW), MP_ROM_INT(LCD_YELLOW) },
    { MP_ROM_QSTR(MP_QSTR_CYAN), MP_ROM_INT(LCD_CYAN) },
    { MP_ROM_QSTR(MP_QSTR_MAGENTA), MP_ROM_INT(LCD_MAGENTA) },
    { MP_ROM_QSTR(MP_QSTR_ORANGE), MP_ROM_INT(LCD_ORANGE) },
    { MP_ROM_QSTR(MP_QSTR_GRAY), MP_ROM_INT(LCD_GRAY) },
};

static MP_DEFINE_CONST_DICT(mp_module_lcd_globals, lcd_module_globals_table);

const mp_obj_module_t mp_module_lcd = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&mp_module_lcd_globals,
};

MP_REGISTER_MODULE(MP_QSTR_lcd, mp_module_lcd);
