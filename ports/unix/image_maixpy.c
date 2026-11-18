#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "py/runtime.h"
#include "py/obj.h"
#include "k_quirc.h"

typedef struct {
    mp_obj_base_t base;
    int w, h;
    unsigned char *buf;
    /* The luma plane, when the frame came from the camera.
     *
     * The decoder and the statistics want the sensor's own luma.
     * Going through RGB565 and back throws it away: R and B are five
     * bits there, so the luma the decoder sees is off by up to five
     * levels on every pixel -- which is most of the contrast on the
     * edge of a small module, and the reason a dense code only read
     * over a narrow band of exposures. */
    unsigned char *gray;
} image_obj_t;

typedef struct {
    mp_obj_base_t base;
    int l_mean, l_stdev, a_mean, a_stdev, b_mean, b_stdev;
    int l_min, l_max, clipped_low, clipped_high;
    /* The mean of the pixels that are within BRIGHT_MARGIN of the
     * brightest pixel of the frame, and how many there are.
     *
     * The auto-exposure needs the brightness of the thing it is
     * pointed at.  A QR code on a screen is a small bright thing in
     * a usually dark picture: the mean of the picture -- or even of
     * the middle of it -- says more about the room than about the
     * code, and the code may not be in the middle of the frame at
     * all.  The bright pixels are the code's white modules wherever
     * the code happens to be. */
    int l_bright_mean;
    int l_bright_count;
    /* The percentile levels of the region (the luma histogram's P50,
     * P90, P95 and P99), and how many pixels were measured.
     *
     * The auto-exposure works from these, not from the bright-pixel
     * mean: that metric has an absolute margin (BRIGHT_MARGIN), so in
     * a dim scene it degenerates to the plain mean, and a few hot
     * pixels move it.  A percentile says "95% of the region is below
     * this level", which is stable in both a dim room and a clipped
     * bright screen, and it makes the clipped fraction meaningful. */
    int l_p50, l_p90, l_p95, l_p99;
    int l_count;
} stats_obj_t;

/* forward declarations */
const mp_obj_type_t stats_type;
const mp_obj_type_t image_type;
static mp_obj_t image_get_statistics(mp_obj_t self_in);
static mp_obj_t image_get_statistics_roi(size_t n_args, const mp_obj_t *args);

static mp_obj_t image_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    image_obj_t *o = m_new_obj(image_obj_t);
    o->base.type = type;
    o->w = 320; o->h = 240;
    if (n_args >= 1 && mp_obj_is_type(args[0], &mp_type_tuple)) {
        mp_obj_tuple_t *t = (mp_obj_tuple_t *)args[0];
        if (t->len >= 2) {
            o->w = mp_obj_get_int(t->items[0]);
            o->h = mp_obj_get_int(t->items[1]);
        }
    }
    o->buf = m_new(uint8_t, o->w * o->h * 2);
    memset(o->buf, 0, o->w * o->h * 2);
    return MP_OBJ_FROM_PTR(o);
}

static mp_obj_t image_width(mp_obj_t self_in) {
    image_obj_t *o = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(o->w);
}
static MP_DEFINE_CONST_FUN_OBJ_1(image_width_obj, image_width);

static mp_obj_t image_height(mp_obj_t self_in) {
    image_obj_t *o = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(o->h);
}
static MP_DEFINE_CONST_FUN_OBJ_1(image_height_obj, image_height);

static mp_obj_t image_to_bytes(mp_obj_t self_in) {
    image_obj_t *o = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_bytes(o->buf, o->w * o->h * 2);
}
static MP_DEFINE_CONST_FUN_OBJ_1(image_to_bytes_obj, image_to_bytes);

static mp_obj_t image_clear(mp_obj_t self_in) {
    image_obj_t *o = MP_OBJ_TO_PTR(self_in);
    memset(o->buf, 0, o->w * o->h * 2);
    return self_in;
}
static MP_DEFINE_CONST_FUN_OBJ_1(image_clear_obj, image_clear);

static mp_obj_t image_from_nv12(size_t n_args, const mp_obj_t *args) {
    if (n_args < 7) return mp_const_none;
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[0], &bufinfo, MP_BUFFER_READ);
    int src_w = mp_obj_get_int(args[1]);
    int src_h = mp_obj_get_int(args[2]);
    int dst_w = mp_obj_get_int(args[5]);
    int dst_h = mp_obj_get_int(args[6]);
    /* args[8] is the chroma plane.  It is not used: see the comment
     * where the picture is built. */

    image_obj_t *o = m_new_obj(image_obj_t);
    o->base.type = &image_type;
    o->w = dst_w; o->h = dst_h;
    o->buf = m_new(uint8_t, dst_w * dst_h * 2);
    memset(o->buf, 0, dst_w * dst_h * 2);
    o->gray = NULL;
    if (src_w == dst_w && src_h == dst_h) {
        /* The luma the sensor produced, kept as it is. */
        o->gray = m_new(uint8_t, dst_w * dst_h);
        memcpy(o->gray, bufinfo.buf, dst_w * dst_h);
    }

    const unsigned char *src = (const unsigned char *)bufinfo.buf;
    int sx_ratio = src_w / dst_w; if (sx_ratio < 1) sx_ratio = 1;
    int sy_ratio = src_h / dst_h; if (sy_ratio < 1) sy_ratio = 1;

    for (int y = 0; y < dst_h; y++) {
        for (int x = 0; x < dst_w; x++) {
            int src_x = x * sx_ratio; if (src_x >= src_w) src_x = src_w - 1;
            int src_y = y * sy_ratio; if (src_y >= src_h) src_y = src_h - 1;
            unsigned char val = src[src_y * src_w + src_x];
            /* The picture is grey: the luma is put in all three
             * channels.  The chroma plane is not used, because it is
             * not the NV12 that a YUV conversion assumes -- its
             * values are centred on sixteen, not on the hundred and
             * twenty-eight a neutral chroma has, so converting it
             * turns the whole picture green.  The luma is kept
             * separately for the decoder and the statistics, which
             * is where the precision matters. */
            unsigned short rgb = ((val >> 3) << 11) | ((val >> 2) << 5) | (val >> 3);
            o->buf[(y * dst_w + x) * 2] = rgb & 0xff;
            o->buf[(y * dst_w + x) * 2 + 1] = (rgb >> 8) & 0xff;
        }
    }
    return MP_OBJ_FROM_PTR(o);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(image_from_nv12_obj, 7, 9, image_from_nv12);

/* ------------------------------------------------------------------ *
 * the shared-memory frame
 *
 * camd publishes the preview luma into /dev/shm/krux-frame; this maps
 * it once and reads it with a seqlock, so the 691 KB socket copy is
 * gone.  Only the luma is read: the picture is built grey from it,
 * and the chroma is not used.
 * ------------------------------------------------------------------ */
#define SHM_PATH "/tmp/krux/frame"
#define SHM_MAGIC 0x4b435246u
struct shm_frame_hdr {
    volatile uint32_t seq;      /* odd = writing, even = stable */
    uint32_t magic;
    uint32_t w, h;
    uint32_t y_size;
    uint32_t frame_seq;
    uint32_t sweeping;
    uint32_t src_w, src_h;      /* the sensor window the preview came from */
};

static int shm_fd = -1;
static unsigned char *shm_base;
static size_t shm_size;

/* The QR retry crop, published by camd on request. */
static int crop_shm_fd = -1;
static unsigned char *crop_shm_base;
static size_t crop_shm_size;

static int shm_attach(void) {
    if (shm_base) return 0;
    shm_fd = open(SHM_PATH, O_RDONLY);
    if (shm_fd < 0) return -1;
    struct stat st;
    if (fstat(shm_fd, &st) < 0 ||
        st.st_size < (off_t)sizeof(struct shm_frame_hdr)) {
        close(shm_fd); shm_fd = -1;
        return -1;
    }
    shm_size = st.st_size;
    shm_base = mmap(NULL, shm_size, PROT_READ, MAP_SHARED, shm_fd, 0);
    if (shm_base == MAP_FAILED) {
        shm_base = NULL; close(shm_fd); shm_fd = -1;
        return -1;
    }
    return 0;
}

/* Build an image from a shared region: a seqlock header followed by
 * the luma.  The picture is grey, built from the luma alone. */
static mp_obj_t image_from_shm_base(unsigned char *base) {
    struct shm_frame_hdr *h = (struct shm_frame_hdr *)base;
    if (h->magic != SHM_MAGIC) return mp_const_none;
    uint32_t w = h->w, hh = h->h;
    if (w == 0 || hh == 0 || h->y_size < w * hh) return mp_const_none;

    image_obj_t *o = m_new_obj(image_obj_t);
    o->base.type = &image_type;
    o->w = w; o->h = hh;
    o->buf = m_new(uint8_t, w * hh * 2);
    o->gray = m_new(uint8_t, w * hh);

    const unsigned char *src = base + sizeof(*h);
    size_t n = (size_t)w * hh;
    for (;;) {
        uint32_t s1 = h->seq;
        if (s1 & 1u) continue;      /* a write is in progress */
        memcpy(o->gray, src, n);
        __sync_synchronize();
        uint32_t s2 = h->seq;
        if (s1 == s2) break;        /* no write happened during the copy */
    }

    for (size_t i = 0; i < n; i++) {
        unsigned char val = o->gray[i];
        unsigned short rgb = ((val >> 3) << 11) | ((val >> 2) << 5) | (val >> 3);
        o->buf[i * 2] = rgb & 0xff;
        o->buf[i * 2 + 1] = (rgb >> 8) & 0xff;
    }
    return MP_OBJ_FROM_PTR(o);
}

static mp_obj_t image_from_shared(void) {
    if (shm_attach() < 0) return mp_const_none;
    return image_from_shm_base(shm_base);
}
static MP_DEFINE_CONST_FUN_OBJ_0(image_from_shared_obj, image_from_shared);

static int crop_shm_attach(void) {
    if (crop_shm_base) return 0;
    crop_shm_fd = open("/tmp/krux/crop", O_RDONLY);
    if (crop_shm_fd < 0) return -1;
    struct stat st;
    if (fstat(crop_shm_fd, &st) < 0 ||
        st.st_size < (off_t)sizeof(struct shm_frame_hdr)) {
        close(crop_shm_fd); crop_shm_fd = -1;
        return -1;
    }
    crop_shm_size = st.st_size;
    crop_shm_base = mmap(NULL, crop_shm_size, PROT_READ, MAP_SHARED,
                         crop_shm_fd, 0);
    if (crop_shm_base == MAP_FAILED) {
        crop_shm_base = NULL; close(crop_shm_fd); crop_shm_fd = -1;
        return -1;
    }
    return 0;
}

static mp_obj_t image_from_crop(void) {
    if (crop_shm_attach() < 0) return mp_const_none;
    return image_from_shm_base(crop_shm_base);
}
static MP_DEFINE_CONST_FUN_OBJ_0(image_from_crop_obj, image_from_crop);

/* (src_w, src_h, out_w, out_h) of the preview, so the firmware can
 * compute the source pixels-per-module of a located code. */
static mp_obj_t image_shared_geometry(void) {
    if (shm_attach() < 0) return mp_const_none;
    struct shm_frame_hdr *h = (struct shm_frame_hdr *)shm_base;
    mp_obj_t t[4] = {
        mp_obj_new_int((mp_int_t)h->src_w),
        mp_obj_new_int((mp_int_t)h->src_h),
        mp_obj_new_int((mp_int_t)h->w),
        mp_obj_new_int((mp_int_t)h->h),
    };
    return mp_obj_new_tuple(4, t);
}
static MP_DEFINE_CONST_FUN_OBJ_0(image_shared_geometry_obj, image_shared_geometry);

/* Drop the shared-memory mappings.  camd recreates/truncates the
 * regions when it restarts, and a mapping made before that can be the
 * wrong size for the new mode -- reading it gives a torn or stale
 * frame.  sensor.reset() calls this so the next snapshot maps the
 * region the running daemon actually publishes. */
static mp_obj_t image_shared_reset(void) {
    if (shm_base) {
        munmap(shm_base, shm_size);
        shm_base = NULL;
    }
    if (shm_fd >= 0) {
        close(shm_fd);
        shm_fd = -1;
    }
    if (crop_shm_base) {
        munmap(crop_shm_base, crop_shm_size);
        crop_shm_base = NULL;
    }
    if (crop_shm_fd >= 0) {
        close(crop_shm_fd);
        crop_shm_fd = -1;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(image_shared_reset_obj, image_shared_reset);

/* (frame_seq, sweeping) from the shared header, so the firmware can
 * throttle and know the autofocus state without a socket round trip. */
static mp_obj_t image_shared_status(void) {
    if (shm_attach() < 0) return mp_const_none;
    struct shm_frame_hdr *h = (struct shm_frame_hdr *)shm_base;
    mp_obj_t t[2] = {
        mp_obj_new_int((mp_int_t)h->frame_seq),
        mp_obj_new_int((mp_int_t)h->sweeping),
    };
    return mp_obj_new_tuple(2, t);
}
static MP_DEFINE_CONST_FUN_OBJ_0(image_shared_status_obj, image_shared_status);

/* QR result object */
typedef struct {
    mp_obj_base_t base;
    k_quirc_result_t res;
    int grid_size;      /* 4*version+17, or 0 when not located */
} qr_obj_t;

static mp_obj_t qr_valid(mp_obj_t self_in) {
    qr_obj_t *o = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_bool(o->res.valid);
}
static MP_DEFINE_CONST_FUN_OBJ_1(qr_valid_obj, qr_valid);

static mp_obj_t qr_grid_size(mp_obj_t self_in) {
    qr_obj_t *o = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(o->grid_size);
}
static MP_DEFINE_CONST_FUN_OBJ_1(qr_grid_size_obj, qr_grid_size);

static mp_obj_t qr_payload(mp_obj_t self_in) {
    qr_obj_t *o = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_str((const char *)o->res.data.payload, o->res.data.payload_len);
}
static MP_DEFINE_CONST_FUN_OBJ_1(qr_payload_obj, qr_payload);

static mp_obj_t qr_x(mp_obj_t self_in) {
    qr_obj_t *o = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(o->res.corners[0].x);
}
static MP_DEFINE_CONST_FUN_OBJ_1(qr_x_obj, qr_x);

static mp_obj_t qr_y(mp_obj_t self_in) {
    qr_obj_t *o = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(o->res.corners[0].y);
}
static MP_DEFINE_CONST_FUN_OBJ_1(qr_y_obj, qr_y);

static mp_obj_t qr_w(mp_obj_t self_in) {
    qr_obj_t *o = MP_OBJ_TO_PTR(self_in);
    /* bounding box width */
    int minx = o->res.corners[0].x, maxx = o->res.corners[0].x;
    for (int i = 1; i < 4; i++) {
        if (o->res.corners[i].x < minx) minx = o->res.corners[i].x;
        if (o->res.corners[i].x > maxx) maxx = o->res.corners[i].x;
    }
    return mp_obj_new_int(maxx - minx);
}
static MP_DEFINE_CONST_FUN_OBJ_1(qr_w_obj, qr_w);

static mp_obj_t qr_h(mp_obj_t self_in) {
    qr_obj_t *o = MP_OBJ_TO_PTR(self_in);
    int miny = o->res.corners[0].y, maxy = o->res.corners[0].y;
    for (int i = 1; i < 4; i++) {
        if (o->res.corners[i].y < miny) miny = o->res.corners[i].y;
        if (o->res.corners[i].y > maxy) maxy = o->res.corners[i].y;
    }
    return mp_obj_new_int(maxy - miny);
}
static MP_DEFINE_CONST_FUN_OBJ_1(qr_h_obj, qr_h);

static mp_obj_t qr_corners(mp_obj_t self_in) {
    qr_obj_t *o = MP_OBJ_TO_PTR(self_in);
    mp_obj_t items[4];
    for (int i = 0; i < 4; i++) {
        mp_obj_t pt[2] = { mp_obj_new_int(o->res.corners[i].x), mp_obj_new_int(o->res.corners[i].y) };
        items[i] = mp_obj_new_tuple(2, pt);
    }
    return mp_obj_new_list(4, items);
}
static MP_DEFINE_CONST_FUN_OBJ_1(qr_corners_obj, qr_corners);

static mp_obj_t qr_rect(mp_obj_t self_in) {
    qr_obj_t *o = MP_OBJ_TO_PTR(self_in);
    int minx = o->res.corners[0].x, miny = o->res.corners[0].y;
    int maxx = minx, maxy = miny;
    for (int i = 1; i < 4; i++) {
        if (o->res.corners[i].x < minx) minx = o->res.corners[i].x;
        if (o->res.corners[i].x > maxx) maxx = o->res.corners[i].x;
        if (o->res.corners[i].y < miny) miny = o->res.corners[i].y;
        if (o->res.corners[i].y > maxy) maxy = o->res.corners[i].y;
    }
    mp_obj_t rect[4] = { mp_obj_new_int(minx), mp_obj_new_int(miny), mp_obj_new_int(maxx-minx), mp_obj_new_int(maxy-miny) };
    return mp_obj_new_tuple(4, rect);
}
static MP_DEFINE_CONST_FUN_OBJ_1(qr_rect_obj, qr_rect);

static const mp_rom_map_elem_t qr_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_payload), MP_ROM_PTR(&qr_payload_obj) },
    { MP_ROM_QSTR(MP_QSTR_x), MP_ROM_PTR(&qr_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_y), MP_ROM_PTR(&qr_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_w), MP_ROM_PTR(&qr_w_obj) },
    { MP_ROM_QSTR(MP_QSTR_h), MP_ROM_PTR(&qr_h_obj) },
    { MP_ROM_QSTR(MP_QSTR_corners), MP_ROM_PTR(&qr_corners_obj) },
    { MP_ROM_QSTR(MP_QSTR_rect), MP_ROM_PTR(&qr_rect_obj) },
    { MP_ROM_QSTR(MP_QSTR_valid), MP_ROM_PTR(&qr_valid_obj) },
    { MP_ROM_QSTR(MP_QSTR_grid_size), MP_ROM_PTR(&qr_grid_size_obj) },
};
static MP_DEFINE_CONST_DICT(qr_locals_dict, qr_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    qr_type,
    MP_QSTR_qrcode,
    0,
    locals_dict, &qr_locals_dict
);

static mp_obj_t image_find_qrcodes(size_t n_args, const mp_obj_t *args) {
    /* args: self, find_inverted (optional) */
    image_obj_t *o = MP_OBJ_TO_PTR(args[0]);
    bool find_inverted = false;
    if (n_args >= 2) find_inverted = mp_obj_get_int(args[1]);

    /* The sensor's own luma, when the frame came from the camera.
     * Only a frame that was built some other way has to be converted,
     * and then the luma is rebuilt from a five-bit-per-channel
     * picture, which is worth up to five levels on every pixel. */
    int npix = o->w * o->h;
    uint8_t *own_gray = NULL;
    const uint8_t *gray = o->gray;
    if (gray == NULL) {
        own_gray = malloc(npix);
        gray = own_gray;
        for (int i = 0; i < npix; i++) {
            unsigned short rgb = o->buf[i*2] | (o->buf[i*2+1] << 8);
            int r = (rgb >> 11) & 0x1f;
            int g = (rgb >> 5) & 0x3f;
            int b = rgb & 0x1f;
            /* Expand RGB565 channels before converting to luma so the decoder
             * receives the expected full-range 8-bit grayscale values. */
            r = (r << 3) | (r >> 2);
            g = (g << 2) | (g >> 4);
            b = (b << 3) | (b >> 2);
            own_gray[i] = (r * 77 + g * 150 + b * 29) >> 8;
        }
    }

    k_quirc_result_t results[8];
    int n = k_quirc_decode_grayscale(gray, o->w, o->h, results, 8, find_inverted);
    free(own_gray);

    mp_obj_t items[n > 0 ? n : 0];
    for (int i = 0; i < n; i++) {
        qr_obj_t *qo = m_new_obj(qr_obj_t);
        qo->base.type = &qr_type;
        qo->res = results[i];
        qo->grid_size = results[i].valid ? results[i].data.version * 4 + 17 : 0;
        items[i] = MP_OBJ_FROM_PTR(qo);
    }
    return mp_obj_new_list(n, items);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR(image_find_qrcodes_obj, 1, image_find_qrcodes);

/* Locate every QR candidate, including the ones that failed to decode,
 * so the caller can crop the region from the full-resolution frame and
 * retry.  Same luma handling as find_qrcodes. */
static mp_obj_t image_qr_scan(size_t n_args, const mp_obj_t *args) {
    image_obj_t *o = MP_OBJ_TO_PTR(args[0]);
    bool find_inverted = false;
    if (n_args >= 2) find_inverted = mp_obj_get_int(args[1]);

    int npix = o->w * o->h;
    uint8_t *own_gray = NULL;
    const uint8_t *gray = o->gray;
    if (gray == NULL) {
        own_gray = malloc(npix);
        gray = own_gray;
        for (int i = 0; i < npix; i++) {
            unsigned short rgb = o->buf[i*2] | (o->buf[i*2+1] << 8);
            int r = (rgb >> 11) & 0x1f;
            int g = (rgb >> 5) & 0x3f;
            int b = rgb & 0x1f;
            r = (r << 3) | (r >> 2);
            g = (g << 2) | (g >> 4);
            b = (b << 3) | (b >> 2);
            own_gray[i] = (r * 77 + g * 150 + b * 29) >> 8;
        }
    }

    /* One decoder for the life of the firmware: the image and pixel
     * buffers are ~1 MB and the QR page scans every frame. */
    static k_quirc_t *qr_ctx;
    if (!qr_ctx) qr_ctx = k_quirc_new();
    k_quirc_scan_t results[8];
    int n = qr_ctx ? k_quirc_scan_ctx(qr_ctx, gray, o->w, o->h, results, 8,
                                      find_inverted) : 0;
    free(own_gray);

    mp_obj_t items[n > 0 ? n : 0];
    for (int i = 0; i < n; i++) {
        qr_obj_t *qo = m_new_obj(qr_obj_t);
        qo->base.type = &qr_type;
        for (int c = 0; c < 4; c++)
            qo->res.corners[c] = results[i].corners[c];
        qo->res.data = results[i].data;
        qo->res.valid = results[i].valid;
        qo->grid_size = results[i].grid_size;
        items[i] = MP_OBJ_FROM_PTR(qo);
    }
    return mp_obj_new_list(n, items);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR(image_qr_scan_obj, 1, image_qr_scan);

/* How close to the brightest pixel a pixel has to be to count as
 * one of the bright ones. */
#define BRIGHT_MARGIN 40

/* The luma of one pixel.  The sensor's own plane when the frame came
 * from the camera, and the RGB565 picture otherwise: the latter
 * rebuilds the luma from five-bit channels, so it is worth up to five
 * levels on every pixel. */
static inline int image_luma(const image_obj_t *o, int i, const uint8_t *gray) {
    if (gray != NULL)
        return gray[i];
    unsigned short rgb = o->buf[i*2] | (o->buf[i*2+1] << 8);
    int r5 = (rgb >> 11) & 0x1f;
    int g6 = (rgb >> 5) & 0x3f;
    int b5 = rgb & 0x1f;
    int r = (r5 << 3) | (r5 >> 2);
    int g = (g6 << 2) | (g6 >> 4);
    int b = (b5 << 3) | (b5 >> 2);
    return (r * 299 + g * 587 + b * 114) / 1000;
}

/* Fill in the percentile levels from a 256-bin histogram of the
 * region.  One pass over the pixels and one over the bins. */
static void stats_percentiles(stats_obj_t *s, const image_obj_t *o,
                              int x0, int y0, int x1, int y1, long n)
{
    static uint32_t hist[256];
    long acc = 0;
    int l;
    int have50 = 0, have90 = 0, have95 = 0, have99 = 0;

    memset(hist, 0, sizeof(hist));
    for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++)
            hist[image_luma(o, y * o->w + x, o->gray)]++;
    s->l_p50 = s->l_p90 = s->l_p95 = s->l_p99 = 0;
    s->l_count = (int)n;
    for (l = 0; l < 256; l++) {
        acc += hist[l];
        if (!have50 && acc * 100 >= n * 50) { s->l_p50 = l; have50 = 1; }
        if (!have90 && acc * 100 >= n * 90) { s->l_p90 = l; have90 = 1; }
        if (!have95 && acc * 100 >= n * 95) { s->l_p95 = l; have95 = 1; }
        if (!have99 && acc * 100 >= n * 99) { s->l_p99 = l; have99 = 1; }
    }
}

static mp_obj_t image_get_statistics(mp_obj_t self_in) {
    image_obj_t *o = MP_OBJ_TO_PTR(self_in);
    stats_obj_t *s = m_new_obj(stats_obj_t);
    s->base.type = &stats_type;
    int n = o->w * o->h;
    long sum = 0;
    double sq = 0;
    int lmin = 255, lmax = 0, clip_low = 0, clip_high = 0;

    const uint8_t *gray = o->gray;
    for (int i = 0; i < n; i++) {
        int l = image_luma(o, i, gray);
        sum += l;
        if (l < lmin) lmin = l;
        if (l > lmax) lmax = l;
        if (l <= 2) clip_low++;
        if (l >= 253) clip_high++;
    }
    s->l_mean = sum / n;
    for (int i = 0; i < n; i++) {
        int l = image_luma(o, i, gray);
        double d = l - s->l_mean;
        sq += d * d;
    }
    s->l_stdev = (int)sqrt(sq / n);
    s->l_min = lmin;
    s->l_max = lmax;
    s->clipped_low = clip_low;
    s->clipped_high = clip_high;
    {
        int thresh = lmax - BRIGHT_MARGIN;
        long bsum = 0;
        int bcount = 0;
        if (thresh < 0)
            thresh = 0;
        for (int i = 0; i < n; i++) {
            int l = image_luma(o, i, o->gray);
            if (l >= thresh) {
                bsum += l;
                bcount++;
            }
        }
        s->l_bright_mean = bcount ? (int)(bsum / bcount) : 0;
        s->l_bright_count = bcount;
    }
    stats_percentiles(s, o, 0, 0, o->w, o->h, n);
    s->a_mean = 128; s->a_stdev = 10;
    s->b_mean = 128; s->b_stdev = 10;
    return MP_OBJ_FROM_PTR(s);
}
static mp_obj_t image_get_statistics_roi(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    image_obj_t *o = MP_OBJ_TO_PTR(args[0]);
    int x0 = mp_obj_get_int(args[1]);
    int y0 = mp_obj_get_int(args[2]);
    int w = mp_obj_get_int(args[3]);
    int h = mp_obj_get_int(args[4]);
    int x1 = x0 + w, y1 = y0 + h;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > o->w) x1 = o->w;
    if (y1 > o->h) y1 = o->h;
    long n = (x1 - x0) * (y1 - y0);

    stats_obj_t *s = m_new_obj(stats_obj_t);
    s->base.type = &stats_type;
    s->a_mean = 128; s->a_stdev = 10;
    s->b_mean = 128; s->b_stdev = 10;
    if (n <= 0) {
        s->l_mean = s->l_stdev = s->l_min = s->l_max = 0;
        s->clipped_low = s->clipped_high = 0;
        s->l_p50 = s->l_p90 = s->l_p95 = s->l_p99 = 0;
        s->l_count = 0;
        return MP_OBJ_FROM_PTR(s);
    }

    long sum = 0;
    int lmin = 255, lmax = 0, clip_low = 0, clip_high = 0;
    for (int y = y0; y < y1; y++) {
        for (int x = x0; x < x1; x++) {
            int l = image_luma(o, y * o->w + x, o->gray);
            sum += l;
            if (l < lmin) lmin = l;
            if (l > lmax) lmax = l;
            if (l <= 2) clip_low++;
            if (l >= 253) clip_high++;
        }
    }
    s->l_mean = sum / n;
    double sq = 0;
    for (int y = y0; y < y1; y++) {
        for (int x = x0; x < x1; x++) {
            int l = image_luma(o, y * o->w + x, o->gray);
            double d = l - s->l_mean;
            sq += d * d;
        }
    }
    s->l_stdev = (int)sqrt(sq / n);
    s->l_min = lmin;
    s->l_max = lmax;
    s->clipped_low = clip_low;
    s->clipped_high = clip_high;
    {
        int thresh = lmax - BRIGHT_MARGIN;
        long bsum = 0;
        int bcount = 0;
        if (thresh < 0)
            thresh = 0;
        for (int y = y0; y < y1; y++) {
            for (int x = x0; x < x1; x++) {
                int l = image_luma(o, y * o->w + x, o->gray);
                if (l >= thresh) {
                    bsum += l;
                    bcount++;
                }
            }
        }
        s->l_bright_mean = bcount ? (int)(bsum / bcount) : 0;
        s->l_bright_count = bcount;
    }
    stats_percentiles(s, o, x0, y0, x1, y1, n);
    return MP_OBJ_FROM_PTR(s);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(image_get_statistics_roi_obj, 5, 5,
                                           image_get_statistics_roi);

/* The MaixPy pages call get_statistics(roi=(x,y,w,h)) with a keyword. */
static mp_obj_t image_get_statistics_kw(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    (void)n_args;
    mp_map_elem_t *e = mp_map_lookup(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_roi), MP_MAP_LOOKUP);
    if (e != NULL) {
        size_t rn = 0; mp_obj_t *r = NULL;
        if (mp_obj_is_type(e->value, &mp_type_tuple) || mp_obj_is_type(e->value, &mp_type_list)) {
            mp_obj_get_array(e->value, &rn, &r);
        }
        if (rn >= 4) {
            mp_obj_t a[5] = { args[0], r[0], r[1], r[2], r[3] };
            return image_get_statistics_roi(5, a);
        }
    }
    return image_get_statistics(args[0]);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(image_get_statistics_kw_obj, 1, image_get_statistics_kw);

static mp_obj_t stats_l_mean(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->l_mean); }
static mp_obj_t stats_l_bright_mean(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->l_bright_mean); }
static mp_obj_t stats_l_bright_count(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->l_bright_count); }
static mp_obj_t stats_l_stdev(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->l_stdev); }
static mp_obj_t stats_a_mean(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->a_mean); }
static mp_obj_t stats_a_stdev(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->a_stdev); }
static mp_obj_t stats_b_mean(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->b_mean); }
static mp_obj_t stats_b_stdev(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->b_stdev); }
static mp_obj_t stats_median(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->l_mean); }
static mp_obj_t stats_l_min(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->l_min); }
static mp_obj_t stats_l_max(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->l_max); }
static mp_obj_t stats_clipped_low(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->clipped_low); }
static mp_obj_t stats_clipped_high(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->clipped_high); }
static mp_obj_t stats_l_p50(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->l_p50); }
static mp_obj_t stats_l_p90(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->l_p90); }
static mp_obj_t stats_l_p95(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->l_p95); }
static mp_obj_t stats_l_p99(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->l_p99); }
static mp_obj_t stats_l_count(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->l_count); }

static MP_DEFINE_CONST_FUN_OBJ_1(stats_l_mean_obj, stats_l_mean);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_l_bright_mean_obj, stats_l_bright_mean);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_l_bright_count_obj, stats_l_bright_count);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_l_stdev_obj, stats_l_stdev);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_a_mean_obj, stats_a_mean);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_a_stdev_obj, stats_a_stdev);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_b_mean_obj, stats_b_mean);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_b_stdev_obj, stats_b_stdev);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_median_obj, stats_median);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_l_min_obj, stats_l_min);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_l_max_obj, stats_l_max);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_clipped_low_obj, stats_clipped_low);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_clipped_high_obj, stats_clipped_high);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_l_p50_obj, stats_l_p50);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_l_p90_obj, stats_l_p90);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_l_p95_obj, stats_l_p95);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_l_p99_obj, stats_l_p99);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_l_count_obj, stats_l_count);

static const mp_rom_map_elem_t stats_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_l_mean), MP_ROM_PTR(&stats_l_mean_obj) },
    { MP_ROM_QSTR(MP_QSTR_l_bright_mean), MP_ROM_PTR(&stats_l_bright_mean_obj) },
    { MP_ROM_QSTR(MP_QSTR_l_bright_count), MP_ROM_PTR(&stats_l_bright_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_l_stdev), MP_ROM_PTR(&stats_l_stdev_obj) },
    { MP_ROM_QSTR(MP_QSTR_l_min), MP_ROM_PTR(&stats_l_min_obj) },
    { MP_ROM_QSTR(MP_QSTR_l_max), MP_ROM_PTR(&stats_l_max_obj) },
    { MP_ROM_QSTR(MP_QSTR_clipped_low), MP_ROM_PTR(&stats_clipped_low_obj) },
    { MP_ROM_QSTR(MP_QSTR_clipped_high), MP_ROM_PTR(&stats_clipped_high_obj) },
    { MP_ROM_QSTR(MP_QSTR_l_p50), MP_ROM_PTR(&stats_l_p50_obj) },
    { MP_ROM_QSTR(MP_QSTR_l_p90), MP_ROM_PTR(&stats_l_p90_obj) },
    { MP_ROM_QSTR(MP_QSTR_l_p95), MP_ROM_PTR(&stats_l_p95_obj) },
    { MP_ROM_QSTR(MP_QSTR_l_p99), MP_ROM_PTR(&stats_l_p99_obj) },
    { MP_ROM_QSTR(MP_QSTR_l_count), MP_ROM_PTR(&stats_l_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_a_mean), MP_ROM_PTR(&stats_a_mean_obj) },
    { MP_ROM_QSTR(MP_QSTR_a_stdev), MP_ROM_PTR(&stats_a_stdev_obj) },
    { MP_ROM_QSTR(MP_QSTR_b_mean), MP_ROM_PTR(&stats_b_mean_obj) },
    { MP_ROM_QSTR(MP_QSTR_b_stdev), MP_ROM_PTR(&stats_b_stdev_obj) },
    { MP_ROM_QSTR(MP_QSTR_median), MP_ROM_PTR(&stats_median_obj) },
};
static MP_DEFINE_CONST_DICT(stats_locals_dict, stats_locals_dict_table);

/* ------------------------------------------------------------------ *
 * The MaixPy image helpers the Tiny Seed (Binary Grid) page needs.
 *
 * find_blobs, get_histogram().get_threshold().value() and draw_rectangle
 * are standard MaixPy methods.  Only QR decoding was ported to this
 * image object, so the page raised AttributeError on its first frame.
 * These are small, self-contained implementations over the same luma
 * plane the statistics and decoder already use.
 * ------------------------------------------------------------------ */

typedef struct {
    mp_obj_base_t base;
    int x, y, w, h;
    int pixels;
} blob_obj_t;

static mp_obj_t blob_rect(mp_obj_t self_in) {
    blob_obj_t *b = MP_OBJ_TO_PTR(self_in);
    mp_obj_t items[4] = {
        mp_obj_new_int(b->x), mp_obj_new_int(b->y),
        mp_obj_new_int(b->w), mp_obj_new_int(b->h),
    };
    return mp_obj_new_tuple(4, items);
}
static mp_obj_t blob_x(mp_obj_t self_in) { return mp_obj_new_int(((blob_obj_t *)MP_OBJ_TO_PTR(self_in))->x); }
static mp_obj_t blob_y(mp_obj_t self_in) { return mp_obj_new_int(((blob_obj_t *)MP_OBJ_TO_PTR(self_in))->y); }
static mp_obj_t blob_w(mp_obj_t self_in) { return mp_obj_new_int(((blob_obj_t *)MP_OBJ_TO_PTR(self_in))->w); }
static mp_obj_t blob_h(mp_obj_t self_in) { return mp_obj_new_int(((blob_obj_t *)MP_OBJ_TO_PTR(self_in))->h); }
static mp_obj_t blob_pixels(mp_obj_t self_in) { return mp_obj_new_int(((blob_obj_t *)MP_OBJ_TO_PTR(self_in))->pixels); }
static MP_DEFINE_CONST_FUN_OBJ_1(blob_rect_obj, blob_rect);
static MP_DEFINE_CONST_FUN_OBJ_1(blob_x_obj, blob_x);
static MP_DEFINE_CONST_FUN_OBJ_1(blob_y_obj, blob_y);
static MP_DEFINE_CONST_FUN_OBJ_1(blob_w_obj, blob_w);
static MP_DEFINE_CONST_FUN_OBJ_1(blob_h_obj, blob_h);
static MP_DEFINE_CONST_FUN_OBJ_1(blob_pixels_obj, blob_pixels);
static const mp_rom_map_elem_t blob_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_rect), MP_ROM_PTR(&blob_rect_obj) },
    { MP_ROM_QSTR(MP_QSTR_x), MP_ROM_PTR(&blob_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_y), MP_ROM_PTR(&blob_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_w), MP_ROM_PTR(&blob_w_obj) },
    { MP_ROM_QSTR(MP_QSTR_h), MP_ROM_PTR(&blob_h_obj) },
    { MP_ROM_QSTR(MP_QSTR_pixels), MP_ROM_PTR(&blob_pixels_obj) },
};
static MP_DEFINE_CONST_DICT(blob_locals_dict, blob_locals_dict_table);
MP_DEFINE_CONST_OBJ_TYPE(blob_type, MP_QSTR_blob, 0, locals_dict, &blob_locals_dict);

/* thresholds is a list of (lo, hi) tuples, as in MaixPy. */
static bool image_pixel_in_thresholds(int val, mp_obj_t thresholds, bool invert) {
    size_t n = 0;
    mp_obj_t *items = NULL;
    if (thresholds != MP_OBJ_NULL) {
        mp_obj_get_array(thresholds, &n, &items);
    }
    bool inside = false;
    for (size_t i = 0; i < n; i++) {
        size_t tn = 0;
        mp_obj_t *t = NULL;
        mp_obj_get_array(items[i], &tn, &t);
        if (tn >= 2) {
            int lo = mp_obj_get_int(t[0]);
            int hi = mp_obj_get_int(t[1]);
            if (val >= lo && val <= hi) { inside = true; break; }
        }
    }
    return invert ? !inside : inside;
}

static mp_obj_t image_find_blobs(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    image_obj_t *o = MP_OBJ_TO_PTR(args[0]);
    mp_obj_t thresholds = (n_args >= 2) ? args[1] : MP_OBJ_NULL;
    int xstride = 1, ystride = 1, area_thresh = 10, invert = 0;
    int rx = 0, ry = 0, rw = o->w, rh = o->h;
    mp_map_elem_t *e;

    e = mp_map_lookup(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_x_stride), MP_MAP_LOOKUP);
    if (e != NULL) xstride = mp_obj_get_int(e->value);
    e = mp_map_lookup(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_y_stride), MP_MAP_LOOKUP);
    if (e != NULL) ystride = mp_obj_get_int(e->value);
    e = mp_map_lookup(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_area_threshold), MP_MAP_LOOKUP);
    if (e != NULL) area_thresh = mp_obj_get_int(e->value);
    e = mp_map_lookup(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_pixels_threshold), MP_MAP_LOOKUP);
    if (e != NULL && mp_obj_get_int(e->value) > area_thresh) area_thresh = mp_obj_get_int(e->value);
    e = mp_map_lookup(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_invert), MP_MAP_LOOKUP);
    if (e != NULL) invert = mp_obj_is_true(e->value) ? 1 : 0;
    e = mp_map_lookup(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_roi), MP_MAP_LOOKUP);
    if (e != NULL) {
        size_t rn = 0; mp_obj_t *r = NULL;
        mp_obj_get_array(e->value, &rn, &r);
        if (rn >= 4) { rx = mp_obj_get_int(r[0]); ry = mp_obj_get_int(r[1]); rw = mp_obj_get_int(r[2]); rh = mp_obj_get_int(r[3]); }
    }
    if (xstride < 1) xstride = 1;
    if (ystride < 1) ystride = 1;
    if (rx < 0) rx = 0;
    if (ry < 0) ry = 0;
    if (rx + rw > o->w) rw = o->w - rx;
    if (ry + rh > o->h) rh = o->h - ry;

    /* The luma plane is the sensor's own Y when this is a camera frame;
     * fall back to the RGB565 buffer when it is not. */
    const uint8_t *gray = o->gray;
    uint8_t *own = NULL;
    if (gray == NULL) {
        own = m_new(uint8_t, o->w * o->h);
        for (int i = 0; i < o->w * o->h; i++) {
            unsigned short rgb = o->buf[i * 2] | (o->buf[i * 2 + 1] << 8);
            int r = (rgb >> 11) & 0x1f, g = (rgb >> 5) & 0x3f, b = rgb & 0x1f;
            own[i] = (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
        }
        gray = own;
    }

    int gw = (rw + xstride - 1) / xstride;
    int gh = (rh + ystride - 1) / ystride;
    if (gw <= 0 || gh <= 0) {
        if (own != NULL) m_del(uint8_t, own, o->w * o->h);
        return mp_obj_new_list(0, NULL);
    }
    uint8_t *visited = m_new0(uint8_t, gw * gh);
    uint32_t *stack = m_new(uint32_t, gw * gh);
    mp_obj_list_t *result = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));

    for (int cy = 0; cy < gh; cy++) {
        for (int cx = 0; cx < gw; cx++) {
            int start = cy * gw + cx;
            if (visited[start]) continue;
            int px = rx + cx * xstride, py = ry + cy * ystride;
            if (!image_pixel_in_thresholds(gray[py * o->w + px], thresholds, invert)) continue;

            int sp = 0;
            stack[sp++] = (uint32_t)start;
            visited[start] = 1;
            int mnx = cx, mxx = cx, mny = cy, mxy = cy, count = 0;
            while (sp > 0) {
                int cell = (int)stack[--sp];
                int x = cell % gw, y = cell / gw;
                count++;
                if (x < mnx) mnx = x;
                if (x > mxx) mxx = x;
                if (y < mny) mny = y;
                if (y > mxy) mxy = y;
                int nb[4] = {x - 1, x + 1, x, x};
                int ny[4] = {y, y, y - 1, y + 1};
                for (int k = 0; k < 4; k++) {
                    if (nb[k] < 0 || nb[k] >= gw || ny[k] < 0 || ny[k] >= gh) continue;
                    int ni = ny[k] * gw + nb[k];
                    if (visited[ni]) continue;
                    int nx = rx + nb[k] * xstride, ny2 = ry + ny[k] * ystride;
                    if (!image_pixel_in_thresholds(gray[ny2 * o->w + nx], thresholds, invert)) continue;
                    visited[ni] = 1;
                    stack[sp++] = (uint32_t)ni;
                }
            }
            int area = count * xstride * ystride;
            if (area >= area_thresh) {
                blob_obj_t *b = m_new_obj(blob_obj_t);
                b->base.type = &blob_type;
                b->x = rx + mnx * xstride;
                b->y = ry + mny * ystride;
                b->w = (mxx - mnx + 1) * xstride;
                b->h = (mxy - mny + 1) * ystride;
                b->pixels = area;
                mp_obj_list_append(MP_OBJ_FROM_PTR(result), MP_OBJ_FROM_PTR(b));
            }
        }
    }
    m_del(uint8_t, visited, gw * gh);
    m_del(uint32_t, stack, gw * gh);
    if (own != NULL) m_del(uint8_t, own, o->w * o->h);
    return MP_OBJ_FROM_PTR(result);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(image_find_blobs_obj, 1, image_find_blobs);

/* get_histogram().get_threshold().value() -- Otsu's method over the luma. */
typedef struct { mp_obj_base_t base; int value; } threshold_obj_t;
typedef struct { mp_obj_base_t base; int otsu; } hist_obj_t;

static mp_obj_t threshold_value(mp_obj_t self_in) { return mp_obj_new_int(((threshold_obj_t *)MP_OBJ_TO_PTR(self_in))->value); }
static MP_DEFINE_CONST_FUN_OBJ_1(threshold_value_obj, threshold_value);
static const mp_rom_map_elem_t threshold_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_value), MP_ROM_PTR(&threshold_value_obj) },
};
static MP_DEFINE_CONST_DICT(threshold_locals_dict, threshold_locals_dict_table);
MP_DEFINE_CONST_OBJ_TYPE(threshold_type, MP_QSTR_threshold, 0, locals_dict, &threshold_locals_dict);

static mp_obj_t hist_get_threshold(mp_obj_t self_in) {
    hist_obj_t *h = MP_OBJ_TO_PTR(self_in);
    threshold_obj_t *t = m_new_obj(threshold_obj_t);
    t->base.type = &threshold_type;
    t->value = h->otsu;
    return MP_OBJ_FROM_PTR(t);
}
static MP_DEFINE_CONST_FUN_OBJ_1(hist_get_threshold_obj, hist_get_threshold);
static const mp_rom_map_elem_t hist_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_get_threshold), MP_ROM_PTR(&hist_get_threshold_obj) },
};
static MP_DEFINE_CONST_DICT(hist_locals_dict, hist_locals_dict_table);
MP_DEFINE_CONST_OBJ_TYPE(hist_type, MP_QSTR_histogram, 0, locals_dict, &hist_locals_dict);

static mp_obj_t image_get_histogram(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    image_obj_t *o = MP_OBJ_TO_PTR(args[0]);
    int rx = 0, ry = 0, rw = o->w, rh = o->h;
    mp_map_elem_t *e = mp_map_lookup(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_roi), MP_MAP_LOOKUP);
    if (e != NULL) {
        size_t rn = 0; mp_obj_t *r = NULL;
        mp_obj_get_array(e->value, &rn, &r);
        if (rn >= 4) { rx = mp_obj_get_int(r[0]); ry = mp_obj_get_int(r[1]); rw = mp_obj_get_int(r[2]); rh = mp_obj_get_int(r[3]); }
    }
    if (rx < 0) rx = 0;
    if (ry < 0) ry = 0;
    if (rx + rw > o->w) rw = o->w - rx;
    if (ry + rh > o->h) rh = o->h - ry;

    uint32_t hist[256] = {0};
    long total = 0;
    const uint8_t *gray = o->gray;
    if (gray != NULL) {
        for (int y = ry; y < ry + rh; y++)
            for (int x = rx; x < rx + rw; x++) { hist[gray[y * o->w + x]]++; total++; }
    } else {
        for (int y = ry; y < ry + rh; y++)
            for (int x = rx; x < rx + rw; x++) {
                unsigned short rgb = o->buf[(y * o->w + x) * 2] | (o->buf[(y * o->w + x) * 2 + 1] << 8);
                int r = (rgb >> 11) & 0x1f, g = (rgb >> 5) & 0x3f, b = rgb & 0x1f;
                hist[(uint8_t)((r * 77 + g * 150 + b * 29) >> 8)]++; total++;
            }
    }
    /* Otsu: maximise the between-class variance. */
    int otsu = 128;
    if (total > 0) {
        double sum = 0;
        for (int i = 0; i < 256; i++) sum += (double)i * hist[i];
        double sumB = 0, maxVar = -1;
        long wB = 0;
        for (int t = 0; t < 256; t++) {
            wB += hist[t];
            if (wB == 0) continue;
            long wF = total - wB;
            if (wF == 0) break;
            sumB += (double)t * hist[t];
            double mB = sumB / wB, mF = (sum - sumB) / wF;
            double between = (double)wB * wF * (mB - mF) * (mB - mF);
            if (between > maxVar) { maxVar = between; otsu = t; }
        }
    }
    hist_obj_t *h = m_new_obj(hist_obj_t);
    h->base.type = &hist_type;
    h->otsu = otsu;
    return MP_OBJ_FROM_PTR(h);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(image_get_histogram_obj, 1, image_get_histogram);

/* draw_rectangle((x,y,w,h), color, thickness=N, fill=N) -- draws into the
 * RGB565 buffer so lcd.display(img) shows the overlay. */
static mp_obj_t image_draw_rectangle(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    image_obj_t *o = MP_OBJ_TO_PTR(args[0]);
    int x = 0, y = 0, w = 0, h = 0, color = 0xFFFF, thickness = 1, fill = 0;
    size_t ai = 1;
    mp_obj_t first = (n_args > 1) ? args[1] : MP_OBJ_NULL;
    size_t rn = 0; mp_obj_t *r = NULL;
    if (first != MP_OBJ_NULL && (mp_obj_is_type(first, &mp_type_tuple) || mp_obj_is_type(first, &mp_type_list))) {
        mp_obj_get_array(first, &rn, &r);
    }
    if (rn >= 4) {
        x = mp_obj_get_int(r[0]); y = mp_obj_get_int(r[1]);
        w = mp_obj_get_int(r[2]); h = mp_obj_get_int(r[3]);
        ai = 2;
    } else if (n_args >= 5) {
        x = mp_obj_get_int(args[1]); y = mp_obj_get_int(args[2]);
        w = mp_obj_get_int(args[3]); h = mp_obj_get_int(args[4]);
        ai = 5;
    }
    if (n_args > ai) color = mp_obj_get_int(args[ai]);
    mp_map_elem_t *e;
    e = mp_map_lookup(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_color), MP_MAP_LOOKUP);
    if (e != NULL) color = mp_obj_get_int(e->value);
    e = mp_map_lookup(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_thickness), MP_MAP_LOOKUP);
    if (e != NULL) thickness = mp_obj_get_int(e->value);
    e = mp_map_lookup(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_fill), MP_MAP_LOOKUP);
    if (e != NULL) fill = mp_obj_is_true(e->value) ? 1 : 0;
    if (thickness < 1) thickness = 1;

    for (int yy = y; yy < y + h && yy < o->h; yy++) {
        for (int xx = x; xx < x + w && xx < o->w; xx++) {
            bool on = fill;
            if (!on) {
                for (int t = 0; t < thickness; t++) {
                    if (yy == y + t || yy == y + h - 1 - t || xx == x + t || xx == x + w - 1 - t) { on = true; break; }
                }
            }
            if (on) {
                int idx = (yy * o->w + xx) * 2;
                o->buf[idx] = color & 0xff;
                o->buf[idx + 1] = (color >> 8) & 0xff;
            }
        }
    }
    return MP_OBJ_FROM_PTR(o);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(image_draw_rectangle_obj, 1, image_draw_rectangle);

static mp_obj_t image_find_qrcodes_method(size_t n_args, const mp_obj_t *args) {
    mp_obj_t a[2] = { args[0], (n_args >= 2) ? args[1] : mp_obj_new_bool(false) };
    return image_find_qrcodes(2, a);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(image_find_qrcodes_method_obj, 1, 2, image_find_qrcodes_method);

static mp_obj_t image_qr_scan_method(size_t n_args, const mp_obj_t *args) {
    mp_obj_t a[2] = { args[0], (n_args >= 2) ? args[1] : mp_obj_new_bool(false) };
    return image_qr_scan(2, a);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(image_qr_scan_method_obj, 1, 2, image_qr_scan_method);

static const mp_rom_map_elem_t image_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_width), MP_ROM_PTR(&image_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_height), MP_ROM_PTR(&image_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_to_bytes), MP_ROM_PTR(&image_to_bytes_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear), MP_ROM_PTR(&image_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_statistics), MP_ROM_PTR(&image_get_statistics_kw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_statistics_roi), MP_ROM_PTR(&image_get_statistics_roi_obj) },
    { MP_ROM_QSTR(MP_QSTR_find_blobs), MP_ROM_PTR(&image_find_blobs_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_histogram), MP_ROM_PTR(&image_get_histogram_obj) },
    { MP_ROM_QSTR(MP_QSTR_draw_rectangle), MP_ROM_PTR(&image_draw_rectangle_obj) },
    { MP_ROM_QSTR(MP_QSTR_find_qrcodes), MP_ROM_PTR(&image_find_qrcodes_method_obj) },
    { MP_ROM_QSTR(MP_QSTR_qr_scan), MP_ROM_PTR(&image_qr_scan_method_obj) },
};
static MP_DEFINE_CONST_DICT(image_locals_dict, image_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    image_type,
    MP_QSTR_Image,
    0,
    make_new, image_make_new,
    locals_dict, &image_locals_dict
);

MP_DEFINE_CONST_OBJ_TYPE(
    stats_type,
    MP_QSTR_statistics,
    0,
    locals_dict, &stats_locals_dict
);

static const mp_rom_map_elem_t image_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR__image) },
    { MP_ROM_QSTR(MP_QSTR_Image), MP_ROM_PTR(&image_type) },
    { MP_ROM_QSTR(MP_QSTR_from_nv12), MP_ROM_PTR(&image_from_nv12_obj) },
    { MP_ROM_QSTR(MP_QSTR_from_shared), MP_ROM_PTR(&image_from_shared_obj) },
    { MP_ROM_QSTR(MP_QSTR_from_crop), MP_ROM_PTR(&image_from_crop_obj) },
    { MP_ROM_QSTR(MP_QSTR_shared_status), MP_ROM_PTR(&image_shared_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_shared_geometry), MP_ROM_PTR(&image_shared_geometry_obj) },
    { MP_ROM_QSTR(MP_QSTR_shared_reset), MP_ROM_PTR(&image_shared_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_find_qrcodes), MP_ROM_PTR(&image_find_qrcodes_obj) },
    { MP_ROM_QSTR(MP_QSTR_qr_scan), MP_ROM_PTR(&image_qr_scan_obj) },
};
static MP_DEFINE_CONST_DICT(image_module_globals, image_module_globals_table);

const mp_obj_module_t image_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&image_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR__image, image_module);
