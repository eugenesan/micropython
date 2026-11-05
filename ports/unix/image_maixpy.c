#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "py/runtime.h"
#include "py/obj.h"
#include "k_quirc.h"

typedef struct {
    mp_obj_base_t base;
    int w, h;
    unsigned char *buf;
} image_obj_t;

typedef struct {
    mp_obj_base_t base;
    int l_mean, l_stdev, a_mean, a_stdev, b_mean, b_stdev;
    int l_min, l_max, clipped_low, clipped_high;
} stats_obj_t;

/* forward declarations */
const mp_obj_type_t stats_type;
const mp_obj_type_t image_type;
static mp_obj_t image_get_statistics(mp_obj_t self_in);

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

    image_obj_t *o = m_new_obj(image_obj_t);
    o->base.type = &image_type;
    o->w = dst_w; o->h = dst_h;
    o->buf = m_new(uint8_t, dst_w * dst_h * 2);
    memset(o->buf, 0, dst_w * dst_h * 2);

    const unsigned char *src = (const unsigned char *)bufinfo.buf;
    int sx_ratio = src_w / dst_w; if (sx_ratio < 1) sx_ratio = 1;
    int sy_ratio = src_h / dst_h; if (sy_ratio < 1) sy_ratio = 1;

    for (int y = 0; y < dst_h; y++) {
        for (int x = 0; x < dst_w; x++) {
            int src_x = x * sx_ratio; if (src_x >= src_w) src_x = src_w - 1;
            int src_y = y * sy_ratio; if (src_y >= src_h) src_y = src_h - 1;
            unsigned char val = src[src_y * src_w + src_x];
            unsigned short rgb = ((val >> 3) << 11) | ((val >> 2) << 5) | (val >> 3);
            o->buf[(y * dst_w + x) * 2] = rgb & 0xff;
            o->buf[(y * dst_w + x) * 2 + 1] = (rgb >> 8) & 0xff;
        }
    }
    return MP_OBJ_FROM_PTR(o);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(image_from_nv12_obj, 7, 8, image_from_nv12);

/* QR result object */
typedef struct {
    mp_obj_base_t base;
    k_quirc_result_t res;
} qr_obj_t;

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

    /* convert RGB565 to grayscale */
    int npix = o->w * o->h;
    uint8_t *gray = malloc(npix);
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
        gray[i] = (r * 77 + g * 150 + b * 29) >> 8;
    }

    k_quirc_result_t results[8];
    int n = k_quirc_decode_grayscale(gray, o->w, o->h, results, 8, find_inverted);
    free(gray);

    mp_obj_t items[n > 0 ? n : 0];
    for (int i = 0; i < n; i++) {
        qr_obj_t *qo = m_new_obj(qr_obj_t);
        qo->base.type = &qr_type;
        qo->res = results[i];
        items[i] = MP_OBJ_FROM_PTR(qo);
    }
    return mp_obj_new_list(n, items);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR(image_find_qrcodes_obj, 1, image_find_qrcodes);

static mp_obj_t image_get_statistics(mp_obj_t self_in) {
    image_obj_t *o = MP_OBJ_TO_PTR(self_in);
    stats_obj_t *s = m_new_obj(stats_obj_t);
    s->base.type = &stats_type;
    int n = o->w * o->h;
    long sum = 0;
    double sq = 0;
    int lmin = 255, lmax = 0, clip_low = 0, clip_high = 0;

    for (int i = 0; i < n; i++) {
        unsigned short rgb = o->buf[i * 2] | (o->buf[i * 2 + 1] << 8);
        int r5 = (rgb >> 11) & 0x1f;
        int g6 = (rgb >> 5) & 0x3f;
        int b5 = rgb & 0x1f;
        int r = (r5 << 3) | (r5 >> 2);
        int g = (g6 << 2) | (g6 >> 4);
        int b = (b5 << 3) | (b5 >> 2);
        int l = (r * 299 + g * 587 + b * 114) / 1000;
        sum += l;
        if (l < lmin) lmin = l;
        if (l > lmax) lmax = l;
        if (l <= 2) clip_low++;
        if (l >= 253) clip_high++;
    }
    s->l_mean = sum / n;
    for (int i = 0; i < n; i++) {
        unsigned short rgb = o->buf[i * 2] | (o->buf[i * 2 + 1] << 8);
        int r5 = (rgb >> 11) & 0x1f;
        int g6 = (rgb >> 5) & 0x3f;
        int b5 = rgb & 0x1f;
        int r = (r5 << 3) | (r5 >> 2);
        int g = (g6 << 2) | (g6 >> 4);
        int b = (b5 << 3) | (b5 >> 2);
        int l = (r * 299 + g * 587 + b * 114) / 1000;
        double d = l - s->l_mean;
        sq += d * d;
    }
    s->l_stdev = (int)sqrt(sq / n);
    s->l_min = lmin;
    s->l_max = lmax;
    s->clipped_low = clip_low;
    s->clipped_high = clip_high;
    s->a_mean = 128; s->a_stdev = 10;
    s->b_mean = 128; s->b_stdev = 10;
    return MP_OBJ_FROM_PTR(s);
}
static MP_DEFINE_CONST_FUN_OBJ_1(image_get_statistics_obj, image_get_statistics);

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
        return MP_OBJ_FROM_PTR(s);
    }

    long sum = 0;
    int lmin = 255, lmax = 0, clip_low = 0, clip_high = 0;
    for (int y = y0; y < y1; y++) {
        for (int x = x0; x < x1; x++) {
            int i = (y * o->w + x) * 2;
            unsigned short rgb = o->buf[i] | (o->buf[i + 1] << 8);
            int r5 = (rgb >> 11) & 0x1f;
            int g6 = (rgb >> 5) & 0x3f;
            int b5 = rgb & 0x1f;
            int r = (r5 << 3) | (r5 >> 2);
            int g = (g6 << 2) | (g6 >> 4);
            int b = (b5 << 3) | (b5 >> 2);
            int l = (r * 299 + g * 587 + b * 114) / 1000;
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
            int i = (y * o->w + x) * 2;
            unsigned short rgb = o->buf[i] | (o->buf[i + 1] << 8);
            int r5 = (rgb >> 11) & 0x1f;
            int g6 = (rgb >> 5) & 0x3f;
            int b5 = rgb & 0x1f;
            int r = (r5 << 3) | (r5 >> 2);
            int g = (g6 << 2) | (g6 >> 4);
            int b = (b5 << 3) | (b5 >> 2);
            int l = (r * 299 + g * 587 + b * 114) / 1000;
            double d = l - s->l_mean;
            sq += d * d;
        }
    }
    s->l_stdev = (int)sqrt(sq / n);
    s->l_min = lmin;
    s->l_max = lmax;
    s->clipped_low = clip_low;
    s->clipped_high = clip_high;
    return MP_OBJ_FROM_PTR(s);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(image_get_statistics_roi_obj, 5, 5,
                                           image_get_statistics_roi);

static mp_obj_t stats_l_mean(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->l_mean); }
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

static MP_DEFINE_CONST_FUN_OBJ_1(stats_l_mean_obj, stats_l_mean);
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

static const mp_rom_map_elem_t stats_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_l_mean), MP_ROM_PTR(&stats_l_mean_obj) },
    { MP_ROM_QSTR(MP_QSTR_l_stdev), MP_ROM_PTR(&stats_l_stdev_obj) },
    { MP_ROM_QSTR(MP_QSTR_l_min), MP_ROM_PTR(&stats_l_min_obj) },
    { MP_ROM_QSTR(MP_QSTR_l_max), MP_ROM_PTR(&stats_l_max_obj) },
    { MP_ROM_QSTR(MP_QSTR_clipped_low), MP_ROM_PTR(&stats_clipped_low_obj) },
    { MP_ROM_QSTR(MP_QSTR_clipped_high), MP_ROM_PTR(&stats_clipped_high_obj) },
    { MP_ROM_QSTR(MP_QSTR_a_mean), MP_ROM_PTR(&stats_a_mean_obj) },
    { MP_ROM_QSTR(MP_QSTR_a_stdev), MP_ROM_PTR(&stats_a_stdev_obj) },
    { MP_ROM_QSTR(MP_QSTR_b_mean), MP_ROM_PTR(&stats_b_mean_obj) },
    { MP_ROM_QSTR(MP_QSTR_b_stdev), MP_ROM_PTR(&stats_b_stdev_obj) },
    { MP_ROM_QSTR(MP_QSTR_median), MP_ROM_PTR(&stats_median_obj) },
};
static MP_DEFINE_CONST_DICT(stats_locals_dict, stats_locals_dict_table);

static mp_obj_t image_find_qrcodes_method(size_t n_args, const mp_obj_t *args) {
    mp_obj_t a[2] = { args[0], (n_args >= 2) ? args[1] : mp_obj_new_bool(false) };
    return image_find_qrcodes(2, a);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(image_find_qrcodes_method_obj, 1, 2, image_find_qrcodes_method);

static const mp_rom_map_elem_t image_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_width), MP_ROM_PTR(&image_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_height), MP_ROM_PTR(&image_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_to_bytes), MP_ROM_PTR(&image_to_bytes_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear), MP_ROM_PTR(&image_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_statistics), MP_ROM_PTR(&image_get_statistics_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_statistics_roi), MP_ROM_PTR(&image_get_statistics_roi_obj) },
    { MP_ROM_QSTR(MP_QSTR_find_qrcodes), MP_ROM_PTR(&image_find_qrcodes_method_obj) },
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
    { MP_ROM_QSTR(MP_QSTR_find_qrcodes), MP_ROM_PTR(&image_find_qrcodes_obj) },
};
static MP_DEFINE_CONST_DICT(image_module_globals, image_module_globals_table);

const mp_obj_module_t image_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&image_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR__image, image_module);
