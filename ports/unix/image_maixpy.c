#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "py/runtime.h"
#include "py/obj.h"

typedef struct {
    mp_obj_base_t base;
    int w, h;
    unsigned char *buf;
} image_obj_t;

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
    o->buf = calloc(o->w * o->h * 2, 1);
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

static mp_obj_t image_from_nv12(size_t n_args, const mp_obj_t *args) __attribute__((used)) {
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
    o->buf = calloc(dst_w * dst_h * 2, 1);

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
static MP_DEFINE_VAR_ARGS_OBJ(image_from_nv12_obj, 7, image_from_nv12);

static mp_obj_t image_find_qrcodes(size_t n_args, const mp_obj_t *args) __attribute__((used)) {
    return mp_obj_new_list(0, NULL);
}
static MP_DEFINE_VAR_ARGS_OBJ(image_find_qrcodes_obj, 0, image_find_qrcodes);

typedef struct {
    mp_obj_base_t base;
    int l_mean, l_stdev, a_mean, a_stdev, b_mean, b_stdev;
} stats_obj_t;

static mp_obj_t image_get_statistics(mp_obj_t self_in) {
    image_obj_t *o = MP_OBJ_TO_PTR(self_in);
    stats_obj_t *s = m_new_obj(stats_obj_t);
    s->base.type = &stats_type;
    long sum = 0;
    for (int i = 0; i < o->w * o->h; i++) {
        sum += o->buf[i * 2 + 1] >> 3;
    }
    s->l_mean = sum / (o->w * o->h);
    s->l_stdev = 10;
    s->a_mean = 128; s->a_stdev = 10;
    s->b_mean = 128; s->b_stdev = 10;
    return MP_OBJ_FROM_PTR(s);
}
static MP_DEFINE_CONST_FUN_OBJ_1(image_get_statistics_obj, image_get_statistics);

static mp_obj_t stats_l_mean(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->l_mean); }
static mp_obj_t stats_l_stdev(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->l_stdev); }
static mp_obj_t stats_a_mean(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->a_mean); }
static mp_obj_t stats_a_stdev(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->a_stdev); }
static mp_obj_t stats_b_mean(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->b_mean); }
static mp_obj_t stats_b_stdev(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->b_stdev); }
static mp_obj_t stats_median(mp_obj_t self_in) { stats_obj_t *s = MP_OBJ_TO_PTR(self_in); return mp_obj_new_int(s->l_mean); }

static MP_DEFINE_CONST_FUN_OBJ_1(stats_l_mean_obj, stats_l_mean);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_l_stdev_obj, stats_l_stdev);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_a_mean_obj, stats_a_mean);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_a_stdev_obj, stats_a_stdev);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_b_mean_obj, stats_b_mean);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_b_stdev_obj, stats_b_stdev);
static MP_DEFINE_CONST_FUN_OBJ_1(stats_median_obj, stats_median);

static const mp_rom_map_elem_t stats_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_l_mean), MP_ROM_PTR(&stats_l_mean_obj) },
    { MP_ROM_QSTR(MP_QSTR_l_stdev), MP_ROM_PTR(&stats_l_stdev_obj) },
    { MP_ROM_QSTR(MP_QSTR_a_mean), MP_ROM_PTR(&stats_a_mean_obj) },
    { MP_ROM_QSTR(MP_QSTR_a_stdev), MP_ROM_PTR(&stats_a_stdev_obj) },
    { MP_ROM_QSTR(MP_QSTR_b_mean), MP_ROM_PTR(&stats_b_mean_obj) },
    { MP_ROM_QSTR(MP_QSTR_b_stdev), MP_ROM_PTR(&stats_b_stdev_obj) },
    { MP_ROM_QSTR(MP_QSTR_median), MP_ROM_PTR(&stats_median_obj) },
};
static MP_DEFINE_CONST_DICT(stats_locals_dict, stats_locals_dict_table);

static const mp_rom_map_elem_t image_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_width), MP_ROM_PTR(&image_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_height), MP_ROM_PTR(&image_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_to_bytes), MP_ROM_PTR(&image_to_bytes_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear), MP_ROM_PTR(&image_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_from_nv12), MP_ROM_PTR(&image_from_nv12_obj) },
    { MP_ROM_QSTR(MP_QSTR_find_qrcodes), MP_ROM_PTR(&image_find_qrcodes_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_statistics), MP_ROM_PTR(&image_get_statistics_obj) },
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
};
static MP_DEFINE_CONST_DICT(image_module_globals, image_module_globals_table);

const mp_obj_module_t image_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&image_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR__image, image_module);