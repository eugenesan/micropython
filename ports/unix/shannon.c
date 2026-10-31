/*
 * shannon -- the image-entropy helper Krux uses on the camera frame.
 *
 * The device image is RGB565, i.e. two bytes per pixel.  entropy_img16b()
 * treats the buffer as 16-bit samples, builds the value histogram and
 * returns Shannon's entropy in bits per pixel.
 */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "py/runtime.h"
#include "py/obj.h"

static mp_obj_t shannon_entropy_img16b(mp_obj_t buf_in) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_READ);

    size_t samples = bufinfo.len / 2;
    if (samples == 0) {
        return mp_obj_new_float(0.0f);
    }

    uint32_t *hist = calloc(65536, sizeof(uint32_t));
    if (hist == NULL) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("shannon histogram"));
    }

    const unsigned char *p = (const unsigned char *)bufinfo.buf;
    for (size_t i = 0; i < samples; i++) {
        uint16_t v = (uint16_t)p[2 * i] | ((uint16_t)p[2 * i + 1] << 8);
        hist[v]++;
    }

    double entropy = 0.0;
    for (int i = 0; i < 65536; i++) {
        if (hist[i] != 0) {
            double prob = (double)hist[i] / (double)samples;
            entropy -= prob * log2(prob);
        }
    }
    free(hist);
    return mp_obj_new_float((mp_float_t)entropy);
}
static MP_DEFINE_CONST_FUN_OBJ_1(shannon_entropy_img16b_obj,
                                 shannon_entropy_img16b);

static const mp_rom_map_elem_t shannon_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_shannon) },
    { MP_ROM_QSTR(MP_QSTR_entropy_img16b),
      MP_ROM_PTR(&shannon_entropy_img16b_obj) },
};
static MP_DEFINE_CONST_DICT(shannon_module_globals,
                            shannon_module_globals_table);

const mp_obj_module_t shannon_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&shannon_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_shannon, shannon_module);
