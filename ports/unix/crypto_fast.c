#include "py/obj.h"
#include "py/runtime.h"
#include "py/builtin.h"
#include "mbedtls/sha256.h"
#include "mbedtls/sha512.h"
#include "mbedtls/md.h"
#include "mbedtls/pkcs5.h"

static mp_obj_t cf_sha256(mp_obj_t data_obj) {
    mp_buffer_info_t buf;
    mp_get_buffer_raise(data_obj, &buf, MP_BUFFER_READ);
    unsigned char out[32];
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);
    mbedtls_sha256_update(&ctx, buf.buf, buf.len);
    mbedtls_sha256_finish(&ctx, out);
    mbedtls_sha256_free(&ctx);
    return mp_obj_new_bytes(out, 32);
}
static MP_DEFINE_CONST_FUN_OBJ_1(cf_sha256_obj, cf_sha256);

static mp_obj_t cf_sha512(mp_obj_t data_obj) {
    mp_buffer_info_t buf;
    mp_get_buffer_raise(data_obj, &buf, MP_BUFFER_READ);
    unsigned char out[64];
    mbedtls_sha512_context ctx;
    mbedtls_sha512_init(&ctx);
    mbedtls_sha512_starts(&ctx, 0);
    mbedtls_sha512_update(&ctx, buf.buf, buf.len);
    mbedtls_sha512_finish(&ctx, out);
    mbedtls_sha512_free(&ctx);
    return mp_obj_new_bytes(out, 64);
}
static MP_DEFINE_CONST_FUN_OBJ_1(cf_sha512_obj, cf_sha512);

static mp_obj_t cf_pbkdf2_hmac(size_t n_args, const mp_obj_t *args) {
    const char *name = mp_obj_str_get_str(args[0]);
    mp_buffer_info_t pw, salt;
    mp_get_buffer_raise(args[1], &pw, MP_BUFFER_READ);
    mp_get_buffer_raise(args[2], &salt, MP_BUFFER_READ);
    mp_int_t iters = mp_obj_get_int(args[3]);
    mp_int_t dklen = mp_obj_get_int(args[4]);
    mbedtls_md_type_t md;
    if (strcmp(name, "sha512") == 0) md = MBEDTLS_MD_SHA512;
    else if (strcmp(name, "sha256") == 0) md = MBEDTLS_MD_SHA256;
    else if (strcmp(name, "sha1") == 0) md = MBEDTLS_MD_SHA1;
    else mp_raise_ValueError(MP_ERROR_TEXT("unsupported hash"));
    if (dklen <= 0) dklen = (md == MBEDTLS_MD_SHA512) ? 64 : 32;
    vstr_t vstr;
    vstr_init_len(&vstr, dklen);
    int rc = mbedtls_pkcs5_pbkdf2_hmac_ext(md, pw.buf, pw.len, salt.buf, salt.len, iters, dklen, (unsigned char *)vstr.buf);
    if (rc != 0) mp_raise_ValueError(MP_ERROR_TEXT("pbkdf2 failed"));
    return mp_obj_new_bytes((const byte *)vstr.buf, dklen);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cf_pbkdf2_hmac_obj, 5, 5, cf_pbkdf2_hmac);

static const mp_rom_map_elem_t crypto_fast_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_crypto_fast) },
    { MP_ROM_QSTR(MP_QSTR_sha256), MP_ROM_PTR(&cf_sha256_obj) },
    { MP_ROM_QSTR(MP_QSTR_sha512), MP_ROM_PTR(&cf_sha512_obj) },
    { MP_ROM_QSTR(MP_QSTR_pbkdf2_hmac), MP_ROM_PTR(&cf_pbkdf2_hmac_obj) },
};
static MP_DEFINE_CONST_DICT(crypto_fast_globals, crypto_fast_globals_table);
const mp_obj_module_t mp_module_crypto_fast = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&crypto_fast_globals,
};
MP_REGISTER_MODULE(MP_QSTR_crypto_fast, mp_module_crypto_fast);
