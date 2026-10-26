// Include required definitions first.
#include "py/obj.h"
#include "py/runtime.h"
#include "py/builtin.h"
#include "qrcodegen.h"
#include "string.h"

static mp_obj_t qrcode_encode(mp_obj_t text_obj) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(text_obj, &bufinfo, MP_BUFFER_READ);

    enum qrcodegen_Ecc errCorLvl = qrcodegen_Ecc_LOW;  // Error correction level
    
    uint8_t qrcode[qrcodegen_BUFFER_LEN_MAX];
    uint8_t tempBuffer[qrcodegen_BUFFER_LEN_MAX];
    memcpy(tempBuffer, bufinfo.buf, bufinfo.len);
    bool ok = false;
    bool is_bytes = false;
    for (size_t i = 0; i < bufinfo.len; i++) {
        if (tempBuffer[i] == 0) { // \x00 will cut the byte array
            is_bytes = true;
        }
    }
    if (is_bytes) {
        ok = qrcodegen_encodeBinary(tempBuffer, bufinfo.len, qrcode, errCorLvl,
                                     qrcodegen_VERSION_MIN, qrcodegen_VERSION_MAX, qrcodegen_Mask_AUTO, true);
    } else {
        ok = qrcodegen_encodeText(bufinfo.buf, tempBuffer, qrcode, errCorLvl,
                                  qrcodegen_VERSION_MIN, qrcodegen_VERSION_MAX, qrcodegen_Mask_AUTO, true);
    }
    if (!ok) {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Failed to encode"));
    }
    int size = qrcodegen_getSize(qrcode);

    size_t bufsize = (size * size + 7) / 8;
    vstr_t vstr;
    vstr_init_len(&vstr, bufsize);
    for (size_t i = 0; i < bufsize - 1; ++i) {
        vstr.buf[i] = qrcode[(i + 1)];
    }
    return mp_obj_new_str_from_vstr(&vstr);
}
static MP_DEFINE_CONST_FUN_OBJ_1(qrcode_encode_obj, qrcode_encode);


static mp_obj_t qrcode_encode_to_string(mp_obj_t text_obj){
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(text_obj, &bufinfo, MP_BUFFER_READ);

    enum qrcodegen_Ecc errCorLvl = qrcodegen_Ecc_LOW;  // Error correction level
    
    uint8_t qrcode[qrcodegen_BUFFER_LEN_MAX];
    uint8_t tempBuffer[qrcodegen_BUFFER_LEN_MAX];
    memcpy(tempBuffer, bufinfo.buf, bufinfo.len);
    bool ok = false;
    bool is_bytes = false;
    for (size_t i = 0; i < bufinfo.len; i++) {
        if(tempBuffer[i] == 0){ // \x00 will cut the byte array
            is_bytes = true;
        }
    }
    if(is_bytes){
        ok = qrcodegen_encodeBinary(tempBuffer, bufinfo.len, qrcode, errCorLvl,
            qrcodegen_VERSION_MIN, qrcodegen_VERSION_MAX, qrcodegen_Mask_AUTO, true);
    }else{
        ok = qrcodegen_encodeText(bufinfo.buf, tempBuffer, qrcode, errCorLvl,
            qrcodegen_VERSION_MIN, qrcodegen_VERSION_MAX, qrcodegen_Mask_AUTO, true);
    }
    if(!ok){
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Failed to encode"));
    }
    int size = qrcodegen_getSize(qrcode);
    size_t bufsize = (size+1)*size;
    vstr_t vstr;
    vstr_init_len(&vstr, bufsize);
    memset((byte*)vstr.buf, '0', bufsize);
    for(int y=0; y<size; y++){
        for(int x=0; x<size; x++){
            if(qrcodegen_getModule(qrcode, x, y)){
                vstr.buf[y*(size+1)+x]='1';
            }
        }
        vstr.buf[y*(size+1)+size]='\n';
    }
    return mp_obj_new_str_from_vstr(&vstr);
}
static MP_DEFINE_CONST_FUN_OBJ_1(qrcode_encode_to_string_obj, qrcode_encode_to_string);
/****************************** MODULE ******************************/

static const mp_rom_map_elem_t qrcode_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_qrcode) },
    { MP_ROM_QSTR(MP_QSTR_encode), MP_ROM_PTR(&qrcode_encode_obj) },
    { MP_ROM_QSTR(MP_QSTR_encode_to_string), MP_ROM_PTR(&qrcode_encode_to_string_obj) },
};
static MP_DEFINE_CONST_DICT(qrcode_module_globals, qrcode_module_globals_table);

// Define module object.
const mp_obj_module_t qrcode_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&qrcode_module_globals,
};

// Register the module to make it available in Python
// The port: MaixPy's own qrcode module, lifted as it is (the QR
// code generator is Nayuki's qrcodegen, which is plain portable C).
// The only change is the registration: the port has no config symbol
// for it.
MP_REGISTER_MODULE(MP_QSTR_qrcode, qrcode_user_cmodule);
