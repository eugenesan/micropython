/*
 * The I2C bus, on Linux — for the Nexus 5 (hammerhead) port of Krux.
 *
 * Krux's firmware talks to its peripherals over MaixPy's `machine.I2C`
 * (krux/src/krux/i2c.py does `from machine import I2C` and builds
 * `I2C(I2C.I2C0, freq=400000, scl=..., sda=...)`).  On Linux the
 * peripherals are on the kernel's I2C bus, so this is the same bus
 * seen from userspace: /dev/i2c-N and the i2c-dev ioctls.
 *
 * This is the port's own class rather than the generic one in
 * extmod/machine_i2c.c, because MaixPy's API differs: the register
 * address width is `mem_size` (not `addrsize`), the bus is chosen
 * with the `I2C.I2C0` class constant, and `writeto` takes the bytes
 * to write rather than a list.
 *
 * The bus numbering is the kernel's.  On the hammerhead:
 *
 *   bus 0   lm3630 (0x38), anx7808 (0x72), bcm2079x — the NFC (0x77)
 *   bus 2   s3350 (0x70)
 *   bus 84  max17048 (0x36), bq24192 (0x6b)
 *
 * The I2C0/I2C1/I2C2 class constants below name those three buses.
 *
 * The i2c-dev ABI (linux/i2c-dev.h, linux/i2c.h) is spelled out here
 * rather than included: the cross build's sysroot has no kernel
 * headers.  The ABI is stable, and the structures below match the
 * kernel's byte for byte on 32-bit ARM (the pointer in struct i2c_msg
 * needs 4-byte alignment, so the structure is 12 bytes).
 *
 * Addressing uses I2C_SLAVE_FORCE, not I2C_SLAVE: i2c-dev refuses
 * I2C_SLAVE for an address that a kernel driver has already claimed
 * (-EBUSY), and on the hammerhead every peripheral of interest is
 * claimed by a driver.
 *
 * The kernel owns the pins and the bus clock, so the `scl`, `sda`
 * and `freq` arguments of the constructor are accepted and ignored.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "py/builtin.h"
#include "py/obj.h"
#include "py/runtime.h"

// For the declaration of machine_i2c_type, which extmod/modmachine.h
// carries for the ports that provide the class.
#include "extmod/modmachine.h"

// From linux/i2c-dev.h.
#define I2C_SLAVE 0x0703
#define I2C_SLAVE_FORCE 0x0706
#define I2C_RDWR 0x0707

// From linux/i2c.h.
#define I2C_M_RD 0x0001

typedef struct {
    uint16_t addr;
    uint16_t flags;
    uint16_t len;
    uint8_t *buf;
} machine_i2c_msg_t;

typedef struct {
    machine_i2c_msg_t *msgs;
    uint32_t nmsgs;
} machine_i2c_rdwr_t;

typedef struct {
    mp_obj_base_t base;
    int fd;
} machine_i2c_obj_t;

// Claim the slave address for the plain read()/write() calls.  The
// FORCE variant is what lets us talk to a chip that a kernel driver
// has already bound.
static void machine_i2c_select(machine_i2c_obj_t *self, uint16_t addr) {
    if (ioctl(self->fd, I2C_SLAVE_FORCE, addr) < 0) {
        mp_raise_OSError(errno);
    }
}

// The register address, as MaixPy's `mem_size` asks for: the width of
// the address field in bits, sent most significant byte first.
static size_t machine_i2c_memaddr(uint8_t *buf, uint32_t memaddr, uint8_t mem_size) {
    if ((mem_size & 7) != 0 || mem_size == 0 || mem_size > 32) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid mem_size"));
    }
    size_t len = 0;
    for (int16_t i = mem_size - 8; i >= 0; i -= 8) {
        buf[len++] = memaddr >> i;
    }
    return len;
}

// bus.readfrom(addr, n) -> bytes
static mp_obj_t machine_i2c_readfrom(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_addr, ARG_nbytes };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_addr, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_nbytes, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args,
        MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    machine_i2c_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    uint16_t addr = args[ARG_addr].u_int;
    size_t n = args[ARG_nbytes].u_int;

    vstr_t vstr;
    vstr_init_len(&vstr, n);

    machine_i2c_select(self, addr);
    size_t done = 0;
    while (done < n) {
        ssize_t ret = read(self->fd, vstr.buf + done, n - done);
        if (ret < 0) {
            vstr_clear(&vstr);
            mp_raise_OSError(errno);
        }
        if (ret == 0) {
            vstr_clear(&vstr);
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2C read returned no data"));
        }
        done += ret;
    }

    return mp_obj_new_bytes_from_vstr(&vstr);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(machine_i2c_readfrom_obj, 1, machine_i2c_readfrom);

// bus.writeto(addr, data)
static mp_obj_t machine_i2c_writeto(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_addr, ARG_data };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_addr, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_data, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args,
        MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    machine_i2c_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    uint16_t addr = args[ARG_addr].u_int;

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_data].u_obj, &bufinfo, MP_BUFFER_READ);

    machine_i2c_select(self, addr);
    size_t done = 0;
    while (done < bufinfo.len) {
        ssize_t ret = write(self->fd, (const uint8_t *)bufinfo.buf + done, bufinfo.len - done);
        if (ret < 0) {
            mp_raise_OSError(errno);
        }
        done += ret;
    }

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(machine_i2c_writeto_obj, 1, machine_i2c_writeto);

// bus.readfrom_mem(addr, reg, n, mem_size=8) -> bytes
static mp_obj_t machine_i2c_readfrom_mem(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_addr, ARG_memaddr, ARG_nbytes, ARG_mem_size };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_addr, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_memaddr, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_nbytes, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_mem_size, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 8} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args,
        MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    machine_i2c_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    uint16_t addr = args[ARG_addr].u_int;
    size_t n = args[ARG_nbytes].u_int;

    uint8_t memaddr_buf[4];
    size_t memaddr_len = machine_i2c_memaddr(&memaddr_buf[0], args[ARG_memaddr].u_int,
        args[ARG_mem_size].u_int);

    vstr_t vstr;
    vstr_init_len(&vstr, n);

    // Write the register, then read it back, without a STOP in
    // between: that is what I2C_RDWR is for.
    machine_i2c_msg_t msgs[2];
    msgs[0].addr = addr;
    msgs[0].flags = 0;
    msgs[0].len = memaddr_len;
    msgs[0].buf = &memaddr_buf[0];
    msgs[1].addr = addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = n;
    msgs[1].buf = (uint8_t *)vstr.buf;

    machine_i2c_rdwr_t data;
    data.msgs = msgs;
    data.nmsgs = 2;

    if (ioctl(self->fd, I2C_RDWR, &data) < 0) {
        vstr_clear(&vstr);
        mp_raise_OSError(errno);
    }

    return mp_obj_new_bytes_from_vstr(&vstr);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(machine_i2c_readfrom_mem_obj, 1, machine_i2c_readfrom_mem);

// bus.writeto_mem(addr, reg, data, mem_size=8)
static mp_obj_t machine_i2c_writeto_mem(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_addr, ARG_memaddr, ARG_data, ARG_mem_size };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_addr, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_memaddr, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_data, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_mem_size, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 8} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args,
        MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    machine_i2c_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    uint16_t addr = args[ARG_addr].u_int;

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_data].u_obj, &bufinfo, MP_BUFFER_READ);

    uint8_t memaddr_buf[4];
    size_t memaddr_len = machine_i2c_memaddr(&memaddr_buf[0], args[ARG_memaddr].u_int,
        args[ARG_mem_size].u_int);

    // The register address and the data go out in one write, so that
    // the device sees them without an intervening STOP.
    uint8_t buf[4 + 64];
    if (memaddr_len + bufinfo.len > sizeof(buf)) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2C write too long"));
    }
    memcpy(&buf[0], &memaddr_buf[0], memaddr_len);
    memcpy(&buf[memaddr_len], bufinfo.buf, bufinfo.len);

    machine_i2c_select(self, addr);
    size_t done = 0;
    size_t len = memaddr_len + bufinfo.len;
    while (done < len) {
        ssize_t ret = write(self->fd, &buf[done], len - done);
        if (ret < 0) {
            mp_raise_OSError(errno);
        }
        done += ret;
    }

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(machine_i2c_writeto_mem_obj, 1, machine_i2c_writeto_mem);

// The MaixPy API takes the bus as the first argument and the pins and
// the frequency as keywords; the kernel owns the pins and the clock,
// so those are accepted and ignored.
static mp_obj_t machine_i2c_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    enum { ARG_id, ARG_freq, ARG_scl, ARG_sda };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_id, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_freq, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_scl, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_sda, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
    };
    mp_arg_val_t vals[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, args, MP_ARRAY_SIZE(allowed_args), allowed_args, vals);

    int bus = vals[ARG_id].u_int;
    char path[32];
    snprintf(path, sizeof(path), "/dev/i2c-%d", bus);

    int fd = open(path, O_RDWR);
    if (fd < 0) {
        mp_raise_msg_varg(&mp_type_OSError,
            MP_ERROR_TEXT("cannot open %s: %s"), path, strerror(errno));
    }

    machine_i2c_obj_t *self = mp_obj_malloc(machine_i2c_obj_t, type);
    self->fd = fd;
    return MP_OBJ_FROM_PTR(self);
}

static void machine_i2c_print(const mp_print_t *print, const mp_obj_t *self_in, mp_print_kind_t kind) {
    machine_i2c_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "I2C(fd=%d)", self->fd);
}

static const mp_rom_map_elem_t machine_i2c_locals_dict_table[] = {
    // The buses of the hammerhead, so that the firmware can ask for
    // "I2C.I2C0" the way it does on MaixPy.
    { MP_ROM_QSTR(MP_QSTR_I2C0), MP_ROM_INT(0) },
    { MP_ROM_QSTR(MP_QSTR_I2C1), MP_ROM_INT(2) },
    { MP_ROM_QSTR(MP_QSTR_I2C2), MP_ROM_INT(84) },

    { MP_ROM_QSTR(MP_QSTR_readfrom), MP_ROM_PTR(&machine_i2c_readfrom_obj) },
    { MP_ROM_QSTR(MP_QSTR_writeto), MP_ROM_PTR(&machine_i2c_writeto_obj) },
    { MP_ROM_QSTR(MP_QSTR_readfrom_mem), MP_ROM_PTR(&machine_i2c_readfrom_mem_obj) },
    { MP_ROM_QSTR(MP_QSTR_writeto_mem), MP_ROM_PTR(&machine_i2c_writeto_mem_obj) },
};
static MP_DEFINE_CONST_DICT(machine_i2c_locals_dict, machine_i2c_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    machine_i2c_type,
    MP_QSTR_I2C,
    MP_TYPE_FLAG_NONE,
    make_new, machine_i2c_make_new,
    print, machine_i2c_print,
    locals_dict, &machine_i2c_locals_dict);
