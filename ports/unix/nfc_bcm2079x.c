/*
 * The NFC chip's device node, on Linux — for the Nexus 5 (hammerhead)
 * port of Krux.
 *
 * The chip is a Broadcom BCM20793 (the hammerhead's NFC controller).
 * It is reached through /dev/bcm2079x, a small misc device with seven
 * ioctls; see standalone/nfc/TRANSPORT.md for the measured contract
 * and standalone/nfc/bcm2079x.h for the ioctl numbers (which are
 * _IO(0xFA, nr) — no argument, no direction bit).
 *
 * Why not I2C: the chip sits on the kernel's I2C bus 0 at address
 * 0x77, but the vendor's bcm2079x driver owns that address, and the
 * chip NACKs (ENOTCONN) for anyone else.  The driver is the only way
 * in, so the port uses it.
 *
 * This module is the low-level half: open the node, drive the power
 * and wake lines, and read and write whole packets.  The packet
 * framing (a 1-byte HCIT type plus a 3-byte NCI header) and the
 * protocol on top of it live in the Python module `nfc`.
 *
 * The kernel driver delivers one whole framed packet per read(), so
 * an ordinary buffered read is enough — no byte-at-a-time header
 * hunting is needed (measured; see TRANSPORT.md).
 *
 * The ioctl numbers are spelled out here rather than included: the
 * cross build's sysroot has no kernel headers, and _IO(0xFA, nr) is
 * simply (0xFA << 8) | nr on Linux.
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "py/obj.h"
#include "py/runtime.h"

// From include/bcm2079x.h (the vendor's own header).
#define BCMNFC_MAGIC 0xFA

#define BCMNFC_POWER_CTL 0xFA01
#define BCMNFC_CHANGE_ADDR 0xFA02
#define BCMNFC_READ_FULL_PACKET 0xFA03
#define BCMNFC_SET_WAKE_ACTIVE_STATE 0xFA04
#define BCMNFC_WAKE_CTL 0xFA05
#define BCMNFC_READ_MULTI_PACKETS 0xFA06
#define BCMNFC_SET_CLIENT_ADDR 0xFA07

#define NFC_DEV "/dev/bcm2079x"

// The largest packet the chip sends: a 255-byte payload plus the
// 4-byte header (1 HCIT byte + the 3-byte NCI header).
#define NFC_MAX_PACKET 259

// The longest the driver will wait for a packet, in milliseconds.
#define NFC_READ_TIMEOUT_MS 2000

static mp_obj_t nfc_open(void) {
    int fd = open(NFC_DEV, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("cannot open " NFC_DEV));
    }
    return mp_obj_new_int(fd);
}

// power(fd, on) -- the chip's power line.
static mp_obj_t nfc_power(mp_obj_t fd_in, mp_obj_t on_in) {
    int fd = mp_obj_get_int(fd_in);
    int on = mp_obj_get_int(on_in);
    if (ioctl(fd, BCMNFC_POWER_CTL, on) < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("power ioctl failed"));
    }
    return mp_const_none;
}

// wake(fd, state) -- the host's wake line (0 = deasserted).
static mp_obj_t nfc_wake(mp_obj_t fd_in, mp_obj_t state_in) {
    int fd = mp_obj_get_int(fd_in);
    int state = mp_obj_get_int(state_in);
    if (ioctl(fd, BCMNFC_WAKE_CTL, state) < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("wake ioctl failed"));
    }
    return mp_const_none;
}

// write(fd, data) -- one whole packet.
static mp_obj_t nfc_write(mp_obj_t fd_in, mp_obj_t data_in) {
    int fd = mp_obj_get_int(fd_in);
    size_t len;
    const char *buf = mp_obj_str_get_data(data_in, &len);
    ssize_t n = write(fd, buf, len);
    if (n < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("write failed"));
    }
    return mp_obj_new_int(n);
}

// read(fd, timeout_ms) -- one whole packet, or None if none arrived.
static mp_obj_t nfc_read(mp_obj_t fd_in, mp_obj_t timeout_in) {
    int fd = mp_obj_get_int(fd_in);
    int timeout = mp_obj_get_int(timeout_in);
    struct pollfd pfd;
    unsigned char buf[NFC_MAX_PACKET];
    ssize_t n;

    pfd.fd = fd;
    pfd.events = POLLIN;
    pfd.revents = 0;

    int ready = poll(&pfd, 1, timeout);
    if (ready < 0) {
        if (errno == EINTR) {
            return mp_const_none;
        }
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("poll failed"));
    }
    if (ready == 0) {
        return mp_const_none;
    }

    n = read(fd, buf, sizeof(buf));
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            return mp_const_none;
        }
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("read failed"));
    }
    if (n == 0) {
        return mp_const_none;
    }
    return mp_obj_new_bytes(buf, n);
}

static mp_obj_t nfc_close(mp_obj_t fd_in) {
    int fd = mp_obj_get_int(fd_in);
    if (fd >= 0) {
        close(fd);
    }
    return mp_const_none;
}

static MP_DEFINE_CONST_FUN_OBJ_0(nfc_open_obj, nfc_open);
static MP_DEFINE_CONST_FUN_OBJ_2(nfc_power_obj, nfc_power);
static MP_DEFINE_CONST_FUN_OBJ_2(nfc_wake_obj, nfc_wake);
static MP_DEFINE_CONST_FUN_OBJ_2(nfc_write_obj, nfc_write);
static MP_DEFINE_CONST_FUN_OBJ_2(nfc_read_obj, nfc_read);
static MP_DEFINE_CONST_FUN_OBJ_1(nfc_close_obj, nfc_close);

static const mp_rom_map_elem_t nfc_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_open), MP_ROM_PTR(&nfc_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_power), MP_ROM_PTR(&nfc_power_obj) },
    { MP_ROM_QSTR(MP_QSTR_wake), MP_ROM_PTR(&nfc_wake_obj) },
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&nfc_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&nfc_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_close), MP_ROM_PTR(&nfc_close_obj) },
};

static MP_DEFINE_CONST_DICT(mp_module_nfc_globals, nfc_module_globals_table);

const mp_obj_module_t mp_module_nfc = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&mp_module_nfc_globals,
};

MP_REGISTER_MODULE(MP_QSTR__nfc, mp_module_nfc);
