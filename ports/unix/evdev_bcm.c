/*
 * The input devices, on Linux -- for the Nexus 5 (hammerhead) port of
 * Krux.
 *
 * The hammerhead has no buttons in the sense the K210 boards have: the
 * keys are the phone's (the power key and the volume keys) and the
 * touch screen is a plain evdev device.  The kernel's input layer
 * gives them all the same interface, so one small module covers them:
 * open a device, read the events with a timeout.
 *
 * The event structure is the kernel's own (linux/input-event-codes.h
 * and linux/input.h): a time stamp, the type, the code and the value.
 * It is spelled out here because the cross build's sysroot has no
 * kernel headers.
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>

#include "py/obj.h"
#include "py/runtime.h"

struct input_event {
    long tv_sec;
    long tv_usec;
    unsigned short type;
    unsigned short code;
    int value;
};

static mp_obj_t evdev_open(mp_obj_t path_in) {
    const char *path = mp_obj_str_get_str(path_in);
    int fd = open(path, O_RDWR | O_NONBLOCK);

    if (fd < 0)
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("cannot open the input device"));
    return mp_obj_new_int(fd);
}

/* read(fd, timeout_ms) -- one event as (type, code, value), or None
 * if nothing arrived in time. */
static mp_obj_t evdev_read(mp_obj_t fd_in, mp_obj_t timeout_in) {
    int fd = mp_obj_get_int(fd_in);
    int timeout = mp_obj_get_int(timeout_in);
    struct pollfd pfd;
    struct input_event ev;
    ssize_t n;

    pfd.fd = fd;
    pfd.events = POLLIN;
    pfd.revents = 0;

    int ready = poll(&pfd, 1, timeout);
    if (ready < 0) {
        if (errno == EINTR)
            return mp_const_none;
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("poll failed"));
    }
    if (ready == 0)
        return mp_const_none;

    n = read(fd, &ev, sizeof(ev));
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            return mp_const_none;
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("read failed"));
    }
    if (n != (ssize_t)sizeof(ev))
        return mp_const_none;

    mp_obj_t tup[3];
    tup[0] = mp_obj_new_int(ev.type);
    tup[1] = mp_obj_new_int(ev.code);
    tup[2] = mp_obj_new_int(ev.value);
    return mp_obj_new_tuple(3, tup);
}

static mp_obj_t evdev_close(mp_obj_t fd_in) {
    int fd = mp_obj_get_int(fd_in);
    if (fd >= 0)
        close(fd);
    return mp_const_none;
}

static MP_DEFINE_CONST_FUN_OBJ_1(evdev_open_obj, evdev_open);
static MP_DEFINE_CONST_FUN_OBJ_2(evdev_read_obj, evdev_read);
static MP_DEFINE_CONST_FUN_OBJ_1(evdev_close_obj, evdev_close);

static const mp_rom_map_elem_t evdev_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_open), MP_ROM_PTR(&evdev_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&evdev_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_close), MP_ROM_PTR(&evdev_close_obj) },
    /* the event types and codes, so that the reader does not have to
     * remember the numbers */
    { MP_ROM_QSTR(MP_QSTR_EV_SYN), MP_ROM_INT(0) },
    { MP_ROM_QSTR(MP_QSTR_EV_KEY), MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_EV_ABS), MP_ROM_INT(3) },
    { MP_ROM_QSTR(MP_QSTR_ABS_X), MP_ROM_INT(0) },
    { MP_ROM_QSTR(MP_QSTR_ABS_Y), MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_ABS_MT_SLOT), MP_ROM_INT(0x2f) },
    { MP_ROM_QSTR(MP_QSTR_ABS_MT_TOUCH_MAJOR), MP_ROM_INT(0x30) },
    { MP_ROM_QSTR(MP_QSTR_ABS_MT_POSITION_X), MP_ROM_INT(0x35) },
    { MP_ROM_QSTR(MP_QSTR_ABS_MT_POSITION_Y), MP_ROM_INT(0x36) },
    /* The tracking id is what says a finger went down (and, with the
     * value -1, that it went up again).  It was MISSING here, and
     * that is why the touch never worked: the touch module asks for
     * evdev.ABS_MT_TRACKING_ID as soon as a tracking event arrives,
     * the module has no such name, and so the poll raised
     * AttributeError and the touch was never seen. */
    { MP_ROM_QSTR(MP_QSTR_ABS_MT_TRACKING_ID), MP_ROM_INT(0x39) },
    { MP_ROM_QSTR(MP_QSTR_ABS_MT_PRESSURE), MP_ROM_INT(0x3a) },
    { MP_ROM_QSTR(MP_QSTR_SYN_REPORT), MP_ROM_INT(0) },
    { MP_ROM_QSTR(MP_QSTR_KEY_VOLUME_UP), MP_ROM_INT(115) },
    { MP_ROM_QSTR(MP_QSTR_KEY_VOLUME_DOWN), MP_ROM_INT(114) },
    { MP_ROM_QSTR(MP_QSTR_KEY_POWER), MP_ROM_INT(116) },
    { MP_ROM_QSTR(MP_QSTR_KEY_ENTER), MP_ROM_INT(28) },
};

static MP_DEFINE_CONST_DICT(mp_module_evdev_globals, evdev_module_globals_table);

const mp_obj_module_t mp_module_evdev = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&mp_module_evdev_globals,
};

MP_REGISTER_MODULE(MP_QSTR_evdev, mp_module_evdev);
