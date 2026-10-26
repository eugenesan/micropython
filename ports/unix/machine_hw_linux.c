/*
 * The rest of the machine module, on Linux -- for the Nexus 5
 * (hammerhead) port of Krux.
 *
 * The firmware's MaixPy `machine` module has three more things that
 * the K210's hardware provides and the hammerhead's does not:
 *
 *   Timer    the K210's hardware timers.  The firmware only asks for
 *            one to drive a PWM channel, so this is a stand-in.
 *   PWM      the K210's PWM, which the firmware uses to dim the
 *            display's backlight.  Here the backlight is a plain
 *            sysfs file, so the duty cycle is written there.  The
 *            "pin" the firmware passes is that file's path (see the
 *            board module).
 *   WDT      the watchdog.  The kernel has one (/dev/watchdog), and
 *            the firmware feeds it while it works, so that a hang
 *            restarts the phone.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "py/obj.h"
#include "py/runtime.h"

// linux/watchdog.h
#ifndef WDIOC_KEEPALIVE
#define WDIOC_KEEPALIVE _IOR('W', 5, int)
#endif
#ifndef WDIOC_SETTIMEOUT
#define WDIOC_SETTIMEOUT _IOR('W', 5, int)
#endif

#define WATCHDOG_NODE "/dev/watchdog"

// ---------------------------------------------------------------- Timer

typedef struct {
    mp_obj_base_t base;
    mp_obj_t timer_id;
    mp_obj_t channel;
    mp_obj_t mode;
    mp_obj_t freq;
} machine_timer_obj_t;

// MicroPython 1.28: the make_new callback takes (type, n_args, n_kw,
// args); the keyword arguments follow the positional ones in args.
static mp_obj_t machine_timer_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    machine_timer_obj_t *self = mp_obj_malloc(machine_timer_obj_t, type);
    (void)n_kw;
    self->timer_id = mp_const_none;
    self->channel = mp_const_none;
    self->mode = mp_const_none;
    self->freq = mp_const_none;

    if (n_args > 0)
        self->timer_id = args[0];
    if (n_args > 1)
        self->channel = args[1];
    if (n_args > 2)
        self->mode = args[2];
    return MP_OBJ_FROM_PTR(self);
}

static void machine_timer_print(const mp_print_t *print, const mp_obj_t *self_in, mp_print_kind_t kind) {
    (void)self_in;
    (void)kind;
    mp_printf(print, "Timer()");
}

static const mp_rom_map_elem_t machine_timer_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_Timer) },
    { MP_ROM_QSTR(MP_QSTR_TIMER0), MP_ROM_INT(0) },
    { MP_ROM_QSTR(MP_QSTR_TIMER1), MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_TIMER2), MP_ROM_INT(2) },
    { MP_ROM_QSTR(MP_QSTR_TIMER3), MP_ROM_INT(3) },
    { MP_ROM_QSTR(MP_QSTR_CHANNEL0), MP_ROM_INT(0) },
    { MP_ROM_QSTR(MP_QSTR_CHANNEL1), MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_CHANNEL2), MP_ROM_INT(2) },
    { MP_ROM_QSTR(MP_QSTR_CHANNEL3), MP_ROM_INT(3) },
    { MP_ROM_QSTR(MP_QSTR_MODE_PWM), MP_ROM_INT(0) },
    { MP_ROM_QSTR(MP_QSTR_MODE_ONE_SHOT), MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_MODE_PERIODIC), MP_ROM_INT(2) },
};

static MP_DEFINE_CONST_DICT(machine_timer_locals_dict, machine_timer_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    machine_timer_type,
    MP_QSTR_Timer,
    MP_TYPE_FLAG_NONE,
    make_new, machine_timer_make_new,
    print, machine_timer_print,
    locals_dict, &machine_timer_locals_dict);

// ------------------------------------------------------------------ PWM

typedef struct {
    mp_obj_base_t base;
    mp_obj_t pin;             // the sysfs file that is the "pin"
    mp_obj_t timer;
    mp_int_t duty;
    mp_int_t freq;
} machine_pwm_obj_t;

static mp_obj_t machine_pwm_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    machine_pwm_obj_t *self = mp_obj_malloc(machine_pwm_obj_t, type);

    self->pin = mp_const_none;
    self->timer = mp_const_none;
    self->duty = 0;
    self->freq = 1000;

    if (n_args > 0)
        self->timer = args[0];

    // the keyword arguments follow the positional ones
    for (size_t i = 0; i < n_kw; i++) {
        mp_obj_t key = args[n_args + i * 2];
        mp_obj_t val = args[n_args + i * 2 + 1];

        if (mp_obj_str_equal(key, MP_OBJ_NEW_QSTR(MP_QSTR_pin))) {
            self->pin = val;
        } else if (mp_obj_str_equal(key, MP_OBJ_NEW_QSTR(MP_QSTR_duty))) {
            self->duty = mp_obj_get_int(val);
        } else if (mp_obj_str_equal(key, MP_OBJ_NEW_QSTR(MP_QSTR_freq))) {
            self->freq = mp_obj_get_int(val);
        }
    }

    // the pin is a file: write the duty to it straight away
    if (self->pin != mp_const_none) {
        const char *path = mp_obj_str_get_str(self->pin);
        FILE *f = fopen(path, "w");

        if (f != NULL) {
            int maximum = 255;
            char name[128];
            char *slash;
            int value = (int)self->duty;

            // the brightness file's maximum: "<dir>/brightness" ->
            // "<dir>/max_brightness"
            if (strlen(path) < sizeof(name)) {
                strcpy(name, path);
                slash = strrchr(name, '/');
                if (slash != NULL) {
                    strcpy(slash + 1, "max_brightness");
                    FILE *m = fopen(name, "r");
                    if (m != NULL) {
                        if (fscanf(m, "%d", &maximum) != 1 || maximum <= 0) {
                            maximum = 255;
                        }
                        fclose(m);
                    }
                }
            }
            if (value < 0) {
                value = 0;
            }
            if (value > 100) {
                value = 100;
            }
            fprintf(f, "%d", value * maximum / 100);
            fclose(f);
        }
    }

    return MP_OBJ_FROM_PTR(self);
}

static mp_obj_t machine_pwm_duty(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    machine_pwm_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    const char *path;
    FILE *f;
    int value, maximum = 255;
    char name[128];
    char *slash;

    self->duty = mp_obj_get_int(args[1]);
    if (self->pin == mp_const_none) {
        return mp_const_none;
    }

    path = mp_obj_str_get_str(self->pin);
    f = fopen(path, "w");
    if (f == NULL) {
        return mp_const_none;
    }
    if (strlen(path) < sizeof(name)) {
        strcpy(name, path);
        slash = strrchr(name, '/');
        if (slash != NULL) {
            strcpy(slash + 1, "max_brightness");
            FILE *m = fopen(name, "r");
            if (m != NULL) {
                if (fscanf(m, "%d", &maximum) != 1 || maximum <= 0) {
                    maximum = 255;
                }
                fclose(m);
            }
        }
    }
    value = (int)self->duty;
    if (value < 0) {
        value = 0;
    }
    if (value > 100) {
        value = 100;
    }
    fprintf(f, "%d", value * maximum / 100);
    fclose(f);
    return mp_const_none;
}

static MP_DEFINE_CONST_FUN_OBJ_VAR(machine_pwm_duty_obj, 1, machine_pwm_duty);

static const mp_rom_map_elem_t machine_pwm_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_PWM) },
    { MP_ROM_QSTR(MP_QSTR_duty), MP_ROM_PTR(&machine_pwm_duty_obj) },
};

static MP_DEFINE_CONST_DICT(machine_pwm_locals_dict, machine_pwm_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    machine_pwm_type,
    MP_QSTR_PWM,
    MP_TYPE_FLAG_NONE,
    make_new, machine_pwm_make_new,
    locals_dict, &machine_pwm_locals_dict);


// ------------------------------------------------------------- SDCard

// The K210 boards have an SD card on their SDMMC bus and MaixPy
// drives it with a `machine.SDCard`.  The hammerhead has no SD slot
// at all (its storage is internal), so the port only has to provide
// the name: the firmware asks for a remount when it wants the card,
// and that has to fail cleanly.

typedef struct {
    mp_obj_base_t base;
} machine_sdcard_obj_t;

static mp_obj_t machine_sdcard_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    (void)args;
    (void)n_args;
    (void)n_kw;
    machine_sdcard_obj_t *self = mp_obj_malloc(machine_sdcard_obj_t, type);
    return MP_OBJ_FROM_PTR(self);
}

// remount() -- the class method the firmware calls
static mp_obj_t machine_sdcard_remount(void) {
    mp_raise_msg(&mp_type_OSError,
        MP_ERROR_TEXT("this device has no SD card"));
}

static MP_DEFINE_CONST_FUN_OBJ_0(machine_sdcard_remount_obj, machine_sdcard_remount);

static const mp_rom_map_elem_t machine_sdcard_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_SDCard) },
    { MP_ROM_QSTR(MP_QSTR_remount), MP_ROM_PTR(&machine_sdcard_remount_obj) },
};

static MP_DEFINE_CONST_DICT(machine_sdcard_locals_dict, machine_sdcard_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    machine_sdcard_type,
    MP_QSTR_SDCard,
    MP_TYPE_FLAG_NONE,
    make_new, machine_sdcard_make_new,
    locals_dict, &machine_sdcard_locals_dict);

// ---------------------------------------------------------------- reset

/*
 * machine.reset() -- power the device off.
 *
 * The firmware's shutdown path (krux/power.py) ends with
 * `machine.reset()`, and on the K210 that is the SoC's reset line: the
 * board powers off.  This port has no such line, and the unix port of
 * MicroPython has no machine.reset() at all (MICROPY_PY_MACHINE_RESET
 * is 0 there), so the firmware's shutdown raised AttributeError, the
 * firmware died, and init -- which restarts a service that exits --
 * started it again.  That is the "shutdown kills the firmware but it
 * comes back" behaviour.
 *
 * The route that works is the one the phase-0 proof used
 * (rootfs/touch-off.c): ask init.  This init has the handler
 *
 *     on property:sys.powerctl=*
 *        powerctl ${sys.powerctl}
 *
 * and the powerctl builtin understands "shutdown" and "reboot".  The
 * property is set with /sbin/setprop, which is toybox here.
 *
 * reboot(2) is tried first: it costs nothing, and on a kernel that
 * allows it to a non-init process it is immediate.  This CAF kernel
 * returns EINVAL to a non-init process, so the setprop is what
 * actually does the work.
 *
 * The call never returns: the firmware expects it not to (it is
 * followed by sys.exit()), and if it did return, the firmware would
 * carry on as if the phone had been reset. */

static void machine_reset_do(const char *what) {
    // The direct route.  LINUX_REBOOT_CMD_POWER_OFF / RESTART, with
    // the magic the kernel checks.  The syscall is used raw because
    // the armel sysroot has no reboot(2) declaration that this static
    // build can rely on; the number is the ARM one (88).
    syscall(SYS_reboot, 0x4321L, 0x28121969L); // the magic from the phase-0 proof

    // The route that works: ask init, which owns the powerctl handler.
    char *argv[] = {(char *)"/sbin/setprop", (char *)"sys.powerctl",
                    (char *)what, NULL};
    char *envp[] = {(char *)"PATH=/sbin:/system/bin", NULL};
    execve("/sbin/setprop", argv, envp);
}

static mp_obj_t machine_reset(void) {
    machine_reset_do("shutdown");
    // execve only returns if it failed.  There is nothing useful left
    // to do, and returning would let the firmware carry on, so the
    // device is asked to stop again and the wait is unbounded.
    for (;;) {
        sleep(1);
        machine_reset_do("shutdown");
    }
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_0(machine_reset_obj, machine_reset);

// ------------------------------------------------------------------ WDT

typedef struct {
    mp_obj_base_t base;
    int fd;
} machine_wdt_obj_t;

static mp_obj_t machine_wdt_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    machine_wdt_obj_t *self = mp_obj_malloc(machine_wdt_obj_t, type);
    int timeout = 30000;

    self->fd = -1;

    // the keyword arguments follow the positional ones
    for (size_t i = 0; i < n_kw; i++) {
        mp_obj_t key = args[n_args + i * 2];
        mp_obj_t val = args[n_args + i * 2 + 1];

        if (mp_obj_str_equal(key, MP_OBJ_NEW_QSTR(MP_QSTR_timeout))) {
            timeout = mp_obj_get_int(val);
        }
    }

    self->fd = open(WATCHDOG_NODE, O_WRONLY);
    if (self->fd >= 0) {
        int t = timeout / 1000;
        ioctl(self->fd, WDIOC_SETTIMEOUT, &t);
    }
    // no watchdog on this device is not an error: the firmware only
    // feeds it
    return MP_OBJ_FROM_PTR(self);
}

static mp_obj_t machine_wdt_feed(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    machine_wdt_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    if (self->fd >= 0) {
        int dummy = 0;
        ioctl(self->fd, WDIOC_KEEPALIVE, &dummy);
    }
    return mp_const_none;
}

static MP_DEFINE_CONST_FUN_OBJ_VAR(machine_wdt_feed_obj, 0, machine_wdt_feed);

static const mp_rom_map_elem_t machine_wdt_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_WDT) },
    { MP_ROM_QSTR(MP_QSTR_feed), MP_ROM_PTR(&machine_wdt_feed_obj) },
};

static MP_DEFINE_CONST_DICT(machine_wdt_locals_dict, machine_wdt_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    machine_wdt_type,
    MP_QSTR_WDT,
    MP_TYPE_FLAG_NONE,
    make_new, machine_wdt_make_new,
    locals_dict, &machine_wdt_locals_dict);
