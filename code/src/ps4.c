#include "ps4.h"
#include <dirent.h>
#include <fcntl.h>
#include <linux/input.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define MAIN_DEV_NAME   "Wireless Controller"
#define MOTION_DEV_NAME "Wireless Controller Motion Sensors"

struct axis_range { int min, max; };

static int main_fd = -1, motion_fd = -1;
static struct axis_range r_lx, r_ly, r_rx, r_ry, r_l2, r_r2;
static struct axis_range r_gyro_x, r_gyro_y, r_gyro_z;
static struct axis_range r_accel_x, r_accel_y, r_accel_z;
static double gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;

static int find_event_device(const char *name)
{
    DIR *dir = opendir("/dev/input");
    if (!dir) { perror("opendir /dev/input"); return -1; }
    struct dirent *entry;
    int fd = -1;
    char path[300], dev_name[256];
    while ((entry = readdir(dir)) != NULL) {
        if (strncmp(entry->d_name, "event", 5) != 0) continue;
        snprintf(path, sizeof(path), "/dev/input/%s", entry->d_name);
        int candidate = open(path, O_RDONLY | O_NONBLOCK);
        if (candidate < 0) continue;
        if (ioctl(candidate, EVIOCGNAME(sizeof(dev_name)), dev_name) >= 0 &&
            strcmp(dev_name, name) == 0) { fd = candidate; break; }
        close(candidate);
    }
    closedir(dir);
    return fd;
}

static struct axis_range get_range(int fd, int code)
{
    struct input_absinfo ai;
    struct axis_range r = {0, 1};
    if (ioctl(fd, EVIOCGABS(code), &ai) >= 0) { r.min = ai.minimum; r.max = ai.maximum; }
    return r;
}

static double normalize_signed(int value, struct axis_range r)
{
    double center = (r.max + r.min) / 2.0;
    double half   = (r.max - r.min) / 2.0;
    if (half <= 0) return 0.0;
    double n = (value - center) / half;
    if (n < -1.0) n = -1.0;
    if (n > 1.0)  n = 1.0;
    return n;
}

static double normalize_unsigned(int value, struct axis_range r)
{
    if (r.max <= r.min) return 0.0;
    double n = (double)(value - r.min) / (r.max - r.min);
    if (n < 0.0) n = 0.0;
    if (n > 1.0) n = 1.0;
    return n;
}

int ps4_init(void)
{
    main_fd = find_event_device(MAIN_DEV_NAME);
    motion_fd = find_event_device(MOTION_DEV_NAME);
    if (main_fd < 0 || motion_fd < 0) {
        fprintf(stderr, "ps4_init: no se encontro el control (¿conectado?).\n");
        return -1;
    }

    r_lx = get_range(main_fd, ABS_X);
    r_ly = get_range(main_fd, ABS_Y);
    r_rx = get_range(main_fd, ABS_RX);
    r_ry = get_range(main_fd, ABS_RY);
    r_l2 = get_range(main_fd, ABS_Z);
    r_r2 = get_range(main_fd, ABS_RZ);

    r_accel_x = get_range(motion_fd, ABS_X);
    r_accel_y = get_range(motion_fd, ABS_Y);
    r_accel_z = get_range(motion_fd, ABS_Z);
    r_gyro_x  = get_range(motion_fd, ABS_RX);
    r_gyro_y  = get_range(motion_fd, ABS_RY);
    r_gyro_z  = get_range(motion_fd, ABS_RZ);

    return 0;
}

// Ver la nota en ds4_arm3d.c: el giroscopio tiene un sesgo constante
// aunque el control este quieto, y ese sesgo es la mayor parte del
// "drift" al integrar velocidad angular en el tiempo. Aqui se mide el
// promedio con el control quieto y se guarda para restarlo despues.
void ps4_calibrate_gyro(void)
{
    if (motion_fd < 0) return;

    printf("Calibrando giroscopio: deja el control quieto y plano...\n");
    fflush(stdout);
    usleep(400000);

    double sum_x = 0, sum_y = 0, sum_z = 0;
    int n_x = 0, n_y = 0, n_z = 0;
    for (int i = 0; i < 100; i++) { // ~500 ms a 5 ms por vuelta
        struct input_event ev;
        while (read(motion_fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
            if (ev.type != EV_ABS) continue;
            if (ev.code == ABS_RX) { sum_x += normalize_signed(ev.value, r_gyro_x); n_x++; }
            else if (ev.code == ABS_RY) { sum_y += normalize_signed(ev.value, r_gyro_y); n_y++; }
            else if (ev.code == ABS_RZ) { sum_z += normalize_signed(ev.value, r_gyro_z); n_z++; }
        }
        usleep(5000);
    }
    gyro_bias_x = n_x ? sum_x / n_x : 0.0;
    gyro_bias_y = n_y ? sum_y / n_y : 0.0;
    gyro_bias_z = n_z ? sum_z / n_z : 0.0;
    printf("Calibracion lista (bias x=%.4f y=%.4f z=%.4f)\n", gyro_bias_x, gyro_bias_y, gyro_bias_z);
    fflush(stdout);
}

void ps4_poll(ps4_state *out)
{
    struct input_event ev;

    while (read(main_fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
        if (ev.type == EV_KEY) {
            int v = ev.value;
            switch (ev.code) {
                case BTN_SOUTH:  out->btn_south  = v; break;
                case BTN_EAST:   out->btn_east   = v; break;
                case BTN_NORTH:  out->btn_north  = v; break;
                case BTN_WEST:   out->btn_west   = v; break;
                case BTN_TL:     out->btn_l1     = v; break;
                case BTN_TR:     out->btn_r1     = v; break;
                case BTN_TL2:    out->btn_l2     = v; break;
                case BTN_TR2:    out->btn_r2     = v; break;
                case BTN_THUMBL: out->btn_l3     = v; break;
                case BTN_THUMBR: out->btn_r3     = v; break;
                case BTN_SELECT: out->btn_select = v; break;
                case BTN_START:  out->btn_start  = v; break;
                case BTN_MODE:   out->btn_mode   = v; break;
                default: break;
            }
        } else if (ev.type == EV_ABS) {
            switch (ev.code) {
                case ABS_X:     out->stick_lx   = normalize_signed(ev.value, r_lx); break;
                case ABS_Y:     out->stick_ly   = normalize_signed(ev.value, r_ly); break;
                case ABS_RX:    out->stick_rx   = normalize_signed(ev.value, r_rx); break;
                case ABS_RY:    out->stick_ry   = normalize_signed(ev.value, r_ry); break;
                case ABS_Z:     out->trigger_l2 = normalize_unsigned(ev.value, r_l2); break;
                case ABS_RZ:    out->trigger_r2 = normalize_unsigned(ev.value, r_r2); break;
                case ABS_HAT0X: out->dpad_x     = ev.value; break;
                case ABS_HAT0Y: out->dpad_y     = ev.value; break;
                default: break;
            }
        }
    }

    while (read(motion_fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
        if (ev.type != EV_ABS) continue;
        switch (ev.code) {
            case ABS_X:  out->accel_x = normalize_signed(ev.value, r_accel_x); break;
            case ABS_Y:  out->accel_y = normalize_signed(ev.value, r_accel_y); break;
            case ABS_Z:  out->accel_z = normalize_signed(ev.value, r_accel_z); break;
            case ABS_RX: out->gyro_x  = normalize_signed(ev.value, r_gyro_x) - gyro_bias_x; break;
            case ABS_RY: out->gyro_y  = normalize_signed(ev.value, r_gyro_y) - gyro_bias_y; break;
            case ABS_RZ: out->gyro_z  = normalize_signed(ev.value, r_gyro_z) - gyro_bias_z; break;
            default: break;
        }
    }
}

void ps4_cleanup(void)
{
    if (main_fd >= 0)   { close(main_fd);   main_fd = -1; }
    if (motion_fd >= 0) { close(motion_fd); motion_fd = -1; }
}
