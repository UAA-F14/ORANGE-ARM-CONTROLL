// Lee los joysticks/botones y el giroscopio/acelerometro de un control
// DS4 (PS4) conectado por Bluetooth o USB, usando evdev puro (sin SDL
// ni otras librerias). Portable entre laptop y Orange Pi: solo depende
// del kernel de Linux.
//
// El DS4 crea DOS dispositivos de entrada separados:
//   "Wireless Controller"                 -> botones, sticks, gatillos, D-pad
//   "Wireless Controller Motion Sensors"   -> acelerometro + giroscopio
//
// Compilar: gcc ds4_read.c -o ds4_read
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define DEV_INPUT_DIR   "/dev/input"
#define MAIN_DEV_NAME   "Wireless Controller"
#define MOTION_DEV_NAME "Wireless Controller Motion Sensors"

// Busca en /dev/input/eventN el dispositivo cuyo nombre (EVIOCGNAME)
// sea exactamente `name`. Regresa el fd abierto o -1 si no lo encuentra.
static int find_event_device(const char *name)
{
    DIR *dir = opendir(DEV_INPUT_DIR);
    if (!dir) {
        perror("opendir " DEV_INPUT_DIR);
        return -1;
    }

    struct dirent *entry;
    int fd = -1;
    char path[300];
    char dev_name[256];

    while ((entry = readdir(dir)) != NULL) {
        if (strncmp(entry->d_name, "event", 5) != 0)
            continue;

        snprintf(path, sizeof(path), "%s/%s", DEV_INPUT_DIR, entry->d_name);
        int candidate = open(path, O_RDONLY | O_NONBLOCK);
        if (candidate < 0)
            continue;

        if (ioctl(candidate, EVIOCGNAME(sizeof(dev_name)), dev_name) >= 0 &&
            strcmp(dev_name, name) == 0) {
            fd = candidate;
            break;
        }
        close(candidate);
    }

    closedir(dir);
    return fd;
}

// Nombre legible para los botones que nos interesan del DS4.
static const char *button_name(unsigned short code)
{
    switch (code) {
        case BTN_SOUTH:  return "X (South)";
        case BTN_EAST:   return "Circulo (East)";
        case BTN_NORTH:  return "Triangulo (North)";
        case BTN_WEST:   return "Cuadro (West)";
        case BTN_TL:     return "L1";
        case BTN_TR:     return "R1";
        case BTN_TL2:    return "L2 (click)";
        case BTN_TR2:    return "R2 (click)";
        case BTN_SELECT: return "Share";
        case BTN_START:  return "Options";
        case BTN_MODE:   return "PS";
        case BTN_THUMBL: return "L3";
        case BTN_THUMBR: return "R3";
        default:         return NULL;
    }
}

static void handle_main_event(const struct input_event *ev)
{
    if (ev->type == EV_KEY) {
        const char *name = button_name(ev->code);
        if (name)
            printf("[BOTON]   %-18s %s\n", name, ev->value ? "presionado" : "soltado");
        return;
    }

    if (ev->type != EV_ABS)
        return;

    switch (ev->code) {
        case ABS_X:     printf("[STICK L] X = %d\n", ev->value); break;
        case ABS_Y:     printf("[STICK L] Y = %d\n", ev->value); break;
        case ABS_RX:    printf("[STICK R] X = %d\n", ev->value); break;
        case ABS_RY:    printf("[STICK R] Y = %d\n", ev->value); break;
        case ABS_Z:     printf("[GATILLO] L2 = %d\n", ev->value); break;
        case ABS_RZ:    printf("[GATILLO] R2 = %d\n", ev->value); break;
        case ABS_HAT0X: printf("[D-PAD]   X = %d\n", ev->value); break;
        case ABS_HAT0Y: printf("[D-PAD]   Y = %d\n", ev->value); break;
        default: break;
    }
}

static void handle_motion_event(const struct input_event *ev)
{
    if (ev->type != EV_ABS)
        return;

    // Acelerometro: ABS_X/Y/Z. Giroscopio: ABS_RX/RY/RZ.
    switch (ev->code) {
        case ABS_X:  printf("[ACEL]    X = %d\n", ev->value); break;
        case ABS_Y:  printf("[ACEL]    Y = %d\n", ev->value); break;
        case ABS_Z:  printf("[ACEL]    Z = %d\n", ev->value); break;
        case ABS_RX: printf("[GIRO]    X = %d\n", ev->value); break;
        case ABS_RY: printf("[GIRO]    Y = %d\n", ev->value); break;
        case ABS_RZ: printf("[GIRO]    Z = %d\n", ev->value); break;
        default: break;
    }
}

int main(void)
{
    int main_fd = find_event_device(MAIN_DEV_NAME);
    if (main_fd < 0) {
        fprintf(stderr, "No se encontro '%s'. ¿Esta conectado el control?\n", MAIN_DEV_NAME);
        return 1;
    }

    int motion_fd = find_event_device(MOTION_DEV_NAME);
    if (motion_fd < 0) {
        fprintf(stderr, "No se encontro '%s'.\n", MOTION_DEV_NAME);
        close(main_fd);
        return 1;
    }

    printf("Control encontrado. Leyendo sticks/botones y giroscopio/acelerometro.\n");
    printf("Ctrl+C para salir.\n\n");

    struct pollfd fds[2] = {
        { .fd = main_fd,   .events = POLLIN },
        { .fd = motion_fd, .events = POLLIN },
    };

    while (1) {
        int ready = poll(fds, 2, -1);
        if (ready < 0) {
            if (errno == EINTR)
                continue;
            perror("poll");
            break;
        }

        struct input_event ev;

        if (fds[0].revents & POLLIN) {
            while (read(main_fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev))
                handle_main_event(&ev);
        }
        if (fds[1].revents & POLLIN) {
            while (read(motion_fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev))
                handle_motion_event(&ev);
        }
    }

    close(main_fd);
    close(motion_fd);
    return 0;
}
