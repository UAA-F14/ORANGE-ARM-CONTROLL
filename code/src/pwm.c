#include "pwm.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

const int PWM_PIN[8] = {29, 0, 33, 35, 37, 32, 36, 38};

struct pwm_channel {
    int  channel;
    char base_path[64];
};

static int write_int(const char *path, long value)
{
    FILE *f = fopen(path, "w");
    if (!f) {
        fprintf(stderr, "No se pudo abrir %s: %s\n", path, strerror(errno));
        return -1;
    }
    fprintf(f, "%ld", value);
    if (fclose(f) != 0) {   // el error real de sysfs aparece aqui
        fprintf(stderr, "Error escribiendo %ld en %s: %s\n", value, path, strerror(errno));
        return -1;
    }
    return 0;
}

pwm_channel *pwm_open(int channel)
{
    struct stat st;
    char export_path[64];
    char base_path[64];

    snprintf(export_path, sizeof(export_path), "%s/export", PWM_CHIP_PATH);
    snprintf(base_path, sizeof(base_path), "%s/pwm%d", PWM_CHIP_PATH, channel);

    // Export solo si el canal no existe todavia (reexportar uno ya
    // activo regresa EBUSY)
    if (stat(base_path, &st) != 0) {
        if (write_int(export_path, channel) != 0)
            return NULL;
        usleep(200000); // esperar a que el kernel cree la carpeta pwmN
    }

    pwm_channel *ch = malloc(sizeof(*ch));
    if (!ch) {
        fprintf(stderr, "pwm_open: sin memoria\n");
        return NULL;
    }
    ch->channel = channel;
    snprintf(ch->base_path, sizeof(ch->base_path), "%s", base_path);

    pwm_disable(ch); // apagar antes de que el caller lo reconfigure
    return ch;
}

int pwm_set_period(pwm_channel *ch, long period_ns)
{
    char path[80];
    snprintf(path, sizeof(path), "%s/period", ch->base_path);
    return write_int(path, period_ns);
}

int pwm_set_duty_cycle(pwm_channel *ch, long duty_ns)
{
    char path[80];
    snprintf(path, sizeof(path), "%s/duty_cycle", ch->base_path);
    return write_int(path, duty_ns);
}

int pwm_enable(pwm_channel *ch)
{
    char path[80];
    snprintf(path, sizeof(path), "%s/enable", ch->base_path);
    return write_int(path, 1);
}

int pwm_disable(pwm_channel *ch)
{
    char path[80];
    snprintf(path, sizeof(path), "%s/enable", ch->base_path);
    return write_int(path, 0);
}

void pwm_close(pwm_channel *ch)
{
    if (!ch) return;

    char unexport_path[64];
    pwm_disable(ch);
    snprintf(unexport_path, sizeof(unexport_path), "%s/unexport", PWM_CHIP_PATH);
    write_int(unexport_path, ch->channel);
    free(ch);
}
