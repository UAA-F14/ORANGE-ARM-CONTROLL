// Demo de la libreria: imprime en vivo los sticks, gatillos y el
// giroscopio/acelerometro ya calibrados. Ctrl+C para salir.
#include "ps4.h"
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static volatile sig_atomic_t running = 1;

static void on_signal(int sig)
{
    (void)sig;
    running = 0;
}

int main(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    if (ps4_init() != 0)
        return 1;

    ps4_calibrate_gyro();

    ps4_state state;
    memset(&state, 0, sizeof(state));

    printf("Leyendo control. Ctrl+C para salir.\n");

    while (running) {
        ps4_poll(&state);
        printf("\rL(%.2f,%.2f) R(%.2f,%.2f) L2=%.2f R2=%.2f  gyro(%.2f,%.2f,%.2f)  accel(%.2f,%.2f,%.2f)  ",
               state.stick_lx, state.stick_ly, state.stick_rx, state.stick_ry,
               state.trigger_l2, state.trigger_r2,
               state.gyro_x, state.gyro_y, state.gyro_z,
               state.accel_x, state.accel_y, state.accel_z);
        fflush(stdout);
        usleep(50000);
    }

    printf("\n");
    ps4_cleanup();
    return 0;
}
