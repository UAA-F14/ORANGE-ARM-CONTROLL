#ifndef PS4_H
#define PS4_H

typedef struct {
    double stick_lx, stick_ly; // -1..1
    double stick_rx, stick_ry; // -1..1
    double trigger_l2, trigger_r2; // 0..1
    int    dpad_x, dpad_y;     // -1, 0, 1

    int btn_south, btn_east, btn_north, btn_west;
    int btn_l1, btn_r1, btn_l2, btn_r2;
    int btn_l3, btn_r3, btn_select, btn_start, btn_mode;

    double gyro_x, gyro_y, gyro_z;     // -1..1, ya con el sesgo restado
    double accel_x, accel_y, accel_z;  // -1..1
} ps4_state;

// Busca "Wireless Controller" y "Wireless Controller Motion Sensors" en
// /dev/input/eventN (por Bluetooth o USB, da igual). Regresa 0 si los
// encontro, -1 si el control no esta conectado.
int ps4_init(void);

// Mide el sesgo del giroscopio -- deja el control quieto y plano antes
// de llamarla. Bloquea aprox. 1 segundo. El sesgo medido se resta
// automaticamente de ahi en adelante en cada ps4_poll().
void ps4_calibrate_gyro(void);

// Lee todos los eventos pendientes (no bloqueante) y actualiza los
// campos de *out que hayan cambiado. Los campos que no tuvieron un
// evento nuevo se quedan con su valor anterior -- por eso *out debe
// ser la MISMA variable en cada llamada, no una recien inicializada.
void ps4_poll(ps4_state *out);

void ps4_cleanup(void);

#endif
