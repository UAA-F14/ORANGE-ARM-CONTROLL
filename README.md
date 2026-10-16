# ORANGE-ARM-CONTROLL

Brazo de 3 servos (codo X, codo Y, gripper) controlado en tiempo real
con un control de PS4 por Bluetooth: lectura de sticks, botones y
giroscopio/acelerómetro por `evdev` puro en C, sin librerías externas
de joystick.

## Estructura

- **`P6PWMLIBRARY/`** — Biblioteca de GPIO (`libgpiod`) y PWM
  (`sysfs`). `pwm_open(canal)` regresa un handle por canal, para poder
  tener varios servos activos a la vez.
- **`P8PS4/`** — Biblioteca de lectura del DS4 por `evdev`: sticks,
  botones, gatillos, D-pad, giroscopio y acelerómetro, con calibración
  del sesgo del giroscopio incluida.
- **`EX2BrazoPS4/`** — Programa final para la Orange Pi: usa las dos
  bibliotecas de arriba para mover 3 servos reales con el control.
- **`PS4 CONTROL CODE/`** — Prototipos de laptop/escritorio (antes de
  tener los servos físicos): lectura por terminal, visualización con
  SDL2, y la simulación 3D del brazo con OpenGL.

## Control

| Entrada | Acción |
|---|---|
| Stick izquierdo | Servo X (base) |
| Stick derecho | Servo Y |
| Girar la muñeca / inclinar el control | Trim que se suma a los sticks (giroscopio) |
| L1 | Reinicia el trim del giroscopio |
| Gatillo R2 | Gripper (a fondo = cerrado) |

## Conectar el DS4 por Bluetooth

```bash
bluetoothctl
agent NoInputNoOutput
default-agent
scan on
pair <MAC>
trust <MAC>
connect <MAC>
```

Esta placa necesita los drivers `hid-sony` y `hid-playstation`
compilados aparte (el kernel vendor no los trae) para que el
giroscopio/acelerómetro aparezcan como un dispositivo de entrada
separado ("Wireless Controller Motion Sensors"). Si da "Permission
denied" en el canal HID aunque ya esté "Bonded: yes", es que el agente
de emparejamiento no forzó "Just Works" — por eso el
`agent NoInputNoOutput` antes de parear.

## Compilar (Orange Pi)

Cada carpeta tiene su propio `CMakeLists.txt`:

```bash
cd P6PWMLIBRARY && cmake -S . -B build && cmake --build build
cd P8PS4         && cmake -S . -B build && cmake --build build
cd EX2BrazoPS4   && cmake -S . -B build && cmake --build build
```

`EX2BrazoPS4` compila los fuentes de las otras dos carpetas
directamente (rutas relativas en su `CMakeLists.txt`), no son copias.

## Compilar (laptop, `PS4 CONTROL CODE/`)

```bash
gcc ds4_read.c -o ds4_read
gcc ds4_gui.c -o ds4_gui $(pkg-config --cflags --libs sdl2) -lm
gcc ds4_arm3d.c -o ds4_arm3d $(pkg-config --cflags --libs sdl2) -lGL -lGLU -lm
```

## Pendiente

Cada servo (X, Y, gripper) necesita su propio pin de PWM por hardware,
y cada canal requiere su propio *overlay* de device tree activado en
`/boot/orangepiEnv.txt` (igual que el canal 0 -> pin físico 29 con el
overlay `pwm0`). Falta configurar los overlays de los canales 1 y 2
cuando estén conectados los 3 servos físicos.
