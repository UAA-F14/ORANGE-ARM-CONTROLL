# ORANGE-ARM-CONTROLL

Brazo de 3 servos (codo X, codo Y, gripper) controlado en tiempo real
con un control de PS4 por Bluetooth: lectura de sticks, botones y
giroscopio/acelerómetro por `evdev` puro en C, sin librerías externas
de joystick.

## Estructura

- **`code/`** — Código para la Orange Pi (el destino final). Una sola
  biblioteca/programa:
  - `inc/` — headers (`gpio.h`, `pwm.h`, `ps4.h`)
  - `src/` — implementación (`gpio.c`, `pwm.c`, `ps4.c`)
  - `main.c` — programa principal: lee el control con `ps4.h` y mueve
    los 3 servos con `pwm.h`
  - `CMakeLists.txt`
- **`demo/`** — Prototipos para correr en laptop/escritorio (Linux
  PC), no en la Orange Pi: lectura por terminal, visualización con
  SDL2, y la simulación 3D del brazo con OpenGL.
- **`design/3DDesign/`** — Modelo 3D del brazo (CAD). Placeholder
  hasta que se comparta el link/archivo del diseño.

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

## Compilar (Orange Pi, `code/`)

```bash
cd code
cmake -S . -B build
cmake --build build
./build/EX2BrazoPS4
```

## Compilar (laptop, `demo/`)

```bash
cd demo
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
