# Brazo de 3 servos controlado con un DS4

Control en tiempo real de un brazo de 3 servos (codo X, codo Y,
gripper) usando un control de PS4 por Bluetooth, en la Orange Pi 4 Pro.

## Estructura

- **`P6PWMLIBRARY/`** — Biblioteca de GPIO (`libgpiod`) y PWM
  (`sysfs`). `pwm_open(canal)` regresa un handle por canal, para poder
  tener varios servos activos a la vez.
- **`P8PS4/`** — Biblioteca de lectura del DS4 por `evdev`: sticks,
  botones, gatillos, D-pad, giroscopio y acelerometro, con calibracion
  del sesgo del giroscopio incluida.
- **`EX2BrazoPS4/`** — Programa final: usa las dos bibliotecas de
  arriba para mover 3 servos reales con el control.

## Control

| Entrada | Accion |
|---|---|
| Stick izquierdo | Servo X (base) |
| Stick derecho | Servo Y |
| Girar la muneca / inclinar el control | Trim que se suma a los sticks (giroscopio) |
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
giroscopio/acelerometro aparezcan como un dispositivo de entrada
separado ("Wireless Controller Motion Sensors").

## Compilar

Cada carpeta tiene su propio `CMakeLists.txt`:

```bash
cd P6PWMLIBRARY && cmake -S . -B build && cmake --build build
cd P8PS4         && cmake -S . -B build && cmake --build build
cd EX2BrazoPS4   && cmake -S . -B build && cmake --build build
```

`EX2BrazoPS4` compila los fuentes de las otras dos carpetas
directamente (rutas relativas en su `CMakeLists.txt`), no son copias.

## Pendiente

Cada servo (X, Y, gripper) necesita su propio pin de PWM por hardware,
y cada canal requiere su propio *overlay* de device tree activado en
`/boot/orangepiEnv.txt` (igual que el canal 0 -> pin fisico 29 con el
overlay `pwm0`). Falta configurar los overlays de los canales 1 y 2
cuando esten conectados los 3 servos fisicos.
