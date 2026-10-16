# Control de brazo con DS4

Lectura del control PS4 (sticks, botones, giroscopio, acelerómetro) por
`evdev` puro en C, sin librerías externas de joystick — pensado para
correr igual en la laptop (pruebas) y en la Orange Pi (destino final,
sin pantalla).

## Archivos

- **`ds4_read.c`** — Lectura por terminal: imprime en vivo los sticks,
  botones, gatillos, D-pad y giroscopio/acelerómetro. El punto de
  partida, sin dependencias gráficas.
  ```
  gcc ds4_read.c -o ds4_read
  ./ds4_read
  ```

- **`ds4_gui.c`** — Visualización con SDL2: círculos para los sticks,
  barras para gatillos y para acelerómetro/giroscopio, botones en
  rombo. Solo para laptop/escritorio.
  ```
  gcc ds4_gui.c -o ds4_gui $(pkg-config --cflags --libs sdl2) -lm
  ./ds4_gui
  ```

- **`ds4_arm3d.c`** — Simulación 3D (SDL2 + OpenGL) de un brazo de 3
  servos (dos codos en ejes perpendiculares tipo gimbal + gripper),
  controlado en vivo con el DS4. Incluye calibración del sesgo del
  giroscopio al arrancar (deja el control quieto y plano un segundo) y
  cámara orbital con el D-pad.
  ```
  gcc ds4_arm3d.c -o ds4_arm3d $(pkg-config --cflags --libs sdl2) -lGL -lGLU -lm
  ./ds4_arm3d
  ```

## Controles (ds4_arm3d)

| Entrada | Accion |
|---|---|
| Stick izquierdo | Codo 1 / servo X (de lado a lado) |
| Stick derecho | Codo 2 / servo Y (arriba/abajo) |
| Girar la muñeca / inclinar el control | Trim que se suma a los sticks (giroscopio) |
| L1 | Reinicia el trim del giroscopio |
| Gatillo R2 | Abre/cierra el gripper |
| D-pad | Orbita la cámara (no mueve ningún servo) |

## Conectar el control por Bluetooth

```
bluetoothctl
agent NoInputNoOutput
default-agent
scan on
pair A0:5A:5E:39:2C:46   # tu MAC puede ser distinta
trust A0:5A:5E:39:2C:46
connect A0:5A:5E:39:2C:46
```

Si da "Permission denied" en el canal HID aunque ya esté "Bonded: yes",
es que el agente de emparejamiento no forzó "Just Works" — por eso el
`agent NoInputNoOutput` antes de parear.

## Pendiente: puerto a servos reales (Orange Pi)

Toda la lectura del control (sticks, giroscopio, calibración, trim) es
matemática pura, no depende de SDL/OpenGL. Para usarlo con servos de
verdad, lo único que cambia es la salida: en vez de `render_frame()`,
llamar una función tipo `pwm_set_servo_angle(canal, grados)` usando el
mismo patrón de sysfs de PWM que ya armamos para el proyecto de la
Orange Pi (pulso de 1-2 ms a 50 Hz típico para servo, en vez del PWM de
100 Hz del motor DC).
