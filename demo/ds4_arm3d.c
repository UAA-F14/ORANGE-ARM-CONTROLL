// Simulacion 3D de un brazo de 3 servos (X, Y, gripper) controlado en
// vivo con un DS4 (lectura evdev, igual que ds4_read.c/ds4_gui.c).
//
// Dos codos en ejes PERPENDICULARES (tipo gimbal/pan-tilt), no en el
// mismo plano:
//   - servo X = codo 1, gira alrededor de un eje VERTICAL (mueve el
//     brazo de lado a lado)
//   - servo Y = codo 2, gira alrededor del eje PERPENDICULAR al
//     anterior, justo despues de el (mueve el brazo arriba/abajo)
//   - el brazo (un solo eslabon) sale despues de los dos codos
//   - servo 3 = gripper, al final del eslabon
//
// Controles:
//   - Stick izquierdo (eje X) -> posicion base del servo X
//   - Stick derecho   (eje Y) -> posicion base del servo Y
//   - Giroscopio   -> "trim" que se SUMA a esa posicion: girar el
//     control en la mano (yaw -> servo X, pitch -> servo Y) mueve el
//     brazo ademas de los sticks. L1 reinicia el trim a 0.
//   - Gatillo R2   -> servo del gripper (apretar = cerrar la pinza)
//   - D-pad        -> orbita la camara alrededor del brazo (no mueve
//     ningun servo, solo cambia el punto de vista)
//
// Nota sobre el giroscopio: da VELOCIDAD angular, no un angulo, asi
// que hay que integrarlo en el tiempo (trim += velocidad * dt) y ese
// trim se va desviando solo ("drift") si el control no esta
// perfectamente quieto -- por eso es un trim que se SUMA al stick y
// se puede resetear con L1, no el control unico. GYRO_SENS de abajo
// es una ganancia ajustada a ojo, no una conversion calibrada a
// grados/segundo reales.
//
// (Se probo tambien sumar el acelerometro -- inclinar el control a
// los lados -- para el eje X, pero el calculo del angulo via atan2()
// da saltos bruscos de +180 a -180 grados en ciertas orientaciones al
// mover el control con la mano, y eso se sentia como que el eje entero
// se volvia loco. Se quito por eso: el eje X se quedo con solo stick +
// giroscopio, igual que el Y.)
//
// Render con OpenGL fijo (immediate mode, sin shaders) via SDL2+GLU:
// suficiente para una demo de cinematica directa, sin la complejidad
// de un pipeline moderno.
//
// Compilar: gcc ds4_arm3d.c -o ds4_arm3d $(pkg-config --cflags --libs sdl2) -lGL -lGLU -lm
#include <dirent.h>
#include <fcntl.h>
#include <linux/input.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <SDL2/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>

#define MAIN_DEV_NAME   "Wireless Controller"
#define MOTION_DEV_NAME "Wireless Controller Motion Sensors"
#define WIN_W 900
#define WIN_H 650

// ---------- Lectura del control (igual que ds4_gui.c) ----------

struct axis_range { int min, max; };

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

// ---------- Cinematica: angulos/estado de los 3 servos ----------

struct arm_state {
    double servo_x_deg;    // -90..90, codo 1 (eje vertical, mueve de lado a lado)
    double servo_y_deg;    // 0..110, codo 2 (eje perpendicular, mueve arriba/abajo)
    double gripper_open;   // 0 (cerrado) .. 1 (abierto)
};

#define GYRO_SENS  1200.0 // grados/segundo a "velocidad" maxima del gyro (ajustar a ojo)
#define SERVO_X_MIN -90.0
#define SERVO_X_MAX  90.0
#define SERVO_Y_MIN   0.0
#define SERVO_Y_MAX 110.0

static double clamp(double v, double lo, double hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ---------- Camara orbital (D-pad) ----------

struct camera {
    double azimuth_deg;   // vuelta horizontal alrededor del brazo
    double elevation_deg; // altura del angulo de vista
    double distance;
};

#define CAM_SPEED 90.0 // grados/segundo mientras se mantiene el D-pad
#define CAM_ELEV_MIN 10.0
#define CAM_ELEV_MAX 85.0

#define BASE_H     1.0
#define ARM_LEN    3.8
#define LINK_R     0.18
#define FINGER_LEN 0.5

static GLUquadric *quad;

static void draw_cylinder(double length, double radius)
{
    glPushMatrix();
    gluCylinder(quad, radius, radius, length, 16, 4);
    gluDisk(quad, 0, radius, 16, 1); // tapa en la base
    glTranslated(0, 0, length);
    gluDisk(quad, 0, radius, 16, 1); // tapa en la punta
    glPopMatrix();
}

static void draw_joint(double radius)
{
    gluSphere(quad, radius * 1.3, 16, 16);
}

static void draw_box(double sx, double sy, double sz)
{
    glPushMatrix();
    glScaled(sx, sy, sz);
    glBegin(GL_QUADS);
        glVertex3d(-0.5,-0.5,-0.5); glVertex3d(0.5,-0.5,-0.5); glVertex3d(0.5,0.5,-0.5); glVertex3d(-0.5,0.5,-0.5);
        glVertex3d(-0.5,-0.5,0.5);  glVertex3d(0.5,-0.5,0.5);  glVertex3d(0.5,0.5,0.5);  glVertex3d(-0.5,0.5,0.5);
        glVertex3d(-0.5,-0.5,-0.5); glVertex3d(-0.5,0.5,-0.5); glVertex3d(-0.5,0.5,0.5); glVertex3d(-0.5,-0.5,0.5);
        glVertex3d(0.5,-0.5,-0.5);  glVertex3d(0.5,0.5,-0.5);  glVertex3d(0.5,0.5,0.5);  glVertex3d(0.5,-0.5,0.5);
        glVertex3d(-0.5,-0.5,-0.5); glVertex3d(0.5,-0.5,-0.5); glVertex3d(0.5,-0.5,0.5); glVertex3d(-0.5,-0.5,0.5);
        glVertex3d(-0.5,0.5,-0.5);  glVertex3d(0.5,0.5,-0.5);  glVertex3d(0.5,0.5,0.5);  glVertex3d(-0.5,0.5,0.5);
    glEnd();
    glPopMatrix();
}

// El brazo se construye encadenando transformaciones sobre la MISMA
// matriz acumulada (sin push/pop entre articulaciones): cada
// glRotate/glTranslate queda heredado por todo lo que se dibuja
// despues. Asi "avanzar hasta la punta del eslabon" es simplemente
// trasladarse a lo largo del mismo eje en el que se dibujo el cilindro
// (Z local), sin tener que recalcular ni repetir ninguna rotacion.
static void draw_arm(const struct arm_state *a)
{
    // Base: cilindro corto, vertical, fija (ya no gira -- sin yaw)
    glColor3f(0.55f, 0.55f, 0.58f);
    glPushMatrix();
    glRotated(-90, 1, 0, 0);
    draw_cylinder(BASE_H, LINK_R * 1.6);
    glPopMatrix();

    glTranslated(0, BASE_H, 0);
    glRotated(a->servo_x_deg, 0, 1, 0); // codo 1 (servo X): eje vertical, de lado a lado
    draw_joint(LINK_R);

    // Codo 2 gira en el eje perpendicular al de codo 1 -- como ya
    // estamos dentro del marco rotado por el yaw de arriba, "rotar en
    // Z local" aqui automaticamente es perpendicular a ese eje
    // vertical, sin importar hacia donde haya girado codo 1.
    glRotated(-a->servo_y_deg, 0, 0, 1); // codo 2 (servo Y): arriba/abajo
    glRotated(-90, 0, 1, 0);             // realinear para que el brazo crezca hacia adelante
    draw_joint(LINK_R * 0.9);

    glColor3f(0.25f, 0.45f, 0.75f);
    draw_cylinder(ARM_LEN, LINK_R);
    glTranslated(0, 0, ARM_LEN);
    draw_joint(LINK_R * 0.8);

    // Gripper: dos dedos que se separan segun gripper_open (0..1)
    glColor3f(0.80f, 0.20f, 0.20f);
    double spread = 0.08 + a->gripper_open * 0.22;
    glPushMatrix();
    glTranslated(spread, 0, FINGER_LEN / 2);
    draw_box(0.06, 0.10, FINGER_LEN);
    glPopMatrix();
    glPushMatrix();
    glTranslated(-spread, 0, FINGER_LEN / 2);
    draw_box(0.06, 0.10, FINGER_LEN);
    glPopMatrix();
}

static void draw_ground_grid(void)
{
    glColor3f(0.75f, 0.75f, 0.72f);
    glBegin(GL_LINES);
    for (int i = -5; i <= 5; i++) {
        glVertex3d(i, 0, -5); glVertex3d(i, 0, 5);
        glVertex3d(-5, 0, i); glVertex3d(5, 0, i);
    }
    glEnd();
}

static void render_frame(const struct arm_state *a, const struct camera *cam)
{
    glClearColor(0.93f, 0.93f, 0.90f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)WIN_W / WIN_H, 0.1, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    double az = cam->azimuth_deg * M_PI / 180.0;
    double el = cam->elevation_deg * M_PI / 180.0;
    double eye_x = cam->distance * cos(el) * sin(az);
    double eye_y = cam->distance * sin(el);
    double eye_z = cam->distance * cos(el) * cos(az);
    gluLookAt(eye_x, eye_y, eye_z,   0, 1.5, 0,   0, 1, 0);

    draw_ground_grid();

    glPushMatrix();
    draw_arm(a);
    glPopMatrix();
}

// El giroscopio casi siempre tiene un pequeno sesgo (bias) constante
// aunque el control este perfectamente quieto; ese sesgo, al
// integrarse en el tiempo, es la mayor parte del "drift". Aqui se
// mide el promedio de las lecturas mientras el control esta quieto y
// plano, y ese promedio se resta de ahi en adelante como correccion.
static void calibrate_gyro(int motion_fd, struct axis_range r_yaw, struct axis_range r_pitch,
                            double *out_yaw_bias, double *out_pitch_bias)
{
    printf("Calibrando giroscopio: deja el control quieto y plano...\n");
    fflush(stdout);
    SDL_Delay(400); // tiempo para acomodar el control antes de muestrear

    double sum_yaw = 0, sum_pitch = 0;
    int n_yaw = 0, n_pitch = 0;
    Uint32 start = SDL_GetTicks();
    while (SDL_GetTicks() - start < 600) {
        struct input_event ev;
        while (read(motion_fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
            if (ev.type != EV_ABS) continue;
            if (ev.code == ABS_RZ) { sum_yaw += normalize_signed(ev.value, r_yaw); n_yaw++; }
            else if (ev.code == ABS_RX) { sum_pitch += normalize_signed(ev.value, r_pitch); n_pitch++; }
        }
        SDL_Delay(5);
    }
    *out_yaw_bias   = n_yaw   ? sum_yaw   / n_yaw   : 0.0;
    *out_pitch_bias = n_pitch ? sum_pitch / n_pitch : 0.0;
    printf("Calibracion lista (bias yaw=%.4f pitch=%.4f)\n", *out_yaw_bias, *out_pitch_bias);
    fflush(stdout);
}

int main(void)
{
    int main_fd = find_event_device(MAIN_DEV_NAME);
    int motion_fd = find_event_device(MOTION_DEV_NAME);
    if (main_fd < 0 || motion_fd < 0) {
        fprintf(stderr, "No se encontro el control (¿conectado?).\n");
        return 1;
    }

    struct axis_range r_stickLx = get_range(main_fd, ABS_X);   // stick izq: servo X
    struct axis_range r_stickRy = get_range(main_fd, ABS_RY);  // stick der: servo Y
    struct axis_range r_r2      = get_range(main_fd, ABS_RZ);
    struct axis_range r_gyro_yaw   = get_range(motion_fd, ABS_RZ); // girar la muneca izq/der
    struct axis_range r_gyro_pitch = get_range(motion_fd, ABS_RX); // inclinar adelante/atras

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }

    double yaw_bias = 0, pitch_bias = 0;
    calibrate_gyro(motion_fd, r_gyro_yaw, r_gyro_pitch, &yaw_bias, &pitch_bias);

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    SDL_Window *win = SDL_CreateWindow("Brazo 3 servos (X, Y, gripper) - DS4",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIN_W, WIN_H,
        SDL_WINDOW_OPENGL);
    SDL_GLContext gl = SDL_GL_CreateContext(win);
    SDL_GL_SetSwapInterval(1); // vsync

    glEnable(GL_DEPTH_TEST);
    quad = gluNewQuadric();

    struct arm_state state = { 0, 30, 0 };
    double stick_x_deg = 0, stick_y_deg = 30;   // posicion base que pone el stick
    double trim_x_deg = 0, trim_y_deg = 0;      // lo que va sumando el giroscopio
    double gyro_yaw_rate = 0, gyro_pitch_rate = 0; // -1..1, ultima lectura del gyro
    int dpad_x = 0, dpad_y = 0; // -1,0,1: estado actual del D-pad

    struct camera cam = { -45.0, 35.0, 11.0 };

    Uint32 last_ticks = SDL_GetTicks();

    int running = 1;
    while (running) {
        SDL_Event sdl_ev;
        while (SDL_PollEvent(&sdl_ev))
            if (sdl_ev.type == SDL_QUIT) running = 0;

        struct input_event ev;
        while (read(main_fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
            if (ev.type == EV_KEY && ev.code == BTN_TL && ev.value) {
                trim_x_deg = 0; // L1: reinicia el trim del gyro (quita el drift acumulado)
                trim_y_deg = 0;
                continue;
            }
            if (ev.type != EV_ABS) continue;
            if (ev.code == ABS_X) {
                double n = normalize_signed(ev.value, r_stickLx);
                stick_x_deg = n * 90.0;
            } else if (ev.code == ABS_RY) {
                double n = normalize_signed(ev.value, r_stickRy); // -1 arriba .. 1 abajo
                stick_y_deg = (n + 1.0) / 2.0 * 110.0;            // 0..110
            } else if (ev.code == ABS_RZ) {
                double n = normalize_unsigned(ev.value, r_r2);    // 0..1
                state.gripper_open = 1.0 - n; // apretar R2 = cerrar
            } else if (ev.code == ABS_HAT0X) {
                dpad_x = ev.value;
            } else if (ev.code == ABS_HAT0Y) {
                dpad_y = ev.value;
            }
        }

        while (read(motion_fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
            if (ev.type != EV_ABS) continue;
            if (ev.code == ABS_RZ)
                gyro_yaw_rate = normalize_signed(ev.value, r_gyro_yaw) - yaw_bias;
            else if (ev.code == ABS_RX)
                gyro_pitch_rate = normalize_signed(ev.value, r_gyro_pitch) - pitch_bias;
        }

        Uint32 now_ticks = SDL_GetTicks();
        double dt = (now_ticks - last_ticks) / 1000.0;
        last_ticks = now_ticks;

        trim_x_deg += gyro_yaw_rate   * GYRO_SENS * dt;
        trim_y_deg += gyro_pitch_rate * GYRO_SENS * dt;

        state.servo_x_deg = clamp(stick_x_deg + trim_x_deg, SERVO_X_MIN, SERVO_X_MAX);
        state.servo_y_deg = clamp(stick_y_deg + trim_y_deg, SERVO_Y_MIN, SERVO_Y_MAX);

        cam.azimuth_deg += dpad_x * CAM_SPEED * dt;
        cam.elevation_deg = clamp(cam.elevation_deg - dpad_y * CAM_SPEED * dt,
                                   CAM_ELEV_MIN, CAM_ELEV_MAX);

        render_frame(&state, &cam);
        SDL_GL_SwapWindow(win);
        SDL_Delay(16);
    }

    gluDeleteQuadric(quad);
    SDL_GL_DeleteContext(gl);
    SDL_DestroyWindow(win);
    SDL_Quit();
    close(main_fd);
    close(motion_fd);
    return 0;
}
