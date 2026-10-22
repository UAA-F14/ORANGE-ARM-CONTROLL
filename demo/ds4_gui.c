// GUI con SDL2 para ver en vivo los joysticks, gatillos, D-pad, botones
// y el giroscopio/acelerometro de un control DS4 (PS4), leidos por evdev
// puro (igual que ds4_read.c). Solo para probar en la laptop/escritorio;
// la Orange Pi al correr sin pantalla seguiria usando la version de
// terminal (ds4_read.c), no esta GUI.
//
// Sin texto (no usa SDL_ttf): todo se distingue por posicion/forma/color.
//   - Circulo izq/der = stick L/R, el punto es la posicion actual.
//   - Barras verticales abajo = gatillos L2/R2.
//   - Cruz chica = D-pad.
//   - Cuadros en rombo (derecha) = X/Circulo/Cuadro/Triangulo.
//   - Barras horizontales abajo = acelerometro (rojo/verde/azul = X/Y/Z)
//     y giroscopio (mismo esquema de color, grupo de abajo).
//
// Compilar: gcc ds4_gui.c -o ds4_gui $(pkg-config --cflags --libs sdl2) -lm
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <SDL2/SDL.h>

#define MAIN_DEV_NAME   "Wireless Controller"
#define MOTION_DEV_NAME "Wireless Controller Motion Sensors"

#define WIN_W 900
#define WIN_H 560

// ---------- Paleta (tema claro) ----------
static const SDL_Color COL_BG      = {245, 245, 240, 255};
static const SDL_Color COL_PANEL   = {255, 255, 255, 255};
static const SDL_Color COL_OUTLINE = {170, 170, 170, 255};
static const SDL_Color COL_DOT     = {40,  110, 200, 255};
static const SDL_Color COL_TEXT_X  = {210, 70,  70,  255}; // rojo = eje X
static const SDL_Color COL_TEXT_Y  = {60,  150, 80,  255}; // verde = eje Y
static const SDL_Color COL_TEXT_Z  = {70,  90,  200, 255}; // azul = eje Z
static const SDL_Color COL_BTN_OFF = {225, 225, 220, 255};
static const SDL_Color COL_BTN_ON  = {230, 160, 40,  255};

// ---------- Estado leido del control ----------
struct ds4_state {
    double stickL_x, stickL_y;   // normalizado -1..1
    double stickR_x, stickR_y;
    double trig_l2, trig_r2;     // 0..1
    int    dpad_x, dpad_y;       // -1,0,1
    int    btn_south, btn_east, btn_north, btn_west;
    int    btn_l1, btn_r1, btn_l3, btn_r3;
    double accel_x, accel_y, accel_z; // normalizado -1..1
    double gyro_x, gyro_y, gyro_z;    // normalizado -1..1
};

static int find_event_device(const char *name)
{
    DIR *dir = opendir("/dev/input");
    if (!dir) { perror("opendir /dev/input"); return -1; }

    struct dirent *entry;
    int fd = -1;
    char path[300], dev_name[256];

    while ((entry = readdir(dir)) != NULL) {
        if (strncmp(entry->d_name, "event", 5) != 0)
            continue;
        snprintf(path, sizeof(path), "/dev/input/%s", entry->d_name);
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

// Centro+rango de un eje, leido una vez al inicio con EVIOCGABS.
struct axis_range { int min, max; };

static struct axis_range get_range(int fd, int code)
{
    struct input_absinfo ai;
    struct axis_range r = {0, 1};
    if (ioctl(fd, EVIOCGABS(code), &ai) >= 0) {
        r.min = ai.minimum;
        r.max = ai.maximum;
    }
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

// ---------- Dibujo ----------

static void set_color(SDL_Renderer *r, SDL_Color c)
{
    SDL_SetRenderDrawColor(r, c.r, c.g, c.b, c.a);
}

static void draw_circle_outline(SDL_Renderer *r, int cx, int cy, int radius)
{
    for (int angle = 0; angle < 360; angle++) {
        double rad = angle * M_PI / 180.0;
        int x = cx + (int)(radius * cos(rad));
        int y = cy + (int)(radius * sin(rad));
        SDL_RenderDrawPoint(r, x, y);
    }
}

static void draw_filled_circle(SDL_Renderer *r, int cx, int cy, int radius)
{
    for (int dy = -radius; dy <= radius; dy++) {
        int dx = (int)sqrt((double)(radius * radius - dy * dy));
        SDL_RenderDrawLine(r, cx - dx, cy + dy, cx + dx, cy + dy);
    }
}

static void draw_stick(SDL_Renderer *ren, int cx, int cy, int radius, double x, double y)
{
    set_color(ren, COL_PANEL);
    draw_filled_circle(ren, cx, cy, radius);
    set_color(ren, COL_OUTLINE);
    draw_circle_outline(ren, cx, cy, radius);

    int dotx = cx + (int)(x * radius);
    int doty = cy + (int)(y * radius);
    set_color(ren, COL_DOT);
    draw_filled_circle(ren, dotx, doty, 10);
}

// Barra vertical que crece hacia arriba desde `base_y`, usada para L2/R2.
static void draw_vbar(SDL_Renderer *ren, int x, int base_y, int w, int max_h, double v, SDL_Color c)
{
    SDL_Rect outline = { x, base_y - max_h, w, max_h };
    set_color(ren, COL_OUTLINE);
    SDL_RenderDrawRect(ren, &outline);

    int h = (int)(v * max_h);
    SDL_Rect fill = { x, base_y - h, w, h };
    set_color(ren, c);
    SDL_RenderFillRect(ren, &fill);
}

// Barra horizontal centrada en `cx`, para acelerometro/giroscopio
// (valores con signo, -1..1).
static void draw_hbar_signed(SDL_Renderer *ren, int cx, int y, int w, int h, double v, SDL_Color c)
{
    SDL_Rect outline = { cx - w / 2, y, w, h };
    set_color(ren, COL_OUTLINE);
    SDL_RenderDrawRect(ren, &outline);

    int half = w / 2;
    int len = (int)(v * half);
    SDL_Rect fill;
    if (len >= 0)
        fill = (SDL_Rect){ cx, y, len, h };
    else
        fill = (SDL_Rect){ cx + len, y, -len, h };
    set_color(ren, c);
    SDL_RenderFillRect(ren, &fill);

    set_color(ren, COL_OUTLINE);
    SDL_RenderDrawLine(ren, cx, y, cx, y + h);
}

static void draw_button(SDL_Renderer *ren, int cx, int cy, int size, int pressed)
{
    SDL_Rect box = { cx - size / 2, cy - size / 2, size, size };
    set_color(ren, pressed ? COL_BTN_ON : COL_BTN_OFF);
    SDL_RenderFillRect(ren, &box);
    set_color(ren, COL_OUTLINE);
    SDL_RenderDrawRect(ren, &box);
}

static void draw_dpad(SDL_Renderer *ren, int cx, int cy, int step, int dx, int dy)
{
    draw_button(ren, cx,        cy - step, 24, dy < 0);
    draw_button(ren, cx,        cy + step, 24, dy > 0);
    draw_button(ren, cx - step, cy,        24, dx < 0);
    draw_button(ren, cx + step, cy,        24, dx > 0);
}

static void render_frame(SDL_Renderer *ren, const struct ds4_state *s)
{
    set_color(ren, COL_BG);
    SDL_RenderClear(ren);

    // Sticks
    draw_stick(ren, 150, 160, 80, s->stickL_x, s->stickL_y);
    draw_stick(ren, 420, 160, 80, s->stickR_x, s->stickR_y);

    // Gatillos L2/R2
    draw_vbar(ren, 600, 260, 40, 140, s->trig_l2, COL_TEXT_X);
    draw_vbar(ren, 660, 260, 40, 140, s->trig_r2, COL_TEXT_Y);

    // D-pad
    draw_dpad(ren, 150, 330, 30, s->dpad_x, s->dpad_y);

    // Botones en rombo (X, Circulo, Cuadro, Triangulo)
    draw_button(ren, 780, 300, 34, s->btn_south); // abajo = X
    draw_button(ren, 820, 260, 34, s->btn_east);  // derecha = Circulo
    draw_button(ren, 740, 260, 34, s->btn_west);  // izquierda = Cuadro
    draw_button(ren, 780, 220, 34, s->btn_north); // arriba = Triangulo

    // L1/R1/L3/R3
    draw_button(ren, 420, 300, 50, s->btn_l1);
    draw_button(ren, 480, 300, 50, s->btn_r1);
    draw_button(ren, 150, 270, 20, s->btn_l3);
    draw_button(ren, 420, 270, 20, s->btn_r3);

    // Acelerometro (fila superior) y giroscopio (fila inferior),
    // rojo/verde/azul = X/Y/Z en ambos casos.
    int bar_w = 700, bar_h = 20, bar_x = 450;
    draw_hbar_signed(ren, bar_x, 380, bar_w, bar_h, s->accel_x, COL_TEXT_X);
    draw_hbar_signed(ren, bar_x, 405, bar_w, bar_h, s->accel_y, COL_TEXT_Y);
    draw_hbar_signed(ren, bar_x, 430, bar_w, bar_h, s->accel_z, COL_TEXT_Z);

    draw_hbar_signed(ren, bar_x, 470, bar_w, bar_h, s->gyro_x, COL_TEXT_X);
    draw_hbar_signed(ren, bar_x, 495, bar_w, bar_h, s->gyro_y, COL_TEXT_Y);
    draw_hbar_signed(ren, bar_x, 520, bar_w, bar_h, s->gyro_z, COL_TEXT_Z);

    SDL_RenderPresent(ren);
}

int main(void)
{
    int main_fd = find_event_device(MAIN_DEV_NAME);
    int motion_fd = find_event_device(MOTION_DEV_NAME);
    if (main_fd < 0 || motion_fd < 0) {
        fprintf(stderr, "No se encontro el control (¿conectado?).\n");
        return 1;
    }

    struct axis_range r_stickLx = get_range(main_fd, ABS_X);
    struct axis_range r_stickLy = get_range(main_fd, ABS_Y);
    struct axis_range r_stickRx = get_range(main_fd, ABS_RX);
    struct axis_range r_stickRy = get_range(main_fd, ABS_RY);
    struct axis_range r_l2      = get_range(main_fd, ABS_Z);
    struct axis_range r_r2      = get_range(main_fd, ABS_RZ);
    struct axis_range r_accelx  = get_range(motion_fd, ABS_X);
    struct axis_range r_accely  = get_range(motion_fd, ABS_Y);
    struct axis_range r_accelz  = get_range(motion_fd, ABS_Z);
    struct axis_range r_gyrox   = get_range(motion_fd, ABS_RX);
    struct axis_range r_gyroy   = get_range(motion_fd, ABS_RY);
    struct axis_range r_gyroz   = get_range(motion_fd, ABS_RZ);

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Window *win = SDL_CreateWindow("DS4 - Joysticks y Giroscopio",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIN_W, WIN_H, 0);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    struct ds4_state st;
    memset(&st, 0, sizeof(st));

    int running = 1;
    while (running) {
        SDL_Event sdl_ev;
        while (SDL_PollEvent(&sdl_ev)) {
            if (sdl_ev.type == SDL_QUIT)
                running = 0;
        }

        struct input_event ev;
        while (read(main_fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
            if (ev.type == EV_ABS) {
                switch (ev.code) {
                    case ABS_X:     st.stickL_x = normalize_signed(ev.value, r_stickLx); break;
                    case ABS_Y:     st.stickL_y = normalize_signed(ev.value, r_stickLy); break;
                    case ABS_RX:    st.stickR_x = normalize_signed(ev.value, r_stickRx); break;
                    case ABS_RY:    st.stickR_y = normalize_signed(ev.value, r_stickRy); break;
                    case ABS_Z:     st.trig_l2  = normalize_unsigned(ev.value, r_l2); break;
                    case ABS_RZ:    st.trig_r2  = normalize_unsigned(ev.value, r_r2); break;
                    case ABS_HAT0X: st.dpad_x   = ev.value; break;
                    case ABS_HAT0Y: st.dpad_y   = ev.value; break;
                }
            } else if (ev.type == EV_KEY) {
                int pressed = ev.value;
                switch (ev.code) {
                    case BTN_SOUTH: st.btn_south = pressed; break;
                    case BTN_EAST:  st.btn_east  = pressed; break;
                    case BTN_NORTH: st.btn_north = pressed; break;
                    case BTN_WEST:  st.btn_west  = pressed; break;
                    case BTN_TL:    st.btn_l1    = pressed; break;
                    case BTN_TR:    st.btn_r1    = pressed; break;
                    case BTN_THUMBL:st.btn_l3    = pressed; break;
                    case BTN_THUMBR:st.btn_r3    = pressed; break;
                }
            }
        }

        while (read(motion_fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
            if (ev.type == EV_ABS) {
                switch (ev.code) {
                    case ABS_X:  st.accel_x = normalize_signed(ev.value, r_accelx); break;
                    case ABS_Y:  st.accel_y = normalize_signed(ev.value, r_accely); break;
                    case ABS_Z:  st.accel_z = normalize_signed(ev.value, r_accelz); break;
                    case ABS_RX: st.gyro_x  = normalize_signed(ev.value, r_gyrox); break;
                    case ABS_RY: st.gyro_y  = normalize_signed(ev.value, r_gyroy); break;
                    case ABS_RZ: st.gyro_z  = normalize_signed(ev.value, r_gyroz); break;
                }
            }
        }

        render_frame(ren, &st);
        SDL_Delay(16); // ~60 fps
    }

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    close(main_fd);
    close(motion_fd);
    return 0;
}
