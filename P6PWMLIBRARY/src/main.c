#include "pwm.h"
#include "gpio.h"
#include <signal.h>
#include <string.h>
#include <unistd.h>

#define BUTTON1 5   // increase duty cycle
#define BUTTON2 6   // decrease duty cycle

#define PERIOD_NS   10000000          // 100 Hz
#define DUTY_STEP   (PERIOD_NS / 20)  // 5% per step
#define REPEAT_US   100000            // step rate while a button is held (100 ms)

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

    // GPIO
    if (gpio_init() != 0)
        return 1;
    gpio_configure(BUTTON1, GPIO_INPUT_PULLUP);
    gpio_configure(BUTTON2, GPIO_INPUT_PULLUP);
    if (gpio_start() != 0) {
        gpio_cleanup();
        return 1;
    }

    // PWM: channel 0, 100 Hz, start at 50% duty cycle
    pwm_channel *ch = pwm_open(0);
    if (!ch) {
        gpio_cleanup();
        return 1;
    }

    int duty = PERIOD_NS / 2;
    if (pwm_set_period(ch, PERIOD_NS) != 0 ||
        pwm_set_duty_cycle(ch, duty) != 0 ||
        pwm_enable(ch) != 0) {
        pwm_close(ch);
        gpio_cleanup();
        return 1;
    }

    while (running) {
        int changed = 0;

        if (gpio_get(BUTTON1) == 0) {           // pressed (active low)
            duty += DUTY_STEP;
            if (duty > PERIOD_NS)
                duty = PERIOD_NS;
            changed = 1;
        } else if (gpio_get(BUTTON2) == 0) {
            duty -= DUTY_STEP;
            if (duty < 0)
                duty = 0;
            changed = 1;
        }

        if (changed) {
            if (pwm_set_duty_cycle(ch, duty) != 0)
                break;
            usleep(REPEAT_US);                  // holding the button keeps stepping
        } else {
            usleep(10000);
        }
    }

    pwm_close(ch);
    gpio_cleanup();
    return 0;
}
