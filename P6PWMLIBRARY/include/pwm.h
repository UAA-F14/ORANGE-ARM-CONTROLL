#ifndef PWM_H
#define PWM_H

#define PWM_CHIP_PATH "/sys/class/pwm/pwmchip0"

/*
PWM Bus     Pin     dtbo
SPWM0-2     7       spwm2
PWM0-0      29      pwm0
PWM0-2      33      pwm2
PWM0-3      35      pwm3
PWM0-4      37      pwm4
PWM0-5      32      pwm5
PWM0-6      36      pwm6
PWM0-7      38      pwm7
*/
extern const int PWM_PIN[8];

// Handle opaco a un canal PWM ya exportado. Se pueden tener varios
// abiertos al mismo tiempo (uno por servo, por ejemplo) -- a
// diferencia de la version anterior de esta libreria, que solo podia
// operar un canal "activo" a la vez.
typedef struct pwm_channel pwm_channel;

// Exporta el canal (0..7, ver tabla de arriba) y regresa un handle
// para usarlo. NULL si fallo.
pwm_channel *pwm_open(int channel);

int  pwm_set_period(pwm_channel *ch, long period_ns);
int  pwm_set_duty_cycle(pwm_channel *ch, long duty_ns);
int  pwm_enable(pwm_channel *ch);
int  pwm_disable(pwm_channel *ch);
void pwm_close(pwm_channel *ch); // disable + unexport + libera el handle

#endif
