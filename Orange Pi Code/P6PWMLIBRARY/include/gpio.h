#ifndef GPIO_H
#define GPIO_H
#define GPIO_INPUT 0
#define GPIO_OUTPUT 1
#define GPIO_INPUT_PULLUP 2
#define GPIO_INPUT_PULLDOWN 3

int gpio_init(void);
int gpio_configure(unsigned int gpio,int mode);
int gpio_start(void);
int gpio_set(unsigned int gpio, int value);
int gpio_get(unsigned int gpio);
void gpio_cleanup(void);
#endif
