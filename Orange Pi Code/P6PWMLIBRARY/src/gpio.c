#include "gpio.h"
#include <stdio.h>
#include <gpiod.h>

static struct gpiod_chip *chip;
static struct gpiod_line_request *request;
static struct gpiod_line_config *config;


int gpio_init(void){

    chip=gpiod_chip_open("/dev/gpiochip1");

    if(chip==NULL){
        perror("error al abrir GPIO chip");
        return -1;
    }
    config=gpiod_line_config_new();
    if(config==NULL){
        fprintf(stderr,"error al crear GPIO config");
        gpiod_chip_close(chip);
        chip=NULL;
        return -1;
    }
    return 0;

}
int gpio_configure(unsigned int gpio,int mode){
    struct gpiod_line_settings *settings;

    settings=gpiod_line_settings_new();
    if(settings==NULL){
        return 1;
    }
    switch(mode){
        case GPIO_INPUT:
        gpiod_line_settings_set_direction(settings,GPIOD_LINE_DIRECTION_INPUT);
        break;
        case GPIO_INPUT_PULLUP:
        gpiod_line_settings_set_direction(settings,GPIOD_LINE_DIRECTION_INPUT);
        gpiod_line_settings_set_bias(settings,GPIOD_LINE_BIAS_PULL_UP);
        break;
        case GPIO_INPUT_PULLDOWN:
        gpiod_line_settings_set_direction(settings,GPIOD_LINE_DIRECTION_INPUT);
        gpiod_line_settings_set_bias(settings,GPIOD_LINE_BIAS_PULL_DOWN);
        break;
        case GPIO_OUTPUT:
        gpiod_line_settings_set_direction(settings,GPIOD_LINE_DIRECTION_OUTPUT);
        break;
        default:
        gpiod_line_settings_free(settings);
        return -1;
        break;
    }
    if(gpiod_line_config_add_line_settings(config,&gpio,1,settings)<0){
        gpiod_line_settings_free(settings);
        return -1;
    }
    gpiod_line_settings_free(settings);
    return 0;
}
int gpio_start(void){
    request= gpiod_chip_request_lines(chip,NULL,config);
    if(request==NULL){
        perror("error al solicitar líneas GPIO");
        return 1;
    }
    return 0;
}
int gpio_set(unsigned int gpio, int value){
    enum gpiod_line_value state;
    if(value){
        state=GPIOD_LINE_VALUE_ACTIVE;
    }else{
        state=GPIOD_LINE_VALUE_INACTIVE;
    }
    return gpiod_line_request_set_value(request,gpio,state);
}
int gpio_get(unsigned int gpio){
    enum gpiod_line_value value;
    value=gpiod_line_request_get_value(request,gpio);
    if(value == GPIOD_LINE_VALUE_ACTIVE){
        return 1;
    }
    if(value == GPIOD_LINE_VALUE_INACTIVE){
        return 0;
    }
    return -1;
}
void gpio_cleanup(void){
    if(request != NULL){
        gpiod_line_request_release(request);
        request=NULL;
    }
    if(config != NULL){
        gpiod_line_config_free(config);
        config=NULL;
    }
    if(chip != NULL){
        gpiod_chip_close(chip);
        chip=NULL;
    }
}
