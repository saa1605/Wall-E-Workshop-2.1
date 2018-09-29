#ifndef SRA18_H
#define SRA18_h

#include <stdio.h>
#include <math.h>
#include <time.h>

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define GPIO_PWM0A_OUT 27   //Set GPIO 15 as PWM0A - Enable
#define GPIO_PWM0B_OUT 14   //Set GPIO 16 as PWM0B 
#define GPIO_NUM0 25  //GPIO to input pin of motor driver
#define GPIO_NUM1 26  //GPIO to input pin of motor driver
#define GPIO_NUM2 16
#define GPIO_NUM3 17

float map(float x, float min_in, float max_in, float min_out, float max_out);

float constrain(float x, float lower_limit, float higher_limit);

float absolute(float number);

void adc1_init();

void mcpwm_gpio_initialize();

void mcpwm_initialize();

void bot_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle1, float duty_cycle2);

void bot_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle1, float duty_cycle2);

void bot_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num);

#endif
