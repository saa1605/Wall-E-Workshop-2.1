#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
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
#define GPIO_NUM2 13
#define GPIO_NUM3 16

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate

adc1_channel_t channel[4] = {ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_0, ADC_CHANNEL_3};
static const adc_atten_t atten = ADC_ATTEN_11db;
static const adc_unit_t unit = ADC_UNIT_1;

int weights[4] = {-3,-1,1,3};

/*
 * Line Following PID Constants
 */
#define kP 3
#define kI 0
#define kD 0.8

/*
 * Motor value constraints
 */
float opt = 60;
float lower_pwm_constrain = 60;
float higher_pwm_constrain = 80;
float left_pwm = 0, right_pwm = 0;

/*
 * Line Following PID Variables
 */
float error=0, prev_error, difference, cumulative_error, correction;

uint32_t adc_reading[4];
float sensor_value[4];





static void adc1_init()
{
    //Check if Two Point or Vref are burned into eFuse

    //Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    for(int i = 0;i < 4;i++)
    {
      adc1_config_channel_atten(channel[i], atten);
    }
}

static void mcpwm_gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}

/**
 * @brief Configure MCPWM module for brushed dc motor
 */
static void mcpwm_initialize()
{
    //1. mcpwm gpio initialization
    mcpwm_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    printf("Configuring pwm_config...\n");
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    printf("Initialize pwm_init...\n");
    gpio_set_direction(GPIO_NUM0, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM1, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM2, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM3, GPIO_MODE_OUTPUT);
    printf("Set direction to GPIO pins...\n");
}

/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
static void set_mcpwm(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle1, float duty_cycle2)
{
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle1);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle2);
    gpio_set_level(GPIO_NUM0,0);
    gpio_set_level(GPIO_NUM1,1);
    gpio_set_level(GPIO_NUM2,0);
    gpio_set_level(GPIO_NUM3,1);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

// /**
//  * @brief motor stop
//  */
// static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
// {
//     mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
//     mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
// }

float map(float x, float min_in, float max_in, float min_out, float max_out)
{
    return (x - min_in) * (max_out - min_out) / (max_in - min_in) + min_out;
}

float constrain(float x, float lower_limit, float higher_limit)
{
    if(x < lower_limit)
        x = lower_limit;
    
    else if(x > higher_limit)
        x = higher_limit;

    return x;
}

float absolute(float number)
{
    if(number < 0)
    {
        return (-1)*number;
    }

    return number;
}

static void read_sensors()
{
  for(int i = 0; i < 4; i++)
    {
        adc_reading[i] = adc1_get_raw(channel[i]);
    }
}

static void calc_sensor_values()
{
    for(int i = 0; i < 4; i++)
    {
        sensor_value[i] = map(adc_reading[i], 1700, 4000, 0, 1000);
        sensor_value[i] = constrain(sensor_value[i],0,1000);
        // if(sensor_value[i] < 0)
        //     sensor_value[i] = 0;
        // else if(sensor_value[i] > 1000)
        //     sensor_value[i] = 1000;
        // printf("Sensor %d\t %d\n",i ,sensor_value[i] );
    }
}

static void calc_error()
{
    int all_black_flag = 1;
    long int weighted_sum = 0, sum = 0, pos = 0;
    
    for(int i = 0; i < 4; i++)
    {
        if(sensor_value[i] > 400)
            all_black_flag = 0;
        
        if(sensor_value[i] > 10)
        {
            weighted_sum += (long)(sensor_value[i]) * (weights[i] * 1000);
            sum += sensor_value[i];
        }
    }
    
    if(sum != 0)
    {
        pos = weighted_sum / sum;
    }

    if(all_black_flag == 1)
    {
        if(error > 0)
            pos = 3000;
        else
            pos = -3000;
    }

    error = pos;
}

static void calc_correction()
{
    error *= 0.01;
    difference = error - prev_error;
    cumulative_error += error;
    
    if(cumulative_error > 30)
    {
        cumulative_error = 30;
    }
    
    else if(cumulative_error < -30)
    {
        cumulative_error = -30;
    }
    
    correction = kP*error + kI*cumulative_error + kD*difference;
    prev_error = error;

    left_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
    right_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
}

void app_main()
{
  adc1_init();
  mcpwm_initialize();

      while(1)
      {
        read_sensors();
        calc_sensor_values();
        calc_error();
        calc_correction();
        set_mcpwm(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
        printf("%d\n", error);
        // printf("Left Pwm: %f\t",left_pwm );
        // printf("Right Pwm: %f\n",right_pwm );
        // printf("Error: %f\t",left_pwm );
        // printf("Error: %f\n",right_pwm );
    }
}