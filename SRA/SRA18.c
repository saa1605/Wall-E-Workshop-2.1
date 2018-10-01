#include "SRA18.h"

//Functions for custom adjustments

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

void enable_buttons()
{
    gpio_set_direction(BUTTON_1,GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_1,GPIO_PULLUP_ONLY);
    gpio_set_direction(BUTTON_2,GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_2,GPIO_PULLUP_ONLY);
}

int pressed_switch(int button_num)
{
    return !gpio_get_level(button_num);    
}

//LED
void led_blink()
{
    gpio_set_direction(LED_1,GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_2,GPIO_MODE_OUTPUT);   
}

//ADC

 void adc1_init(adc1_channel_t channel[4])
{
    //Configure ADC

    static const adc_atten_t atten = ADC_ATTEN_11db;

    adc1_config_width(ADC_WIDTH_BIT_12);
    
    for(int i = 0;i < 4;i++)
    {
      adc1_config_channel_atten(channel[i], atten);
    }
}

//MCPWM

 void mcpwm_gpio_initialize()
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}

/**
 * @brief Configure MCPWM module for brushed dc motor
 */
 void mcpwm_initialize()
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


 void bot_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle1, float duty_cycle2)
{
    // printf("%s\n","BOT FORWARD");
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle1);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle2);
    gpio_set_level(GPIO_NUM0,0);
    gpio_set_level(GPIO_NUM1,1);
    gpio_set_level(GPIO_NUM2,0);
    gpio_set_level(GPIO_NUM3,1);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

 void bot_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle1, float duty_cycle2)
{
    // printf("%s\n","BOT BACKWARD");
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle1);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle2);
    gpio_set_level(GPIO_NUM0,1);
    gpio_set_level(GPIO_NUM1,0);
    gpio_set_level(GPIO_NUM2,1);
    gpio_set_level(GPIO_NUM3,0);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

 void bot_spot_left(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle1, float duty_cycle2)
{
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle1);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle2);
    gpio_set_level(GPIO_NUM0,1);
    gpio_set_level(GPIO_NUM1,0);
    gpio_set_level(GPIO_NUM2,0);
    gpio_set_level(GPIO_NUM3,1);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

 void bot_spot_right(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle1, float duty_cycle2)
{
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle1);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle2);
    gpio_set_level(GPIO_NUM0,0);
    gpio_set_level(GPIO_NUM1,1);
    gpio_set_level(GPIO_NUM2,1);
    gpio_set_level(GPIO_NUM3,0);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

 void bot_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}