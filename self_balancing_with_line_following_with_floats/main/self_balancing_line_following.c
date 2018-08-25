/* i2c/mpu6050 - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "esp_system.h"
#include "driver/i2c.h"

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

#define ALPHA 0.9
#define RAD_TO_DEG 57.27272727

#define BUFF_SIZE 6
#define DELAY_TIME_BETWEEN_ITEMS_MS   5 /*!< delay time between different test items, dt should be small */

#define I2C_MASTER_SCL_IO    21   /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    22    /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ    100000     /*!< I2C master clock frequency */

#define MPU6050_ADDR  0x68    /*!< slave address for BH1750 sensor */
#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

#define ACCE_START_ADD 0x3B
#define GYRO_START_ADD 0x43

int sensor_min_array[] = {3493,3954,4031,3976};
int sensor_max_array[] = {2694,1778,1778,1860};

/*
 * Line Following PID Constants
 */
#define yaw_kP 1
#define yaw_kI 0
#define yaw_kD 0

float counter = 0;
uint32_t timer = 0;
float dt = 0;
float complimentary_angle[2] = {0, 0};

bool balanced = false;
float absolute_pitch_angle = 0;
float yaw_error, yaw_prev_error, yaw_difference, yaw_cumulative_error, yaw_correction;

adc1_channel_t channel[4] = {ADC_CHANNEL_0, ADC_CHANNEL_3, ADC_CHANNEL_6, ADC_CHANNEL_7};
static const adc_atten_t atten = ADC_ATTEN_11db;
static const adc_unit_t unit = ADC_UNIT_1;

int weights[4] = {-3,-1,1,3};

uint32_t adc_reading[4];
float sensor_value[4];

float acce_angle[2];
float setpoint = 0;
float initial_angle = -11;
float forward_angle = 7; 


float pitch_angle = 0, pitch_error, prevpitch_error, pitchDifference, pitchCumulativeError, pitch_correction; 

float pitchKp = 4;
float pitchKi = 0.5;
float pitchKd = 1;

float lower_pwm_constrain = 60;
float higher_pwm_constrain = 100;
float left_pwm = 0, right_pwm = 0;

float absolute_pitch_correction = 0;

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

static void adc1_init()
{
    //Configure ADC

    adc1_config_width(ADC_WIDTH_BIT_12);
    for(int i = 0;i < 4;i++)
    {
      adc1_config_channel_atten(channel[i], atten);
    }
}

static void mcpwm_gpio_initialize()
{
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
static void bot_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle1, float duty_cycle2)
{
    printf("%s\n","BOT FORWARD");
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle1);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle2);
    gpio_set_level(GPIO_NUM0,0);
    gpio_set_level(GPIO_NUM1,1);
    gpio_set_level(GPIO_NUM2,0);
    gpio_set_level(GPIO_NUM3,1);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

static void bot_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle1, float duty_cycle2)
{
    printf("%s\n","BOT BACKWARD");
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle1);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle2);
    gpio_set_level(GPIO_NUM0,1);
    gpio_set_level(GPIO_NUM1,0);
    gpio_set_level(GPIO_NUM2,1);
    gpio_set_level(GPIO_NUM3,0);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}
/**
 * @brief motor stop
 */
static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}


/**
 * @brief mpu6050_init, inittialize MPU6050
 */
esp_err_t mpu6050_init(i2c_port_t i2c_num)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU6050_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x6B, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief mpu6050_read_acce Read raw acce values
 */
esp_err_t mpu6050_read_acce(i2c_port_t i2c_num, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU6050_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ACCE_START_ADD, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU6050_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief mpu6050_read_gyro Read raw gyro values
 */
esp_err_t mpu6050_read_gyro(i2c_port_t i2c_num, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU6050_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, GYRO_START_ADD, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU6050_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
void i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test function to show buffer
 */
void disp_buf(int16_t* buf, int len)
{
    int i;
    for(i = 0; i < len; i++) {
        printf("%d ", buf[i]);
    }
    printf("\n");
}

/**
 * @brief shitf_buf make raw values by shift and merge raw values
 */
void shift_buf(uint8_t* buf_1, int16_t* buf_2, int len)
{
    buf_2[0] = ((buf_1[0] << 8) + buf_1[1]);
    buf_2[1] = ((buf_1[2] << 8) + buf_1[3]);
    buf_2[2] = ((buf_1[4] << 8) + buf_1[5]);
}

uint32_t msec()
{
	uint32_t time;
    time = system_get_time();
	return time;
}

esp_err_t complimentory_filter(int16_t* acce_raw_value, int16_t* gyro_raw_value, float complimentary_angle[], int len)
{
    
    float gyro_angle[2];
    float gyro_rate[2];
    int i;
    
    counter++;
    if(counter == 1) {
        acce_angle[0] = (atan2(acce_raw_value[1], acce_raw_value[2]) * RAD_TO_DEG);
        acce_angle[1] = (atan2(-(acce_raw_value[0]), acce_raw_value[2]) * RAD_TO_DEG);
        for( i = 0; i < 2; i++) {
            complimentary_angle[i] =  acce_angle[i];
        }
        timer = system_get_time();
        return ESP_OK;
    }
    dt = (float) (msec() - timer) / 1000000;
    timer = msec();
    acce_angle[0] = (atan2(acce_raw_value[1], acce_raw_value[2]) * RAD_TO_DEG);
    acce_angle[1] = (atan2(-(acce_raw_value[0]), acce_raw_value[2]) * RAD_TO_DEG);
    for( i = 0; i < 2; i++) {
        // printf("gyro:%f\n",gyro_angle[i] );
        // printf("acce:%f\n",acce_angle[i] );
        gyro_rate[i] = gyro_raw_value[i]/131;
        gyro_angle[i] = gyro_rate[i] * dt;
        complimentary_angle[i] = (ALPHA * (complimentary_angle[i] + gyro_angle[i])) + ((1-ALPHA) * acce_angle[i]);    
    }
    return ESP_OK;
}


static void read_sensors()
{
  for(int i = 0; i < 4; i++)
    {
        adc_reading[i] = adc1_get_raw(channel[i]);
        // printf("%d\t",adc_reading[i]);

    }

}

static void calc_sensor_values()
{
    for(int i = 0; i < 4; i++)
    {
        sensor_value[i] = map(adc_reading[i], 1700, 4000, 0, 1000);
        sensor_value[i] = constrain(sensor_value[i],0,1000);
    }

}

static void calculate_yaw_error()
{
    int all_black_flag = 1;
    float weighted_sum = 0, sum = 0, pos = 0;
    
    for(int i = 0; i < 4; i++)
    {
        if(sensor_value[i] > 400)
        {
            all_black_flag = 0;
        }

        weighted_sum += (float)(sensor_value[i]) * (weights[i] * 1000);
        sum += sensor_value[i];
        
    }
    
    if(sum != 0)
    {
        pos = weighted_sum / sum;
    }

    if(all_black_flag == 1)
    {
        if(yaw_error < 0)
            pos = 3000;
        else
            pos = -3000;
    }

    yaw_error = pos;

}

static void calculate_yaw_correction()
{
    yaw_error *= 0.01;
    yaw_difference = yaw_error - yaw_prev_error;
    yaw_cumulative_error += yaw_error;
    
    if(yaw_cumulative_error > 30)
    {
        yaw_cumulative_error = 30;
    }
    
    else if(yaw_cumulative_error < -30)
    {
        yaw_cumulative_error = -30;
    }
    
    yaw_correction = yaw_kP*yaw_error + yaw_kI*yaw_cumulative_error - yaw_kD*yaw_difference;
    yaw_prev_error = yaw_error;

    // left_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
    // right_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
}

float absolute(float number)
{
    if(number < 0)
    {
        return (-1)*number;
    }

    return number;
}


void calculate_pitch_error()
{
    pitch_error = setpoint-pitch_angle;   
    pitchDifference = pitch_error - prevpitch_error;
    pitchCumulativeError += pitch_error;
    if(pitchCumulativeError>40)
        pitchCumulativeError = 40;
    else if(pitchCumulativeError<-40)
        pitchCumulativeError = -40;
    pitch_correction = pitchKp * pitch_error + pitchKi * pitchCumulativeError - pitchKd * pitchDifference;
    prevpitch_error = pitch_error;
}

void print_info()
{        
    // printf("Angles by complimentary filter...\n");
    // printf("ROLL: %f\n", complimentary_angle[0]);
    // printf("Pitch Angle:%f\t",pitch_angle);
    printf("Pitch Error%f\t",pitch_error);
    // printf("YAW CORRECTION: %f\t",yaw_correction);
    // printf("PITCH INPUT%f\n",absolute_pitch_correction );
    // printf("PITCH CORRECTION %f\n",pitch_correction );
    // printf("Absolute Pitch Correction: %f\t",absolute_pitch_correction);
    // printf("Yaw Correction: %f\n", yaw_correction);
    printf("LEFT PWM: %f\t",left_pwm);
    // printf("RIGHT PWM: %f\n",right_pwm);
    printf("\n");
}

void app_main()
{

    //Initialize I2C 
    i2c_master_init();
    setpoint = initial_angle;

    //adc testing
    while(0)
    {

        read_sensors();
        calc_sensor_values();

    }

    // Motor Testing 
    while(0)
    {
        mcpwm_gpio_initialize();
        mcpwm_initialize();
        // for(long int i=0;i< 100000;i++)
        // {
        //     bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 90, 90);
        // }
        // for(long int i=0;i< 100000;i++)
        // {
        //     bot_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 90, 90);
        // }

    }

    // Main code 
    while(1)
    {
        int ret;
        uint8_t* acce_rd = (uint8_t*) malloc(BUFF_SIZE);
        uint8_t* gyro_rd = (uint8_t*) malloc(BUFF_SIZE);
        int16_t* acce_raw_value = (int16_t*) malloc(BUFF_SIZE/2);
        int16_t* gyro_raw_value = (int16_t*) malloc(BUFF_SIZE/2);
        float complimentary_angle[2] = {0, 0};
        ret = mpu6050_init(I2C_MASTER_NUM);
        while(ret != ESP_OK) 
        {
            printf("INIT FAILED... Retry\n");
            vTaskDelay(100/ portTICK_RATE_MS);
            ret = mpu6050_init(I2C_MASTER_NUM);
        }
        printf("INIT SUCESS...\n");
        vTaskDelay(100/ portTICK_RATE_MS);
        mcpwm_gpio_initialize();
        mcpwm_initialize();
        while (1) 
        {
            /*Read raw gyro values*/
            ret = mpu6050_read_gyro(I2C_MASTER_NUM, gyro_rd, BUFF_SIZE);
            shift_buf(gyro_rd, gyro_raw_value, BUFF_SIZE/2);


            /*Read raw acce values*/
            ret = mpu6050_read_acce(I2C_MASTER_NUM, acce_rd, BUFF_SIZE);
            shift_buf(acce_rd, acce_raw_value, BUFF_SIZE/2);

            //Get pitch angle using compimentor filter
            complimentory_filter(acce_raw_value, gyro_raw_value, complimentary_angle, BUFF_SIZE/2);

            pitch_angle = complimentary_angle[1];


            //Calulate PITCH and YAW error
            calculate_pitch_error();
            read_sensors();
            calc_sensor_values();
            calculate_yaw_error();
            calculate_yaw_correction();
            absolute_pitch_correction = absolute(pitch_correction);
            absolute_pitch_correction = constrain(absolute_pitch_correction,0,80);

            
            //if bot is not balance, balance it at 190. Once it is balanced shift the the set-point ahead 

            if(!balanced)
            {
                // printf("%s\n","Not balanced : ");
                if (pitch_error > 2)
                {
                    bot_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);

                }

                else if (pitch_error < -2)
                {
                    bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
                }
                else
                { 
                    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                    pitchCumulativeError = 0;
                    // setpoint = forward_angle;
                    // balanced = true;
                }
                // left_pwm = constrain(absolute_pitch_correction - yaw_correction, lower_pwm_constrain, higher_pwm_constrain);
                // right_pwm = constrain(absolute_pitch_correction + yaw_correction, lower_pwm_constrain, higher_pwm_constrain);
                left_pwm = constrain((absolute_pitch_correction), lower_pwm_constrain, higher_pwm_constrain);
                right_pwm = constrain((absolute_pitch_correction), lower_pwm_constrain, higher_pwm_constrain);            
            }
            else
            {
                printf("%s\n","Perfectly balanced, as all things should be");
                if (pitch_error > 2)
                {
                    bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
                }

                else if (pitch_error < -2)
                {
                    bot_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
                }
                else
                { 
                    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                }

                if(pitch_error>2.5 || pitch_error < -5)
                {
                    setpoint = initial_angle;
                    balanced = false;
                    printf("%s\n","out of forward angle range");
                }

                left_pwm = constrain((absolute_pitch_correction - yaw_correction), lower_pwm_constrain, higher_pwm_constrain);
                right_pwm = constrain((absolute_pitch_correction + yaw_correction), lower_pwm_constrain, higher_pwm_constrain);
                // left_pwm = constrain((absolute_pitch_correction), lower_pwm_constrain, higher_pwm_constrain);
                // right_pwm = constrain((absolute_pitch_correction), lower_pwm_constrain, higher_pwm_constrain);
            }
            print_info();
        }

    }
    
}

