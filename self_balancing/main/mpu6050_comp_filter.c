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

#define PKp 8
#define PKi 0
#define PKd 1

int counter = 0;
uint32_t timer = 0;
float dt = 0;
float complimentary_angle[2] = {0, 0};
float acce_angle[2];

float setpoint = 180; 
float pitchAngle = 0, pitchError, prevPitchError, pitchDifference, pitchCumulativeError, pitchCorrection; 
float pitchKp = PKp;
float pitchKi = PKi;
float pitchKd = PKd;

float lower_pwm_constrain = 60;
float higher_pwm_constrain = 70;
float left_pwm = 0, right_pwm = 0;

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
static void bot_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle1, float duty_cycle2)
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

static void bot_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle1, float duty_cycle2)
{
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

int map(uint32_t x, int min_in, int max_in, int min_out, int max_out)
{
    return ((int)x - min_in) * (max_out - min_out) / (max_in - min_in) + min_out;
}

float constrain(float x, float lower_limit, float higher_limit)
{
    if(x < lower_limit)
        x = lower_limit;
    
    else if(x > higher_limit)
        x = higher_limit;

    return x;
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

void mpu6050_complimentary_filter(void* arg)
{
    int ret;
    uint8_t* acce_rd = (uint8_t*) malloc(BUFF_SIZE);
    uint8_t* gyro_rd = (uint8_t*) malloc(BUFF_SIZE);
    int16_t* acce_raw_value = (int16_t*) malloc(BUFF_SIZE/2);
    int16_t* gyro_raw_value = (int16_t*) malloc(BUFF_SIZE/2);
    float complimentary_angle[2] = {0, 0};
    ret = mpu6050_init(I2C_MASTER_NUM);
    while(ret != ESP_OK) {
        printf("INIT FAILED... Retry\n");
        vTaskDelay(100/ portTICK_RATE_MS);
        ret = mpu6050_init(I2C_MASTER_NUM);
    }
    printf("INIT SUCESS...\n");
    vTaskDelay(100/ portTICK_RATE_MS);
    mcpwm_gpio_initialize();
    mcpwm_initialize();
    while (1) {
        /*Read raw gyro values*/
        ret = mpu6050_read_gyro(I2C_MASTER_NUM, gyro_rd, BUFF_SIZE);
        shift_buf(gyro_rd, gyro_raw_value, BUFF_SIZE/2);


        /*Read raw acce values*/
        ret = mpu6050_read_acce(I2C_MASTER_NUM, acce_rd, BUFF_SIZE);
        shift_buf(acce_rd, acce_raw_value, BUFF_SIZE/2);

        complimentory_filter(acce_raw_value, gyro_raw_value, complimentary_angle, BUFF_SIZE/2);
        // printf("Angles by complimentary filter...\n");
        // printf("ROLL: %f\n", complimentary_angle[0]);
        // printf("PITCH: %f\n", acce_angle[1]);

        pitchAngle = acce_angle[1];

        pitchError = setpoint-abs(pitchAngle);                                                
        pitchDifference = pitchError - prevPitchError;
        pitchCumulativeError += pitchError;
        if(pitchCumulativeError>40)
            pitchCumulativeError = 40;
        else if(pitchCumulativeError<-40)
            pitchCumulativeError = -40;
        pitchCorrection = pitchKp * pitchError + pitchKi * pitchCumulativeError - pitchKd * pitchDifference;
        prevPitchError = pitchError;

        // printf("%f\n", pitchCorrection);

        double absolutePitchCorrection = abs(pitchCorrection);
      

        
        // printf("absolutePitchCorrection %f\n",absolutePitchCorrection);


        left_pwm = constrain(absolutePitchCorrection, lower_pwm_constrain, higher_pwm_constrain);
        right_pwm = constrain(absolutePitchCorrection, lower_pwm_constrain, higher_pwm_constrain);


        if (pitchAngle<178 && pitchAngle>0)
            bot_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
        else if (pitchAngle>(-178) && pitchAngle < 0)
            bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
        else
        { 
            brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
            printf("%s\n","perfect");
        }
        // printf("left_pwm%f\n", left_pwm);
        // printf("right_pwm%f\n", right_pwm);

        // bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm)

    }
}

// void calculatePitchError()
// {
//   pitchError = pitchAngle;                                                
//   pitchDifference = pitchError - prevPitchError;
//   pitchCumulativeError += pitchError;
//   if(pitchCumulativeError>40)
//     pitchCumulativeError = 40;
//   else if(pitchCumulativeError<-40)
//     pitchCumulativeError = -40;
//   pitchCorrection = pitchKp * pitchError + pitchKi * pitchCumulativeError + pitchKd * pitchDifference;
//   prevPitchError = pitchError;
// }

// void self_balance()
// {
//     while(1)
//     {
        

//         printf("%f\n",pitchAngle);



//         printf("PitchCorrection %f\n",pitchCorrection);

//         double absolutePitchCorrection = abs(pitchCorrection);
      
//         if (pitchAngle > 0.15)
//             bot_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
//         else if (pitchAngle < -0.15)
//             bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
//         else brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        
//         printf("absolutePitchCorrection %f\n",absolutePitchCorrection);


//         left_pwm = constrain(absolutePitchCorrection, lower_pwm_constrain, higher_pwm_constrain);
//         right_pwm = constrain(absolutePitchCorrection, lower_pwm_constrain, higher_pwm_constrain);

//         // printf("left_pwm %f\n",left_pwm);
//         // printf("right_pwm %f\n",right_pwm);
//         vTaskDelay(100);
//     }      
// }

void app_main()
{

    i2c_master_init();

    xTaskCreate(mpu6050_complimentary_filter, "i2c_mpu6050_complimentary_filter", 4096 * 2, NULL, 1, NULL);
    // xTaskCreatePinnedToCore(self_balance, "self_balance", 2048 * 2, NULL, 2, NULL,1);
}

