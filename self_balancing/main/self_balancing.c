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

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include <stdlib.h>

#include "MPU.h"
#include "SRA18.h"
#include "TUNING.h"

adc1_channel_t channel[4] = {ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_0, ADC_CHANNEL_3};

float yaw_kP= 3;
float yaw_kI= 0;
float yaw_kD= 0.5;

float pitchKp=  5.85;       
float pitchKi=  0.095;          
float pitchKd=  2.8;

int MAX_PITCH_CORRECTION =100;
int MAX_INTEGRAL_ERROR= 90;

int MAX_PWM = 100; 
int MIN_PWM = 60;

float setpoint = 8;
float initial_acce_angle = 8;
float forward_angle = -4.5; 

int is_forward = 1;

bool balanced = false;

float absolute_pitch_correction = 0,absolute_pitch_angle = 0,pitch_angle = 0, pitch_error, prevpitch_error, pitchDifference, pitchCumulativeError, pitch_correction,integral_term;

float left_pwm = 0, right_pwm = 0;

float complimentary_angle[2] = {0,0};

void calculate_pitch_error()
{
    pitch_error = pitch_angle; 
    pitchDifference = (pitch_error - prevpitch_error);
    pitchCumulativeError += pitch_error;

    //RESET CUMULATIVE ERROR EVERYTIME BOT CROSSES THE ORIGIN
    if(pitch_error*prevpitch_error < 0)
    {
        pitchCumulativeError = 0;
    }  

    integral_term = pitchCumulativeError*pitchKi;   

    if(integral_term>MAX_INTEGRAL_ERROR)
        integral_term = MAX_INTEGRAL_ERROR;
    else if(integral_term<-MAX_INTEGRAL_ERROR)
        integral_term= -MAX_INTEGRAL_ERROR;
    
    pitch_correction = pitchKp * pitch_error + integral_term + pitchKd * pitchDifference;
    prevpitch_error = pitch_error;

    absolute_pitch_correction = absolute(pitch_correction);
    absolute_pitch_correction = constrain(absolute_pitch_correction,0,MAX_PITCH_CORRECTION);
}

void print_info()
{        
    printf("PITCH ANGLE:%f\t",pitch_angle);
    printf("PITCH ERROR%f\t",pitch_error);
    printf("PITCH CORRECTION %f\n",pitch_correction);
    printf("ABSOLUTE PITCH CORRECTION: %f\t",absolute_pitch_correction);
    printf("LEFT PWM: %f\t",left_pwm);
    printf("RIGHT PWM: %f\n",right_pwm);

    printf("\n");
}

void http_server(void *arg)
{
    printf("%s\n", "http task");
    struct netconn *conn, *newconn;
    err_t err;
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 80);
    netconn_listen(conn);
    do {
     err = netconn_accept(conn, &newconn);
     if (err == ERR_OK) {
       http_server_netconn_serve(newconn,&setpoint,&pitchKp,&pitchKd,&pitchKi,&yaw_kP,&yaw_kD,& yaw_kI);
       netconn_delete(newconn);
     }
    } while(err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
}

void balance_task(void *arg)
{
    uint8_t* acce_rd = (uint8_t*) malloc(BUFF_SIZE);
    uint8_t* gyro_rd = (uint8_t*) malloc(BUFF_SIZE);
    int16_t* acce_raw_value = (int16_t*) malloc(BUFF_SIZE/2);
    int16_t* gyro_raw_value = (int16_t*) malloc(BUFF_SIZE/2);

    i2c_master_init();
    start_mpu();
    mcpwm_initialize();
    
    vTaskDelay(100/ portTICK_RATE_MS);

    while (1) 
    {
      calculate_pitch_angle(acce_rd,gyro_rd,acce_raw_value,gyro_raw_value,initial_acce_angle,complimentary_angle);

      pitch_angle = complimentary_angle[1];

      calculate_pitch_error();

      if(!balanced)
      {
          left_pwm = constrain((absolute_pitch_correction), MIN_PWM, MAX_PWM);
          right_pwm = constrain((absolute_pitch_correction), MIN_PWM, MAX_PWM);

          if (pitch_error > 1)
          {
              bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
          }

          else if (pitch_error < -1)
          {                     
              bot_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
          }
          else
          { 
              bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
          }
      }
    }      
}

void app_main()
{
    
    nvs_flash_init();
    initialise_wifi();
    
    xTaskCreate(balance_task,"balance task",100000,NULL,1,NULL);
    xTaskCreate(&http_server, "http_server", 10000, NULL, 2, NULL);
}
