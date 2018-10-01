//C Headers

#include <stdio.h>
#include <math.h>
#include <time.h>

//Components

#include "MPU.h"
#include "SRA18.h"
#include "TUNING.h"

//Limiting Variables
#define MAX_PITCH_CORRECTION 80
#define MAX_INTEGRAL_ERROR 75

#define MAX_PWM 90 
#define MIN_PWM 60

//Array to store channels of ADC
adc1_channel_t channel[4] = {ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_0, ADC_CHANNEL_3};

//Line Following Tuning Parameters
float yaw_kP= 3;
float yaw_kI= 0;
float yaw_kD= 0.5;

//Self Balancing Tuning Parameters
float pitch_kP=  5.85;       
float pitch_kI=  0.095;          
float pitch_kD=  3.8;

//Configuration Variables
float setpoint = 6;
float initial_acce_angle = 6;
float forward_angle = -4.5; 
int is_forward = 1;
bool balanced = false;

//Error and correction values
float absolute_pitch_correction = 0,absolute_pitch_angle = 0,pitch_angle = 0,roll_angle = 0,pitch_error, prevpitch_error, pitchDifference, pitchCumulativeError, pitch_correction,integral_term;

float left_pwm = 0, right_pwm = 0;

//Pitch and Roll angles
float complimentary_angle[2] = {0,0};


/*
  Calculate the motor inputs according to angle of the MPU
*/
void calculate_pitch_error()
{
    pitch_error = pitch_angle; 
    pitchDifference = (pitch_error - prevpitch_error);
    pitchCumulativeError += pitch_error;

    integral_term = pitchCumulativeError*pitch_kI;   

    if(integral_term>MAX_INTEGRAL_ERROR)
        integral_term = MAX_INTEGRAL_ERROR;
    else if(integral_term<-MAX_INTEGRAL_ERROR)
        integral_term= -MAX_INTEGRAL_ERROR;
    
    pitch_correction = pitch_kP * pitch_error + integral_term + pitch_kD * pitchDifference;
    prevpitch_error = pitch_error;

    absolute_pitch_correction = absolute(pitch_correction);
    absolute_pitch_correction = constrain(absolute_pitch_correction,0,MAX_PITCH_CORRECTION);
}

//Debugging
void print_info()
{        
    printf("PITCH ANGLE:%f\t",pitch_angle);
    printf("setpoint%f",setpoint);
    printf("initial_acce_angle%f",initial_acce_angle);
    // printf("PITCH ERROR%f\t",pitch_error);
    // printf("PITCH CORRECTION %f\n",pitch_correction);
    // printf("ABSOLUTE PITCH CORRECTION: %f\t",absolute_pitch_correction);
    // printf("LEFT PWM: %f\t",left_pwm);
    // printf("RIGHT PWM: %f\n",right_pwm);

    printf("\n");
}

/*
  Create an HTTP server to tune variables wirelessly 
*/
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
       http_server_netconn_serve(newconn,&setpoint,&forward_angle,&pitch_kP,&pitch_kD,&pitch_kI,&yaw_kP,&yaw_kD,&yaw_kI);
       netconn_delete(newconn);
     }
    } while(err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
}

/*
  The main task to balance the robot
*/
void balance_task(void *arg)
{
    uint8_t* acce_rd = (uint8_t*) malloc(BUFF_SIZE);
    uint8_t* gyro_rd = (uint8_t*) malloc(BUFF_SIZE);
    int16_t* acce_raw_value = (int16_t*) malloc(BUFF_SIZE/2);
    int16_t* gyro_raw_value = (int16_t*) malloc(BUFF_SIZE/2);

    i2c_master_init();  //Initialise the I2C interface
    start_mpu();        //Intialise the MPU 
    mcpwm_initialize(); 
    
    vTaskDelay(100/ portTICK_RATE_MS);

    while (1) 
    {
      initial_acce_angle = setpoint;
      calculate_angle(acce_rd,gyro_rd,acce_raw_value,gyro_raw_value,initial_acce_angle,&roll_angle,&pitch_angle);  //Function to calculate pitch angle based on intial accelerometer angle
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
