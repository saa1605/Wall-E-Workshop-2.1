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

#include "MPU.h"
#include "SRA18.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////
//ALL VARIABLES BEYOND THIS 


float yaw_kP= 2;
float yaw_kI= 0;
float yaw_kD= 0.75;

float pitchKp=  5.85;       
float pitchKi=  0.095;          
float pitchKd=  2.8;

int MAX_PITCH_CORRECTION =90;
int MAX_INTEGRAL_ERROR= 90;

int MAX_PWM = 100; 
int MIN_PWM = 60;


float setpoint = 6.5;
float initial_acce_angle = 6.5;
float forward_angle = 7.5; 
float MAX_PITCH_ERROR = -2 ;


////////////////////////////////////////////////////////////////////////////////////////////////////////// 


// FOR BALANCING
int is_forward = 1;
int forward_count = 0;

bool balanced = false;
float absolute_pitch_angle = 0;

float pitch_angle = 0, pitch_error, prevpitch_error, pitchDifference, pitchCumulativeError, pitch_correction,integral_term;


//FOR LINE FOLLOWING
float yaw_error, yaw_prev_error, yaw_difference, yaw_cumulative_error, yaw_correction;
int weights[4] = {-3,-1,1,3};

uint32_t adc_reading[4];
float sensor_value[4];

float left_pwm = 0, right_pwm = 0;

float absolute_pitch_correction = 0;


static void read_sensors()
{
  for(int i = 0; i < 4; i++)
    {
        adc_reading[i] = adc1_get_raw(channel[i]);
        // printf("%d\t",adc_reading[i]);

    }
    // printf("\n");

}

static void calc_sensor_values()
{
    for(int i = 0; i < 4; i++)
    {
        sensor_value[i] = map(adc_reading[i], 1700, 4000, 0, 1000);
        sensor_value[i] = constrain(sensor_value[i],0,1000);
        // printf("%f\t",sensor_value[i]);
    }

    // printf("\n");
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

        weighted_sum += (float)(sensor_value[i]) * (weights[i]);
        sum += sensor_value[i];
        
    }
    
    if(sum != 0)
    {
        pos = weighted_sum / sum;
    }

    if(all_black_flag == 1)
    {
        if(yaw_error > 0)
            pos = 3000;
        else
            pos = -3000;
    }

    yaw_error = pos;

}

static void calculate_yaw_correction()
{
    yaw_error *= 10;
    yaw_difference = (yaw_error - yaw_prev_error);
    yaw_cumulative_error += yaw_error;
    
    if(yaw_cumulative_error > 30)
    {
        yaw_cumulative_error = 30;
    }
    
    else if(yaw_cumulative_error < -30)
    {
        yaw_cumulative_error = -30;
    }
    
    yaw_correction = yaw_kP*yaw_error + yaw_kI*yaw_cumulative_error + yaw_kD*yaw_difference;
    yaw_prev_error = yaw_error;
}

void calculate_pitch_error()
{
    pitch_error = pitch_angle; 
    pitchDifference = (pitch_error - prevpitch_error);
    pitchCumulativeError += pitch_error;

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
    
    // printf("ROLL ANGLE: %f\n", complimentary_angle[0]);
    printf("PITCH ANGLE:%f\t",pitch_angle);
    // printf("PITCH ERROR%f\t",pitch_error);
    // printf("YAW ERROR: %f\t",yaw_error);
    // printf("YAW CORRECTION: %f\t",yaw_correction);  
    // printf("PITCH CORRECTION %f\n",pitch_correction);
    // printf("ABSOLUTE PITCH CORRECTION: %f\t",absolute_pitch_correction);
    // printf("YAW CORRECTION: %f\t", yaw_correction);
    // printf("LEFT PWM: %f\t",left_pwm);
    // printf("RIGHT PWM: %f\n",right_pwm);

    printf("\n");
}

void balance_task(void *arg)
{
    i2c_master_init();
    initial_acce_angle = setpoint;

    uint8_t* acce_rd = (uint8_t*) malloc(BUFF_SIZE);
    uint8_t* gyro_rd = (uint8_t*) malloc(BUFF_SIZE);
    int16_t* acce_raw_value = (int16_t*) malloc(BUFF_SIZE/2);
    int16_t* gyro_raw_value = (int16_t*) malloc(BUFF_SIZE/2);


    start_mpu();
    
    vTaskDelay(100/ portTICK_RATE_MS);
    
    //INIT PWM
    mcpwm_initialize();
    

    //SELF BALANCING AND LINE FOLLOWING
    while (1) 
    {
        calculate_pitch_angle(acce_rd,gyro_rd,acce_raw_value,gyro_raw_value,initial_acce_angle);

        //complimentary_angle[1] corresponds to the pitch angle
        pitch_angle = complimentary_angle[1];


        //Calulate PITCH and YAW error
        calculate_pitch_error();
        read_sensors();
        calc_sensor_values();
        calculate_yaw_error();
        calculate_yaw_correction();

        if(pitch_error>0)
        {
            is_forward = 1;
        }
        else
        {
            is_forward = -1;
        }

        left_pwm = constrain((absolute_pitch_correction +50 + is_forward*yaw_correction), MIN_PWM, MAX_PWM);
        right_pwm = constrain((absolute_pitch_correction +50 - is_forward*yaw_correction), MIN_PWM, MAX_PWM);


        if(!balanced)
        {
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
                initial_acce_angle = forward_angle;
                balanced = true;
            }
        }

        else
        {

            //CHECK DIRECTION OF BOT MOTION FOR ADDING OR SUBTRACTING YAW CORRECTION


            // SET DIRECTION OF BOT FOR BALANCING
            if (pitch_error > 0)
            {   
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
            }

            else if (pitch_error < 0)
            {
                bot_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);      
            }
            else
            { 
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
            }

            //FORWARD COUNT TO BRING THE BOT OUT OF BALANCED CONDITION
            if(pitch_error>1 || pitch_error < MAX_PITCH_ERROR)
            {
                initial_acce_angle = setpoint;
                balanced = false;

            }

        }
        print_info();
    }

       
}

void app_main()
{
    xTaskCreate(balance_task,"balance task",100000,NULL,1,NULL);
}
