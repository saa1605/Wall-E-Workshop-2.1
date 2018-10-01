//C Headers

#include <stdio.h>
#include <math.h>
#include <time.h>

//Components

#include "MPU.h"
#include "SRA18.h"
#include "TUNING.h"

//4Limiting Parameters
#define MAX_PITCH_CORRECTION 80
#define MAX_INTEGRAL_ERROR 75

#define MAX_PWM 90 
#define MIN_PWM 60

#define MAX_PITCH_ERROR -2.5 

//ADC Channels
adc1_channel_t channel[4] = {ADC_CHANNEL_3,ADC_CHANNEL_0,ADC_CHANNEL_6,ADC_CHANNEL_7};

//Line Following Tuning Parameters
float yaw_kP= 2;
float yaw_kI= 0;
float yaw_kD= 1;

//Self Balancing Tuning Parameters
float pitch_kP=  5.85;       
float pitch_kI=  0.095;          
float pitch_kD=  3.8;

float setpoint = 6;
float initial_acce_angle = 6;
float forward_angle = 7.5; 

//FOR BALANCING
int is_forward = 1;
int forward_count = 0;

bool balanced = false;
float absolute_pitch_angle = 0;

float pitch_angle = 0,roll_angle = 0,absolute_pitch_correction = 0, pitch_error, prevpitch_error, pitchDifference, pitchCumulativeError, pitch_correction,integral_term;

//FOR LINE FOLLOWING
float yaw_error, yaw_prev_error, yaw_difference, yaw_cumulative_error, yaw_correction;
int weights[4] = {-3,-1,1,3};

uint32_t adc_reading[4];
float sensor_value[4];

float left_pwm = 0, right_pwm = 0;

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
            pos = 30;
        else
            pos = -30;
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

void print_info()
{        
    
    // printf("ROLL ANGLE: %f\n", complimentary_angle[0]);
    // printf("PITCH ANGLE:%f\t",pitch_angle);
    // printf("PITCH ERROR%f\t",pitch_error);
    // printf("YAW ERROR: %f\t",yaw_error);
    // printf("YAW CORRECTION: %f\t",yaw_correction);  
    // printf("PITCH CORRECTION %f\n",pitch_correction);
    // printf("ABSOLUTE PITCH CORRECTION: %f\t",absolute_pitch_correction);
    // printf("YAW CORRECTION: %f\t", yaw_correction);
    printf("LEFT PWM: %f\t",left_pwm);
    printf("RIGHT PWM: %f\n",right_pwm);

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

void balance_with_line_follow_task(void *arg)
{
    uint8_t* acce_rd = (uint8_t*) malloc(BUFF_SIZE);
    uint8_t* gyro_rd = (uint8_t*) malloc(BUFF_SIZE);
    int16_t* acce_raw_value = (int16_t*) malloc(BUFF_SIZE/2);
    int16_t* gyro_raw_value = (int16_t*) malloc(BUFF_SIZE/2);

    i2c_master_init();
    start_mpu();
    mcpwm_initialize();

    vTaskDelay(100/ portTICK_RATE_MS);
    

    //SELF BALANCING AND LINE FOLLOWING
    while (1) 
    {

        calculate_angle(acce_rd,gyro_rd,acce_raw_value,gyro_raw_value,initial_acce_angle,&roll_angle,&pitch_angle);

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

        
        if(!balanced)
        {
            // SET DIRECTION OF BOT FOR BALANCING
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

            left_pwm = constrain((absolute_pitch_correction),MIN_PWM,MAX_PWM);
            right_pwm = constrain((absolute_pitch_correction),MIN_PWM,MAX_PWM);

        }

        else
        {
            left_pwm = constrain((absolute_pitch_correction + is_forward*yaw_correction), MIN_PWM, MAX_PWM);
            right_pwm = constrain((absolute_pitch_correction - is_forward*yaw_correction), MIN_PWM, MAX_PWM);


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

            
            if(pitch_error>0.5 || pitch_error < MAX_PITCH_ERROR)
            {
                initial_acce_angle = setpoint;
                balanced = false;
            }

        }
    }

       
}

void app_main()
{
    nvs_flash_init();
    initialise_wifi();
    xTaskCreate(&balance_with_line_follow_task,"self_balancing with line_following",100000,NULL,1,NULL);
    xTaskCreate(&http_server,"server",10000,NULL,2,NULL);

}
