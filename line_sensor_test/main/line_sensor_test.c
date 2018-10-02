//C Headers

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>


//Components

#include "MPU.h"
#include "SRA18.h"
#include "TUNING.h"

//Declare the the channel array consisting of the ADC channel inputs

adc1_channel_t channel[4] = {ADC_CHANNEL_7,ADC_CHANNEL_6,ADC_CHANNEL_3,ADC_CHANNEL_0};

int adc_reading[4];
float sensor_values[4];

void blink_task(void *arg)
{

	while(1)
	{
		for(int i =0;i<4;i++)
		{
			adc_reading[i] = adc1_get_raw(channel[i]);
			sensor_values[i] = map(adc_reading[i],1700,4000,0,1000);
			sensor_values[i] = constrain(sensor_values[i],0,1000);
		}		

		for(int i=0;i<4;i++)
		{
			printf("SENSOR RAW: %d\t",adc_reading[i]);
		}
		printf("\n\n");
		for(int i=0;i<4;i++)
		{
			printf("SENSOR VALUE: %f\t",sensor_values[i]);
		}
	}
	
}

void app_main()
{
	/*
		Basic Function for task creation
	*/

    xTaskCreate(&blink_task,"blink task",4096,NULL,1,NULL);
}
