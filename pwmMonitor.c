#include <time.h>
#include <string.h>
#include <sys/statvfs.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "gpiolib.h"
#include <termios.h>

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include <unistd.h>

#include "gpio.h"

#define PWM_INPUT_PORT   	32 //imx7 E1_pin?? 

#define uint8_t u_int8_t
#define int8_t  int8_t
#define uint16_t u_int16_t
#define int16_t  int16_t
#define uint32_t u_int32_t
#define int32_t  int32_t

#define GPIO_IN  0
#define GPIO_OUT 1

struct timespec synchTime;

typedef struct
{
	u_int8_t machState;	
	
	double period;
	double riseEdgeTriggerTime;
	
}PWM_IN;

PWM_IN PwmIn;


void PwmIsr();

/*
*|----------------------------------------------------------------------------
*|  Routine:PwmMonitorInit
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void PwmMonitorInit()
{
	pthread_t pwmThread;
    int  iret1;
	
	memset(&PwmIn,0x00, sizeof(PwmIn) );
	
	gpio_export(PWM_INPUT_PORT);
	gpio_direction(PWM_INPUT_PORT,GPIO_IN);
	
    /* Create independent thread each of which will execute function */
	iret1 = pthread_create( &pwmThread, NULL, PwmIsr, (void*)NULL);
	
/* init a PWM out for testing */
system("cd /sys/class/pwm/pwmchip1/");	
system("echo 0 > export");

/* Select the period of the PWM signal. Value is in nanoseconds. */
system("echo 1000000 > pwm0/period");

/* Select the duty cycle. Value is in nanoseconds and must be less than the period. */
systme("echo 500000 > pwm0/duty_cycle");

/* Enable/disable the PWM signal, use 1 or 0 respectively: */
system("echo 1 > pwm0/enable");

}

/*
*|----------------------------------------------------------------------------
*|  Routine:PwmIsr
*|  Description: this is a threaded function
*|  Retval:
*|----------------------------------------------------------------------------
*/
void PwmIsr()
{		
	u_int8_t inputState;
	
	for(;;)
	{		
		inputState =gpio_read(PWM_INPUT_PORT);
		
		switch(PwmIn.machState)
		{
			case 0:
				if(inputState==1)
				{
					clock_gettime(CLOCK_MONOTONIC, &synchTime);
					
					PwmIn.riseEdgeTriggerTime =(double)(synchTime.tv_sec + (double)synchTime.tv_nsec/1000000000);
					
					PwmIn.machState=1; 
					
					printf("PWM rise %6.8lf\n", PwmIn.riseEdgeTriggerTime);	
				}
				break;
			case 1:
				if(inputState==0)
				{
					/* falling edge */
					PwmIn.machState =2;
				}
				break;
			case 2:
				if(inputState==1)
				{				
					PwmIn.period =clock_gettime(CLOCK_MONOTONIC, &synchTime) -PwmIn.riseEdgeTriggerTime;
				
					PwmIn.machState=0; 
					
					printf("PWM period %6.8lf\n", PwmIn.period);	
				}
				break;
		}	
	}
}

	
	
