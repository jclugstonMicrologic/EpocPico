//#include <mraa.h>
//#include <mraa/gpio.h>
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

#include <gpio.h>

#define RED_LED_PORT		9  //imx7 E1_pin35 GPIO1_IO09 (real board will be gp12/mraa20/pwm0)
#define GREEN_LED_PORT		8  //imx7 E1_pin33 GPIO1_IO08 (real board will be gp13/mraa14/pwm1)
#define BUZZER_PORT	    	10 //imx7 E1_pin37 GPIO1_IO10 (real board will be gp182/mraa0/pwm2)

#define GPS_ON_PORT	    	33 //imx7 E1_pin26 GPIO2_IO01 (real board will be gp45/mraa45)
#define CAMERA_TRIGGER_PORT	34 //imx7 E1_pin28 GPIO2_IO02 (real board will be gp46/mraa32)

#define SW_TRIGGER_PORT		36 //imx7 E1_pin32 GPIO2_IO04 (real board will be gp48/mraa33)

#define GPS_VALID_PORT	    37 //imx7 E1_pin25 GPIO2_IO06 (real board will be gp165/mraa15)
#define EVENT_TRIGGER_PORT  35 //imx7 E1_pin30 GPIO2_IO03 (real board will be gp47/mraa46)

#define PPS_PORT 			32 //imx7 E1_pin24 GPIO2_IO00 (real board will be gp44/mraa31)

#define POWER_SW_PORT		39 //imx7 E1_pin48 GPIO2_IO07 (real board will be gp14/mraa36)
#define EJECT_PORT			44 //imx7 E1_pin42 GPIO2_IO12 (real board will be gp15/mraa48)

#define LO_BAT_PORT			41 //imx7 E1_pin54 ??? (gp41/mraa51)

#define SPARK_FUN_TEST_PORT	31

#define uint8_t u_int8_t
#define int8_t  int8_t
#define uint16_t u_int16_t
#define int16_t  int16_t
#define uint32_t u_int32_t
#define int32_t  int32_t

//#define GPIO_IN  0
//#define GPIO_OUT 1
//mraa_gpio_context RedLedPin;
//mraa_gpio_context GreenLedPin;
//mraa_gpio_context GpsValidPin;
//mraa_gpio_context EventTriggerPin;

//mraa_gpio_context GpsOnPin;
//mraa_gpio_context BuzzerPin; 
//mraa_gpio_context VccOnPin;
//mraa_gpio_context CameraTriggerPin;
//mraa_gpio_context SwTriggerPin;
//mraa_gpio_context PowerSwTriggerPin;
//mraa_gpio_context PpsTriggerPin;
//mraa_gpio_context EjectTriggerPin;
//mraa_gpio_context LoBatPin;

//mraa_pwm_context RedLedPin;
//mraa_pwm_context GreenLedPin;

#ifdef AC_BUZZER	
mraa_pwm_context BuzzerPin;   /* the PWM pin context */
#else
//mraa_gpio_context BuzzerPin; 
#endif

//mraa_i2c_context i2c;
//mraa_gpio_context SparkFunTestPin;
	
int PwrSwitchAssert;
int EventTriggerCount;
int CameraTriggerCount;

int GpsWeek;
int GpsTime;

struct timespec synchTime;

double PpsTriggerTime =0;
double EventTime;
double CameraTime;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}IMU_DATA;

typedef struct
{
	float heading;
	float pitch;
	float roll;
}SPATIAL_DATA;

IMU_DATA ImuAccel;
IMU_DATA ImuGyro;
IMU_DATA ImuMag;

SPATIAL_DATA SpatialData;

uint16_t ReadI2CAccel(uint8_t coord);
uint16_t ReadI2CGyro(uint8_t coord);
uint16_t ReadI2CMag(uint8_t coord);

void ImuTest(void);
float ReadI2CQuaternion(uint8_t coord);
uint8_t ReadI2CUARTReg( uint8_t reg );

/*
*|----------------------------------------------------------------------------
*|  Routine:PpsIsrCallback
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void PpsIsrCallback()
{	
	clock_gettime(CLOCK_MONOTONIC, &synchTime);
	PpsTriggerTime =(double)(synchTime.tv_sec + (double)synchTime.tv_nsec/1000000000);	
	
	if( ++GpsTime >=604800)
	{
		/* week has expired	*/ 
		GpsTime =0;
		GpsWeek ++;
	}
	
	//printf("PPS isr %6.8lf\n", PpsTriggerTime);	
}

/*
*|----------------------------------------------------------------------------
*|  Routine:EventTriggerIsrCallback
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void EventTriggerIsrCallback()
{
	EventTriggerCount ++;
	
	clock_gettime(CLOCK_MONOTONIC, &synchTime);
	
	if( PpsTriggerTime ==0 )
		PpsTriggerTime = (double)(synchTime.tv_sec + (double)synchTime.tv_nsec/1000000000);	;
	
	EventTime =(double)(GpsTime + ( (double)(synchTime.tv_sec + (double)synchTime.tv_nsec/1000000000) -PpsTriggerTime ));

#ifdef MPU_9250
	ImuAccel.x =ReadI2CGyro(0);
	ImuAccel.y =ReadI2CGyro(1);
	ImuAccel.z =ReadI2CGyro(2);
	
	ImuGyro.x =ReadI2CGyro(0);
	ImuGyro.y =ReadI2CGyro(1);
	ImuGyro.z =ReadI2CGyro(2);

	ImuMag.x =ReadI2CGyro(0);
	ImuMag.y =ReadI2CGyro(1);
	ImuMag.z =ReadI2CGyro(2);
#endif

#ifdef EM7180	
	SpatialData.heading=ReadI2CQuaternion(0);
	SpatialData.pitch =ReadI2CQuaternion(1);
	SpatialData.roll =ReadI2CQuaternion(2);
	
	/* read twice */
	SpatialData.heading=ReadI2CQuaternion(0);
	SpatialData.pitch =ReadI2CQuaternion(1);
	SpatialData.roll =ReadI2CQuaternion(2);	
#endif

#ifdef VN100	
	//SpatialData.heading=ReadI2CUARTReg(3);
	//SpatialData.pitch =ReadI2CUARTReg(3);
	//SpatialData.roll =ReadI2CUARTReg(3);	
#endif
	
	//printf("ev trigger isr %f\n", EventTime);	
} 

/*
*|----------------------------------------------------------------------------
*|  Routine:CameraTriggerIsrCallback
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void CameraTriggerIsrCallback()
{
	CameraTriggerCount ++;
	
	clock_gettime(CLOCK_MONOTONIC, &synchTime);
	
	if( PpsTriggerTime ==0 )
		PpsTriggerTime = (double)(synchTime.tv_sec + (double)synchTime.tv_nsec/1000000000);	;
	
	CameraTime =(double)(GpsTime + ( (double)(synchTime.tv_sec + (double)synchTime.tv_nsec/1000000000) -PpsTriggerTime ));
	
	//printf("tr trigger isr %f\n", CameraTime);	
} 
 
/*
*|----------------------------------------------------------------------------
*|  Routine:GetEventTime
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void GetEventTime(double *pValue)
{
	*pValue =EventTime;
}	

/*
*|----------------------------------------------------------------------------
*|  Routine:GetCameraTriggerTime
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void GetCameraTriggerTime(double *pValue)
{
	*pValue =CameraTime;
}	


/*
*|----------------------------------------------------------------------------
*|  Routine:GetPpsTriggerTime
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void GetPpsTriggerTime(double *pValue)
{
	*pValue =PpsTriggerTime;
}	

/*
*|----------------------------------------------------------------------------
*|  Routine:SetGpsTime
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void SetGpsTime(int gpsW, int gpsT)
{
	GpsWeek =gpsW;
	GpsTime =gpsT;
}

/*****************************************************/

/*
*|----------------------------------------------------------------------------
*|  Routine:PwrSwIsrCallback
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void PwrSwIsrCallback()
{
	PwrSwitchAssert =1;
	printf("pwr sw isr\n");	
} 

//#define GPSON_INVERTED

/*
*|----------------------------------------------------------------------------
*|  Routine: InitGpio
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void InitGpio()
{	
	//mraa_init();
	
	//int gpio_pin;
	
	/********* outputs **************/		
	//GpsOnPin = mraa_gpio_init(GPS_ON_PORT);
	//mraa_gpio_dir(GpsOnPin, MRAA_GPIO_OUT);

	gpio_export(GPS_ON_PORT);
	gpio_direction(GPS_ON_PORT,GPIO_OUT);
	
	
#ifdef GPSON_INVERTED	
	/* set output high */
	//mraa_gpio_write(GpsOnPin, 1);
	gpio_write(GPS_ON_PORT, 1);
#else
	/* set output low */
	//mraa_gpio_write(GpsOnPin, 0);
	gpio_write(GPS_ON_PORT, 0);	
#endif	
		
	gpio_export(SW_TRIGGER_PORT);
	gpio_direction(SW_TRIGGER_PORT,GPIO_OUT);		
	/* set output low */	
	gpio_write(SW_TRIGGER_PORT, 0);
	
	//SwTriggerPin = mraa_gpio_init(SW_TRIGGER_PORT);
	//mraa_gpio_dir(SwTriggerPin, MRAA_GPIO_OUT);				
	/* set output low */	
	//mraa_gpio_write(SwTriggerPin, 0);
	
#ifndef AC_BUZZER	
	//BuzzerPin = mraa_gpio_init(BUZZER_PORT);
	//mraa_gpio_dir(BuzzerPin, MRAA_GPIO_OUT);				
		
	gpio_export(BUZZER_PORT);
	gpio_direction(BUZZER_PORT,GPIO_OUT);			
#endif	

	/********* inputs **************/
	//GpsValidPin = mraa_gpio_init(GPS_VALID_PORT);
	//mraa_gpio_dir(GpsValidPin, MRAA_GPIO_IN);			
	gpio_export(GPS_VALID_PORT);
	gpio_direction(GPS_VALID_PORT,GPIO_IN);

	//EventTriggerPin = mraa_gpio_init(EVENT_TRIGGER_PORT);
	//mraa_gpio_dir(EventTriggerPin, MRAA_GPIO_IN);
	//mraa_gpio_isr(EventTriggerPin, MRAA_GPIO_EDGE_RISING, &EventTriggerIsrCallback, NULL);	
	gpio_export(EVENT_TRIGGER_PORT);
	gpio_direction(EVENT_TRIGGER_PORT,GPIO_IN);
	
	//CameraTriggerPin = mraa_gpio_init(CAMERA_TRIGGER_PORT);
	//mraa_gpio_dir(CameraTriggerPin, MRAA_GPIO_IN);
	//mraa_gpio_isr(CameraTriggerPin, MRAA_GPIO_EDGE_RISING, &CameraTriggerIsrCallback, NULL); // switch to rising edge Aug, 2018	
	gpio_export(CAMERA_TRIGGER_PORT);
	gpio_direction(CAMERA_TRIGGER_PORT,GPIO_IN);

	//PpsTriggerPin = mraa_gpio_init(PPS_PORT);
	//mraa_gpio_dir(PpsTriggerPin, MRAA_GPIO_IN);
	//mraa_gpio_isr(PpsTriggerPin, MRAA_GPIO_EDGE_FALLING, &PpsIsrCallback, NULL);	
	gpio_export(PPS_PORT);
	gpio_direction(PPS_PORT,GPIO_IN);		
	
	
	//PowerSwTriggerPin = mraa_gpio_init(POWER_SW_PORT);	
	//mraa_gpio_dir(PowerSwTriggerPin, MRAA_GPIO_IN);		
	//mraa_gpio_mode(PowerSwTriggerPin, MRAA_GPIO_PULLUP);
	gpio_export(POWER_SW_PORT);
	gpio_direction(POWER_SW_PORT,GPIO_IN);			

	//EjectTriggerPin = mraa_gpio_init(EJECT_PORT);
	//mraa_gpio_dir(EjectTriggerPin, MRAA_GPIO_IN);	
	//mraa_gpio_mode(EjectTriggerPin, MRAA_GPIO_PULLUP);
	gpio_export(EJECT_PORT);
	gpio_direction(EJECT_PORT,GPIO_IN);			


	/********* remove *****************/
//	SparkFunTestPin = mraa_gpio_init(SPARK_FUN_TEST_PORT);
//	mraa_gpio_dir(SparkFunTestPin, MRAA_GPIO_OUT);				
	
	PwrSwitchAssert =0;	
}	

/*
*|----------------------------------------------------------------------------
*|  Routine:GpsOn
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void GpsOn(void)
{
#ifdef GPSON_INVERTED
	//mraa_gpio_write(GpsOnPin, 0);
	gpio_write(GPS_ON_PORT, 0);
#else
	printf("\n!!!GPS on not inverted, ON ==1\n\n");
	//mraa_gpio_write(GpsOnPin, 1);
	gpio_write(GPS_ON_PORT, 1);
#endif
}

void GpsOff(void)
{
	
#ifdef GPSON_INVERTED
	//mraa_gpio_write(GpsOnPin, 1);
	gpio_write(GPS_ON_PORT, 1);
#else	
	printf("\n!!!GPS off not inverted, OFF ==0\n\n");
	//mraa_gpio_write(GpsOnPin, 0);		
	gpio_write(GPS_ON_PORT, 0);
#endif	
}

#if 0
/* May use this pin as trigger lock out for PIC */
/*
*|----------------------------------------------------------------------------
*|  Routine:LoBatOn
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void LoBatOn(void)
{
	mraa_gpio_write(LoBatPin, 1);
}

/*
*|----------------------------------------------------------------------------
*|  Routine:LoBatOff
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void LoBatOff(void)
{
	
	mraa_gpio_write(LoBatPin, 0);	
}
#endif

void VccOn(void)
{
//	mraa_gpio_write(VccOnPin, 1);	
}

void VccOff(void)
{
//	mraa_gpio_write(VccOnPin, 0);		
}

void SwTriggerPulse(void)
{
//	mraa_gpio_write(SwTriggerPin, 1);		
	gpio_write(SW_TRIGGER_PORT, 1);
	usleep(100000);
//	mraa_gpio_write(SwTriggerPin, 0);
	gpio_write(SW_TRIGGER_PORT, 0);
}

void SwTriggerOn(void)
{
	//mraa_gpio_write(SwTriggerPin, 1);		
	gpio_write(SW_TRIGGER_PORT, 1);
}

void SwTriggerOff(void)
{
    //mraa_gpio_write(SwTriggerPin, 0);
	gpio_write(SW_TRIGGER_PORT, 0);		
}

/*
*|----------------------------------------------------------------------------
*|  Routine:RedLedOn
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void RedLedOn(int solid)
{
#if 0
	if(solid)
	{
		mraa_pwm_pulsewidth_ms(RedLedPin, 200);
	}
	else		
	{
		mraa_pwm_pulsewidth_ms(RedLedPin, 100);
	}

	mraa_pwm_enable(RedLedPin, 1);
#endif
}

/*
*|----------------------------------------------------------------------------
*|  Routine:RedLedOff
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void RedLedOff(void)
{
#if 0
	mraa_pwm_pulsewidth_ms(RedLedPin, 0);			
	//mraa_pwm_pulsewidth_us(RedLedPin, 1);
	mraa_pwm_enable(RedLedPin, 1);
#endif
}


void GreenLedOn(int solid)
{
#if 0
	/* enable the PWM pulse on the pin */
	if(solid)
	{
		mraa_pwm_pulsewidth_ms(GreenLedPin, 200);	
	}
	else
	{		
		mraa_pwm_pulsewidth_ms(GreenLedPin, 100);	
	}
	
	mraa_pwm_enable(GreenLedPin, 1);
#endif
}

void GreenLedOff(void)
{
#if 0
	mraa_pwm_pulsewidth_ms(GreenLedPin, 0);	
	//mraa_pwm_pulsewidth_us(GreenLedPin, 1);	
	mraa_pwm_enable(GreenLedPin, 1);
#endif
}

void YellowLedOn(int solid)
{
#if 0
	if(solid)
	{
		mraa_pwm_pulsewidth_ms(RedLedPin, 200);
		mraa_pwm_pulsewidth_ms(GreenLedPin, 200);	
	}
	else
	{	
		mraa_pwm_pulsewidth_ms(RedLedPin, 100);
		mraa_pwm_pulsewidth_ms(GreenLedPin, 100);	
	}
	
	mraa_pwm_enable(RedLedPin, 1);
	mraa_pwm_enable(GreenLedPin, 1);
#endif
}

void YellowLedOff(void)
{
#if 0
	RedLedPin = mraa_gpio_init(RED_LED_PORT);
	mraa_gpio_dir(RedLedPin, MRAA_GPIO_OUT);
	mraa_gpio_write(RedLedPin, 0);
	
	GreenLedPin = mraa_gpio_init(GREEN_LED_PORT);
	mraa_gpio_dir(GreenLedPin, MRAA_GPIO_OUT);
	mraa_gpio_write(GreenLedPin, 0);
	
	usleep(50000);
	
	InitPwm();	
#else
 #if 0
	usleep(75000);
	mraa_pwm_pulsewidth_ms(RedLedPin, 0);
	usleep(75000);
	//mraa_pwm_pulsewidth_us(RedLedPin, 1);
	mraa_pwm_enable(RedLedPin, 1);
		
	usleep(75000);
	mraa_pwm_pulsewidth_ms(GreenLedPin, 0);	
	//mraa_pwm_pulsewidth_us(GreenLedPin, 1);
	usleep(75000);
	mraa_pwm_enable(GreenLedPin, 1);		
	
	usleep(75000);
	mraa_pwm_enable(RedLedPin, 0);
	usleep(75000);
	mraa_pwm_enable(GreenLedPin, 0);	
 #endif
#endif	
}

/*
*|----------------------------------------------------------------------------
*|  Routine:BuzzerOn
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void BuzzerOn(int level)
{
#ifdef AC_BUZZER
	/* set the pulse width (duty cycle) */
	if(level ==0)
		mraa_pwm_pulsewidth_us(BuzzerPin, 10);
	else
		mraa_pwm_pulsewidth_us(BuzzerPin, 200);
	
	/* enable the PWM pulse on the pin */
    mraa_pwm_enable(BuzzerPin, 1);	
#else	
	//mraa_gpio_write(BuzzerPin, 1);		
	gpio_write(BUZZER_PORT, 1);
#endif	
}

/*
*|----------------------------------------------------------------------------
*|  Routine:BuzzerOff
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void BuzzerOff(void)
{
#ifdef AC_BUZZER	
	/* set the pulse width (duty cycle) */
	//mraa_pwm_enable(BuzzerPin, 1);
	mraa_pwm_pulsewidth_us(BuzzerPin, 0);
	/* disable the PWM pulse on the pin */
	mraa_pwm_enable(BuzzerPin, 0);	    
#else
//	mraa_gpio_write(BuzzerPin, 0);					     
	gpio_write(BUZZER_PORT, 0);
#endif	
}

/*
*|----------------------------------------------------------------------------
*|  Routine:BuzzerChirp
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void BuzzerChirp(int nbrChirps)
{
	int j;
	
	for(j=0; j<nbrChirps; j++)
	{
		BuzzerOn(1);
		usleep(75000);
		BuzzerOff();
		usleep(75000);
	}	
}

/*
*|----------------------------------------------------------------------------
*|  Routine:GpsPosValid
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int GpsPosValid(void)
{
	//int value =mraa_gpio_read(GpsValidPin);	
	int value =gpio_read(GPS_VALID_PORT);
	return value;
}

int ReadPowerSw(void)
{
	//int value =mraa_gpio_read(PowerSwTriggerPin);	
	int value =gpio_read(POWER_SW_PORT);
	return value;
}

int EventTrigger(void)
{
	//int value =mraa_gpio_read(EventTriggerPin);	
	int value =gpio_read(EVENT_TRIGGER_PORT);
	return value;
}

int ReadEjectSw(void)
{
	//int value =mraa_gpio_read(EjectTriggerPin);
	int value =gpio_read(EJECT_PORT);		
	
	return value;
}

/*
*|----------------------------------------------------------------------------
*|  Routine:SetEventTriggerCount
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void SetEventTriggerCount(int value)
{
	EventTriggerCount =value;
}

/*
*|----------------------------------------------------------------------------
*|  Routine:GetEventTriggerCount
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int GetEventTriggerCount(void)
{
	return EventTriggerCount;
}

/*
*|----------------------------------------------------------------------------
*|  Routine:SetCameraTriggerCount
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void SetCameraTriggerCount(void)
{
	CameraTriggerCount =0;
}

/*
*|----------------------------------------------------------------------------
*|  Routine:GetCameraTriggerCount
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int GetCameraTriggerCount(void)
{
	return CameraTriggerCount;
}

/*
*|----------------------------------------------------------------------------
*|  Routine:EventTriggerTestOn
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int EventTriggerTestOn(void)
{
	//mraa_gpio_write(EventTriggerPin, 1);	
	gpio_write(EVENT_TRIGGER_PORT, 1);	
}

/*
*|----------------------------------------------------------------------------
*|  Routine:EventTriggerTestOff
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int EventTriggerTestOff(void)
{
	//mraa_gpio_write(EventTriggerPin, 0);
	gpio_write(EVENT_TRIGGER_PORT, 0);		
}


/*
*|----------------------------------------------------------------------------
*|  Routine: InitPwm
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void InitPwm(void)
{	
#if 0   
	/* intialize the mraa system */    
	mraa_init();
    
#ifdef AC_BUZZER	
    /* initialize buzzer PWM operation */
    BuzzerPin = mraa_pwm_init(BUZZER_PORT);
	
	/* set the period on the PWM pin */
    mraa_pwm_period_us(BuzzerPin, 350);      // Set the period as ms, Hz
    	
	/* set the pulse width (duty cycle) */
	mraa_pwm_pulsewidth_us(BuzzerPin, 200);
#endif
	
    /* initialize Red LED for PWM operation */
    RedLedPin = mraa_pwm_init(RED_LED_PORT);
	
	mraa_pwm_enable(RedLedPin, 0);
	
	/* set the period on the PWM pin */
    mraa_pwm_period_ms(RedLedPin, 200);      // Set the period as 100ms, 10Hz
    	
	/* set the pulse width (duty cycle) */
	mraa_pwm_pulsewidth_ms(RedLedPin, 100);		
	
    /* initialize Green LED for PWM operation */
    GreenLedPin = mraa_pwm_init(GREEN_LED_PORT);
	
	mraa_pwm_enable(GreenLedPin, 0);
	
	/* set the period on the PWM pin */
    mraa_pwm_period_ms(GreenLedPin, 200);      // Set the period as 100ms, 10Hz
    	
	/* set the pulse width (duty cycle) */
	mraa_pwm_pulsewidth_ms(GreenLedPin, 100);
	
	
    /* enable the PWM pulse on the pin */
    //mraa_pwm_enable(BuzzerPin, 1);	
	
    /* use the function 'mraa_pwm_write' to set the duty cycle */
	//duty =0.25;
    //mraa_pwm_write(BuzzerPin, duty);
#endif	
}	



void SparkFunTestOn(void)
{
	//mraa_gpio_write(SparkFunTestPin, 1);	
}	

void SparkFunTestOff(void)
{
	//mraa_gpio_write(SparkFunTestPin, 0);	
}	
	
	
		
	
#if 0
/************************* IMU I2C stuff ***********************************************/

/*
*|----------------------------------------------------------------------------
*|  Routine:SetupImu
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
uint8_t SetupImu()
{
	uint8_t id, hwversion, j;
	uint8_t result, status, sensor;
	uint16_t romversion, ramversion;
	uint8_t imuStatus =0;
	
	status =mraa_i2c_address(i2c, EM7180_ADDRESS); // or 0x69
	id = mraa_i2c_read_word_data(i2c,0x90);	 //117
	if( status !=MRAA_SUCCESS )
		printf("IMU Address Failure\n");
		
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
	hwversion = mraa_i2c_read_word_data(i2c,0x91);
	
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
	romversion =(mraa_i2c_read_word_data(i2c,0x70)<<8);
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
	romversion |= mraa_i2c_read_word_data(i2c,0x71);		
	
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
	ramversion =( mraa_i2c_read_word_data(i2c,0x72)<<8);
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
	ramversion |= mraa_i2c_read_word_data(i2c,0x73);			
	
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
	status = mraa_i2c_read_word_data(i2c,SENTRAL_STATUS_REGISTER);
		
	printf("EM7180\nProduct ID: 0x%x\nRevision ID: 0x%x\nRom Version: 0x%x\nRam Version: 0x%x\nSENtral Status: 0x%x\n\n",id, hwversion, romversion, ramversion, status);	
	
	if( id ==0x80 )	  
		imuStatus =1;
	
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x00, 0x34);
	
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x00, PASS_THRU_CONTROL_REGISTER);	
	
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x01, 0x34);

	mraa_i2c_address(i2c, EM7180_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x00, 0x34);

	mraa_i2c_address(i2c, EM7180_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x1e, MAG_RATE_REGISTER);
	
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x0a, ACCEL_RATE_REGISTER);
	
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x0f, GYRO_RATE_REGISTER);

	//mraa_i2c_address(i2c, EM7180_ADDRESS); 
    //mraa_i2c_write_byte_data(i2c, 0x01, 0x32);
	
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x00, ENABLE_EVENTS_REGISTER);

#ifndef PASS_THRU_ENABLE
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x00, ALG_CONTROL_REGISTER);
	
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x01, HOST_CONTROL_REGISTER);
#endif		

	mraa_i2c_address(i2c, EM7180_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x03, 0x5b);	

	mraa_i2c_address(i2c, EM7180_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x03, 0x5c);	
	
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x80, ALG_CONTROL_REGISTER);
	
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x04, ALG_CONTROL_REGISTER);
	
	status =0;
	int failCnt =0;
	while(status ==0 )
	{
		mraa_i2c_address(i2c, EM7180_ADDRESS); 
		status = mraa_i2c_read_word_data(i2c,MAG_ACTUAL_RATE_REGISTER);
		
		mraa_i2c_address(i2c, EM7180_ADDRESS); // or 0x69
		id = mraa_i2c_read_word_data(i2c,ACCEL_ACTUAL_RATE_REGISTER);	 //117
		
		mraa_i2c_address(i2c, EM7180_ADDRESS); 
		hwversion = mraa_i2c_read_word_data(i2c,GYRO_ACTUAL_RATE_REGISTER);
	
		printf("Actual Mag Rate: %d\nActual Accel Rate: %d\nActual Gyro Rate: %d\n\n",status, id, hwversion);
	
		usleep(1000000);	
		
		if( failCnt++ >5 )
		{
			printf("IMU sensor setup failure!!!\n");
			break;
		}
	}	
	
#ifdef PASS_THRU_ENABLE	
	/* setup EM7180 for pass thru mode */
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x00, ALG_CONTROL_REGISTER);	
	
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x01, PASS_THRU_CONTROL_REGISTER);	
		
	mraa_i2c_address(i2c, EM7180_ADDRESS); 
	hwversion = mraa_i2c_read_word_data(i2c,PASS_THRU_STATUS_REGISTER);		
	
	printf("pass thru: %d\n",hwversion);
#else
	
#endif	
	
	return imuStatus;

}


/*
*|----------------------------------------------------------------------------
*|  Routine:TransmitVCMMessage
*|  Description: velocity compensation measurement message for the VN100
*|  Retval:
*|----------------------------------------------------------------------------
*/
void TransmitVCMMessage( uint16_t  Vx )
{
	char txBuffer[32];
	int j;
	
	memset( txBuffer, 0x00, sizeof(txBuffer) );
	
	sprintf(txBuffer, "$VNWRG,50,%d,0,0*XX\r\n", Vx);	
	
	for(j=0; j<strlen(txBuffer); j++)
	{
		mraa_i2c_address(i2c,I2C_UART_ADDRESS); 
		mraa_i2c_write_byte_data(i2c, txBuffer[j], THR_REGISTER);
	}
	
	//strcpy(txBuffer, "$VNWRG,50,0,0,0*XX\r\n");
	printf("tx data buffer %s\n",txBuffer);	
}

/*
*|----------------------------------------------------------------------------
*|  Routine:GetI2CUARTRxData
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
uint8_t GetI2CUARTRxData( uint8_t *pRxData)
{
	uint8_t status =0, i;
	uint8_t nbrByteRx =0;
	
	mraa_i2c_address(i2c, I2C_UART_ADDRESS); 	
	mraa_i2c_write_byte(i2c,LSR_REGISTER);
	status = mraa_i2c_read_byte(i2c);
	
	while( (status &0x01) ==0x01 )
	{
		mraa_i2c_address(i2c, I2C_UART_ADDRESS); // or 0x69
		//mraa_i2c_write_byte(i2c,RHR_REGISTER); // and 68 Gyro_Xout
		//*pRxData++ =( mraa_i2c_read_byte(i2c));
		*pRxData++ =( mraa_i2c_read_byte_data(i2c,RHR_REGISTER));
	
		mraa_i2c_address(i2c, I2C_UART_ADDRESS); 	
		//mraa_i2c_write_byte(i2c,LSR_REGISTER); // and 68 Gyro_Xout
		//status = mraa_i2c_read_byte(i2c);
		status = mraa_i2c_read_byte_data(i2c, LSR_REGISTER);
		
		nbrByteRx ++;
		
		//printf("byte %x\n", *pRxData);
		
		if( (status &0x03) ==0x03 )
			printf("OVERRUN\n");
	
	}
	
	return nbrByteRx;
}

/*
*|----------------------------------------------------------------------------
*|  Routine:InitI2CUart
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
uint8_t InitI2CUart()
{
	uint8_t status =0;	
	uint8_t data;
	uint8_t rxData;
	uint8_t nbrBytes =0;
	int j;
	uint8_t rxDataBuf[64];
	
	/* disable interrupts */
	mraa_i2c_address(i2c,I2C_UART_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x00, IER_REGISTER);
	
	/* enable divisor latch, and set to 8,N,1 */
	mraa_i2c_address(i2c,I2C_UART_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x83, LCR_REGISTER);

	/* read back register */
	//mraa_i2c_address(i2c, I2C_UART_ADDRESS); 
	//data = mraa_i2c_read_word_data(i2c,LCR_REGISTER);	
	//printf("I2C LCR Register 0x%x\n", data);
		
	/* set baud rate */
	/* divisor =3686400/(16*BaudRate) so for 115200bps we get divisor =2 */
	
	/* write baudrae DLL register */
	mraa_i2c_address(i2c,I2C_UART_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x02, DLL_REGISTER);
	
	/* write baudrae DLH register */
	mraa_i2c_address(i2c,I2C_UART_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x00, DLH_REGISTER);

	/* read back register */
	//mraa_i2c_address(i2c, I2C_UART_ADDRESS); 
	//data = mraa_i2c_read_word_data(i2c,DLL_REGISTER);	
	//printf("I2C DLL Register 0x%x\n", data);

	/* read back register */
	//mraa_i2c_address(i2c, I2C_UART_ADDRESS); 
	//data = mraa_i2c_read_word_data(i2c,DLH_REGISTER);	
	//printf("I2C DLH Register 0x%x\n", data);
	
	/* enable tx, and rx FIFO */
   	mraa_i2c_address(i2c,I2C_UART_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x03, FCR_REGISTER);

	/* disable divisor latch, leave as 8,N,1 */
   	mraa_i2c_address(i2c,I2C_UART_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x03, LCR_REGISTER);
		
    /*  get the receive register status*/
	mraa_i2c_address(i2c, I2C_UART_ADDRESS); 
	data = mraa_i2c_read_byte_data(i2c,LSR_REGISTER);
   
    printf("I2C LSR status: 0x%x\n", data);
	
	//status =mraa_i2c_address(i2c, I2C_UART_ADDRESS); // 0x90
	//mraa_i2c_write_byte(i2c,j);  // 
	//data =(mraa_i2c_read_byte(i2c));	
	
	for(j=0; j<10; j++)
	{
		status =mraa_i2c_address(i2c, I2C_UART_ADDRESS); 
		data = mraa_i2c_read_byte_data(i2c,(j<<3));
		
		printf("I2C UART Register %d data 0x%x\n", j, data);
		
		usleep(100000);
	}
	
#if 0
	while(1)
	{
	mraa_i2c_address(i2c, I2C_UART_ADDRESS); 
	data =mraa_i2c_read_byte_data(i2c,LSR_REGISTER); 
	
	mraa_i2c_address(i2c, I2C_UART_ADDRESS); 
	rxData = mraa_i2c_read_byte_data(i2c,RHR_REGISTER);
	
	printf("LSR 0x%x Rx data 0x%x\n", data, rxData);
				
	mraa_i2c_address(i2c,I2C_UART_ADDRESS); 
    mraa_i2c_write_byte_data(i2c, 0x55, THR_REGISTER);
	
	usleep(10000);
	}
#else
	
	TransmitVCMMessage(0);

	memset( rxDataBuf, 0x00, sizeof(rxDataBuf));
	nbrBytes =GetI2CUARTRxData(rxDataBuf);
	
	if( nbrBytes >0 )
	{		
		printf("rx data buffer %d %s\n", nbrBytes, rxDataBuf);
	}
	else 
	{
		printf("No Vector Nav data available\n");
	}	
	
#endif
	

	return 1;
}

/*
*|----------------------------------------------------------------------------
*|  Routine:ReadI2CUARTReg
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
uint8_t ReadI2CUARTReg( uint8_t reg )
{
	uint8_t data;
	
	//mraa_i2c_address(i2c, I2C_UART_ADDRESS); 
	//rxData = mraa_i2c_read_byte_data(i2c,reg);	
	
	return data;
}

/*
*|----------------------------------------------------------------------------
*|  Routine:InitI2C
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
uint8_t InitI2C()
{
	uint8_t id, hwversion, j;
	uint8_t result, status, sensor;
	int imuTest =0;
	uint8_t imuStatus =0;	
	uint8_t data;
		
	i2c = mraa_i2c_init(I2C_PORT);
	result =mraa_i2c_frequency(i2c, MRAA_I2C_STD);//I2C_PORT); //400kHz  MRAA_I2C_FAST

	if( i2c ==NULL )
	{
		printf("\nWarning I2C port not initialized!!!\n");
		return 0;
	}	
	
#ifdef VN100 // vector nav
	imuStatus =InitI2CUart();
#else	
	imuStatus =SetupImu();
#endif
	
#ifdef PASS_THRU_ENABLE
	/* get IMU data now */
	mraa_i2c_address(i2c, IMU_ADDRESS); // or 0x69
	id = mraa_i2c_read_word_data(i2c,117);
	
	ImuAccel.x =ReadI2CAccel(0);
	ImuAccel.y =ReadI2CAccel(1);
	ImuAccel.z =ReadI2CAccel(2);	
	printf("IMU Accel data: %d\naX: %d\naY: %d\naZ: %d\n\n", id, ImuAccel.x, ImuAccel.y, ImuAccel.z);			
	
	ImuGyro.x =ReadI2CGyro(0);
	ImuGyro.y =ReadI2CGyro(1);
	ImuGyro.z =ReadI2CGyro(2);	
	printf("IMU Gyro data: %d\ngX: %d\ngY: %d\ngZ: %d\n\n", id, ImuGyro.x, ImuGyro.y, ImuGyro.z);			
	
	ImuMag.x =ReadI2CMag(0);
	ImuMag.y =ReadI2CMag(1);
	ImuMag.z =ReadI2CMag(2);
	printf("IMU Mag data: %d\nmX: %d\nmY: %d\nmZ: %d\n\n", id, ImuMag.x, ImuMag.y, ImuMag.z);
#endif

//#define TEST_IMU
#ifdef TEST_IMU
	//for(imuTest =0; imuTest<100; imuTest++)
	while(1)
	{
		ImuTest();
	}
#endif		
	
	return imuStatus;
}


/*
*|----------------------------------------------------------------------------
*|  Routine: ReadI2CQuaternion
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
float ReadI2CQuaternion(uint8_t coord)
{
	uint8_t data[4];
	float thedata =0;
	uint16_t imuReg =0;
	
	if( i2c ==NULL )
	{
		printf("\nWarning I2C port not initialized!!!\n");
		return 0;
	}
	
	if( coord ==0 )
	{
		imuReg=0;
	}
	else if( coord ==1 )
	{
		imuReg=4;
	}	
	else
	{
		imuReg=8;
	}	
		
	mraa_i2c_address(i2c, EM7180_ADDRESS); // or 0x69
	mraa_i2c_write_byte(i2c,imuReg); // and 68 Gyro_Xout
	data[0] =( mraa_i2c_read_byte(i2c));
			
	mraa_i2c_address(i2c, EM7180_ADDRESS); // or 0x69
	mraa_i2c_write_byte(i2c,(imuReg+1)); // and 68 Gyro_Xout
	data[1] = (mraa_i2c_read_byte(i2c));
	
	mraa_i2c_address(i2c, EM7180_ADDRESS); // or 0x69
	mraa_i2c_write_byte(i2c,(imuReg+2)); // and 68 Gyro_Xout
	data[2] = (mraa_i2c_read_byte(i2c));

	mraa_i2c_address(i2c, EM7180_ADDRESS); // or 0x69
	mraa_i2c_write_byte(i2c,(imuReg+3)); // and 68 Gyro_Xout
	data[3] = mraa_i2c_read_byte(i2c);
	
	thedata = *(float *)data;
	
	//printf("IMU Mag Pitch Array: %d %d %d %d\n\n", data[0],data[1],data[2],data[3]);
	return thedata;
}

/*
*|----------------------------------------------------------------------------
*|  Routine:ReadI2CAccel
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
uint16_t ReadI2CAccel(uint8_t coord)
{
	uint16_t data;
	uint16_t imuReg;
	
	switch(coord)
	{
		case 0:
			imuReg =59;
			break;
		case 1:
			imuReg =61;
			break;
		case 2:
			imuReg =63;
			break;
	}

	mraa_i2c_address(i2c, IMU_ADDRESS); // or 0x69
	mraa_i2c_write_byte(i2c,imuReg); // and 68 Gyro_Xout
	data =( mraa_i2c_read_byte(i2c) <<8);
			
	mraa_i2c_address(i2c, IMU_ADDRESS); // or 0x69
	mraa_i2c_write_byte(i2c,(imuReg+1)); // and 68 Gyro_Xout
	data |= mraa_i2c_read_byte(i2c);
	
	return data;
}

/*
*|----------------------------------------------------------------------------
*|  Routine:ReadI2CGyro
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
uint16_t ReadI2CGyro(uint8_t coord)
{
	uint16_t data;
	uint16_t imuReg;
	
	switch(coord)
	{
		case 0:
			imuReg =67;
			break;
		case 1:
			imuReg =69;
			break;
		case 2:
			imuReg =71;
			break;
	}

	mraa_i2c_address(i2c, IMU_ADDRESS); // or 0x69
	mraa_i2c_write_byte(i2c,imuReg); // and 68 Gyro_Xout
	data =( mraa_i2c_read_byte(i2c) <<8);
			
	mraa_i2c_address(i2c, IMU_ADDRESS); // or 0x69
	mraa_i2c_write_byte(i2c,(imuReg+1)); // and 68 Gyro_Xout
	data |= mraa_i2c_read_byte(i2c);
	
	return data;
}

/*
*|----------------------------------------------------------------------------
*|  Routine:ReadI2CMag
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
uint16_t ReadI2CMag(uint8_t coord)
{
	uint16_t data;
	uint16_t imuReg;
	
	switch(coord)
	{
		case 0:
			imuReg =3;
			break;
		case 1:
			imuReg =5;
			break;
		case 2:
			imuReg =7;
			break;
	}

	mraa_i2c_address(i2c, IMU_ADDRESS); // or 0x69
	mraa_i2c_write_byte(i2c,imuReg); // and 68 Gyro_Xout
	data =( mraa_i2c_read_byte(i2c));
			
	mraa_i2c_address(i2c, IMU_ADDRESS); // or 0x69
	mraa_i2c_write_byte(i2c,(imuReg+1)); // and 68 Gyro_Xout
	data |=( mraa_i2c_read_byte(i2c)<<8);
	
	return data;
}

/*
*|----------------------------------------------------------------------------
*|  Routine:GetImuAccel
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void GetImuAccel(IMU_DATA *pImuData)//(uint16_t *pX, uint16_t *pY, uint16_t *pZ)
{
	pImuData->x =ImuAccel.x;
	pImuData->y =ImuAccel.y;
	pImuData->z =ImuAccel.z;
}

/*
*|----------------------------------------------------------------------------
*|  Routine:GetImuGyro
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void GetImuGyro(IMU_DATA *pImuData) //uint16_t *pX, uint16_t *pY, uint16_t *pZ)
{
	pImuData->x =ImuGyro.x;
	pImuData->y =ImuGyro.y;
	pImuData->z =ImuGyro.z;
}

/*
*|----------------------------------------------------------------------------
*|  Routine:GetImuMag
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void GetImuMag(IMU_DATA *pImuData) //uint16_t *pX, uint16_t *pY, uint16_t *pZ)
{
	pImuData->x =ImuMag.x;
	pImuData->y =ImuMag.y;
	pImuData->z =ImuMag.z;
}

/*
*|----------------------------------------------------------------------------
*|  Routine:GetSpatialData
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void GetSpatialData(SPATIAL_DATA *pSpatialData)
{
	pSpatialData->heading =SpatialData.heading;
	pSpatialData->pitch =SpatialData.pitch;
	pSpatialData->roll =SpatialData.roll;
}

/*
*|----------------------------------------------------------------------------
*|  Routine:ImuTest
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void ImuTest(void)
{
	uint8_t status;	
#ifdef MPU_9250
	while(1)
	{
		usleep(500000);	
		ImuAccel.x =ReadI2CAccel(0);
		ImuAccel.y =ReadI2CAccel(1);
		ImuAccel.z =ReadI2CAccel(2);
		
		printf("IMU Accel data: %d %d %d\n", ImuAccel.x, ImuAccel.y, ImuAccel.z);		
		
		ImuGyro.x =ReadI2CGyro(0);
		ImuGyro.y =ReadI2CGyro(1);
		ImuGyro.z =ReadI2CGyro(2);
		
		printf("IMU Gyro data: %d %d %d\n", ImuGyro.x, ImuGyro.y, ImuGyro.z);

		ImuMag.x =ReadI2CMag(0);
		ImuMag.y =ReadI2CMag(1);
		ImuMag.z =ReadI2CMag(2);
		
		printf("IMU Mag data: %d %d %d\n\n", ImuMag.x, ImuMag.y, ImuMag.z);
	}	
#else	
//	while(1)
	{
		mraa_i2c_address(i2c, EM7180_ADDRESS); 
		status = mraa_i2c_read_word_data(i2c,EVENT_STATUS_REGISTER);

		if( (status &0x3c) ==0x3c )
		{
			SpatialData.heading=ReadI2CQuaternion(0);
			SpatialData.pitch =ReadI2CQuaternion(1);
			SpatialData.roll =ReadI2CQuaternion(2);
			
			printf("EM7180 Spatial Data H P R: %f %f %f %d\n", SpatialData.heading*57.2958,SpatialData.pitch*57.2958,SpatialData.roll*57.2958,status);
		}
		if( (status &0x03)  )
		{
			mraa_i2c_address(i2c, EM7180_ADDRESS); 
			mraa_i2c_write_byte_data(i2c, 0x01, 0x9b);
			
			printf("CPU Reset %d\n", status);

			usleep(1000000);			
			SetupImu();
		}
		
		usleep(500000);	
	
	}	
#endif	
}
#endif

	
	
