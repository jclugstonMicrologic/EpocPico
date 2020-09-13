#include <mraa.h>
#include <mraa/gpio.h>
#include <string.h>

#define I2C_PORT	6

#define IMU_ADDRESS (0x68) 

/* VN100 */
#define I2C_UART_ADDRESS  (0x48) //(0x90>>1) 
/* I2C uart registers need to be shifted by 3*/
#define RHR_REGISTER	  (0x00<<3) 
#define THR_REGISTER	  (0x00<<3) 
#define IER_REGISTER	  (0x01<<3) 
#define FCR_REGISTER 	  (0x02<<3) 
#define LCR_REGISTER 	  (0x03<<3) 
#define MCR_REGISTER 	  (0x04<<3) 
#define LSR_REGISTER 	  (0x05<<3) 

/* special registers */
#define DLL_REGISTER	  (0x00<<3) 
#define DLH_REGISTER	  (0x01<<3) 
/* end I2C UART */

#define EM7180_ADDRESS 		  0x28
#define HOST_CONTROL_REGISTER 0x34
#define ALG_CONTROL_REGISTER  0x54

#define MAG_RATE_REGISTER 	  0x55
#define ACCEL_RATE_REGISTER	  0x56
#define GYRO_RATE_REGISTER 	  0x57

#define MAG_ACTUAL_RATE_REGISTER   0x45
#define ACCEL_ACTUAL_RATE_REGISTER 0x46
#define GYRO_ACTUAL_RATE_REGISTER  0x47

#define QRATE_DIVISOR_REGISTER  0x32
#define ENABLE_EVENTS_REGISTER  0x33
#define EVENT_STATUS_REGISTER   0x35

#define SENTRAL_STATUS_REGISTER 0x37


#define PASS_THRU_CONTROL_REGISTER 0xa0
#define PASS_THRU_STATUS_REGISTER  0x9e

//#define PASS_THRU_ENABLE
//#define MPU_9250
//#define EM7180
#define VN100 // vector nav

mraa_i2c_context i2c;

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

void TransmitVCMMessage( double Vx );

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
void TransmitVCMMessage( double Vx )
{
	char txBuffer[64];
	int j;
	
	memset( txBuffer, 0x00, sizeof(txBuffer) );
		
	sprintf(txBuffer, "$VNWRG,50,%3.6f,0,0*XX\r\n", Vx);	
	
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
	
	TransmitVCMMessage(0.0);

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