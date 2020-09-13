#include <time.h>
#include <string.h>
#include <sys/statvfs.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "gpiolib.h"
#include "gpio.h"
#include "pwmMonitor.h"
#include <termios.h>

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <dirent.h>

#define TAGR_VERSION "TAGR PICO v0.00 05/11/2020\n"  

#define SWIFTNAV
//#define OEM719
#ifdef SWIFTNAV
 #define RAW_GPS_BAUD_RATE  460800 //230400 
#else
 #define RAW_GPS_BAUD_RATE  460800 //921600 
#endif

#define POWER_UP_TIME			5
#define PRE_FLIGHT_TIMEOUT		180 //120
#define LOSS_COMM_TIMEOUT		5
#define POWER_DOWN_ASSSERT_TIME	1 /* how long power down button must be held before accepting it */

#define AMOUNT_TO_COPY 		86400   /* how far back, 24hrs */
#define SELF_TEST_PHOTO_COUNT	3

#define GNNS_MSGS_COMPLETE		2 //7

#define MIN_DISK_SPACE			150000000 //(150MB of ???MB)
#define MANAGE_DISK_SPACE		100000000 //(100MB of ???MB), if we reach this limit start deleting oldest files

/* define these for normal operation */
#define USE_OTG_MSD
//#define TURN_OFF_WIFI
//#define USE_IMU	
//#define HISTORY_LOG

/* test defines */
#define AUTO_START
#define DISABLE_PV_DETECT
#define SIM_GPS_DATE_TIME
//#define SIM_RAW_DATA
#define SKIP_SELF_TEST
//#define FILE_WRITE_TEST
//#define TEST_IMU_OUTPUT
//#define LED_TEST_ENABLED
//#define INIT_GNSS_ENABLED
//#define TEST_VCM_MESSAGE
//#define GPS_DISABLED
#define REDUCED_PREFLIGHT_TIME
//#define FLIGHT_SIM_WITH_REPEAT

#define uint8_t u_int8_t
#define int8_t  int8_t

#define uint16_t u_int16_t
#define int16_t  int16_t

#define uint32_t u_int32_t
#define int32_t  int32_t

/** Local Constants and Types *************************************************/
enum
{
	LED_BLINK =0,
	LED_SOLID 
};

typedef enum
{
	SYSTEM_STATE_INIT =0,	
	SYSTEM_STATE_STARTUP,
	SYSTEM_STATE_IDLE,	
	SYSTEM_STATE_POWERED,
	SYSTEM_STATE_WAIT_GPS_TIME,
	SYSTEM_STATE_SELF_TEST,
//	SYSTEM_STATE_SELF_TEST_SETTLE,
	SYSTEM_STATE_SELF_TEST_FAIL,
	SYSTEM_STATE_SELF_TEST_PASS,
	SYSTEM_STATE_READY,
	SYSTEM_STATE_DEBOUNCE_POWER_DWN,
	SYSTEM_STATE_READY_GPS_LOST,
	SYSTEM_STATE_POWER_DOWN,
	SYSTEM_STATE_FINALIZE_DATA_TRANSFER,
	SYSTEM_STATE_COPY_FILES,
	SYSTEM_STATE_WAIT_USER_FAIL_ACK,
	SYSTEM_STATE_OFF,
	
	
	SYSTEM_STATE_SETUP_COMNAV_STATE1,
	SYSTEM_STATE_SETUP_COMNAV_STATE2,
	SYSTEM_STATE_SETUP_COMNAV_STATE3,
	SYSTEM_STATE_SETUP_COMNAV_STATE4,
	SYSTEM_STATE_SETUP_COMNAV_STATE5,		
	
	SYSTEM_STATE_LED_TEST_STATE1,
	SYSTEM_STATE_LED_TEST_STATE2,
	SYSTEM_STATE_LED_TEST_STATE3,
	SYSTEM_STATE_LED_TEST_STATE4,	
	
	SYSTEM_STATE_LAST
	
}SystemStateTypeEnum;

typedef struct
{
	unsigned int stateTimer;
	SystemStateTypeEnum machState;
}MAIN_SYSTEM;

typedef struct
{
	int year;
	int month;
	int day;
	int hour;
	int min;
	int sec;
}GPS_DATE_TIME;

MAIN_SYSTEM MainSystem;
GPS_DATE_TIME GpsDateTime;

/** Local Variable Declarations ***********************************************/
//int FdGpsValid=0;
FILE* fp;
char FileExt[] ="   ";

typedef struct
{
	int week;
	int time;	
}GPS_TOW;

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

GPS_TOW GpsTow;


char filename1[255];
char ImuFilename[255];

#ifdef HISTORY_LOG
char HistoryLogFilename[255];
#endif

char logFileName[128];
char fileCpyStr[255];

time_t GpsEpoch;	
unsigned int TotalFileSize =0;

unsigned long long GetOtgDiskSpace(void);
unsigned long GetFileSize(char *pFileName);

/*
*|----------------------------------------------------------------------------
*|  Routine: CheckFileExists
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int CheckFileExists(char *filename)
{
	FILE *pFile;	
	
	pFile =fopen(filename,"r");
	
    if( pFile !=0 )
	{		
		fclose(pFile);			
		return 1;
	}	
	
	else
		return 0;
}

/*
*|----------------------------------------------------------------------------
*|  Routine: ManageLogfiles
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int ManageLogfiles(void)
{
	DIR *d;
	char st[80];
	char oldest[80];
    struct dirent *dir;
	int nbrFiles =0;
	
	//system("ls /home/root/msd/ -lt");
	
	struct stat filestat;
	struct stat filestat2;
	time_t epoch;
	time_t epoch2;
	struct tm fileTime;    
	int cnt =0;
	unsigned long long DiskFreeSpace;
	
	int  memoryCheck =1;
//	int retval =0;	
//	system("mkdir /home/root/msd");
//	system("mkdir /media/usb");

	//system("umount /home/root/msd");
//	retval = system("mount -o offset=8192 /dev/mmcblk0p9 /home/root/msd");
//	printf("%d\n", retval);

	int fileValid =0;
	
	TotalFileSize =0;	
	
	d = opendir("/home/root/msd");
	
//	while(memoryCheck)
	{
		if (d)
		{		
			while ((dir = readdir(d)) != NULL)
			{
				//printf("%s %d\n", dir->d_name, nbrFiles);
		
				strcpy(st, "/home/root/msd/");	
				strcat(st, dir->d_name);
				
				stat(st,&filestat);
				
				cnt =0;
				int value[6];
				int multiplier =1;
				int digCnt =0;
			
				fileValid =0;
				
				while(1)
				{	
					//printf("%x\n", st[cnt]);		
					//if(st[cnt] =='u' )
					
					if(st[++cnt] == '_')
					{	
						//while(digCnt <6 )
						//{
						cnt++;					
						fileTime.tm_year  =0;					
						while(st[cnt] !='_' )
						{
							fileTime.tm_year =(fileTime.tm_year*multiplier) + (st[cnt]-0x30);
							
							value[digCnt] =(value[digCnt]*multiplier) + (st[cnt]-0x30);
							//printf("%x\n", st[cnt]);
							multiplier =10;
							
							cnt++;
						}
						
						/* sanity check */
						if( fileTime.tm_year<2017 || fileTime.tm_year >2100 )
						{
							fileValid =0;
							epoch2 =0;
							printf("filename error\n");
							break;
						}
						else
						{
							fileValid =1;
							nbrFiles ++;
						}
						
						digCnt ++;
						//}
					#if 1	
						cnt ++;
						fileTime.tm_mon =0;
						while(st[cnt] !='_' )
						{
							fileTime.tm_mon =(fileTime.tm_mon*multiplier) + (st[cnt]-0x30);
							
							//printf("%x\n", st[cnt]);
							multiplier =10;
							
							cnt++;
						}					
						
						cnt ++;
						fileTime.tm_mday =0;
						while(st[cnt] !='_' )
						{
							fileTime.tm_mday =(fileTime.tm_mday*multiplier) + (st[cnt]-0x30);
							
							//printf("%x\n", st[cnt]);
							multiplier =10;
							
							cnt++;
						}										

						cnt ++;
						fileTime.tm_hour =0;
						while(st[cnt] !='_' )
						{
							fileTime.tm_hour =(fileTime.tm_hour*multiplier) + (st[cnt]-0x30);
							
							//printf("%x\n", st[cnt]);
							multiplier =10;
							
							cnt++;
						}	
						
						cnt ++;
						fileTime.tm_min =0;
						while(st[cnt] !='_' )
						{
							fileTime.tm_min =(fileTime.tm_min*multiplier) + (st[cnt]-0x30);
							
							//printf("%x\n", st[cnt]);
							multiplier =10;
							
							cnt++;
						}						
						
						cnt ++;
						fileTime.tm_sec =0;
						while(st[cnt] !='.' )
						{
							fileTime.tm_sec =(fileTime.tm_sec*multiplier) + (st[cnt]-0x30);
							
							//printf("%x\n", st[cnt]);
							multiplier =10;
							
							cnt++;
						}					
						
						fileTime.tm_year -=1900;					
						fileTime.tm_mon -=1;					
					#else				
						fileTime.tm_year =value[0] -1900;					
						fileTime.tm_mon =value[1]-1;
						fileTime.tm_mday =value[2];
						fileTime.tm_hour =value[3];
						fileTime.tm_min =value[4];
						fileTime.tm_sec =value[5];
					#endif
						//printf("%d %d %d %d %d %d\n", fileTime.tm_year, fileTime.tm_mon, fileTime.tm_mday, fileTime.tm_hour,fileTime.tm_min,fileTime.tm_sec);
						
						epoch = mktime(&fileTime);		 
						printf("%d %s\n", epoch, dir->d_name);
				
						break;
					}	
					
					if( cnt >50 )
					{
						//printf("break %s %d\n", dir->d_name, cnt);
						cnt =0;
						break;
					}
				}

				//if( fileValid ==1 )
				{
					if( (GpsEpoch -epoch)<AMOUNT_TO_COPY &&  (GpsEpoch -epoch)>=0 )
					{					
						TotalFileSize +=GetFileSize(st);
						printf("Total file size %ld %s\n", TotalFileSize, st);
					}				
								
					if(nbrFiles ==1 && fileValid==1 )
					{
						epoch2 =epoch;
						
						strcpy(oldest, "/home/root/msd/");	
						strcat(oldest, dir->d_name);				
					}			
					double timeDiff =(epoch-epoch2);
					
					if( timeDiff<0 )// || timeDiff>1000000000 )
					{
						//filestat2.st_mtime =filestat.st_mtime;
						//printf("oldest file %s", ctime(&filestat.st_mtime));
						epoch2 =epoch;
						strcpy(oldest, "/home/root/msd/");	
						strcat(oldest, dir->d_name);
					}			
				}
			}		
						
			printf("\nthe oldest file is %s\n", oldest);
			
			DiskFreeSpace =GetOtgDiskSpace();	
			
			//printf("\n %ld %ld\n", DiskFreeSpace, MANAGE_DISK_SPACE);
			
			if( DiskFreeSpace<MANAGE_DISK_SPACE) /* 100MB */
			{
				int retval =remove(oldest);			
				
				if(retval ==0)
				{
					memoryCheck =0;
					printf("deleted file%s %d\n", oldest, retval);
				}
				else 
				{
					memoryCheck =1;
					printf("failed to delete file%s %d\n", oldest, retval);
				}
			}
			else
			{
				memoryCheck =1;
			}
			
			closedir(d);
		}	
	}
	
	return memoryCheck;
}

/*
*|----------------------------------------------------------------------------
*|  Routine: CreateLogFile
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int CreateLogFile(char *pFileExt)
{	
	time_t t = time(NULL);		
	struct tm *tm = localtime(&t);	
	int retval =0;
	struct tm gpsfileTime;
#ifdef FLIGHT_SIM_WITH_REPEAT	
	static int repeatCnt =0;
#endif

#ifdef USE_OTG_MSD	
	sprintf(filename1, "/home/root/msd/u_%d_%d_%d_%d_%d_%d.%s",GpsDateTime.year,GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec,pFileExt);		
	sprintf(logFileName, "u_%d_%d_%d_%d_%d_%d.%s",GpsDateTime.year,GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec,pFileExt);
#else
	// use the SD card
	sprintf(filename1, "/media/sdcard/%d_%d_%d_%d_%d_%d.log",GpsDateTime.year,GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec);	
#endif

#ifdef FLIGHT_SIM_WITH_REPEAT	       
	repeatCnt ++;
	sprintf(filename1, "/home/root/msd/REPEAT_TEST_%d.log", repeatCnt);	
#endif

	gpsfileTime.tm_year =GpsDateTime.year -1900;					
	gpsfileTime.tm_mon =GpsDateTime.month-1;
	gpsfileTime.tm_mday =GpsDateTime.day;
	gpsfileTime.tm_hour =GpsDateTime.hour;
	gpsfileTime.tm_min =GpsDateTime.min;
	gpsfileTime.tm_sec =GpsDateTime.sec;
				
	GpsEpoch = mktime(&gpsfileTime);		 
	printf("%d %s\n", GpsEpoch, filename1);		
	
#ifdef USE_OTG_MSD	
	system("mkdir /home/root/msd");
	system("mkdir /media/usb");

	//retval = system("mount -o offset=8192 /dev/mmcblk2 /home/root/msd");
	//retval = system("mount -o offset=8192 /dev/mmcblk2p2 /home/root/msd");
	retval = system("mount /dev/mmcblk2p4 /home/root/msd");
	printf("%d\n", retval);	

	while( ManageLogfiles() ==0 ){}
#else
	retval = system("mount /dev/mmcblk1p1 /media/sdcard");
	printf("%d\n", retval);
#endif	
			
#if 0			
	if( CheckFileExists(filename1) )
	{
		printf("\nFile exists, rename\n\n");		
		
		// rename to year +1, will make it obvious this has happened
		sprintf(filename1, "/home/root/msd/%d_%d_%d_%d_%d_%d.log",(tm->tm_year+1900),(tm->tm_mon+1),tm->tm_mday,tm->tm_hour,tm->tm_min,tm->tm_sec);	
	    //sprintf(filename1, "/home/root/msd/%d_%d_%d_%d_%d_%d.log",(GpsDateTime.year+1),GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec);			
	}
#endif
	
	fp = fopen(filename1,"w");
	
	if( fp ==0 )
	{
		printf("File create failed");
		
		return 0;
	}
	else
	{	
		retval =fprintf(fp, TAGR_VERSION);
	
		fclose(fp);	
		
		if( retval >0 )
		{
			printf("File Init\n");
			return 1;
		}
		else
		{			
			printf("Write Fail\n");
			return 0;
		}
	}		
}

/*
*|----------------------------------------------------------------------------
*|  Routine: CreateImuLogFile
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int CreateImuLogFile(uint8_t imuHwEnabled)
{
	if( !imuHwEnabled )
		return 1;

	time_t t = time(NULL);		
	struct tm *tm = localtime(&t);	
	int retval =0;

	sprintf(ImuFilename, "/home/root/msd/%d_%d_%d_%d_%d_%d.imu",GpsDateTime.year,GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec);	

	printf("%s\n", ImuFilename);
	
	system("mkdir /home/root/msd");

	retval = system("mount -o offset=8192 /dev/mmcblk0p9 /home/root/msd");
	printf("%d\n", retval);
	
	if( CheckFileExists(ImuFilename) )
	{
		printf("\nIMU File exists, rename\n\n");		
		
		// rename to year +1, will make it obvious this has happened
		sprintf(ImuFilename, "/home/root/msd/%d_%d_%d_%d_%d_%d.imu",(tm->tm_year+1900),(tm->tm_mon+1),tm->tm_mday,tm->tm_hour,tm->tm_min,tm->tm_sec);	
	    //sprintf(ImuFilename, "/home/root/msd/%d_%d_%d_%d_%d_%d.log",(GpsDateTime.year+1),GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec);			
	}
	
	fp = fopen(ImuFilename,"w");
	
	if( fp ==0 )
	{
		printf("IMU File create failed");
		
		return 0;
	}
	else
	{	
		retval =fprintf(fp, TAGR_VERSION);
	
		fclose(fp);	
		
		if( retval >0 )
		{
			printf("IMU File Init\n");
			return 1;
		}
		else
		{			
			printf("IMU Write Fail\n");
			return 0;
		}
	}			
}

/*
*|----------------------------------------------------------------------------
*|  Routine: CreateSystemLogFile
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int CreateSystemLogFile(void)
{
#ifdef HISTORY_LOG	
	time_t t = time(NULL);		
	struct tm *tm = localtime(&t);	
	int retval =0;

	//sprintf(HistoryLogFilename, "/home/root/msd/%d_%d_%d_%d_%d_%d.imu",GpsDateTime.year,GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec);	
	strcpy(HistoryLogFilename, "/home/root/msd/History.log");	

	printf("%s\n", HistoryLogFilename);
	
	system("mkdir /home/root/msd");

	retval = system("mount -o offset=8192 /dev/mmcblk0p9 /home/root/msd");
	printf("%d\n", retval);
	
	if( CheckFileExists(HistoryLogFilename) )
	{
		printf("\nSystem Log File exists\n\n");		
		
		// rename to year +1, will make it obvious this has happened
		//sprintf(HistoryLogFilename);	
	}
	
	fp = fopen(HistoryLogFilename,"a");
	
	if( fp ==0 )
	{
		printf("System log file create failed");
		
		return 0;
	}
	else
	{	
		retval =fprintf(fp, "\n\n************HISTORY LOG START *****************************\n");
		retval =fprintf(fp, TAGR_VERSION);	
	
		fclose(fp);	
		
		if( retval >0 )
		{
			printf("System log file Init\n");
			return 1;
		}
		else
		{			
			printf("System log file Write Fail\n");
			return 0;
		}
	}			
#endif	
}

/*
*|----------------------------------------------------------------------------
*|  Routine: WriteSystemLog
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void WriteSystemLog(char *str)
{
#ifdef HISTORY_LOG
	fp = fopen(HistoryLogFilename,"a");
	fprintf(fp, str);
	fclose(fp);		
#endif	
}

/*
*|----------------------------------------------------------------------------
*|  Routine: SimGpsTime
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void SimGpsTime()
{
	int value;
	
	GpsTow.week =1949;
	GpsTow.time =321548;

	GpsDateTime.year =2019;
	
	value =rand() % 12;
	GpsDateTime.month =11;//value;
	
	value =rand() % 30;
	GpsDateTime.day  =18;//value;
	
	value =rand() % 12;
	GpsDateTime.hour =6;//value;
	
	value =rand() % 60;
	GpsDateTime.min =value;
	
	value =rand() % 60;
	GpsDateTime.sec =value;	
}

/*
*|----------------------------------------------------------------------------
*|  Routine: ParseGpsTime
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int ParseGpsTime(char *pRxBuf)
{
	// TIMEA,COM1,0,56.5,FINESTEERING,1948,43207.000,00000000,9924,14039;VALID,1.744794222e-09,1.285034918e-09,-18.00000000000,2017,5,7,11,59,49000,VALID*8e686cbf	
	// 5 commas, GpsWeek
	// 6 commas, GpsTime
	
	int index;	
	int commaCnt =0;	

	#ifdef SWIFTNAV	
		strcpy( FileExt, "sbp");
		return 1;
	#elif COMNAV
		strcpy( FileExt, "cnb");
	#else
		strcpy( FileExt, "log");
	#endif
	
	if(strstr(pRxBuf, "INVALID*") !=0 ||
	   strstr(pRxBuf, "ERROR") !=0
	  )
	{
		printf("GPS Time INVALID!!!\n");
		return 0;
	}
	
	#ifdef COMNAV
	if(strstr(pRxBuf, "Command accepted!") !=0 )
	{
		printf("ComNav GNSS Detected!!!\n");
		
		strcpy(FileExt,"cnb");
	}
	#endif	

	for( index=0; index<255; index++)
	{
		if( *(pRxBuf++) == ',' )
		{
			switch( commaCnt )	
			{
				case 4:
					//GpsWeek =1948;
					//memcpy( &GpsTow.week, pRxBuf, sizeof(GpsTow.week) );
					GpsTow.week =atoi(pRxBuf);
					printf("\nGPS Week %d\n", GpsTow.week);
					break;
				case 5:
					//GpsTime =390600;
					//memcpy( &GpsTow.time, pRxBuf, sizeof(GpsTow.time) );
					GpsTow.time =atoi(pRxBuf);
					printf("\nGPS Time %d\n", GpsTow.time);
					break;					
				case 12:					
					GpsDateTime.year =atoi(pRxBuf);
					break;							
				case 13:					
					GpsDateTime.month =atoi(pRxBuf);
					break;							
				case 14:					
					GpsDateTime.day =atoi(pRxBuf);
					break;							
				case 15:					
					GpsDateTime.hour =atoi(pRxBuf);
					break;							
				case 16:					
					GpsDateTime.min =atoi(pRxBuf);
					break;							
				case 17:					
					GpsDateTime.sec =atoi(pRxBuf);
					GpsDateTime.sec /=1000;
					break;												
			}
			
			commaCnt ++;
			
			if( commaCnt >17 )
			{				
				printf("\nGPS Time %d,%d,%d %d:%d:%d\n", GpsDateTime.year,GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec);
				
				if( GpsDateTime.year ==0 || GpsDateTime.month ==0 || GpsDateTime.day ==0 )
				{
					return 0;
				}
				else
					return 1;
			}
		}
	}
	
	return 0;
}


/*
*|----------------------------------------------------------------------------
*|  Routine: ParseSwiftNavTime
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
uint8_t ParseSwiftNavTime(char *pRxBuf)
{
	int k;
	uint8_t swiftNavTime =0;
	
	uint8_t status;
	
	for(k=2; k<4096; k++)
	{
		//printf("%x", pRxBuf[k]);
		if( pRxBuf[k] ==0x01 && pRxBuf[k-1] ==0x02 && pRxBuf[k-2] ==0x55 )
		{
			//printf("SwiftNav GPS TIME %x %x %x %x %x %x %x %x\n", pRxBuf[k+1], pRxBuf[k+2], pRxBuf[k+4], pRxBuf[k+5], pRxBuf[k+6], pRxBuf[k+7],pRxBuf[k+8], pRxBuf[k+9]);
										
			GpsTow.week =(uint8_t)(pRxBuf[k+5]&0x000000ff)<<8 | (uint8_t)pRxBuf[k+4]&0x000000ff;
			GpsTow.time =(uint32_t)((uint8_t)(pRxBuf[k+9]&0x000000ff)<<24 | (uint8_t)(pRxBuf[k+8]&0x000000ff)<<16 | (uint8_t)(pRxBuf[k+7]&0x000000ff)<<8 | (uint8_t)(pRxBuf[k+6]));
						
			GpsTow.time /=1000;
						
			if( GpsTow.week >1990 && GpsTow.week <3000 )
			{
				swiftNavTime |=0x01;						
			}
						
			GpsDateTime.year =GpsTow.week;
			GpsDateTime.month =GpsTow.time;
						
			//GpsTow.time =gpsTime;
			//GpsTow.time /=1000;
			printf("GPS TIME %d %d\n", GpsTow.week, GpsTow.time);						
		//	break;
		}	
		if( pRxBuf[k] ==0x01 && pRxBuf[k-1] ==0x03 && pRxBuf[k-2] ==0x55)
		{
			//printf("SwiftNav UTC TIME %x %x %x %x %x %x %x %x\n", pRxBuf[k+1], pRxBuf[k+2], pRxBuf[k+4], pRxBuf[k+5], pRxBuf[k+6], pRxBuf[k+7],pRxBuf[k+8], pRxBuf[k+9]);						
			status =(uint8_t)(pRxBuf[k+4]); 
			
			printf("\nstatus %d\n", status);
			
			if( (status&0x01) ==0x01 )
			{
				GpsTow.time =(uint8_t)(pRxBuf[k+8]&0x00ff)<<24 | (uint8_t)(pRxBuf[k+7]&0x00ff)<<16 | (uint8_t)(pRxBuf[k+6]&0x00ff)<<8 | (uint8_t)pRxBuf[k+5]&0x00ff;
				GpsTow.time /=1000;
				
				GpsDateTime.year =(uint8_t)(pRxBuf[k+10]&0x00ff)<<8 | (uint8_t)pRxBuf[k+9]&0x00ff;
				GpsDateTime.month =(uint8_t)pRxBuf[k+11]&0x00ff;
				GpsDateTime.day =(uint8_t)pRxBuf[k+12]&0x00ff;
							
				GpsDateTime.hour =(uint8_t)pRxBuf[k+13]&0x00ff;
				GpsDateTime.min =(uint8_t)pRxBuf[k+14]&0x00ff;
				GpsDateTime.sec =(uint8_t)pRxBuf[k+15]&0x00ff;
							
				printf("\nUTC TIME status %d %d,%d,%d %d:%d:%d GPS TIME %d\n", status, GpsDateTime.year,GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec,GpsTow.time);
				
				//if( GpsDateTime.year ==0 || GpsDateTime.month ==0 || GpsDateTime.day ==0 )				
					//swiftNavTime =0x00;				
				//else
				swiftNavTime |=0x02;
			}		
			break;
		}
	}	
	
	return swiftNavTime;
}

/*
*|----------------------------------------------------------------------------
*|  Routine: GetOtgDiskSpace
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
unsigned long long GetOtgDiskSpace(void)
{
	unsigned long long result = 0;		
	struct statvfs sfs;
	
	if ( statvfs ( "/home/root/msd/", &sfs) != -1 )
	{	
		result = (unsigned long long)sfs.f_bsize * sfs.f_blocks;
		printf("disk size: %lld\n", result);

		result = (unsigned long long)sfs.f_bsize * sfs.f_bfree;
		printf("disk space available: %lld\n", result);	
	}		
	
	return result;
}

/*
*|----------------------------------------------------------------------------
*|  Routine: CopyLogToSdCard
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void CopyLogToSdCard(void)
{
	/* copy log file to SD card */
	sprintf(fileCpyStr, "cp /home/root/msd/%s /media/sdcard/%s",logFileName,logFileName);
	system(fileCpyStr);
	printf("%s", fileCpyStr);
	printf("\n");		
}

void *print_message_function( void *ptr )
{
	for(;;)
	{
		char *message;
		message = (char *) ptr;
		printf("%s \n", message);
	 
		usleep(500000); 
	}
}

void *blink_led( void *ptr )
{
    int ledstate =0;
    int gpio_pin =37;
    gpio_export(gpio_pin);
    gpio_direction(gpio_pin,1);


    for(;;)
    {
        //printf("LED %d\n", ledstate);
        gpio_write(gpio_pin, ledstate);
	
	if(ledstate == 0 )ledstate =1;
	else ledstate =0;
	
        usleep(500000); 
    }
}

/*
*|----------------------------------------------------------------------------
*|  Routine: UartInit
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int UartInit(char *portname)
{
    /** init the uart **/
    int msgcnt =0;
    int uartStream = -1;
    uartStream = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
    //uartStream = open("/dev/ttymcu1", O_RDWR | O_NOCTTY | O_NDELAY); //Open in non blocking read/write mode

    if (uartStream <0)
    {
	//ERROR - CAN'T OPEN SERIAL PORT
	printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    }	
    
    struct termios oldtio,newtio;
    //newtio.c_cflag = B115200 | CRTSCTS | CS8 | CLOCAL | CREAD;
    newtio.c_cflag = B115200 | CS8;
    /* raw output */
    //newtio.c_oflag = 0;
 
    /* now clean the modem line and activate the settings for the port */
    tcflush(uartStream, TCIFLUSH);
    //cfsetospeed(&newtio,B115200);
    //cfsetispeed(&newtio,B115200);
    tcsetattr(uartStream,TCSANOW,&newtio);

    return uartStream;
}


/*
*|----------------------------------------------------------------------------
*|  Routine: GetUsbDiskSpace
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
float GetUsbDiskSpace(void)
{
	//unsigned long long result = 0;		
	unsigned long long result = 0;		
	struct statvfs sfs;
	
	float avalue =0.0;
	
	if ( statvfs ("/media/usb/", &sfs) != -1 )
	{	
		//result =(unsigned long long)( (unsigned long long)sfs.f_bsize * (unsigned long long)sfs.f_blocks);
		//result =(unsigned long long)(sfs.f_bsize * sfs.f_blocks);
		
		avalue =(float)((float)sfs.f_bsize/(float)1024.0/1024.0 *sfs.f_blocks);
		//result /=(1024*1024);
		//printf("disk size: %d %ld\n", sfs.f_bsize, sfs.f_blocks);
		printf("disk size: %4.2fMB\n", avalue);

		//result = (unsigned long long)sfs.f_bsize * sfs.f_bfree;
		//result = (unsigned long long)(sfs.f_bsize * sfs.f_bfree);
		
		avalue =(float)((float)sfs.f_bsize/(float)1024.0/1024.0 *sfs.f_bfree);
		//printf("disk space available: %d %d\n",sfs.f_bsize, sfs.f_bfree);
		printf("disk space available: %4.2fMB\n",avalue);
	}		
	
	return avalue; // result;
}

/*
*|----------------------------------------------------------------------------
*|  Routine: MountUsbDrive
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int MountUsbDrive(void)
{
	int retval =0;
	
	/* mount USB drive */
	system("umount /media/usb");
	retval = system("mount -t vfat /dev/sda /media/usb");
	printf("%d\n", retval);
					
	if( retval !=0 )
	{
		retval = system("mount -t vfat /dev/sda1 /media/usb");
		printf("%d\n", retval);						
	}
				
	if( retval !=0 )
	{
		retval = system("mount -t vfat /dev/sdb /media/usb");
		printf("%d\n", retval);						
	}	

	if( retval !=0 )
	{
		retval = system("mount -t vfat /dev/sdb1 /media/usb");
		printf("%d\n", retval);						
	}					

	return retval;
}

/*
*|----------------------------------------------------------------------------
*|  Routine: GetFileSize
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
unsigned long GetFileSize(char *pFileName)
{	
	struct stat st;
	stat(pFileName, &st);
	unsigned long fileSize = st.st_size;
	printf("Log file size %d\n", fileSize);	
		
	return fileSize;
}

/*
*|----------------------------------------------------------------------------
*|  Routine: CopyLogToUsbDrive
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void CopyLogToUsbDrive(void)
{	
	DIR *d;
	char st[80];
	char oldest[80];
    struct dirent *dir;
    d = opendir("/home/root/msd");
	int nbrFiles =0;
	
	//system("ls /home/root/msd/ -lt");
	
	struct stat filestat;
	struct stat filestat2;
	time_t epoch;
	time_t epoch2;
	struct tm fileTime;    
	int cnt =0;
	unsigned long long DiskFreeSpace;
	
	if (d)
	{		
		while ((dir = readdir(d)) != NULL)
		{
			nbrFiles ++;
			//printf("%s %d\n", dir->d_name, nbrFiles);
		
			strcpy(st, "/home/root/msd/");	
			strcat(st, dir->d_name);
				
			stat(st,&filestat);
				
			cnt =0;
			int value[6];
			int multiplier =1;
			int digCnt =0;
			
			epoch =0;
			
			while(1)
			{	
				//printf("%x\n", st[cnt]);		
				//if(st[cnt] =='u' )
					
				/* first convert filename to linux time */ 
				if(st[++cnt] == '_')
				{	
					//while(digCnt <6 )
					//{
					cnt++;					
					fileTime.tm_year  =0;					
					while(st[cnt] !='_' )
					{
						fileTime.tm_year =(fileTime.tm_year*multiplier) + (st[cnt]-0x30);
						
						value[digCnt] =(value[digCnt]*multiplier) + (st[cnt]-0x30);
						//printf("%x\n", st[cnt]);
						multiplier =10;
							
						cnt++;
					}
						
					/* sanity check */
					if( fileTime.tm_year<2017 || fileTime.tm_year >2100 )
					{
						printf("filename error %d\n", fileTime.tm_year);
						break;
					}
						
					digCnt ++;
						//}
				#if 1	
					cnt ++;
					fileTime.tm_mon =0;
					while(st[cnt] !='_' )
					{
						fileTime.tm_mon =(fileTime.tm_mon*multiplier) + (st[cnt]-0x30);
						
						//printf("%x\n", st[cnt]);
						multiplier =10;
							
						cnt++;
					}					
						
					cnt ++;
					fileTime.tm_mday =0;
					while(st[cnt] !='_' )
					{
						fileTime.tm_mday =(fileTime.tm_mday*multiplier) + (st[cnt]-0x30);
							
							//printf("%x\n", st[cnt]);
						multiplier =10;
							
						cnt++;
					}										

					cnt ++;
					fileTime.tm_hour =0;
					while(st[cnt] !='_' )
					{
						fileTime.tm_hour =(fileTime.tm_hour*multiplier) + (st[cnt]-0x30);
							
						//printf("%x\n", st[cnt]);
						multiplier =10;
							
						cnt++;
					}	
						
					cnt ++;
					fileTime.tm_min =0;
					while(st[cnt] !='_' )
					{
						fileTime.tm_min =(fileTime.tm_min*multiplier) + (st[cnt]-0x30);
							
						//printf("%x\n", st[cnt]);
						multiplier =10;
							
						cnt++;
					}						
						
					cnt ++;
					fileTime.tm_sec =0;
					while(st[cnt] !='.' )
					{
						fileTime.tm_sec =(fileTime.tm_sec*multiplier) + (st[cnt]-0x30);
						
						//printf("%x\n", st[cnt]);
						multiplier =10;
							
						cnt++;
					}					
						
					fileTime.tm_year -=1900;					
					fileTime.tm_mon -=1;					
				#else				
					fileTime.tm_year =value[0] -1900;					
					fileTime.tm_mon =value[1]-1;
					fileTime.tm_mday =value[2];
					fileTime.tm_hour =value[3];
					fileTime.tm_min =value[4];
					fileTime.tm_sec =value[5];
				#endif
					//printf("%d %d %d %d %d %d\n", fileTime.tm_year, fileTime.tm_mon, fileTime.tm_mday, fileTime.tm_hour,fileTime.tm_min,fileTime.tm_sec);
						
					epoch = mktime(&fileTime);		 
					printf("%d %s\n", epoch, dir->d_name);

					break;
				}	
					
				if( cnt >50 )
				{
					//printf("break %s %d\n", dir->d_name, cnt);
					cnt =0;
					break;
				}
			}
										
			if( (GpsEpoch -epoch)<AMOUNT_TO_COPY && (GpsEpoch -epoch)>=0 )
			{
				//printf("%ld %ld %s\n", GpsEpoch, epoch, dir->d_name);
	
				sprintf(fileCpyStr, "cp /home/root/msd/%s /media/usb/%s",dir->d_name,dir->d_name);
				system(fileCpyStr);
				printf(fileCpyStr);
				printf("\n");						
			}
		}		
						
		closedir(d);
	}	
}

/*
*|----------------------------------------------------------------------------
*|  Routine: main
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int main(void)
{
	typedef struct
	{
		unsigned int print;	
		unsigned int selftest;
		unsigned int serialComm;
		unsigned int i2cUart;
			
	}TIMER;
	
	TIMER Timer;
	
	int i,j;
	int GpsPositionValid =0;

	uint16_t LoopCounter =0;

	static unsigned long NsecTimer=0;
	static unsigned long usecTime=0;
		
	
	static int Toggle =0;
	static int BuzzerToggle =0;

	static struct timeval tv;		

	static char CopyFileString[255];// = asctime(tm);
		  
    time_t s;  // seconds
    struct timespec spec;	

	//char txBuffer[64];
	char rxBuffer[4096];
	uint8_t rxDataBuf[64];
	
	int nbrBytes=0;
	int retval =0;
	
	int eventCount =0;
	int cameraTriggerCount =0;
	int selfTestPhotoCount =0;
	
	int currentEvCount =0;
	int currentTrCount =0;

	double prevEvTime =0;
	int prevTrCnt =0;
	
	int TestEvCount =0;
	
	uint8_t ImuHwEnabled =0;
 
    uint8_t SwiftNavTime =0;
	
	float MemAvail =0.0;
	unsigned long LogFileSize=0;
	
	double horspd =0.0;
	double trkgnd =0.0;

int count =0;
char tempbuf[25];

#if 1
/*** GPIO ***/
char aBuf[32]; 
int gpio_pin =32; //E1_24, GPIO2_IO00, so gpion_pin =(2-1)*32 + 0 =32
gpio_export(gpio_pin);
gpio_direction(gpio_pin,1);
gpio_write(gpio_pin, 0);
printf("WRITE BUZZER\n");
usleep(1000000); 
gpio_write(gpio_pin, 1);

gpio_pin =37; //E1_34, GPIO2_IO05, so gpion_pin =(2-1)*32 + 5 =37
gpio_export(gpio_pin);
gpio_direction(gpio_pin,1);
gpio_write(gpio_pin, 1);
printf("WRITE GPS\n");
usleep(1000000); 
gpio_write(gpio_pin, 0);

gpio_pin =49;
gpio_export(gpio_pin);
gpio_direction(gpio_pin,0);
int value =gpio_read(gpio_pin);
printf("READ %d\n", value);

/*** THREAD ***/
/** start a thread */
pthread_t thread1, thread2;
char *message1 = "Thread 1";
char *message2 = "Thread 2";
int  iret1, iret2;
int led;
/* Create independent threads each of which will execute function */
//iret1 = pthread_create( &thread1, NULL, print_message_function, (void*) message1);
	
iret1 = pthread_create( &thread2, NULL, blink_led, (void*)led); 

//pthread_join( thread1, NULL);
	 
/*** I2C ***/
int i2cstream;
char filename[20];
int addr = 0x48; // 0b1010011; /* The I2C address */
uint16_t dataAddr = 0x1234;
uint8_t val = 0x5c;
uint8_t buf[3];

//sprintf(filename,"/dev/i2c-1",0);
char *i2cname = "/dev/i2c-1";
if ((i2cstream = open(i2cname,I2C_RDWR)) < 0)
{
	printf("I2C ERR1\n");
    //exit(1);
}

if (ioctl(i2cstream,I2C_SLAVE,addr) < 0)
{
	printf("I2C ERR2\n");
    //exit(2);
}

buf[0] = dataAddr >> 8;
buf[1] = dataAddr & 0xff;
buf[2] = val;

if (write(i2cstream, buf, 3) != 3)
{
	printf("I2C ERR3\n");
//    exit(3);
}


#if 0
int msgcnt =0;
char *portname = "/dev/ttymxc5";
int uartStream = -1;
uartStream =UartInit(portname);

/*** TEST PERIPHERALS ***/
while(1)
{
msgcnt++;
memset(&aBuf, 0x55, sizeof(aBuf));
strcpy(aBuf, "Hello PICO-PI, MXC driver");
write(uartStream, aBuf, 32);
usleep(1000000); 
memset(&aBuf, 0x00, 32);

nbrBytes =0;
memset(&rxBuffer, 0x00, 255);
nbrBytes = read(uartStream,rxBuffer,255); 
rxBuffer[nbrBytes]=0;             /* set end of string, so we can printf */
printf("UART %s:  %s:%d:count %d\n", portname, rxBuffer, nbrBytes, msgcnt);

usleep(100000); 
}      		
#endif
#endif
	
	printf(TAGR_VERSION);

	//InitGpio();
	printf("GPIO NOT Init\n");
			
	InitPwm();	
	printf("PWM Init\n");
			
#ifdef USE_IMU	
	ImuHwEnabled =InitI2C();		// this inits IMU as well	
	printf("I2C Init\n");
	
	if(ImuHwEnabled)
		printf("\nIMU Detected\n");
	else
		printf("\n!!!No IMU Detected\n");
		
#endif
	
#ifdef TEST_IMU_OUTPUT	
 #ifdef MPU_9250	
	IMU_DATA imuAccel;
	GetImuAccel(&imuAccel);							
	printf("\nIMU Accel Data: X %d Y %d Z %d\n", imuAccel.x,imuAccel.y,imuAccel.z);	
	
	IMU_DATA imuGyro;
	GetImuGyro(&imuGyro);							
	printf("\nIMU Gyro Data: X %d Y %d Z %d\n", imuGyro.x,imuGyro.y,imuGyro.z);	
	
	IMU_DATA imuMag;
	GetImuMag(&imuMag);							
	printf("\nIMU Mag Data: X %d Y %d Z %d\n", imuMag.x,imuMag.y,imuMag.z);			
 #else	
	SPATIAL_DATA spatialData;
	GetSpatialData(&spatialData);				
	printf("\nSpatial Data: H %f P %f R %f\n", spatialData.heading,spatialData.pitch,spatialData.roll);
 #endif	
#endif

#ifdef TURN_OFF_WIFI	
	/* turn off WiFi */
	system("systemctl stop wpa_supplicant");	
	printf("kill WiFi\n");
#else
	BuzzerChirp(10);
	printf("\n!!!Warning WiFi Enabled!!!\n\n");
#endif	

#ifdef DISABLE_PV_DETECT	
	printf("\n!!!Warning PV Detect Override Enabled!!!\n\n");
#endif

#ifdef SIM_GPS_DATE_TIME	
	printf("\n!!!Warning GPS Data Time Simulation Enabled!!!\n\n");
#endif

#ifdef SWIFTNAV	
	printf("\n!!!Swiftnav GNSS module enabled, baudrate %dbps!!!\n\n", RAW_GPS_BAUD_RATE);
#elif COMNAV	
	printf("\n!!!ComNav GNSS module enabled, baudrate %dbps!!!\n\n", RAW_GPS_BAUD_RATE);
#else
	printf("\n!!!Novatel GNSS module enabled, baudrate %dbps!!!\n\n", RAW_GPS_BAUD_RATE);
#endif
	
#ifdef GPS_DISABLED
	printf("\n!!!Warning GNSS module disabled!!!\n\n");
#endif

    char *portname = "/dev/ttymxc5";
    int uartStream = -1;
    uartStream =UartInit(portname);

#if 1
	if (uartStream <0)
	{		
		fprintf(stderr, "UART failed to setup\n");
			
		usleep(1000000);
		
		BuzzerOn(1);
		
		YellowLedOff();
		
		RedLedOn(LED_SOLID);
		
		MainSystem.machState =SYSTEM_STATE_SELF_TEST_FAIL;
		//return EXIT_FAILURE;
	}
	else
	{ 
		printf("UART Init\n");	
		
		MainSystem.stateTimer =0;
		
		MainSystem.machState =SYSTEM_STATE_INIT;//SYSTEM_STATE_SELF_TEST_PASS;
	}	
#endif


	//char txBuffer[] ="Hello Edison 1234567890 Hello Edison 0987654321 Hello Edison 1234567890 Hello Edison 0987654321\n";	
	char txBuffer[] ="LOG TIMEA ONCE\r\n"; // get gps time command
	int MsgCount =0;
	char GnssMsg[10][255];
#ifdef SETUP_COMNAV	
	strcpy(GnssMsg[2],"log com1 rawephemb ontime 10\r\n");
	strcpy(GnssMsg[4],"log com1 glorawephemb ontime 10\r\n");
	strcpy(GnssMsg[3],"log com1 ionutcb ontime 10\r\n");	
	strcpy(GnssMsg[1],"log com1 rangecmpb ontime 1\r\n");
	strcpy(GnssMsg[0],"set rtkfreq 20\r\n");
	
	strcpy(GnssMsg[5],"SAVECONFIG\r\n");
#else
	strcpy(GnssMsg[0],"log com1 rangecmpb ontime 0.05\r\n");
#endif	
	strcpy(GnssMsg[6],"LOG TIMEA ONCE\r\n");
	
	unsigned long long DiskFreeSpace;
	
	printf("start main loop\n");

	
	#ifdef LED_TEST_ENABLED
	printf("\n!!!Warning LED Test Enabled!!!\n\n");
	MainSystem.machState =SYSTEM_STATE_LED_TEST_STATE1;
	#endif


	int k=0;
	uint16_t vmcCnt =0;
		
	while(1) // main program loop
	{
		clock_gettime(CLOCK_REALTIME, &spec);
					
		switch(MainSystem.machState)
		{
			case SYSTEM_STATE_INIT:
				GpsTow.week =0;
				GpsTow.time =0;
				GpsPositionValid =0;
				SwiftNavTime =0;

				GpsOff();
				
				YellowLedOff();			
				//RedLedOff();
				//GreenLedOff();
				
				BuzzerOff();
				SwTriggerOff();	
				//SwTriggerOn();

#ifdef HISTORY_LOG			
	printf("\nhistory log enabled\n");
	CreateSystemLogFile();	
#endif	
				
				MainSystem.stateTimer =spec.tv_sec;
				MainSystem.machState =SYSTEM_STATE_STARTUP;				
				break;
			case SYSTEM_STATE_STARTUP:
			#ifndef AUTO_START				
				if( ReadPowerSw() ==0 ) // power turned on, let's start
			#endif
				{								
					printf("\npwr sw assert\n");
					RedLedOn(LED_BLINK);
				
					/* power on GNSS receiver */
					GpsOn();			
								
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_IDLE;
					
#ifdef HISTORY_LOG		
fp = fopen(HistoryLogFilename,"a");
fprintf(fp, "Power on %d\n", spec.tv_sec);
fclose(fp);	
#endif
				}
				break;
			case SYSTEM_STATE_IDLE:
				if( (spec.tv_sec -MainSystem.stateTimer) >=POWER_UP_TIME )
				{
					/* if GPS powered on and data received, turn Vcc on (staggered) */
					//VccOn();
												
					MainSystem.stateTimer =spec.tv_sec;
					
				#ifndef INIT_GNSS_ENABLED
					MainSystem.machState =SYSTEM_STATE_POWERED;
				#else
					MsgCount =0;
					strcpy( txBuffer, GnssMsg[MsgCount]); //"log com1 rawephemb onchanged\r\n"); 
					
					printf("%s\n", txBuffer);
					
					//mraa_uart_flush(uart);															
					//mraa_uart_write(uart, txBuffer, strlen(txBuffer)); //40);
					write(uartStream, txBuffer, strlen(txBuffer));
					
					MainSystem.machState =SYSTEM_STATE_SETUP_COMNAV_STATE1;
				#endif
				}
				break;
			case SYSTEM_STATE_POWERED: /* wait for GPS fix Position valid */
			#ifndef DISABLE_PV_DETECT			
				if( GpsPositionValid )//  || (spec.tv_sec -MainSystem.stateTimer) >=15 )  //!!!!!! 30
			#endif				
				{						
					YellowLedOff();										
					RedLedOn(LED_SOLID);
									
					MainSystem.stateTimer =spec.tv_sec;					
					
					Timer.selftest =spec.tv_sec;
	
				#ifndef SWIFTNAV
					/* get GPS time (tow/sow, time of week) */ 
					strcpy( txBuffer, GnssMsg[6]);
					mraa_uart_flush(uart);
					//mraa_uart_write(uart, txBuffer, sizeof(txBuffer)); //32);
					write(uartStream, txBuffer, strlen(txBuffer));					
				#endif					
				
				printf("\nGet GPS time\n");
				printf("%s", txBuffer);				
				
					MainSystem.machState =SYSTEM_STATE_WAIT_GPS_TIME;

#ifdef HISTORY_LOG							
fp = fopen(HistoryLogFilename,"a");
fprintf(fp, "PV Detect %d\n", spec.tv_sec);
fclose(fp);				
#endif
				}								
				break;
			case SYSTEM_STATE_WAIT_GPS_TIME:
			#ifdef SIM_GPS_DATE_TIME								
				SimGpsTime();

				SetGpsTime(GpsTow.week, GpsTow.time);
				printf("\nGPS SIM time %s\n%d %d\n", rxBuffer, GpsTow.week, GpsTow.time);		

				CreateLogFile("sbp");
				
				CreateImuLogFile(ImuHwEnabled);
				
				DiskFreeSpace =GetOtgDiskSpace();				
				
				if( DiskFreeSpace <MIN_DISK_SPACE)
					printf("Drive capacity reaching limit!!!\n");
				else
					printf("Drive capacity OK\n");				

				SwTriggerOn();
				//SwTriggerOff(); /* signal PIC we are entering self test */
				BuzzerOn(1);
				
				currentEvCount =0;
				currentTrCount =0;
						
				SetEventTriggerCount(0);
				SetCameraTriggerCount();
					
				Timer.selftest =spec.tv_sec;				
				
				MainSystem.machState =SYSTEM_STATE_SELF_TEST;						
			#else	
				//if(strstr(rxBuffer, "accepted") !=0 )
						
			#ifndef SWIFTNAV					
				/* there is binary data in the rxBuffer due to the raw data coming in as well
				   so replace the binary data in the buffer with ASCII character '~'
				   so as that strstr() will work */
				for(k=0; k<sizeof(rxBuffer); k++)
				{
					if( rxBuffer[k] <0x20 || rxBuffer[k] > 0x7f)
						rxBuffer[k] ='~';
				}
				
				char *pch;
								
				pch =strstr(rxBuffer, "TIMEA");
			#else				
				SwiftNavTime |=ParseSwiftNavTime(rxBuffer);
			
				char *pch =0;				
			#endif
				
				if( pch !=0 || (SwiftNavTime &0x02) ==0x02 )		
				{
					if( pch !=0 )
						printf("%.150s\n", pch);//rxBuffer);
					
					if( ParseGpsTime(pch) )
					{					
						SetGpsTime(GpsTow.week, GpsTow.time);
						//printf("\nGPS time received %s\n%d %d\n", rxBuffer, GpsTow.week, GpsTow.time);
						//printf("\nGPS time received %d %d\n", GpsTow.week, GpsTow.time);
						
						/* create our raw log file */
						if( CreateLogFile(FileExt) )	
						{	
#ifdef HISTORY_LOG							
fp = fopen(HistoryLogFilename,"a");
fprintf(fp, "Log file name %s\n", filename1);
fclose(fp);	
#endif
					
							CreateImuLogFile(ImuHwEnabled);	

							DiskFreeSpace =GetOtgDiskSpace();						
							
							if( DiskFreeSpace <MIN_DISK_SPACE)
								printf("Drive capacity reaching limit!!!\n");
							else
								printf("Drive capacity OK\n");							
												
							currentEvCount =0;
							currentTrCount =0;
							
							selfTestPhotoCount =0;
							prevEvTime =0;
							prevTrCnt =0;
							
							SetEventTriggerCount(0);
							SetCameraTriggerCount();	
							
							SwTriggerOn();	
							//SwTriggerPulse();
							BuzzerOn(1);							
							
							Timer.selftest =spec.tv_sec;
		
#ifdef HISTORY_LOG		
fp = fopen(HistoryLogFilename,"a");
fprintf(fp, "GPS Time Received %d\n", spec.tv_sec);
fclose(fp);													
#endif

							MainSystem.machState =SYSTEM_STATE_SELF_TEST;						
						}
						else
						{
							Timer.selftest =spec.tv_sec;
												
							MainSystem.machState =SYSTEM_STATE_SELF_TEST_FAIL;
						}
					}
					else
						MainSystem.machState =SYSTEM_STATE_POWERED;						
				}
				
				if( (spec.tv_sec -Timer.selftest) >=5 )
				{
					/* no message received, try again */
					printf("\nGPS time not received, try again!!!\n");
//					printf("%s %d\n", rxBuffer, nbrBytes);
					MainSystem.machState =SYSTEM_STATE_POWERED;						
				}
			#endif
				break;
			case SYSTEM_STATE_SELF_TEST:					
				
				eventCount =GetEventTriggerCount();
				cameraTriggerCount =GetCameraTriggerCount();
								
				//if( currentTrCount !=cameraTriggerCount )
				if( currentEvCount !=eventCount )
				{
					// log to file
					double tr, ev, pps;
					double deltat;
					
					GetEventTime(&ev);
					GetCameraTriggerTime(&tr);
					GetPpsTriggerTime(&pps);

#ifdef HISTORY_LOG							
fp = fopen(HistoryLogFilename,"a");
fprintf(fp, "\nTimes %f %f %f\n", ev, tr, pps);
fclose(fp);				
#endif																				
			
					fp = fopen(filename1,"a");					
					fprintf(fp, "TR Time,%f,EV time,%f,TR Count,%d,EV Count,%d\n", tr, ev, cameraTriggerCount, eventCount);
					fclose(fp);	
										
					printf("\nTR Time %f EV time %f TR Count %d Ev Count %d\n", tr, ev, cameraTriggerCount, eventCount);
										
					currentEvCount =eventCount;								
					
					/* if time from last self test photo is 28.5-31.5 seconds */
					deltat =(ev-prevEvTime);
					
					if( deltat >28.5 && deltat <31.5 &&
						(cameraTriggerCount-prevTrCnt) ==1
					  )
					{
						selfTestPhotoCount ++;
						
						printf("\nSelf test photo #%d accepted\n", selfTestPhotoCount);
					}
					else
					{
						selfTestPhotoCount =1;							
						
						printf("\nSelf test photo start\n");

#ifdef HISTORY_LOG								
fp = fopen(HistoryLogFilename,"a");
fprintf(fp, "\nSelf test photo start %d GpsTime %d\n", spec.tv_sec, GpsTow.time);
fclose(fp);				
#endif																				
					}																	
					
					prevEvTime =ev;
					prevTrCnt =cameraTriggerCount;
					
					//currentTrCount =cameraTriggerCount;
					
					// buzzer chirp (2 beep chirp)											
					BuzzerOff();
					usleep(75000);
					BuzzerChirp(2);
				}
				
				//if( eventCount >=SELF_TEST_PHOTO_COUNT )
				if( selfTestPhotoCount >=SELF_TEST_PHOTO_COUNT )
				{			
					printf("\nTest Complete\nTR Count %d,EV Count %d\n", cameraTriggerCount, eventCount );
										
					RedLedOff();
					GreenLedOn(LED_BLINK);

					BuzzerOff();
									
					SwTriggerOff();
					
					memset(rxBuffer, 0x00, sizeof(rxBuffer) );
					
					MainSystem.stateTimer =spec.tv_sec;					
					
					Timer.serialComm =spec.tv_sec;					

				#ifdef USE_IMU	
					/* read now to clear buffer */
					GetI2CUARTRxData(rxDataBuf);
				#endif
				
					/* clear this buffer out as well */
					#if 0
					if( mraa_uart_data_available(uart, 10) )					
					{
						mraa_uart_read(uart, rxBuffer, sizeof(rxBuffer));		
					}
					#endif
					nbrBytes = read(uartStream,rxBuffer,255); 
										
					MainSystem.machState =SYSTEM_STATE_SELF_TEST_PASS;	
		
#ifdef HISTORY_LOG		
fp = fopen(HistoryLogFilename,"a");
fprintf(fp, "\nSelf test passed %d GpsTime %d\n", spec.tv_sec, GpsTow.time);
fclose(fp);																		
#endif
				}
				if( (spec.tv_sec -Timer.selftest) >=1 )
				{
					//printf("\nTR off\n");
					BuzzerOff();
					SwTriggerOff();
				}
				if( (spec.tv_sec -Timer.selftest) >=30 )
				{
				//	printf("\nNo EV, try again!!!\n");
					SwTriggerOn();
					//SwTriggerPulse();
					
				#ifndef AUTO_START					
					BuzzerOn(1);
				#endif
					Timer.selftest =spec.tv_sec;					
				}
				
			#ifdef SKIP_SELF_TEST
				selfTestPhotoCount =SELF_TEST_PHOTO_COUNT;
				SetEventTriggerCount(SELF_TEST_PHOTO_COUNT);
					
				MainSystem.machState =SYSTEM_STATE_SELF_TEST_PASS;	
			#else				
				if( DiskFreeSpace <MIN_DISK_SPACE )
				{
					MainSystem.machState =SYSTEM_STATE_SELF_TEST_FAIL;
				}
			#endif
				break;
			case SYSTEM_STATE_SELF_TEST_PASS:
				if( (spec.tv_sec%3) ==0)
				{
				#ifndef AUTO_START	
					BuzzerOn(1);			
				#endif
				}
				else
					BuzzerOff();
				
				#ifdef REDUCED_PREFLIGHT_TIME
				if( (spec.tv_sec -MainSystem.stateTimer) >=5 ) 
				#else
				if( (spec.tv_sec -MainSystem.stateTimer) >=PRE_FLIGHT_TIMEOUT ) 
				#endif
				{
					BuzzerOff();
										
					GreenLedOff();
					
					GreenLedOn(LED_SOLID);
					
					SetEventTriggerCount(0);
					
					SetCameraTriggerCount();
					
					// buzzer chirp (4 beep chirp)
					BuzzerChirp(4);				
					
					MainSystem.stateTimer=spec.tv_sec;
					
					MainSystem.machState =SYSTEM_STATE_READY;	

#ifdef HISTORY_LOG							
fp = fopen(HistoryLogFilename,"a");
fprintf(fp, "Flight ready %d\n", spec.tv_sec);
fclose(fp);				
#endif
				}				
				break;
			case SYSTEM_STATE_READY: // flight time	
			#ifdef FLIGHT_SIM_WITH_REPEAT
				if( (spec.tv_sec -MainSystem.stateTimer) >=1800 )  /* repeat test timer */
				{
					MainSystem.stateTimer =0;
					MainSystem.machState =SYSTEM_STATE_DEBOUNCE_POWER_DWN;					
				}
			#else
				if( ReadPowerSw() ==0 ) // potential power down, lets confirm
				{
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_DEBOUNCE_POWER_DWN;
				}
			#endif
				break;
			case SYSTEM_STATE_DEBOUNCE_POWER_DWN:
			
				if( ReadPowerSw() ==1 ) // false power down
				{
					MainSystem.machState =SYSTEM_STATE_READY;										
				}
				else if( (spec.tv_sec -MainSystem.stateTimer) >=POWER_DOWN_ASSSERT_TIME ) 
				{
					/* power turned off, let's stop */
#ifdef HISTORY_LOG							
fp = fopen(HistoryLogFilename,"a");
fprintf(fp, "Power Down %d\n", spec.tv_sec);
fclose(fp);	
#endif
					YellowLedOn(LED_SOLID);
			
					BuzzerChirp(6);
#ifdef HISTORY_LOG		
fp = fopen(HistoryLogFilename,"a");
fprintf(fp, "Yellow LED, and buzz %d\n", spec.tv_sec);
fclose(fp);	
#endif

					MainSystem.stateTimer =spec.tv_sec;
					
					eventCount =GetEventTriggerCount();
					cameraTriggerCount =GetCameraTriggerCount();
							
					fp = fopen(filename1,"a");	
					//fwrite(eventCount, 1, sizeof(eventCount), fp);
					//retval =fprintf(fp, "\nEV Count: %d   TR count: %d\n", eventCount, cameraTriggerCount);					
					if( fprintf(fp, "\nEV Count: %d   TR count: %d\n", eventCount, cameraTriggerCount) <0 )
					{
						printf("\nFILE WRITE ERROR (shutdown)!!!\n");												
					}						
					fclose(fp);	
		
#ifdef HISTORY_LOG				
fp = fopen(HistoryLogFilename,"a");
fprintf(fp, "Time %d File write status %d, EV count: %d, TR count: %d\n", spec.tv_sec, retval,eventCount,cameraTriggerCount);
fclose(fp);
#endif					
				
					MainSystem.machState =SYSTEM_STATE_COPY_FILES;
				}	
				
				break;		
			case SYSTEM_STATE_COPY_FILES:													
				MainSystem.stateTimer =spec.tv_sec;
			
				retval =MountUsbDrive();
																				
				if( retval ==0 )
				{
					YellowLedOff();
					
					/* drive mounted, check available space */
					MemAvail =GetUsbDiskSpace();
						
					LogFileSize =GetFileSize(filename1);
						
					TotalFileSize +=LogFileSize;
					TotalFileSize =TotalFileSize/1000000 +1; // add 1MB for precation
						
					printf("Total size of data to copy %ldMB; Mem avail: %4.2fMB\n", TotalFileSize, MemAvail);
						
					if( MemAvail >TotalFileSize )
					{
						/* if space okay, write to disk, blink green led  */						
						GreenLedOn(LED_BLINK);
						CopyLogToUsbDrive();							
						//GreenLedOn(LED_SOLID);
							
						MainSystem.machState =SYSTEM_STATE_FINALIZE_DATA_TRANSFER;								
					}
					else
					{						
						RedLedOn(LED_BLINK);
						
						BuzzerToggle =0;
						BuzzerOn(1);
						
						printf("No space available\n");		
						MainSystem.machState =SYSTEM_STATE_WAIT_USER_FAIL_ACK;		
					}
					
					/* umount USB drive */
					retval = system("umount /media/usb");																		
				}
				else
				{
					//RedLedOn(LED_BLINK);
					//MainSystem.machState =SYSTEM_STATE_WAIT_USER_FAIL_ACK;	

					MainSystem.machState =SYSTEM_STATE_POWER_DOWN;							
				}
				
				printf("\npwr down\n");						
					
				SwTriggerOn(); /* signal PIC we are done flight */																	
				break;	
			case  SYSTEM_STATE_READY_GPS_LOST:
				if( GpsPosValid() ==1 )
				{
					YellowLedOff();
					GreenLedOn(LED_SOLID);
					MainSystem.machState =SYSTEM_STATE_READY;
				}
				break;
			case SYSTEM_STATE_WAIT_USER_FAIL_ACK:
				if( ReadPowerSw() ==0 ) // power down
				{	
				#if 0
					RedLedOn(LED_SOLID);
					
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_POWER_DOWN;	
				#else
					BuzzerOff();
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_COPY_FILES;	
				#endif
				}
				
				if( (spec.tv_sec -MainSystem.stateTimer) >=1 ) 
				{
					if(BuzzerToggle ==0)
					{
						BuzzerOff();
						BuzzerToggle =1;						
					}				
					else
					{
						BuzzerOn(1);
						BuzzerToggle =0;
					}
					
					MainSystem.stateTimer =spec.tv_sec;
				}					
				break;
			case SYSTEM_STATE_FINALIZE_DATA_TRANSFER:
				GreenLedOn(LED_SOLID);
				MainSystem.stateTimer =spec.tv_sec;
				MainSystem.machState =SYSTEM_STATE_POWER_DOWN;						
				break;
			case SYSTEM_STATE_POWER_DOWN:								
				if( (spec.tv_sec -MainSystem.stateTimer) >=3 ) 
				{
					YellowLedOff();
					GpsOff();
					VccOff();
					
					//YellowLedOn(LED_BLINK);
		
#ifdef HISTORY_LOG				
fp = fopen(HistoryLogFilename,"a");
fprintf(fp, "GPS off, status LED off %d\n", spec.tv_sec);
fclose(fp);				
#endif

					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_OFF;//SYSTEM_STATE_INIT;
					
				#ifdef USE_OTG_MSD	
					system("umount /home/root/msd");
				#else
					system("umount /media/sdcard");
				#endif							
				}
				break;
			case SYSTEM_STATE_OFF:
			#ifdef FLIGHT_SIM_WITH_REPEAT
				if( (spec.tv_sec -MainSystem.stateTimer) >=15 ) 
					MainSystem.machState =SYSTEM_STATE_INIT;
			#endif
				
				/* stay here forever, system must be reset/power cycled */
				break;
			case SYSTEM_STATE_SELF_TEST_FAIL:
				if( ReadPowerSw() ==0 )
				{
					MainSystem.machState =SYSTEM_STATE_INIT;								
				}
				BuzzerChirp(2);							
				break;						
			case SYSTEM_STATE_LED_TEST_STATE1:
				BuzzerOff();
				RedLedOn(LED_BLINK);
				
				MainSystem.stateTimer =spec.tv_sec;
				MainSystem.machState =SYSTEM_STATE_LED_TEST_STATE2;
				break;
			case SYSTEM_STATE_LED_TEST_STATE2:	
				if( (spec.tv_sec -MainSystem.stateTimer) >=2 )
				{
					YellowLedOff();										
					RedLedOn(LED_SOLID);					
					
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_LED_TEST_STATE3;
				}				
				break;			
			case SYSTEM_STATE_LED_TEST_STATE3:
				if( (spec.tv_sec -MainSystem.stateTimer) >=2 )
				{
					RedLedOff();
					GreenLedOn(LED_BLINK);
					
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_LED_TEST_STATE4;
				}				
				break;
			case SYSTEM_STATE_LED_TEST_STATE4:
				if( (spec.tv_sec -MainSystem.stateTimer) >=5 )
				{
					YellowLedOff();
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_LED_TEST_STATE1;
				}				
				break;
			case SYSTEM_STATE_SETUP_COMNAV_STATE1:
				if(strstr(rxBuffer, "accepted") !=0 )
				{
					printf("%s\n", rxBuffer);
					
					MsgCount++;
					
					strcpy( txBuffer, GnssMsg[MsgCount]); //"log com1 glorawephemb onchanged\r\n");
					
					printf("%s\n", txBuffer);
					
					//mraa_uart_write(uart, txBuffer, 40);
					//mraa_uart_flush(uart);					
					write(uartStream, txBuffer, 40);
					
					if( MsgCount ==GNNS_MSGS_COMPLETE)
						MainSystem.machState =SYSTEM_STATE_POWERED;
				}			
				else if( (spec.tv_sec -MainSystem.stateTimer) >=5 )
				{
					printf("%s\n", rxBuffer);
										
					strcpy( txBuffer, GnssMsg[MsgCount]); //"log com1 rawephemb onchanged\r\n"); 									
				
					printf("%d %s\n", MsgCount, txBuffer);
				
					//mraa_uart_write(uart, txBuffer, 35);
					//mraa_uart_flush(uart);					
				    write(uartStream, txBuffer, 35);
					
					MainSystem.stateTimer =spec.tv_sec;			
				}
				break;
			case SYSTEM_STATE_SETUP_COMNAV_STATE2:
				break;				
			case SYSTEM_STATE_SETUP_COMNAV_STATE3:
				break;				
			case SYSTEM_STATE_SETUP_COMNAV_STATE4:
				break;							
			case SYSTEM_STATE_SETUP_COMNAV_STATE5:
				break;								
				
		} // end switch(machState)
			
	#ifdef FILE_WRITE_TEST	
		/* !!!!!!!!!!!!!!!!!!! for testing !!!!!!!!!!!!!!!!!!! */		
		if( MainSystem.machState >=SYSTEM_STATE_SELF_TEST_PASS &&
			MainSystem.machState < SYSTEM_STATE_POWER_DOWN
		  )
		{			
			usleep(50000);			
	
			fp = fopen(filename1,"ab");	
			
			retval =fwrite(&txBuffer, 1, 512, fp);
			
			fclose(fp);		

			if(retval <=0 )
			{
				printf("write fail\n");
			}
		}
	#endif
	
	#ifdef SERIAL_LOOP_BACK	
		/* !!!!!!!!!!!!!!!!!!! for testing !!!!!!!!!!!!!!!!!!! */
		if( MainSystem.machState >=SYSTEM_STATE_SELF_TEST_PASS )
		{
			//mraa_uart_write(uart, txBuffer, 32);//sizeof(txBuffer));
			//mraa_uart_flush(uart);
			write(uartStream, txBuffer, 32);
			usleep(10000);
		}
	#endif	
		
		nbrBytes=0;
		
	#if 0
		if( mraa_uart_data_available(uart, 10) )					
		{
			nbrBytes =mraa_uart_read(uart, rxBuffer, sizeof(rxBuffer));		
		}
	#else
		nbrBytes = read(uartStream,rxBuffer,sizeof(rxBuffer));
	#endif		
	

	#ifdef SIM_RAW_DATA		
		nbrBytes = sizeof(rxBuffer);
		strcpy( rxBuffer, "RAW DATA TEST RAW DATA TEST RAW DATA TEST RAW DATA TEST RAW DATA TEST RAW DATA TEST RAW DATA TEST RAW DATA TEST RAW DATA TEST\n");
	#endif
	
		if( nbrBytes >0 )
		{		
			//printf( "%d\n", nbrBytes);
			//printf( "%s %d\n", rxBuffer, nbrBytes);
			
			fflush( stdout );

			char *pch;
			pch =strstr(rxBuffer, "\xaa\x44\x12\x1c\x63\x00");			
			
			if( pch !=0 )
			{	
				/* bestvel message received, get horizontal speed, and track over ground */		
				horspd = *(double *)&rxBuffer[44];
				trkgnd = *(double *)&rxBuffer[52];
				//printf("trkgnd %f %x %x %x %x %x %x %x %x\n", trkgnd, rxBuffer[52], rxBuffer[53], rxBuffer[54], rxBuffer[55],rxBuffer[56], rxBuffer[57], rxBuffer[58], rxBuffer[59]);
				//horspd =0.034429;
				//TransmitVCMMessage(horspd);
				printf("horspd trkgnd %f %f\n", horspd, trkgnd);
			}
			
			if( MainSystem.machState >=SYSTEM_STATE_SELF_TEST_PASS &&
				MainSystem.machState <SYSTEM_STATE_SETUP_COMNAV_STATE1
			  )
			{					
				fp = fopen(filename1,"a");
		
				//count++;							
				//sprintf(tempbuf, "\r\nepoc%d", count);
				//fwrite(tempbuf, 1, strlen(tempbuf), fp);
				
				if( fwrite(&rxBuffer, 1, nbrBytes, fp) !=nbrBytes )
				{
					printf("File write error\n");
					BuzzerChirp(2);
					
					fclose(fp);

					//MainSystem.machState =SYSTEM_STATE_POWER_DOWN;
#ifdef HISTORY_LOG							
fp = fopen(HistoryLogFilename,"a");
fprintf(fp, "!!!Log file write error!!! %d\n", spec.tv_sec);
fclose(fp);				
#endif	
				}
				else
					fclose(fp);								
			}
			
			/* reset timer */
			Timer.serialComm =spec.tv_sec;		
			
			//mraa_uart_flush(uart);
		}
		else
		{
			/* handle error here */
			if( MainSystem.machState >=SYSTEM_STATE_SELF_TEST_PASS &&
				(spec.tv_sec-Timer.serialComm) >LOSS_COMM_TIMEOUT
			  )
			{
				
				/* reset timer */
				Timer.serialComm =spec.tv_sec;					
		
#ifdef HISTORY_LOG				
fp = fopen(HistoryLogFilename,"a");
fprintf(fp, "!!!No UART data available!!! %d\n", spec.tv_sec);
fclose(fp);		
#endif		
																	
				//printf("No UART data available\n");				
				#ifndef LED_TEST_ENABLED
				//BuzzerChirp(2);
				#endif
			}
		}		

		memset( rxDataBuf, 0x00, sizeof(rxDataBuf) );
				
	#ifdef USE_IMU	
		nbrBytes =GetI2CUARTRxData(rxDataBuf);	
		
		if(ImuHwEnabled)
		{
			if( MainSystem.machState >=SYSTEM_STATE_SELF_TEST_PASS )		
			{
				//memset( rxDataBuf, 0x00, sizeof(rxDataBuf));		
				//nbrBytes =GetI2CUARTRxData(rxDataBuf);
				
				/* check if we have VN100 data */
				if( nbrBytes >0 )
				{	
					char *pch;								
					pch =strstr(rxDataBuf, "$VN");
					
					//if( pch ==0 )
					{
						fp = fopen(ImuFilename,"a");
						fwrite(&rxDataBuf, 1, nbrBytes, fp);
						fwrite(&trkgnd, 1, 8, fp);						
						fclose(fp);													
					}		
					// else do not log the VN100 velocity response to the imu log file???

					/* transmit horizontal speed to VN100 */
					//TransmitVCMMessage(horspd);
					
					/* diagnostic stuff for console */			
					//printf("horspd trkgnd %f %f\n", horspd, trkgnd);
					//printf("VN100 data %d\n", nbrBytes);
					
				}
			}
		}
	#endif
	
		if( spec.tv_sec-Timer.print >=1)
		{			
			/* diagnostic stuff for console */
			printf("Mach state %d timer %d   PV %d   EvCnt %d   TRCnt %d   sec %ld\n", MainSystem.machState, MainSystem.stateTimer, GpsPositionValid, GetEventTriggerCount(), GetCameraTriggerCount(), spec.tv_sec);	
			
			Timer.print =spec.tv_sec;
			
		#ifdef DISABLE_PV_DETECT						
			//#error PV DETECT DISABLED
			//PpsIsrCallback();
			//EventTriggerIsrCallback();
		#endif
		
			/* read the hardware pin */
			GpsPositionValid = GpsPosValid();		
			
			if( Toggle ==0 )
			{
				Toggle =1;
			}
			else
			{			
				Toggle =0;		
			}
		
		#ifdef SIM_RAW_DATA	
			if( MainSystem.machState >=SYSTEM_STATE_SELF_TEST_PASS )
				SetEventTriggerCount(TestEvCount++);
		#endif
		}	

	#ifdef TEST_VCM_MESSAGE
		if( MainSystem.machState >=SYSTEM_STATE_SELF_TEST_PASS)
			TransmitVCMMessage(vmcCnt++);
	#endif			

//char c;
//printf("Enter character: ");
//c = getc(stdin);
//printf("Character entered: ");
//putc(c, stdout);	
	}

    //mraa_uart_stop(uart);
    //mraa_deinit();
	
	if(fp !=0 )
		fclose(fp);
	
	printf("\nDONE\n");

	YellowLedOff();
//#endif
		
	return 0;
}

//https://github.com/intel-iot-devkit/mraa/tree/master/examples
//https://iotdk.intel.com/docs/mraa/v0.9.5/edison.html              // I/O map
