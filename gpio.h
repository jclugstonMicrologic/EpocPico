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

#define GPIO_IN  0
#define GPIO_OUT 1

void InitGpio(void);
void InitPwm(void);

void GpsOn(void);
void GpsOff(void);
void RedLedOn(int solid);
void RedLedOff(void);
void GreenLedOn(int solid);
void GreenLedOff(void);
void YellowLedOn(int solid);
void YellowLedOff(void);

void BuzzerOn(int level);
void BuzzerOff(void);
void BuzzerChirp(int nbrChirps);

void SwTriggerOn(void);
void SwTriggerOff(void);

void SetGpsTime(int gpsW, int gpsT);

void SetEventTriggerCount(int value);

void SetCameraTriggerCount(void);
int GetCameraTriggerCount(void);
int GetEventTriggerCount(void);
void GetCameraTriggerTime(double *pValue);

void GetEventTime(double *pValue);
void GetPpsTriggerTime(double *pValue);

int ReadPowerSw(void);
int GpsPosValid(void);

void VccOn(void);
void VccOff(void);




















