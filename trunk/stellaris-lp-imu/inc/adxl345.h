//*****************************************************************************
//
//				Stellaris IMU (FREEIMU PORT)
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
//		Author 				: Terence Ang - terenceang@mac.com
//		Original Code 		:  Hari Nair- hair.nair@gmail.com
//		Original i2c code 	: JOERG QUINTEN (aBUGSworstnightmare)
//		Original ftoa code	: JOERG QUINTEN (aBUGSworstnightmare)
//
//					1st Draft .. 09/March/2013
//
//*****************************************************************************

#include "type.h"
#include "inttypes.h"
#include "inc/hw_types.h"

#ifndef ADXL345_h
#define ADXL345_h

/* ------- Register names ------- */
#define ADXL345_DEVID 0x00
#define ADXL345_RESERVED1 0x01
#define ADXL345_THRESH_TAP 0x1d
#define ADXL345_OFSX 0x1e
#define ADXL345_OFSY 0x1f
#define ADXL345_OFSZ 0x20
#define ADXL345_DUR 0x21
#define ADXL345_LATENT 0x22
#define ADXL345_WINDOW 0x23
#define ADXL345_THRESH_ACT 0x24
#define ADXL345_THRESH_INACT 0x25
#define ADXL345_TIME_INACT 0x26
#define ADXL345_ACT_INACT_CTL 0x27
#define ADXL345_THRESH_FF 0x28
#define ADXL345_TIME_FF 0x29
#define ADXL345_TAP_AXES 0x2a
#define ADXL345_ACT_TAP_STATUS 0x2b
#define ADXL345_BW_RATE 0x2c
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_INT_ENABLE 0x2e
#define ADXL345_INT_MAP 0x2f
#define ADXL345_INT_SOURCE 0x30
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37
#define ADXL345_FIFO_CTL 0x38
#define ADXL345_FIFO_STATUS 0x39

#define ADXL345_BW_1600 0xF // 1111
#define ADXL345_BW_800  0xE // 1110
#define ADXL345_BW_400  0xD // 1101  
#define ADXL345_BW_200  0xC // 1100
#define ADXL345_BW_100  0xB // 1011  
#define ADXL345_BW_50   0xA // 1010 
#define ADXL345_BW_25   0x9 // 1001 
#define ADXL345_BW_12   0x8 // 1000 
#define ADXL345_BW_6    0x7 // 0111
#define ADXL345_BW_3    0x6 // 0110

/* 
 Interrupt PINs
 INT1: 0
 INT2: 1
 */
#define ADXL345_INT1_PIN 0x00
#define ADXL345_INT2_PIN 0x01

/* 
 Interrupt bit position
 */
#define ADXL345_INT_DATA_READY_BIT 0x07
#define ADXL345_INT_SINGLE_TAP_BIT 0x06
#define ADXL345_INT_DOUBLE_TAP_BIT 0x05
#define ADXL345_INT_ACTIVITY_BIT   0x04
#define ADXL345_INT_INACTIVITY_BIT 0x03
#define ADXL345_INT_FREE_FALL_BIT  0x02
#define ADXL345_INT_WATERMARK_BIT  0x01
#define ADXL345_INT_OVERRUNY_BIT   0x00

#define ADXL345_DATA_READY 0x07
#define ADXL345_SINGLE_TAP 0x06
#define ADXL345_DOUBLE_TAP 0x05
#define ADXL345_ACTIVITY   0x04
#define ADXL345_INACTIVITY 0x03
#define ADXL345_FREE_FALL  0x02
#define ADXL345_WATERMARK  0x01
#define ADXL345_OVERRUNY   0x00

#define MAX_CAL_SAMPLES         10
// assumes 15Hz sample interval
#define ADXL345_SAMPLE_DELAY_MS    70

typedef struct ADXL345_CALIB_DATA_ {
    short x0g;
    short y0g;
    short xp1g;
    short xm1g;
    short yp1g;
    short ym1g;
    short zp1g;
    short zm1g;
} ADXL345_CALIB_DATA;


typedef struct ADXL345_DATA_ {
    ADXL345_CALIB_DATA calib;
    int z0g;
    int xSens;
    int ySens;
    int zSens;
    int x2g;
    int y2g;
    int z2g;
} ADXL345_DATA;

extern ADXL345_DATA gADXL345;

void adxl345_Config(void);
void adxl345_PowerOn(void);
void adxl345_ReadXYZRawData(int *pxraw, int* pyraw, int* pzraw);
void adxl345_GetCorrectedData(int ax, int ay, int az, float* pacx, float * pacy, float* pacz);
void adxl345_GetAveragedRawData(int numSamples, int* pXavg, int* pYavg, int* pZavg);
void adxl345_CalcXYZGData(int xraw, int yraw, int zraw, int* pgx, int * pgy, int* pgz);
void adxl345_SetTapThreshold(int tapThreshold);
int adxl345_GetTapThreshold(void);
void adxl345_SetAxisGains(double *_gains);
void adxl345_GetAxisGains(double *_gains);
void adxl345_SetAxisOffset(int x, int y, int z);
void adxl345_GetAxisOffset(int* x, int* y, int*z);
void adxl345_SetTapDuration(int tapDuration);
int adxl345_GetTapDuration(void);
void adxl345_SetDoubleTapLatency(int doubleTapLatency);
int adxl345_GetDoubleTapLatency(void);
void adxl345_SetDoubleTapWindow(int doubleTapWindow);
int adxl345_GetDoubleTapWindow(void);
void adxl345_SetActivityThreshold(int activityThreshold);
int adxl345_GetActivityThreshold(void);
void adxl345_SetInactivityThreshold(int inactivityThreshold);
int adxl345_GetInactivityThreshold(void);
void adxl345_SetTimeInactivity(int timeInactivity);
int adxl345_GetTimeInactivity(void);
void adxl345_SetFreeFallThreshold(int freeFallthreshold);
int adxl345_GetFreeFallThreshold(void);
void adxl345_SetFreeFallDuration(int freeFallDuration);
int adxl345_GetFreeFallDuration(void);

int adxl345_IsActivityXEnabled(void);
int adxl345_IsActivityYEnabled(void);
int adxl345_IsActivityZEnabled(void);
int adxl345_IsInactivityXEnabled(void);
int adxl345_IsInactivityYEnabled(void);
int adxl345_IsInactivityZEnabled(void);
int adxl345_IsActivityAc(void);
int adxl345_IsInactivityAc(void);
void adxl345_SetActivityAc(int state);
void adxl345_SetInactivityAc(int state);

void adxl345_BypassFifo(void);

int adxl345_GetSuppressBit(void);
void adxl345_SetSuppressBit(int state);
int adxl345_IsTapDetectionOnX(void);
void adxl345_SetTapDetectionOnX(int state);
int adxl345_IsTapDetectionOnY(void);
void adxl345_SetTapDetectionOnY(int state);
int adxl345_IsTapDetectionOnZ(void);
void adxl345_SetTapDetectionOnZ(int state);

void adxl345_SetActivityX(int state);
void adxl345_SetActivityY(int state);
void adxl345_SetActivityZ(int state);
void adxl345_SetInactivityX(int state);
void adxl345_SetInactivityY(int state);
void adxl345_SetInactivityZ(int state);

int adxl345_IsActivitySourceOnX(void);
int adxl345_IsActivitySourceOnY(void);
int adxl345_IsActivitySourceOnZ(void);
int adxl345_IsTapSourceOnX(void);
int adxl345_IsTapSourceOnY(void);
int adxl345_IsTapSourceOnZ(void);
int adxl345_IsAsleep(void);

int adxl345_IsLowPower(void);
void adxl345_SetLowPower(int state);
double adxl345_GetRate(void);
void adxl345_SetRate(double rate);
void adxl345_SetBw(u08 bw_code);
u08 adxl345_GetBwCode(void);


int adxl345_Triggered(u08 interrupts, int mask);


u08 adxl345_GetInterruptSource(void);
int adxl345_GetInterruptSourceBit(u08 interruptBit);
int adxl345_GetInterruptMapping(u08 interruptBit);
void adxl345_SetInterruptMapping(u08 interruptBit, int interruptPin);
int adxl345_IsInterruptEnabled(u08 interruptBit);
void adxl345_SetInterrupt(u08 interruptBit, int state);

int adxl345_GetRangeSetting(void);
void adxl345_SetRangeSetting(int val);
int adxl345_GetSelfTestBit(void);
void adxl345_SetSelfTestBit(int selfTestBit);
int adxl345_GetSpiBit(void);
void adxl345_SetSpiBit(int spiBit);
int adxl345_GetInterruptLevelBit(void);
void adxl345_SetInterruptLevelBit(int interruptLevelBit);
int adxl345_GetFullResBit(void);
void adxl345_SetFullResBit(int fullResBit);
int adxl345_GetJustifyBit(void);
void adxl345_SetJustifyBit(int justifyBit);

void adxl345_SetRegisterBit(u08 regAdress, int bitPos, int state);
int adxl345_GetRegisterBit(u08 regAdress, int bitPos);

#endif

