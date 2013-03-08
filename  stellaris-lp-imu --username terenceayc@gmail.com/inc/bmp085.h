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

#ifndef BMP085_H_
#define BMP085_H_

#define BMP085_DEVID 0xD0

// maximum oversampling
#ifdef BMP085_UNIT_TEST
#define BMP085_OSS					0
#else
#define BMP085_OSS					3
#endif

// pressure sensor sampling states
#define BMP085_READ_TEMPERATURE 		11
#define BMP085_READ_PRESSURE			22

// 14.7 samples per second
// ~ 3 second averaging window

#define BMP085_SAMPLES_PER_SECX10     147 // bogus, fix
#define BMP085_NUM_Z_SAMPLES             10

extern int gnSmpCnt;
extern s32 gAltZBuf[];
extern s32 gnAltAvgM;
extern s32 gnAltCm;
extern s32 gnAltAvgCm;
extern s32 gnCps;
extern s16 gnTempC;
extern s32 gnPa;


void bmp085_TriggerPressureSample(void);
void bmp085_TriggerTemperatureSample(void);
u32  bmp085_ReadPressureSample(void);
u32  bmp085_ReadTemperatureSample(void);
s16  bmp085_CalcTemperatureCx10(void);
s32  bmp085_CalcPressurePa(void);
void bmp085_DeltaCmPerSec(void);
s32  bmp085_Pa2Cm(s32 pa);
void bmp085_AverageAltitude(void);
void bmp085_Config(void);
void bmp085_InitData(void);
void bmp085_InitWindowBuffers(void);
void bmp085_ReadCoeffs(void);
void bmp085_AcquireAveragedSample(int nSamples);

#ifdef BMP085_UNIT_TEST
void bmp085_UnitTest(void);
#endif

#ifdef BMP085_NOISE_EVAL
void bmp085_DumpAltitude(int nSamples, int pzInx);
#endif

void bmp085_SampleContinuous(void);


#endif /* BMP085_H_ */
