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

#ifndef HMC6883L_H_
#define HMC6883L_H_


#define  HMC5883L_CONFIG_A      0
#define  HMC5883L_CONFIG_B      1
#define  HMC5883L_MODE          2
#define  HMC5883L_DATA          3

#define  HMC5883L_SCALE_09      0
#define  HMC5883L_SCALE_13      1
#define  HMC5883L_SCALE_19      2
#define  HMC5883L_SCALE_25      3
#define  HMC5883L_SCALE_40      4
#define  HMC5883L_SCALE_47      5
#define  HMC5883L_SCALE_56      6
#define  HMC5883L_SCALE_81      7

#define HMC5883L_DEVID			0x0F

// At 75Hz continuous measurement rate, the delay between samples = 1/75 = 13mS
#define  HMC5883L_MEAS_DELAY_MS     15
#define  HMC5883L_MEAS_NORMAL       0

#define HMC5883L_MEAS_CONTINUOUS        0
#define HMC5883L_MEAS_SINGLE_SHOT       1
#define HMC5883L_MEAS_IDLE              3

#define HMC5883L_LOCAL_DECLINATION  ((double)-0.031125)  // W 1 deg 47 minutes at Bangalore airport

#define TWO_PI               6.283185
#define _180_DIV_PI         57.295779

#define MAX_MAG_SAMPLES 10

typedef struct HMC5883L_CALIB_DATA_ {
    int xMax;
    int xMin;
    int yMax;
    int yMin;
    int zMax;
    int zMin;
} HMC5883L_CALIB_DATA;

typedef struct HMC_DATA_ {
    HMC5883L_CALIB_DATA calib;
    int xRange;
    int yRange;
    int zRange;
    int scaleIndex;
    int headingDeg;
    } HMC_DATA;

extern HMC_DATA gHMC5883L;


void hmc5883l_Config(void);
void hmc5883l_SetScale(int scale);
void hmc5883l_ReadXYZRawData(int* pmx, int* pmy, int* pmz);
void hmc5883l_GetGaussReadings(float* pmx, float* pmy, float * pmz);
void hmc5883l_SetMeasurementMode(int mode);
void hmc5883l_SetOperatingMode(int mode);
int  hmc5883l_GetHeadingDeg(int mx, int my, float dec);
void hmc5883l_GetAveragedRawData(int numSamples, int* pXavg, int* pYavg, int* pZavg);
void hmc5883l_GetCorrectedData(int mx, int my, int mz, float *pcmx, float* pcmy, float* pcmz);

#endif

