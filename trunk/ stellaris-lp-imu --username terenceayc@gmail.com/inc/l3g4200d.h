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


#ifndef L3G4200D_H_
#define L3G4200D_H_


#define L3G_WHO_AM_I            0x0F
#define L3G_CTRL_REG1           0x20
#define L3G_CTRL_REG2           0x21
#define L3G_CTRL_REG3           0x22
#define L3G_CTRL_REG4           0x23
#define L3G_CTRL_REG5           0x24
#define L3G_REF_DATA_CAPTURE    0x25
#define L3G_OUT_TEMP            0x26
#define L3G_STATUS_REG          0x27
#define L3G_OUT_X_L             0x28
#define L3G_OUT_Y_L             0x2A
#define L3G_OUT_Z_L             0x2C
#define L3G_FIFO_CTRL_REG       0x2E
#define L3G_FIFO_SRC_REG        0x2F
#define L3G_INT1_CFG            0x30
#define L3G_INT1_SRC            0x31
#define L3G_INT1_THS_XH         0x32
#define L3G_INT1_THS_XL         0x33
#define L3G_INT1_THS_YH         0x34
#define L3G_INT1_THS_YL         0x35
#define L3G_INT1_THS_ZH         0x36
#define L3G_INT1_THS_ZL         0x37
#define L3G_INT1_DURATION       0x38

#define L3G_MAX_CAL_SAMPLES     10

#define L3G_SAMPLE_DELAY_MS     10

#define L3G_SENSITIVITY_0250DPS  (0.00875f)
#define L3G_SENSITIVITY_0500DPS  (0.01750f)
#define L3G_SENSITIVITY_2000DPS  (0.07000f)

typedef struct L3G_CALIB_DATA_ {
    int xOffset;
    int yOffset;
    int zOffset;
    int xOffsetSigma;
    int yOffsetSigma;
    int zOffsetSigma;
} L3G_CALIB_DATA;

typedef struct L3G_DATA_ {
    L3G_CALIB_DATA calib;
    int xThreshold;
    int yThreshold;
    int zThreshold;
} L3G_DATA;

extern L3G_DATA gL3G;

void l3g_Config(void);
void l3g_ReadXYZRawData(int *pxraw, int* pyraw, int* pzraw);
void l3g_GetAveragedRawData(int numSamples, int* pXavg, int* pYavg, int* pZavg);
void l3g_GetCalibStatsRawData(int numSamples, int* pXavg, int* pYavg, int* pZavg, int* pXSigma, int* pYSigma, int* pZSigma);
void l3g_GetCorrectedData(int xraw, int yraw, int zraw, float* pgcx, float* pgcy, float* pgcz);

#endif
