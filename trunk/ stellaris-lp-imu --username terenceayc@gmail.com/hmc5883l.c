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

#include "common.h"
#include "type.h"
#include "hmc5883l.h"
#include "i2c_IMU.h"
#include <math.h>


HMC_DATA gHMC5883L;
extern int gXBuf[];
extern int gYBuf[];
extern int gZBuf[];

int gGaussScale[8] = {
    1370,   //  0.9
    1090,   //  1.3
    820,    //  1.9
    660,    //  2.5
    440,    //  4.0
    390,    //  4.7
    330,    //  5.6
    230     //  8.1
    };


void hmc5883l_Config(void) {
    u08 b;
    hmc5883l_SetScale(HMC5883L_SCALE_13);
    b = 0x78; // "01111000" clear CRA7, 8 sample average, 75Hz data rate, normal measurement flow
    i2c_XmtByte(I2C_ID_HMC5883L, HMC5883L_CONFIG_A,b);
    hmc5883l_SetOperatingMode(HMC5883L_MEAS_CONTINUOUS);
    }

void hmc5883l_SetScale(int scale) {
    u08 b;
    gHMC5883L.scaleIndex = scale;
    b = (((u08)scale)<<5);
    i2c_XmtByte(I2C_ID_HMC5883L, HMC5883L_CONFIG_B,b);
    }

void hmc5883l_ReadXYZRawData(int* pmx, int* pmy, int* pmz) {
    u08 buf[6];
    s16 x,y,z;
    i2c_RcvBuf(I2C_ID_HMC5883L, HMC5883L_DATA,6,buf);
    x = (s16)((((u16)buf[0]) << 8) | (u16)buf[1]); // NOTE : data is MSB first !!
    *pmx = (int)x;
    z = (s16)((((u16)buf[2]) << 8) | (u16)buf[3]); // NOTE : order is x,z,y!!
    *pmz = (int)z;
    y = (s16)((((u16)buf[4]) << 8) | (u16)buf[5]);
    *pmy = (int)y;
    }


void hmc5883l_GetAveragedRawData(int numSamples, int* pXavg, int* pYavg, int* pZavg)  {
    int cnt;
    for (cnt = 0; cnt < numSamples; cnt++){
        hmc5883l_ReadXYZRawData(&gXBuf[cnt], &gYBuf[cnt], &gZBuf[cnt]);
        DELAY_MS(HMC5883L_MEAS_DELAY_MS);
        }
    *pXavg = util_AverageSamples(gXBuf,numSamples);
    *pYavg = util_AverageSamples(gYBuf,numSamples);
    *pZavg = util_AverageSamples(gZBuf,numSamples);
    }

//void hmc5883l_GetCorrectedData(int mx, int my, int mz, float *pcmx, float* pcmy, float* pcmz) {
//    *pcmx = mx;
//    *pcmy = my;
//    *pcmz = mz;
//    }

void hmc5883l_GetCorrectedData(int mx, int my, int mz, float *pcmx, float* pcmy, float* pcmz) {
    *pcmx = ((float)(mx - gHMC5883L.calib.xMin)/(float)gHMC5883L.xRange) - 0.5f;
    *pcmy = ((float)(my - gHMC5883L.calib.yMin)/(float)gHMC5883L.yRange) - 0.5f;
    *pcmz = ((float)(mz - gHMC5883L.calib.zMin)/(float)gHMC5883L.zRange) - 0.5f;
    }

int hmc5883l_GetHeadingDeg(int mx, int my, float dec) {
    float x, y, heading;

    x = ((float)(mx - gHMC5883L.calib.xMin)/(float)gHMC5883L.xRange) - 0.5;
    y = ((float)(my - gHMC5883L.calib.yMin)/(float)gHMC5883L.yRange) - 0.5;

    heading = atan2(y,x);
    heading += dec;

    if(heading < 0.0){
        heading += TWO_PI;
        }

    if(heading > TWO_PI){
        heading -= TWO_PI;
        }
    heading *= _180_DIV_PI;

    return ((int) (heading+0.5));
    }

void hmc5883l_SetMeasurementMode(int mode) {
    u08 b;
    b = i2c_RcvByte(I2C_ID_HMC5883L, HMC5883L_CONFIG_A);
    b &= 0x7c; // "01111100" clear CRA7, keep existing averaging and datarate settings, clear measuremode bits
    b |= ((u08)mode);
    //b = (u08)mode;
    i2c_XmtByte(I2C_ID_HMC5883L, HMC5883L_CONFIG_A,b);
    }

void hmc5883l_SetOperatingMode(int mode) {
    u08 b;
    b = (u08)mode;
    i2c_XmtByte(I2C_ID_HMC5883L, HMC5883L_MODE,b);
    }


