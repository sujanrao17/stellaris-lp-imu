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

#include <math.h>
#include "common.h"
#include "i2c_IMU.h"
#include "l3g4200d.h"

L3G_DATA gL3G;

short GXAvgBuf[];
short GYAvgBuf[];
short GZAvgBuf[];

void l3g_Config(void) {
	i2c_XmtByte(I2C_ID_L3G4200D, L3G_CTRL_REG1, 0x3e); // "00011111"- Data rate 100Hz, BW 25Hz, power up, enable all axes
	i2c_XmtByte(I2C_ID_L3G4200D, L3G_CTRL_REG4, 0x90); // "10010000" update output registers only after MSB/LSB are read, data lsb at lower address, 500dps, disable selftest
}

void l3g_ReadXYZRawData(short *pxraw, short *pyraw, short* pzraw) {
	u08 buf[6];
	s16 x, y, z;
	i2c_RcvBuf(I2C_ID_L3G4200D, L3G_OUT_X_L | (1 << 7), 6, buf); //read the acceleration data from the L3G4200D
	// each axis reading comes in 13 bit 2's complement format. lsb first, msb has sign bits extended
	x = (s16) ((((u16) buf[1]) << 8) | (u16) buf[0]);
	*pxraw = (int) x;
	y = (s16) ((((u16) buf[3]) << 8) | (u16) buf[2]);
	*pyraw = (int) y;
	z = (s16) ((((u16) buf[5]) << 8) | (u16) buf[4]);
	*pzraw = (int) z;
}

void l3g_GetAveragedRawData(char numSamples, short* pXavg, short* pYavg, short* pZavg) {
	int cnt;

	for (cnt = 0; cnt < numSamples; cnt++) {
		l3g_ReadXYZRawData(&GXAvgBuf[cnt], &GYAvgBuf[cnt], &GZAvgBuf[cnt]);
		DELAY_MS(L3G_SAMPLE_DELAY_MS);
	}
	*pXavg = util_AverageSamples(GXAvgBuf, numSamples);
	*pYavg = util_AverageSamples(GYAvgBuf, numSamples);
	*pZavg = util_AverageSamples(GZAvgBuf, numSamples);
}

void l3g_GetCalibStatsRawData(int numSamples, short* pXavg, short* pYavg,
		short* pZavg, short* pXSigma, short* pYSigma, short* pZSigma) {
	int cnt;

	for (cnt = 0; cnt < numSamples; cnt++) {
		l3g_ReadXYZRawData(&GXAvgBuf[cnt], &GYAvgBuf[cnt], &GZAvgBuf[cnt]);
		DELAY_MS(L3G_SAMPLE_DELAY_MS);
	}
	*pXavg = util_AverageSamples(GXAvgBuf, numSamples);
	*pYavg = util_AverageSamples(GYAvgBuf, numSamples);
	*pZavg = util_AverageSamples(GZAvgBuf, numSamples);
	*pXSigma = util_SigmaSamples(GXAvgBuf, numSamples, *pXavg);
	*pYSigma = util_SigmaSamples(GYAvgBuf, numSamples, *pYavg);
	*pZSigma = util_SigmaSamples(GZAvgBuf, numSamples, *pZavg);
}

void l3g_GetCorrectedData(int xraw, int yraw, int zraw, float* pgcx,
		float* pgcy, float* pgcz) {
	int xCorr, yCorr, zCorr;
	xCorr = xraw - gL3G.calib.xOffset;
	*pgcx = (ABS(xCorr) > gL3G.xThreshold ?(float) xCorr * L3G_SENSITIVITY_0500DPS : 0.0f);
	yCorr = yraw - gL3G.calib.yOffset;
	*pgcy = (ABS(yCorr) > gL3G.yThreshold ? (float) yCorr * L3G_SENSITIVITY_0500DPS : 0.0f);
	zCorr = zraw - gL3G.calib.zOffset;
	*pgcz = (ABS(zCorr) > gL3G.zThreshold ?(float) zCorr * L3G_SENSITIVITY_0500DPS : 0.0f);
}

