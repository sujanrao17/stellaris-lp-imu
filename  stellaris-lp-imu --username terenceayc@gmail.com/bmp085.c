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
#include "bmp085.h"
#include "i2c_IMU.h"

s16 gAC1;
s16 gAC2;
s16 gAC3;
u16 gAC4;
u16 gAC5;
u16 gAC6;
s16 gB1;
s16 gB2;
s32 gB5;
s16 gMb;
s16 gMc;
s16 gMd;

int gnSensorState;
int gnSmpCnt;
s32 gAltZBuf[BMP085_NUM_Z_SAMPLES];
s32 gnAltAvgM;
s32 gnAltAvgCm;
s32 gnAltCm;
s32 gnCps;
u32 gnTraw;
u32 gnPraw;
s16 gnTempC;
s32 gnSum_t;
s32 gnSum_t2;
s32 gnDen;
s32 gnPa;

void bmp085_Config(void) {
	gnSmpCnt = 0;
	gnAltAvgM = 0;
	gnAltCm = 0;
	gnCps = 0;
	gnTempC = 0;
	gnPa = 0;
	// following parameters used in linear regression computation
	// for climb and sink rate, need to be computed only once.
	gnSum_t = -((s32) BMP085_NUM_Z_SAMPLES * (BMP085_NUM_Z_SAMPLES - 1)) / 2;
	gnSum_t2 = ((s32) BMP085_NUM_Z_SAMPLES * (BMP085_NUM_Z_SAMPLES - 1)
			* (2 * BMP085_NUM_Z_SAMPLES - 1)) / 6;
	gnDen = ((s32) BMP085_NUM_Z_SAMPLES * gnSum_t2) - (gnSum_t * gnSum_t);
	bmp085_ReadCoeffs();
#ifdef UART_DBG
#ifdef SENSOR_UNIT_TEST
	bmp085_UnitTest();
#endif
#endif
}

void bmp085_ReadCoeffs(void) {
	u08 b[22];
	i2c_RcvBuf(I2C_ID_BMP085, 0xAA, 22, b);

	gAC1 = (s16) ((((u16) b[0]) << 8) | (u16) b[1]);
	gAC2 = (s16) ((((u16) b[2]) << 8) | (u16) b[3]);
	gAC3 = (s16) ((((u16) b[4]) << 8) | (u16) b[5]);

	gAC4 = (((u16) b[6]) << 8) | (u16) b[7];
	gAC5 = (((u16) b[8]) << 8) | (u16) b[9];
	gAC6 = (((u16) b[10]) << 8) | (u16) b[11];

	gB1 = (s16) ((((u16) b[12]) << 8) | (u16) b[13]);
	gB2 = (s16) ((((u16) b[14]) << 8) | (u16) b[15]);
	gMb = 0;
	gMc = (s16) ((((u16) b[18]) << 8) | (u16) b[19]);
	gMd = (s16) ((((u16) b[20]) << 8) | (u16) b[21]);

}

void bmp085_InitData(void) {
	bmp085_InitWindowBuffers();
	gnCps = 0;
	gnSmpCnt = BMP085_NUM_Z_SAMPLES - 1;

	bmp085_TriggerTemperatureSample();
	gnSensorState = BMP085_READ_TEMPERATURE;
}

#ifdef SENSOR_UNIT_TEST

// Bosch SensorTec test to validate the code used to compute temperature
// and pressure from pressure sensor readings 

u16 ac5[9] = {32758, 32758, 32758, 30774, 30774, 30774, 26431, 26431, 26431};
u16 ac6[9] = {23153, 23153, 23153, 9469, 9469, 9469, 40099, 40099, 40099};

u32 up[9] = {25827, 23843, 21476, 25634, 23656, 21302, 26060, 24056, 21657};
u32 ut[9] = {24524, 27898, 34787, 9389, 14520, 21853, 40438, 45980, 54518};

s32 pResult[9]= {70369, 69964, 69961, 66140, 69406, 69385, 68346, 70599, 70559};
s16 tResult[9] = {-177, 150, 650, -404, 150, 650, -338, 150, 650};

void bmp085_UnitTest(void) {
	int cnt;
	s16 t;
	s32 p;

	gAC1 = 408;
	gAC2 = -72;
	gAC3 = -14383;
	gAC4 = 32741;
	gB1 = 6190;
	gB2 = 4;
	gMb = 0;
	gMc = -8711;
	gMd = 2868;
	for (cnt = 0; cnt < 9; cnt++) {
		gAC5 = ac5[cnt];
		gAC6 = ac6[cnt];
		gnTraw = ut[cnt];
		gnPraw = up[cnt];
		t = bmp085_CalcTemperatureCx10();
		p = bmp085_CalcPressurePa();
		sprintf(gszBuf,"%d T %d %d P %ld %ld\r\n",cnt, tResult[cnt],t, pResult[cnt], p);
		uart_Print(gszBuf);
	}
}
#endif

void bmp085_InitWindowBuffers() {
	int n;
	bmp085_AcquireAveragedSample(4);
	n = BMP085_NUM_Z_SAMPLES;
	while (n--) {
		gAltZBuf[n] = gnAltCm;
	}
}

void bmp085_TriggerPressureSample(void) {
	i2c_XmtByte(I2C_ID_BMP085, 0xF4, 0x34 + (BMP085_OSS << 6));
}

void bmp085_TriggerTemperatureSample(void) {
	i2c_XmtByte(I2C_ID_BMP085, 0xF4, 0x2E);
}

#define PRESSURE_SAMPLE_WAIT_MS     30
#define TEMPERATURE_SAMPLE_WAIT_MS  30

void bmp085_AcquireAveragedSample(int nSamples) {
	s32 pa, tc, pAccum, tAccum, n;
	pAccum = 0;
	tAccum = 0;
	n = nSamples;
	while (n--) {
		bmp085_TriggerTemperatureSample();
		DELAY_MS(TEMPERATURE_SAMPLE_WAIT_MS);
		gnTraw = bmp085_ReadTemperatureSample();
		tc = bmp085_CalcTemperatureCx10();
		bmp085_TriggerPressureSample();
		DELAY_MS(PRESSURE_SAMPLE_WAIT_MS);
		gnPraw = bmp085_ReadPressureSample();
		pa = bmp085_CalcPressurePa();
		pAccum += pa;
		tAccum += tc;
	}
	tc = tAccum / nSamples;
	gnTempC = (tc >= 0 ? (tc + 5) / 10 : (tc - 5) / 10);
	pa = pAccum / nSamples;
	gnPa = pa;
	gnAltCm = bmp085_Pa2Cm(pa);
	gnAltAvgM = (gnAltCm >= 0 ? (gnAltCm + 50L) / 100L : (gnAltCm - 50L) / 100L);

}

#ifdef SENSOR_NOISE_EVAL

void bmp085_DumpAltitude(int nSamples, int pzInx) {
	s32 pa,tc,n;
	n = nSamples;
	while (n--) {
		bmp085_TriggerTemperatureSample();
		while(!gpiobmp085Eoc);
		gnTraw = bmp085_ReadTemperatureSample();
		tc = bmp085_CalcTemperatureCx10();
		bmp085_TriggerPressureSample();
		while(!gpiobmp085Eoc);
		gnPraw = bmp085_ReadPressureSample();
		pa = bmp085_CalcPressurePa();
		gnTempC = (tc >= 0 ? (tc+5)/10 : (tc-5)/10);
		gnAltCm = bmp085_Pa2Cm(pa, pzInx);

}
#endif

u32 bmp085_ReadTemperatureSample(void) {
	u08 b[2];
	u32 w;
	i2c_RcvBuf(I2C_ID_BMP085, 0xF6, 2, b);
	w = (((u32) b[0]) << 8) | (u32) b[1];
	return w;
}

u32 bmp085_ReadPressureSample(void) {
	u08 b[3];
	u32 dw;
	i2c_RcvBuf(I2C_ID_BMP085, 0xF6, 3, b);
	dw = (((u32) b[0]) << 16) | (((u32) b[1]) << 8) | (u32) b[2];
	dw = (dw >> (8 - BMP085_OSS));
	return dw;
}

s16 bmp085_CalcTemperatureCx10(void) {
	s32 x1, x2;
	s16 t;
	x1 = (((s32) gnTraw - (s32) gAC6) * (s32) gAC5) >> 15;
	x2 = ((s32) gMc << 11) / (x1 + gMd);
	gB5 = x1 + x2;
	t = ((gB5 + 8) >> 4);
	return t;
}

s32 bmp085_CalcPressurePa(void) {
	s32 x1, x2, x3, b3, b6, p;
	u32 b4, b7;

	b6 = gB5 - 4000L;
	x1 = (b6 * b6) >> 12;
	x1 *= gB2;
	x1 >>= 11;

	x2 = (gAC2 * b6);
	x2 >>= 11;
	x3 = x1 + x2;
	b3 = (((((s32) gAC1) * 4 + x3) << BMP085_OSS) + 2) >> 2;

	x1 = (gAC3 * b6) >> 13;
	x2 = (gB1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (gAC4 * (u32) (x3 + 32768L)) >> 15;

	b7 = ((u32) (gnPraw - b3) * (50000L >> BMP085_OSS));
	if (b7 < (u32) 0x80000000) {
		p = (b7 << 1) / b4;
	} else {
		p = (b7 / b4) << 1;
	}

	x1 = p >> 8;
	x1 *= x1;
	x1 = (x1 * 3038L) >> 16;
	x2 = (p * -7357L) >> 16;
	p += ((x1 + x2 + 3791L) >> 4);	// p in Pa  
	return p;
}

void bmp085_AverageAltitude(void) {
	int cnt;
	s32 avg;
	cnt = BMP085_NUM_Z_SAMPLES;
	avg = 0;
	while (cnt--) {
		avg += gAltZBuf[cnt];
	}
	avg /= BMP085_NUM_Z_SAMPLES;
	gnAltAvgCm = avg;
	gnAltAvgM = ((avg >= 0) ? (avg + 50L) / 100L : (avg - 50L) / 100L);
	return;
}

void bmp085_DeltaCmPerSec(void) {
	int n, trel;
	s32 z, sum_zt;

	sum_zt = 0;
	n = BMP085_NUM_Z_SAMPLES;
	while (n--) {
		z = gAltZBuf[n] - gnAltAvgCm;
		trel = n - gnSmpCnt;	// time origin is the current sample in window
		if (n > gnSmpCnt) {
			trel -= BMP085_NUM_Z_SAMPLES;
		}
		sum_zt += ((s32) trel * z);
	}
	gnCps = (sum_zt * (s32) (BMP085_SAMPLES_PER_SECX10 * BMP085_NUM_Z_SAMPLES))
			/ (10 * gnDen);
	return;
}

#include "pztbl.txt"

s32 bmp085_Pa2Cm(s32 pa) {
	s32 inx, pa1, z1, z2, z;
	if (pa > PA_INIT) {
		z = gPZTbl[0];
	} else {
		inx = (PA_INIT - pa) >> 10;
		if (inx >= PZLUT_ENTRIES - 1) {
			z = gPZTbl[PZLUT_ENTRIES - 1];
		} else {
			pa1 = PA_INIT - (inx << 10);
			z1 = gPZTbl[inx];
			z2 = gPZTbl[inx + 1];
			z = z1 + (((pa1 - pa) * (z2 - z1)) >> 10);
		}
	}
	return z;
}

