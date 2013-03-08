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
#include "tmrsys.h"
#include "fpu.h"
#include "i2c_IMU.h"
#include "imu.h"
#include "adxl345.h"
#include "l3g4200d.h"
#include "hmc5883l.h"
#include "bmp085.h"
#include "stdio_console.h"
#include "utils/uartstdio.h"
#include "driverlib/systick.h"


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, unsigned long ulLine) {
}
#endif

#define MAX_CALIB_SAMPLES   100

int gXBuf[MAX_CALIB_SAMPLES];
int gYBuf[MAX_CALIB_SAMPLES];
int gZBuf[MAX_CALIB_SAMPLES];

int gAXRawAvg, gAYRawAvg, gAZRawAvg;
int gGXRawAvg, gGYRawAvg, gGZRawAvg;
int gMXRawAvg, gMYRawAvg, gMZRawAvg;
float gAXC, gAYC, gAZC;
float gGXC, gGYC, gGZC;
float gMXC, gMYC, gMZC;

#define MAX_AVG_SAMPLES 8
#define NUM_AVG_SAMPLES 4

int gAXBuf[MAX_AVG_SAMPLES];
int gAYBuf[MAX_AVG_SAMPLES];
int gAZBuf[MAX_AVG_SAMPLES];
int gMXBuf[MAX_AVG_SAMPLES];
int gMYBuf[MAX_AVG_SAMPLES];
int gMZBuf[MAX_AVG_SAMPLES];
int gGXBuf[MAX_AVG_SAMPLES];
int gGYBuf[MAX_AVG_SAMPLES];
int gGZBuf[MAX_AVG_SAMPLES];

float gQuaternion[4];
float gYPR[3];
float gYPRref[3];

long gnPA;

volatile int xcounter = 0;
volatile int ycounter = 0;

void util_FillAveragingBuffers(void);
void ui_SetDefaultUserParams(void);
void ui_ADXL345Calibrate(void);
void ui_HMC5883LCalibrate(void);
void ui_L3GCalibrate(void);
void mcu_Config(void);
void ui_ADXL345Calibrate(void);
void ui_HMC5883LCalibrate(void);
void ui_L3GCalibrate(void);
void ui_printframe(void);
int util_WaitBtnPressTimeout(int seconds);

uint8_t ftoa(float f, uint8_t numchar, char *buf);

int main(void) {

	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	//
	ROM_FPULazyStackingEnable ();

	//
	// Set the clocking to run directly from the crystal.
	// 16Mhz XTAL --> PLL = 400Mhz --> /2 --> SysDIV_2_5 = 80Mhz.,
	//
	ROM_SysCtlClockSet (
			SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ
					| SYSCTL_OSC_MAIN);

	/*
	 * System configs.
	 */
	FPUEnable();				// FPU disabled by default.
	tmrsys_Config();//Start SYSTIK timer and buttons, using systik for denounce.
	InitConsole();				//Init BT UART Console
	i2c_Config();				// Setup i2c -- i2c_port 1

	LED_INIT()
	;					// Setup launchpad LEDs

	UARTprintf("\n\nGY-80 Initialising.\n");
	UARTprintf("Checking Sensors on i2c bus.\n\n");

	/*
	 * Ping i2c device to make sure they are alive.
	 */
	UARTprintf("L34200D (3-Axis gyroscope) : ");
	if (i2c_RcvByte(I2C_ID_L3G4200D, L3G_WHO_AM_I) == 0xD3) {
		UARTprintf("ok\n");
	} else {
		UARTprintf("missing\n");
	}

	UARTprintf("ADXL345 (3-Axis Accelerometer) : ");
	if (i2c_RcvByte(I2C_ID_ADXL345, ADXL345_DEVID) == 0xE5) {
		UARTprintf("ok\n");
	} else {
		UARTprintf("missing\n");
	}

	UARTprintf("HMC5883L (3-Axis Digital Compass) : ");
	if (i2c_RcvByte(I2C_ID_HMC5883L, HMC5883L_DEVID) == 0x3C) {
		UARTprintf("ok\n");
	} else {
		UARTprintf("missing\n");
	}

	UARTprintf("BMP085 (Barometric Pressure Sensor) : ");

	if (i2c_RcvByte(I2C_ID_BMP085, BMP085_DEVID) == 0x55) {
		UARTprintf("ok\n");
	} else {
		UARTprintf("missing\n");
	}
	UARTprintf("%c[?25l", ASCII_ESC);

	DELAY_MS(1000);
	/*
	 * Print screen frame
	 */
	ui_printframe();
	/*
	 * IMU routines
	 */

	float IMU_Heading;

	char stringfloat1[10];
	char stringfloat2[10];
	char stringfloat3[10];
	char stringfloat4[10];

	int lx, ly, lz;
	int smpCnt = 0;

	ui_SetDefaultUserParams();		//Load default calibrations
	imu_Init();						//Setup all sensors Acc, Gryo, Mag, Alt

	util_FillAveragingBuffers();	//Fill Averaging Buffer with Data

	while (1) {
		if (gbSysTickFlag) {								//trigger every 10ms
			gbSysTickFlag = 0;

//			LED_TOGGLE(RED);

			bmp085_AcquireAveragedSample(4);				//Average 4 sample.
			bmp085_AverageAltitude();

			UARTprintf("%c[4;0H", ASCII_ESC);

			ftoa((float) (gnAltCm / 100.0), 4, (char*) stringfloat1);
			UARTprintf("|    %2d*C     |   %sm    |  %08dpa  |             |",
					gnTempC, stringfloat1, gnPa);

			hmc5883l_ReadXYZRawData(&lx, &ly, &lz);
			IMU_Heading = hmc5883l_GetHeadingDeg(lx, ly, 0.0457);

			UARTprintf("%c[8;0H", ASCII_ESC);
			ftoa(IMU_Heading, 4, (char*) stringfloat4);
			UARTprintf("|   %05d     |    %05d    |    %05d     |   %s*   |",
					lx, ly, lz, stringfloat4);

			UARTprintf("%c[12;0H", ASCII_ESC);
			adxl345_ReadXYZRawData(&lx, &ly, &lz);
			UARTprintf(
					"|   %05d     |    %05d    |    %05d     |             |",
					lx, ly, lz);

			UARTprintf("%c[16;0H", ASCII_ESC);

			l3g_ReadXYZRawData(&lx, &ly, &lz);
			UARTprintf(
					"|   %05d     |    %05d    |    %05d     |             |",
					lx, ly, lz);

			UARTprintf("%c[20;0H", ASCII_ESC);

			/*
			 * Sensor fusion stuff
			 */

			adxl345_ReadXYZRawData(&gAXBuf[smpCnt], &gAYBuf[smpCnt],
					&gAZBuf[smpCnt]);
			l3g_ReadXYZRawData(&gGXBuf[smpCnt], &gGYBuf[smpCnt],
					&gGZBuf[smpCnt]);
			hmc5883l_ReadXYZRawData(&gMXBuf[smpCnt], &gMYBuf[smpCnt],
					&gMZBuf[smpCnt]);

			gAXRawAvg = util_AverageSamples(gAXBuf, NUM_AVG_SAMPLES);
			gAYRawAvg = util_AverageSamples(gAYBuf, NUM_AVG_SAMPLES);
			gAZRawAvg = util_AverageSamples(gAZBuf, NUM_AVG_SAMPLES);

			gGXRawAvg = util_AverageSamples(gGXBuf, NUM_AVG_SAMPLES);
			gGYRawAvg = util_AverageSamples(gGYBuf, NUM_AVG_SAMPLES);
			gGZRawAvg = util_AverageSamples(gGZBuf, NUM_AVG_SAMPLES);

			gMXRawAvg = util_AverageSamples(gMXBuf, NUM_AVG_SAMPLES);
			gMYRawAvg = util_AverageSamples(gMYBuf, NUM_AVG_SAMPLES);
			gMZRawAvg = util_AverageSamples(gMZBuf, NUM_AVG_SAMPLES);

			adxl345_GetCorrectedData(gAXRawAvg, gAYRawAvg, gAZRawAvg, &gAXC,
					&gAYC, &gAZC);
			l3g_GetCorrectedData(gGXRawAvg, gGYRawAvg, gGZRawAvg, &gGXC, &gGYC,
					&gGZC);
			hmc5883l_GetCorrectedData(gMXRawAvg, gMYRawAvg, gMZRawAvg, &gMXC,
					&gMYC, &gMZC);

			imu_UpdateData(gAXC, gAYC, gAZC, gGXC, gGYC, gGZC, gMXC, gMYC,
					gMZC);
			imu_GetYawPitchRoll(gYPR);

			/*
			 *
			 */

			ftoa(gYPR[0], 5, (char *) stringfloat1);
			ftoa(gYPR[1], 5, (char *) stringfloat2);
			ftoa(gYPR[2], 5, (char *) stringfloat3);
			UARTprintf("|   %s    |    %s   |     %s    |             |",
					stringfloat1, stringfloat2, stringfloat3);
			smpCnt++;

			if (smpCnt == NUM_AVG_SAMPLES)
				smpCnt = 0;

			if (gbBtnPressed) {

				UARTprintf("%c[2J", ASCII_ESC);
				UARTprintf("%c[H", ASCII_ESC);

				UARTprintf(
						"To calibrate accelerometer, press button within 5 seconds!\r\n");
				if (util_WaitBtnPressTimeout(5)) {
					ui_ADXL345Calibrate();
				}

				UARTprintf(
						"To calibrate compass, press button within 5 seconds!\r\n");
				if (util_WaitBtnPressTimeout(5)) {
					ui_HMC5883LCalibrate();
				}

				UARTprintf(
						"To calibrate gyroscope, press button within 5 seconds!\r\n");
				if (util_WaitBtnPressTimeout(5)) {
					ui_L3GCalibrate();
				}
				SysTickIntDisable();
				DELAY_MS(3000);
				UARTprintf("%c[2J", ASCII_ESC);
				ui_printframe();
				gbBtnPressed = 0;
				SysTickIntEnable();

			}
//			LED(BLUE, off);
//			LED(GREEN, off);

		}
	}

}

void util_FillAveragingBuffers(void) {
	int x, y, z, cnt;

	adxl345_GetAveragedRawData(4, &x, &y, &z);
	for (cnt = 0; cnt < NUM_AVG_SAMPLES; cnt++) {
		gAXBuf[cnt] = x;
		gAYBuf[cnt] = y;
		gAZBuf[cnt] = z;
	}
	l3g_GetAveragedRawData(4, &x, &y, &z);
	for (cnt = 0; cnt < NUM_AVG_SAMPLES; cnt++) {
		gGXBuf[cnt] = x;
		gGYBuf[cnt] = y;
		gGZBuf[cnt] = z;
	}
	hmc5883l_GetAveragedRawData(4, &x, &y, &z);
	for (cnt = 0; cnt < NUM_AVG_SAMPLES; cnt++) {
		gMXBuf[cnt] = x;
		gMYBuf[cnt] = y;
		gMZBuf[cnt] = z;
	}
}

void ui_SetDefaultUserParams(void) {

	gADXL345.calib.x0g = 0;
	gADXL345.calib.y0g = 0;
	gADXL345.calib.xp1g = 160;
	gADXL345.calib.xm1g = -300;
	gADXL345.calib.yp1g = 41;
	gADXL345.calib.ym1g = -587;
	gADXL345.calib.zp1g = 1316;
	gADXL345.calib.zm1g = 732;

	gHMC5883L.calib.xMax = 806;
	gHMC5883L.calib.xMin = -460;
	gHMC5883L.calib.yMax = 402;
	gHMC5883L.calib.yMin = -937;
	gHMC5883L.calib.zMax = 669;
	gHMC5883L.calib.zMin = -624;

	gHMC5883L.xRange = gHMC5883L.calib.xMax - gHMC5883L.calib.xMin;
	gHMC5883L.yRange = gHMC5883L.calib.yMax - gHMC5883L.calib.yMin;
	gHMC5883L.zRange = gHMC5883L.calib.zMax - gHMC5883L.calib.zMin;

	gL3G.calib.xOffset = 19;
	gL3G.calib.xOffsetSigma = 9;
	gL3G.calib.yOffset = 12;
	gL3G.calib.yOffsetSigma = 9;
	gL3G.calib.zOffset = 14;
	gL3G.calib.zOffsetSigma = 9;

	gL3G.xThreshold = 3 * gL3G.calib.xOffsetSigma;
	gL3G.yThreshold = 3 * gL3G.calib.yOffsetSigma;
	gL3G.zThreshold = 3 * gL3G.calib.zOffsetSigma;

}

//*****************************************************************************
//
//! \brief Float to ASCII
//!
//! Converts a floating point number to ASCII. Note that buf must be
//! large enough to hold
//!
//! \param f is the floating point number.
//! \param buf is the buffer in which the resulting string is placed.
//! \return None.
//!
//! \par Example:
//! ftoa(3.14) returns "3.14"
//!
//
//*****************************************************************************
uint8_t ftoa(float f, uint8_t decPlaces, char *buf) {

	int pos = 0, ix, dp, num, total;
	if (f < 0) {
		buf[pos++] = '-';
		f = -f;
	}
	dp = 0;
	while (f >= 10.0) {
		f = f / 10.0;
		dp++;
	}

	total = dp + decPlaces;

	for (ix = 1; ix < total; ix++) {
		num = (int) f;
		f = f - num;
		if (num > 9)
			buf[pos++] = '#';
		else
			buf[pos++] = '0' + num;
		if (dp == 0)
			buf[pos++] = '.';
		f = f * 10.0;
		dp--;
	}

	buf[pos++] = 0;  // null

	return pos;
}

// Ensure the OFSTX,Y,Z registers are set to 0
// Step A
// Place each axis in +1g and -1g orientation and get the test axis
// raw 13bit 2's complement averaged readings
// (total 6 readings, 2 per axis)
// The difference of the +1g and -1g readings divided by 2 is the axis sensitivity in LSB/g
// The inverse of the sensitivity is the axis scale factor in g's

// Step B
// Place board horizontally and get the X,Y and Z averaged readings
// The X and Y readings are in 0g, therefore these are the 0g offsets. Subtract them from
// the X and Y raw readigs to get the corrected X and Y readings
// The Z reading is in +1g, so subtract the Z sensitivity (LSb/g) found in Step A
// to get the Z axis 0g offset.
// Subtract the Z axis 0g offset from the Z raw reading to get the
// corrected Z reading.
//
// To get the acceleration in g's, multiply the (0g corrected) sensor output by the scale
// To get the acceleration in cm/s*s, multiply the g's by by 980cm/s*s

void ui_ADXL345Calibrate(void) {

	int x, y, z;
	UARTprintf("Orient board Z axis UP and hit C\r\n");
	while (!gbBtnPressed)
		;;
	gbBtnPressed = 0;
	adxl345_GetAveragedRawData(10, &x, &y, &z);
	gADXL345.calib.x0g = x;
	gADXL345.calib.y0g = y;
	gADXL345.calib.zp1g = z;

	UARTprintf("Orient board Z axis DOWN and hit C\r\n");
	while (!gbBtnPressed)
		;
	gbBtnPressed = 0;
	adxl345_GetAveragedRawData(10, &x, &y, &z);
	gADXL345.calib.zm1g = z;

	UARTprintf("Orient board X axis UP and hit C\r\n");
	while (!gbBtnPressed)
		;
	gbBtnPressed = 0;
	adxl345_GetAveragedRawData(10, &x, &y, &z);
	gADXL345.calib.xp1g = x;

	UARTprintf("Orient board X axis DOWN and hit C\r\n");
	while (!gbBtnPressed)
		;
	gbBtnPressed = 0;
	adxl345_GetAveragedRawData(10, &x, &y, &z);
	gADXL345.calib.xm1g = x;

	UARTprintf("Orient board Y axis UP and hit C\r\n");
	while (!gbBtnPressed)
		;
	gbBtnPressed = 0;
	adxl345_GetAveragedRawData(10, &x, &y, &z);
	gADXL345.calib.yp1g = y;

	UARTprintf("Orient board Y axis DOWN and hit C\r\n");
	while (!gbBtnPressed)
		;
	gbBtnPressed = 0;
	adxl345_GetAveragedRawData(10, &x, &y, &z);
	gADXL345.calib.ym1g = y;

	UARTprintf("ADXL345 Accelerometer Calibration Data\r\n");
	UARTprintf(" x0g = %d\r\n", gADXL345.calib.x0g);
	UARTprintf(" y0g = %d\r\n", gADXL345.calib.y0g);
	UARTprintf(" xp1g = %d\r\n", gADXL345.calib.xp1g);
	UARTprintf(" xm1g = %d\r\n", gADXL345.calib.xm1g);
	UARTprintf(" yp1g = %d\r\n", gADXL345.calib.yp1g);
	UARTprintf(" ym1g = %d\r\n", gADXL345.calib.ym1g);
	UARTprintf(" zp1g = %d\r\n", gADXL345.calib.zp1g);
	UARTprintf(" zm1g = %d\r\n", gADXL345.calib.zm1g);

}

void ui_HMC5883LCalibrate(void) {
	UARTprintf(
			"Orient in all directions slowly. If you know roughly where north is, try to point\r\n");
	UARTprintf(
			"each axis (+ and -) of the board towards north and shift the board slightly in all directions\r\n");
	UARTprintf(
			"around this approximate position. The goal is to get the max and min values for each axis\r\n");
	UARTprintf("Press C when ready to start\r\n");
	while (!gbBtnPressed)
		;;
	gbBtnPressed = 0;
	UARTprintf(
			"Calibrating, Press C after you are done orienting the board with all axes ...\r\n");

	gHMC5883L.calib.xMax = gHMC5883L.calib.yMax = gHMC5883L.calib.zMax = -32768;
	gHMC5883L.calib.xMin = gHMC5883L.calib.yMin = gHMC5883L.calib.zMin = 32767;

	while (!gbBtnPressed) {
		hmc5883l_ReadXYZRawData(&gMXRawAvg, &gMYRawAvg, &gMZRawAvg);
		if (gMXRawAvg > gHMC5883L.calib.xMax)
			gHMC5883L.calib.xMax = gMXRawAvg;
		if (gMYRawAvg > gHMC5883L.calib.yMax)
			gHMC5883L.calib.yMax = gMYRawAvg;
		if (gMZRawAvg > gHMC5883L.calib.zMax)
			gHMC5883L.calib.zMax = gMZRawAvg;
		if (gMXRawAvg < gHMC5883L.calib.xMin)
			gHMC5883L.calib.xMin = gMXRawAvg;
		if (gMYRawAvg < gHMC5883L.calib.yMin)
			gHMC5883L.calib.yMin = gMYRawAvg;
		if (gMZRawAvg < gHMC5883L.calib.zMin)
			gHMC5883L.calib.zMin = gMZRawAvg;
		DELAY_MS(HMC5883L_MEAS_DELAY_MS);
	}
	gbBtnPressed = 0;
	gHMC5883L.xRange = gHMC5883L.calib.xMax - gHMC5883L.calib.xMin;
	gHMC5883L.yRange = gHMC5883L.calib.yMax - gHMC5883L.calib.yMin;
	gHMC5883L.zRange = gHMC5883L.calib.zMax - gHMC5883L.calib.zMin;

	UARTprintf("HMC5883L Compass Calibration Data\r\n");
	UARTprintf(" xMax = %d xMin = %d\r\n", gHMC5883L.calib.xMax,
			gHMC5883L.calib.xMin);
	UARTprintf(" yMax = %d yMin = %d\r\n", gHMC5883L.calib.yMax,
			gHMC5883L.calib.yMin);
	UARTprintf(" zMax = %d zMin = %d\r\n\r\n", gHMC5883L.calib.zMax,
			gHMC5883L.calib.zMin);

}

void ui_L3GCalibrate(void) {
	UARTprintf(
			"Orient board in neutral position and keep it still until calibration is complete\r\n");
	UARTprintf("Press C when ready to start calibration\r\n");
	while (!gbBtnPressed)
		;
	gbBtnPressed = 0;
	DELAY_MS(2000);
	l3g_GetCalibStatsRawData(100, &gL3G.calib.xOffset, &gL3G.calib.yOffset,
			&gL3G.calib.zOffset, &gL3G.calib.xOffsetSigma,
			&gL3G.calib.yOffsetSigma, &gL3G.calib.zOffsetSigma);
	gL3G.xThreshold = 3 * gL3G.calib.xOffsetSigma;
	gL3G.yThreshold = 3 * gL3G.calib.yOffsetSigma;
	gL3G.zThreshold = 3 * gL3G.calib.zOffsetSigma;

	UARTprintf("L3G4200D Gyroscope Calibration Data\r\n");
	UARTprintf(" xOffset = %d xOffsetSigma = %d\r\n", gL3G.calib.xOffset,
			gL3G.calib.xOffsetSigma);
	UARTprintf(" yOffset = %d yOffsetSigma = %d\r\n", gL3G.calib.yOffset,
			gL3G.calib.yOffsetSigma);
	UARTprintf(" zOffset = %d zOffsetSigma = %d\r\n\r\n", gL3G.calib.zOffset,
			gL3G.calib.zOffsetSigma);

}

void ui_printframe(void) {

	UARTprintf("%c[2J", ASCII_ESC);
	UARTprintf("%c[H", ASCII_ESC);
	UARTprintf("\nBMP085 Barometer Readings ");
	UARTprintf("\n| Temperature |  Altitude   |   Pressure   |             |");

	UARTprintf("%c[6;0H", ASCII_ESC);
	UARTprintf("HMC5883L Compass Readings");
	UARTprintf("\n|   X-Raw     |    Y-Raw    |    Z-Raw     |   Heading   |");

	UARTprintf("%c[10;0H", ASCII_ESC);
	UARTprintf("ADXL345 Accelerometer Readings");
	UARTprintf("\n|   X-Axis    |    Y-Axis   |    Z-Axis    |             |");

	UARTprintf("%c[14;0H", ASCII_ESC);
	UARTprintf("L3G4200D Gryoscope Readings");
	UARTprintf("\n|   X-Axis    |    Y-Axis   |    Z-Axis    |             |");

	UARTprintf("%c[18;0H", ASCII_ESC);
	UARTprintf("IMU fusion");
	UARTprintf("\n|    Roll     |    Pitch    |     Yaw      |   Heading   |");

}

int util_WaitBtnPressTimeout(int seconds) {
	u32 tick, delayTicks;
	delayTicks = (seconds * 1000) / TMRSYS_TICK_MS;
	tick = gnSysTick;
	gbBtnPressed = 0;
	while ((!gbBtnPressed) && ((gnSysTick - tick) < delayTicks))
		;;
	if (gbBtnPressed) {
		gbBtnPressed = 0;
		return 1;
	} else {
		return 0;
	}
}
