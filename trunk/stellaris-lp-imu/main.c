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
#include "i2c_IMU.h"
#include "imu.h"
#include "adxl345.h"
#include "l3g4200d.h"
#include "hmc5883l.h"
#include "bmp085.h"
#include "stdio_console.h"
#include "utils/uartstdio.h"
#include "driverlib/systick.h"
#include "driverlib/eeprom.h"

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, unsigned long ulLine) {
}
#endif

float gAXC, gAYC, gAZC;
float gGXC, gGYC, gGZC;
float gMXC, gMYC, gMZC;

short gAXRaw, gAYRaw, gAZRaw;
short gMXRaw, gMYRaw, gMZRaw;
short gGXRaw, gGYRaw, gGZRaw;

float gQuaternion[4];
float gYPR[3];

long gnPA;

volatile int xcounter = 0;
volatile int ycounter = 0;

typedef struct {
	ADXL345_CALIB_DATA adxl345_calib;
	HMC5883L_CALIB_DATA hmc5883L_calib;
	L3G_CALIB_DATA l3g4200d_calib;
} EE_STORE;

EE_STORE gNvdBuf;

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
void ui_ReadUserParams(void);
int ui_Check_Params(void);
void ui_ReadUserParams(void);
int ui_WriteUserParams(void);

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
	tmrsys_Config();			//Start SYSTIK timer and buttons, using systik for denounce.
	InitConsole();				//Init BT UART Console
	i2c_Config();				// Setup i2c -- i2c_port 1

	ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_EEPROM0);
	EEPROMInit();

//	EEPROMMassErase();

	LED_INIT();					// Setup launchpad LEDs

	UARTprintf("\n\nGY-80 Initialising.\n");
	UARTprintf("Checking Sensors on i2c bus.\n\n");

	// Ping i2c device to make sure they are alive.

#if DEBUG

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


#endif


	if (ui_Check_Params()) {
		ui_ReadUserParams();
		LED(RED, off);
	} else {
		ui_SetDefaultUserParams();
		UARTprintf("corrupted Calibration data in EEPROM!\n");
		LED(RED, on);
	}

#if DEBUG
	/*
	 * Print screen frame
	 */
	ui_printframe();

#endif

	/*
	 * IMU routines
	 */

	char stringfloat1[10];
	char stringfloat2[10];
	char stringfloat3[10];

	imu_Init();						//Setup all sensors Acc, Gryo, Mag, Alt

	while (1) {
		if (gbSysTickFlag) {								//trigger every 10ms
			gbSysTickFlag = 0;

			/*
			 * Sensor fusion stuff
			 */

			adxl345_ReadXYZRawData(&gAXRaw, &gAYRaw, &gAZRaw);					// get Raw Acc Data
			l3g_ReadXYZRawData(&gGXRaw, &gGYRaw, &gGZRaw);						// get Raw Gyro Data
			hmc5883l_ReadXYZRawData(&gMXRaw, &gMYRaw, &gMZRaw);					// get Raw Magn Data


			adxl345_GetCorrectedData(gAXRaw, gAYRaw, gAZRaw, &gAXC, &gAYC, &gAZC);    	//update with calibration Data
			l3g_GetCorrectedData(gGXRaw, gGYRaw, gGZRaw, &gGXC, &gGYC, &gGZC);			//update with calibration Data
			hmc5883l_GetCorrectedData(gMXRaw, gMYRaw, gMZRaw, &gMXC, &gMYC, &gMZC);		//update with calibration Data

			imu_UpdateData(gAXC, gAYC, gAZC, gGXC, gGYC, gGZC, gMXC, gMYC, gMZC);		//update realdata.

	//		imu_UpdateData(0, gAYC, 0, 0, gGYC, 0, 0, gMYC, 0);		//update realdata.

//			imu_GetEuler(gYPR);
			imu_GetYawPitchRoll(gYPR);

#if DEBUG

	        float IMU_Heading;
	        short lx, ly, lz;
	        char stringfloat4[10];

            bmp085_AcquireAveragedSample(4);                                //Average 4 sample.
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

			ftoa(gYPR[0], 4, (char *) stringfloat1);
            ftoa(gYPR[1], 4, (char *) stringfloat2);
            ftoa(gYPR[2], 4, (char *) stringfloat3);
            UARTprintf("|   %s    |    %s   |     %s    |             |",
                            stringfloat1, stringfloat2, stringfloat3);
#else
			ftoa(gYPR[0], 4, (char *) stringfloat1);
			ftoa(gYPR[1], 4, (char *) stringfloat2);
			ftoa(gYPR[2], 4, (char *) stringfloat3);
//			ftoa(0, 4, (char *) stringfloat1);
//			ftoa(0, 4, (char *) stringfloat3);

			UARTprintf("%s,%s,%s!", stringfloat3, stringfloat2, stringfloat1);		//send out roll, pitch, yaw!
#endif


			if (gbBtnPressed) {

				UARTprintf("%c[2J", ASCII_ESC);
				UARTprintf("%c[H", ASCII_ESC);

				UARTprintf(
						"To calibrate accelerometer, press button within 3 seconds!\r\n");
				if (util_WaitBtnPressTimeout(3)) {
					ui_ADXL345Calibrate();
				}

				UARTprintf(
						"To calibrate compass, press button within 3 seconds!\r\n");
				if (util_WaitBtnPressTimeout(3)) {
					ui_HMC5883LCalibrate();
				}

				UARTprintf(
						"To calibrate gyroscope, press button within 3 seconds!\r\n");
				if (util_WaitBtnPressTimeout(3)) {
					ui_L3GCalibrate();
				}
				SysTickIntDisable();

				ui_WriteUserParams();
				DELAY_MS(1000);

				UARTprintf("%c[2J", ASCII_ESC);
				ui_printframe();
				gbBtnPressed = 0;

				SysTickIntEnable();

			}

		}
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
		if (dp > 10)
			f = 0;
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

	short x, y, z;
	UARTprintf("Orient board Z axis UP and hit C\r\n");
	while (!gbBtnPressed)
		;
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
		hmc5883l_ReadXYZRawData(&gMXRaw, &gMYRaw, &gMZRaw);

		if (gMXRaw > gHMC5883L.calib.xMax)
			gHMC5883L.calib.xMax = gMXRaw;
		if (gMYRaw > gHMC5883L.calib.yMax)
			gHMC5883L.calib.yMax = gMYRaw;
		if (gMZRaw > gHMC5883L.calib.zMax)
			gHMC5883L.calib.zMax = gMZRaw;
		if (gMXRaw < gHMC5883L.calib.xMin)
			gHMC5883L.calib.xMin = gMXRaw;
		if (gMYRaw < gHMC5883L.calib.yMin)
			gHMC5883L.calib.yMin = gMYRaw;
		if (gMZRaw < gHMC5883L.calib.zMin)
			gHMC5883L.calib.zMin = gMZRaw;
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
	delayTicks = (seconds * 1000000);   // ticks in uS.
	tick = sys_us;
	gbBtnPressed = 0;
	while ((!gbBtnPressed) && ((sys_us - tick) < delayTicks))
		;;
	if (gbBtnPressed) {
		gbBtnPressed = 0;
		return 1;
	} else {
		return 0;
	}
}

int ui_Check_Params(void) {
	unsigned long EE_CRC, CURRENT_CRC;
	EE_CRC = (sizeof(gNvdBuf) / 2);
	EEPROMRead((unsigned long*) &(gNvdBuf.adxl345_calib.x0g),
			EEPROMAddrFromBlock(1), sizeof(gNvdBuf));
	EEPROMRead(&EE_CRC, EEPROMAddrFromBlock(1) + sizeof(gNvdBuf),
			sizeof(EE_CRC));
	CURRENT_CRC = ROM_Crc16Array ((sizeof(gNvdBuf) / 4),
			(unsigned long*) &(gNvdBuf.adxl345_calib.x0g));
	if (CURRENT_CRC != EE_CRC)
		return 0;
	return 1;
}

void ui_ReadUserParams(void) {
	util_MemCpy((u08*) &(gADXL345.calib.x0g), (u08*) &(gNvdBuf.adxl345_calib),
			sizeof(gADXL345.calib));

	util_MemCpy((u08*) &(gHMC5883L.calib.xMax),
			(u08*) &(gNvdBuf.hmc5883L_calib), sizeof(gHMC5883L.calib));

	gHMC5883L.xRange = gHMC5883L.calib.xMax - gHMC5883L.calib.xMin;
	gHMC5883L.yRange = gHMC5883L.calib.yMax - gHMC5883L.calib.yMin;
	gHMC5883L.zRange = gHMC5883L.calib.zMax - gHMC5883L.calib.zMin;

	util_MemCpy((u08*) &(gL3G.calib.xOffset), (u08*) &(gNvdBuf.l3g4200d_calib),
			sizeof(gL3G.calib));

	gL3G.xThreshold = 3 * gL3G.calib.xOffsetSigma;
	gL3G.yThreshold = 3 * gL3G.calib.yOffsetSigma;
	gL3G.zThreshold = 3 * gL3G.calib.zOffsetSigma;

	gADXL345.xSens = (gADXL345.calib.xp1g - gADXL345.calib.xm1g) / 2;
	gADXL345.ySens = (gADXL345.calib.yp1g - gADXL345.calib.ym1g) / 2;
	gADXL345.zSens = (gADXL345.calib.zp1g - gADXL345.calib.zm1g) / 2;

	gADXL345.z0g = gADXL345.calib.zp1g - gADXL345.zSens;
	gADXL345.x2g = 10000L / (s32) gADXL345.xSens;
	gADXL345.y2g = 10000L / (s32) gADXL345.ySens;
	gADXL345.z2g = 10000L / (s32) gADXL345.zSens;
}

int ui_WriteUserParams(void) {
	unsigned long CURRENT_CRC;
	util_MemCpy((u08*) &(gNvdBuf), (u08*) &(gADXL345.calib.x0g),
			sizeof(gADXL345.calib));
	util_MemCpy((u08*) &gNvdBuf + sizeof(gADXL345.calib),
			(u08*) &(gHMC5883L.calib.xMax), sizeof(gHMC5883L.calib));
	util_MemCpy(
			(u08*) &gNvdBuf + sizeof(gADXL345.calib) + sizeof(gHMC5883L.calib),
			(u08*) &(gL3G.calib.xOffset), sizeof(gL3G.calib));

	EEPROMProgram((unsigned long*) &(gNvdBuf.adxl345_calib.x0g),
			EEPROMAddrFromBlock(1), (unsigned long) sizeof(gNvdBuf));
	CURRENT_CRC = ROM_Crc16Array ((sizeof(gNvdBuf) / 4),
			(unsigned long*) &(gNvdBuf));
	EEPROMProgram(&CURRENT_CRC, EEPROMAddrFromBlock(1) + sizeof(gNvdBuf), 4);

	if (ui_Check_Params()) {
		UARTprintf("\nCalibration Parameters saved");
		LED(RED, off);
		return 1;
	};

	UARTprintf("\nWrite fail, Calibration not saved!\n");
	LED(RED, 1);
	return 0;

}
