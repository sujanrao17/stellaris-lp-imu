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
#include "i2c_IMU.h"
#include "adxl345.h"
#include <math.h>

ADXL345_DATA gADXL345;
extern int gXBuf[];
extern int gYBuf[];
extern int gZBuf[];

void adxl345_Config(void) {
	adxl345_PowerOn();
	adxl345_BypassFifo();
	i2c_XmtByte(I2C_ID_ADXL345, ADXL345_DATA_FORMAT, 0);
	adxl345_SetRangeSetting(16);
	adxl345_SetFullResBit(1);
	adxl345_SetAxisOffset(0, 0, 0);
}

void adxl345_BypassFifo(void) {
	u08 b;
	b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_FIFO_CTL);
	b &= 0x3F;
	i2c_XmtByte(I2C_ID_ADXL345, ADXL345_FIFO_CTL, b);
}

void adxl345_PowerOn(void) {
	//i2c_XmtByte(I2C_ID_ADXL345, ADXL345_POWER_CTL, 0);
	//i2c_XmtByte(I2C_ID_ADXL345, ADXL345_POWER_CTL, 16);
	i2c_XmtByte(I2C_ID_ADXL345, ADXL345_POWER_CTL, 8);
}

// Reads the raw data into three variable x, y and z
void adxl345_ReadXYZRawData(int *pxraw, int* pyraw, int* pzraw) {
	u08 buf[6];
	s16 x, y, z;
	i2c_RcvBuf(I2C_ID_ADXL345, ADXL345_DATAX0, 6, buf); //read the acceleration data from the ADXL345
	// each axis reading comes in 13 bit 2's complement format. lsb first, msb has sign bits extended
	x = (s16) ((((u16) buf[1]) << 8) | (u16) buf[0]);
	*pxraw = (int) x;
	y = (s16) ((((u16) buf[3]) << 8) | (u16) buf[2]);
	*pyraw = (int) y;
	z = (s16) ((((u16) buf[5]) << 8) | (u16) buf[4]);
	*pzraw = (int) z;
}

void adxl345_CalcXYZGData(int xraw, int yraw, int zraw, int* pgx, int * pgy,
		int* pgz) {
	*pgx = (((xraw - gADXL345.calib.x0g) * gADXL345.x2g * 98L) / 1000L);
	*pgy = (((yraw - gADXL345.calib.y0g) * gADXL345.x2g * 98L) / 1000L);
	*pgz = (((zraw - gADXL345.z0g) * gADXL345.z2g * 98L) / 1000L);
}

void adxl345_GetCorrectedData(int ax, int ay, int az, float* pacx, float * pacy,
		float* pacz) {
	*pacx = (float) (ax - gADXL345.calib.x0g) / (float) gADXL345.xSens;
	*pacy = (float) (ay - gADXL345.calib.y0g) / (float) gADXL345.ySens;
	*pacz = (float) (az - gADXL345.z0g) / (float) gADXL345.zSens;
}

void adxl345_GetAveragedRawData(int numSamples, int* pXavg, int* pYavg,
		int* pZavg) {
	int cnt;
	for (cnt = 0; cnt < numSamples; cnt++) {
		adxl345_ReadXYZRawData(&gXBuf[cnt], &gYBuf[cnt], &gZBuf[cnt]);
		DELAY_MS(ADXL345_SAMPLE_DELAY_MS)
	}
	*pXavg = util_AverageSamples(gXBuf, numSamples);
	*pYavg = util_AverageSamples(gYBuf, numSamples);
	*pZavg = util_AverageSamples(gZBuf, numSamples);
}

// Gets the range setting and return it into rangeSetting
// it can be 2, 4, 8 or 16
int adxl345_GetRangeSetting(void) {
	u08 b;
	b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_DATA_FORMAT);
	return (int) (b & 0x3);
}

// Sets the range setting, possible values are: 2, 4, 8, 16
void adxl345_SetRangeSetting(int val) {
	u08 s, b;

	switch (val) {
	case 2:
	default:
		s = 0X00; 			//"00000000"
		break;
	case 4:
		s = 0X01;				//"00000001"
		break;
	case 8:
		s = 0x02;			//"00000010"
		break;
	case 16:
		s = 0x03;			//"00000011"
		break;
	}
	b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_DATA_FORMAT);
	s |= (b & 0xEC);    //"11101100"
	i2c_XmtByte(I2C_ID_ADXL345, ADXL345_DATA_FORMAT, s);
}

// gets the state of the SELF_TEST bit
int adxl345_GetSelfTestBit(void) {
	return adxl345_GetRegisterBit(ADXL345_DATA_FORMAT, 7);
}

// Sets the SELF-TEST bit
// if set to 1 it applies a self-test force to the sensor causing a shift in the output data
// if set to 0 it disables the self-test force
void adxl345_SetSelfTestBit(int selfTestBit) {
	adxl345_SetRegisterBit(ADXL345_DATA_FORMAT, 7, selfTestBit);
}

// Gets the state of the SPI bit
int adxl345_GetSpiBit(void) {
	return adxl345_GetRegisterBit(ADXL345_DATA_FORMAT, 6);
}

// Sets the SPI bit
// if set to 1 it sets the device to 3-wire mode
// if set to 0 it sets the device to 4-wire SPI mode
void adxl345_SetSpiBit(int spiBit) {
	adxl345_SetRegisterBit(ADXL345_DATA_FORMAT, 6, spiBit);
}

// Gets the state of the INT_INVERT bit
int adxl345_GetInterruptLevelBit() {
	return adxl345_GetRegisterBit(ADXL345_DATA_FORMAT, 5);
}

// Sets the INT_INVERT bit
// if set to 0 sets the interrupts to active high
// if set to 1 sets the interrupts to active low
void adxl345_SetInterruptLevelBit(int interruptLevelBit) {
	adxl345_SetRegisterBit(ADXL345_DATA_FORMAT, 5, interruptLevelBit);
}

// Gets the state of the FULL_RES bit
int adxl345_GetFullResBit() {
	return adxl345_GetRegisterBit(ADXL345_DATA_FORMAT, 3);
}

// Sets the FULL_RES bit
// if set to 1, the device is in full resolution mode, where the output resolution increases with the
//   g range set by the range bits to maintain a 4mg/LSB scal factor
// if set to 0, the device is in 10-bit mode, and the range buts determine the maximum g range
//   and scale factor
void adxl345_SetFullResBit(int fullResBit) {
	adxl345_SetRegisterBit(ADXL345_DATA_FORMAT, 3, fullResBit);
}

// Gets the state of the justify bit
int adxl345_GetJustifyBit() {
	return adxl345_GetRegisterBit(ADXL345_DATA_FORMAT, 2);
}

// Sets the JUSTIFY bit
// if sets to 1 selects the left justified mode
// if sets to 0 selects right justified mode with sign extension
void adxl345_SetJustifyBit(int justifyBit) {
	adxl345_SetRegisterBit(ADXL345_DATA_FORMAT, 2, justifyBit);
}

// Sets the THRESH_TAP value
// it should be between 0 and 255
// the scale factor is 62.5 mg/LSB
// A value of 0 may result in undesirable behavior
void adxl345_SetTapThreshold(int tapThreshold) {
	u08 b;
	CLAMP(tapThreshold, 0, 255);
	b = (u08) tapThreshold;
	i2c_XmtByte(I2C_ID_ADXL345, ADXL345_THRESH_TAP, b);
}

// Gets the THRESH_TAP u08 value
// return value is comprised between 0 and 255
// the scale factor is 62.5 mg/LSB
int adxl345_GetTapThreshold(void) {
	u08 b;
	b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_THRESH_TAP);
	return (int) b;
}

// Sets the OFSX, OFSY and OFSZ bytes
// OFSX, OFSY and OFSZ are user offset adjustments in twos complement format with
// a scale factor of 15,6mg/LSB
// OFSX, OFSY and OFSZ should be comprised between
void adxl345_SetAxisOffset(int x, int y, int z) {
	i2c_XmtByte(I2C_ID_ADXL345, ADXL345_OFSX, (u08) x);
	i2c_XmtByte(I2C_ID_ADXL345, ADXL345_OFSY, (u08) y);
	i2c_XmtByte(I2C_ID_ADXL345, ADXL345_OFSZ, (u08) z);
}

// Gets the OFSX, OFSY and OFSZ bytes
void adxl345_GetAxisOffset(int* px, int* py, int* pz) {
	u08 b;
	b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_OFSX);
	*px = (int) b;
	b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_OFSY);
	*py = (int) b;
	b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_OFSZ);
	*pz = (int) b;
}

// Sets the DUR u08
// The DUR u08 contains an unsigned time value representing the maximum time
// that an event must be above THRESH_TAP threshold to qualify as a tap event
// The scale factor is 625�s/LSB
// A value of 0 disables the tap/double tap funcitons. Max value is 255.
void adxl345_SetTapDuration(int tapDuration) {
	u08 b;
	CLAMP(tapDuration, 0, 255);
	b = (u08) tapDuration;
	i2c_XmtByte(I2C_ID_ADXL345, ADXL345_DUR, b);
}

// Gets the DUR u08
int adxl345_GetTapDuration(void) {
	u08 b;
	b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_DUR);
	return (int) b;
}

// Sets the latency (latent register) which contains an unsigned time value
// representing the wait time from the detection of a tap event to the start
// of the time window, during which a possible second tap can be detected.
// The scale factor is 1.25ms/LSB. A value of 0 disables the double tap function.
// It accepts a maximum value of 255.
void adxl345_SetDoubleTapLatency(int doubleTapLatency) {
	u08 b;
	b = (u08) doubleTapLatency;
	i2c_XmtByte(I2C_ID_ADXL345, ADXL345_LATENT, b);
}

// Gets the Latent value
int adxl345_GetDoubleTapLatency(void) {
	u08 b;
	b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_LATENT);
	return (int) b;
}

// Sets the Window register, which contains an unsigned time value representing
// the amount of time after the expiration of the latency time (Latent register)
// during which a second valud tap can begin. The scale factor is 1.25ms/LSB. A
// value of 0 disables the double tap function. The maximum value is 255.
void adxl345_SetDoubleTapWindow(int doubleTapWindow) {
	u08 b;
	CLAMP(doubleTapWindow, 0, 255);
	b = (u08) doubleTapWindow;
	i2c_XmtByte(I2C_ID_ADXL345, ADXL345_WINDOW, b);
}

// Gets the Window register
int adxl345_GetDoubleTapWindow(void) {
	u08 b;
	b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_WINDOW);
	return (int) b;
}

// Sets the THRESH_ACT u08 which holds the threshold value for detecting activity.
// The data format is unsigned, so the magnitude of the activity event is compared
// with the value is compared with the value in the THRESH_ACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
// activity interrupt is enabled. The maximum value is 255.
void adxl345_SetActivityThreshold(int activityThreshold) {
	u08 b;
	CLAMP(activityThreshold, 0, 255);
	b = (u08) activityThreshold;
	i2c_XmtByte(I2C_ID_ADXL345, ADXL345_THRESH_ACT, b);
}

// Gets the THRESH_ACT u08
int adxl345_GetActivityThreshold() {
	u08 b;
	b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_THRESH_ACT);
	return (int) b;
}

// Sets the THRESH_INACT u08 which holds the threshold value for detecting inactivity.
// The data format is unsigned, so the magnitude of the inactivity event is compared
// with the value is compared with the value in the THRESH_INACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
// inactivity interrupt is enabled. The maximum value is 255.
void adxl345_SetInactivityThreshold(int inactivityThreshold) {
	u08 b;
	CLAMP(inactivityThreshold, 0, 255);
	b = (u08) inactivityThreshold;
	i2c_XmtByte(I2C_ID_ADXL345, ADXL345_THRESH_INACT, b);
}

// Gets the THRESH_INACT u08
int adxl345_GetInactivityThreshold(void) {
	u08 b;
	b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_THRESH_INACT);
	return (int) b;
}

// Sets the TIME_INACT register, which contains an unsigned time value representing the
// amount of time that acceleration must be less thant the value in the THRESH_INACT
// register for inactivity to be declared. The scale factor is 1sec/LSB. The value must
// be between 0 and 255.
void adxl345_SetTimeInactivity(int timeInactivity) {
	u08 b;
	CLAMP(timeInactivity, 0, 255);
	b = (u08) timeInactivity;
	i2c_XmtByte(I2C_ID_ADXL345, ADXL345_TIME_INACT, b);
}

// Gets the TIME_INACT register
int adxl345_GetTimeInactivity(void) {
	u08 b;
	b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_TIME_INACT);
	return (int) b;
}

// Sets the THRESH_FF register which holds the threshold value, in an unsigned format, for
// free-fall detection. The root-sum-square (RSS) value of all axes is calculated and
// compared whith the value in THRESH_FF to determine if a free-fall event occured. The
// scale factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the free-fall
// interrupt is enabled. The maximum value is 255.
void adxl345_SetFreeFallThreshold(int freeFallThreshold) {
	u08 b;
	CLAMP(freeFallThreshold, 0, 255);
	b = (u08) freeFallThreshold;
	i2c_XmtByte(I2C_ID_ADXL345, ADXL345_THRESH_FF, b);
}

// Gets the THRESH_FF register.
int adxl345_GetFreeFallThreshold(void) {
	u08 b;
	b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_THRESH_FF);
	return (int) b;
}

// Sets the TIME_FF register, which holds an unsigned time value representing the minimum
// time that the RSS value of all axes must be less than THRESH_FF to generate a free-fall
// interrupt. The scale factor is 5ms/LSB. A value of 0 may result in undesirable behavior if
// the free-fall interrupt is enabled. The maximum value is 255.
void adxl345_SetFreeFallDuration(int freeFallDuration) {
	u08 b;
	CLAMP(freeFallDuration, 0, 255);
	b = (u08) freeFallDuration;
	i2c_XmtByte(I2C_ID_ADXL345, ADXL345_TIME_FF, b);
}

// Gets the TIME_FF register.
int adxl345_GetFreeFallDuration(void) {
	u08 b;
	b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_TIME_FF);
	return (int) b;
}

int adxl345_IsActivityXEnabled(void) {
	return adxl345_GetRegisterBit(ADXL345_ACT_INACT_CTL, 6);
}

int adxl345_IsActivityYEnabled(void) {
	return adxl345_GetRegisterBit(ADXL345_ACT_INACT_CTL, 5);
}

int adxl345_IsActivityZEnabled(void) {
	return adxl345_GetRegisterBit(ADXL345_ACT_INACT_CTL, 4);
}

int adxl345_IsInactivityXEnabled(void) {
	return adxl345_GetRegisterBit(ADXL345_ACT_INACT_CTL, 2);
}

int adxl345_IsInactivityYEnabled(void) {
	return adxl345_GetRegisterBit(ADXL345_ACT_INACT_CTL, 1);
}

int adxl345_IsInactivityZEnabled(void) {
	return adxl345_GetRegisterBit(ADXL345_ACT_INACT_CTL, 0);
}

void adxl345_SetActivityX(int state) {
	adxl345_SetRegisterBit(ADXL345_ACT_INACT_CTL, 6, state);
}

void adxl345_SetActivityY(int state) {
	adxl345_SetRegisterBit(ADXL345_ACT_INACT_CTL, 5, state);
}

void adxl345_SetActivityZ(int state) {
	adxl345_SetRegisterBit(ADXL345_ACT_INACT_CTL, 4, state);
}

void adxl345_SetInactivityX(int state) {
	adxl345_SetRegisterBit(ADXL345_ACT_INACT_CTL, 2, state);
}

void adxl345_SetInactivityY(int state) {
	adxl345_SetRegisterBit(ADXL345_ACT_INACT_CTL, 1, state);
}

void adxl345_SetInactivityZ(int state) {
	adxl345_SetRegisterBit(ADXL345_ACT_INACT_CTL, 0, state);
}

int adxl345_IsActivityAc(void) {
	return adxl345_GetRegisterBit(ADXL345_ACT_INACT_CTL, 7);
}

int adxl345_IsInactivityAc(void) {
	return adxl345_GetRegisterBit(ADXL345_ACT_INACT_CTL, 3);
}

void adxl345_SetActivityAc(int state) {
	adxl345_SetRegisterBit(ADXL345_ACT_INACT_CTL, 7, state);
}

void adxl345_SetInactivityAc(int state) {
	adxl345_SetRegisterBit(ADXL345_ACT_INACT_CTL, 3, state);
}

int adxl345_GetSuppressBit(void) {
	return adxl345_GetRegisterBit(ADXL345_TAP_AXES, 3);
}

void adxl345_SetSuppressBit(int state) {
	adxl345_SetRegisterBit(ADXL345_TAP_AXES, 3, state);
}

int adxl345_IsTapDetectionOnX(void) {
	return adxl345_GetRegisterBit(ADXL345_TAP_AXES, 2);
}

void adxl345_SetTapDetectionOnX(int state) {
	adxl345_SetRegisterBit(ADXL345_TAP_AXES, 2, state);
}

int adxl345_IsTapDetectionOnY(void) {
	return adxl345_GetRegisterBit(ADXL345_TAP_AXES, 1);
}

void adxl345_SetTapDetectionOnY(int state) {
	adxl345_SetRegisterBit(ADXL345_TAP_AXES, 1, state);
}

int adxl345_IsTapDetectionOnZ(void) {
	return adxl345_GetRegisterBit(ADXL345_TAP_AXES, 0);
}

void adxl345_SetTapDetectionOnZ(int state) {
	adxl345_SetRegisterBit(ADXL345_TAP_AXES, 0, state);
}

int adxl345_IsActivitySourceOnX(void) {
	return adxl345_GetRegisterBit(ADXL345_ACT_TAP_STATUS, 6);
}

int adxl345_IsActivitySourceOnY(void) {
	return adxl345_GetRegisterBit(ADXL345_ACT_TAP_STATUS, 5);
}
int adxl345_IsActivitySourceOnZ(void) {
	return adxl345_GetRegisterBit(ADXL345_ACT_TAP_STATUS, 4);
}

int adxl345_IsTapSourceOnX(void) {
	return adxl345_GetRegisterBit(ADXL345_ACT_TAP_STATUS, 2);
}

int adxl345_IsTapSourceOnY(void) {
	return adxl345_GetRegisterBit(ADXL345_ACT_TAP_STATUS, 1);
}

int adxl345_IsTapSourceOnZ(void) {
	return adxl345_GetRegisterBit(ADXL345_ACT_TAP_STATUS, 0);
}

int adxl345_IsAsleep(void) {
	return adxl345_GetRegisterBit(ADXL345_ACT_TAP_STATUS, 3);
}

int adxl345_IsLowPower(void) {
	return adxl345_GetRegisterBit(ADXL345_BW_RATE, 4);
}

void adxl345_SetLowPower(int state) {
	adxl345_SetRegisterBit(ADXL345_BW_RATE, 4, state);
}

double adxl345_GetRate(void) {
	u08 b;
	b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_BW_RATE);
	b &= 0x0F;   // "00001111"
	return (pow(2, ((int) b) - 6)) * 6.25;
}

void adxl345_SetRate(double rate) {
	u08 b, s;
	int v = (int) (rate / 6.25);
	int r = 0;
	while (v >>= 1) {
		r++;
	}
	if (r <= 9) {
		b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_BW_RATE);
		s = (u08) ((r + 6) | (b & 0xF0));  //"11110000"
		i2c_XmtByte(I2C_ID_ADXL345, ADXL345_BW_RATE, s);
	}
}

void adxl345_SetBw(u08 bw_code) {
	if ((bw_code < ADXL345_BW_3) || (bw_code > ADXL345_BW_1600)) {
		return;
	} else {
		i2c_XmtByte(I2C_ID_ADXL345, ADXL345_BW_RATE, bw_code);
	}
}

u08 adxl345_GetBwCode(void) {
	u08 bwCode;
	bwCode = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_BW_RATE);
	return bwCode;
}

//Used to check if action was triggered in interrupts
//Example triggered(interrupts, ADXL345_SINGLE_TAP);
int adxl345_Triggered(u08 interrupts, int mask) {
	return (int) ((interrupts >> mask) & 1);
}

/*
 ADXL345_DATA_READY
 ADXL345_SINGLE_TAP
 ADXL345_DOUBLE_TAP
 ADXL345_ACTIVITY
 ADXL345_INACTIVITY
 ADXL345_FREE_FALL
 ADXL345_WATERMARK
 ADXL345_OVERRUNY
 */

u08 adxl345_GetInterruptSource(void) {
	u08 b;
	b = i2c_RcvByte(I2C_ID_ADXL345, ADXL345_INT_SOURCE);
	return b;
}

int adxl345_GetInterruptSourceBit(u08 interruptBit) {
	return adxl345_GetRegisterBit(ADXL345_INT_SOURCE, interruptBit);
}

int adxl345_GetInterruptMapping(u08 interruptBit) {
	return adxl345_GetRegisterBit(ADXL345_INT_MAP, interruptBit);
}

// Set the mapping of an interrupt to pin1 or pin2
// eg: setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT,ADXL345_INT2_PIN);
void adxl345_SetInterruptMapping(u08 interruptBit, int interruptPin) {
	adxl345_SetRegisterBit(ADXL345_INT_MAP, interruptBit, interruptPin);
}

int adxl345_IsInterruptEnabled(u08 interruptBit) {
	return adxl345_GetRegisterBit(ADXL345_INT_ENABLE, interruptBit);
}

void adxl345_SetInterrupt(u08 interruptBit, int state) {
	adxl345_SetRegisterBit(ADXL345_INT_ENABLE, interruptBit, state);
}

void adxl345_SetRegisterBit(u08 regAdress, int bitPos, int state) {
	u08 b;
	b = i2c_RcvByte(I2C_ID_ADXL345, regAdress);
	if (state) {
		b |= (1 << bitPos); // forces nth bit of _b to be 1.  all other bits left alone.
	} else {
		b &= ~(1 << bitPos); // forces nth bit of _b to be 0.  all other bits left alone.
	}
	i2c_XmtByte(I2C_ID_ADXL345, regAdress, b);
}

int adxl345_GetRegisterBit(u08 regAdress, int bitPos) {
	u08 b;
	b = i2c_RcvByte(I2C_ID_ADXL345, regAdress);
	return (int) ((b >> bitPos) & 1);
}

