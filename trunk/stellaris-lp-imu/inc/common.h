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


#ifndef GLOBAL_H_
#define GLOBAL_H_

#define DEBUG 0
#define BLINK 0

#include "inc/hw_gpio.h"
#include "inttypes.h"
#include "driverlib/rom.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "type.h"
#include "fpu.h"

#define DELAY_MS(ms)	{ROM_SysCtlDelay(ms * (ROM_SysCtlClockGet() / 3000));} // multiplier for SysCtlGet() store in global variables
#define DELAY_US(us)	{ROM_SysCtlDelay(us * ((ROM_SysCtlClockGet() / 3000)/1000));} // multiplier for SysCtlGet() store in global variables

#define I2C_PORT I2C1_MASTER_BASE

#define RED GPIO_PIN_1
#define GREEN GPIO_PIN_2
#define BLUE GPIO_PIN_3

#define on 1
#define off 0

#define LED_INIT()  		{ ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED | GREEN | BLUE);}
#define LED(color,sw)    	{HWREG(GPIO_PORTF_BASE +  (color << 2)) = (((color >> 01) << sw) & color);}
#define LED_TOGGLE(color)    	{HWREG(GPIO_PORTF_BASE +  (GPIO_O_DATA +(color << 2))) ^= color;}

#define BUTTON_1 GPIO_PIN_0
#define BUTTON_2 GPIO_PIN_4

#define APP_SYSTICKS_PER_SEC            32
#define APP_BUTTON_POLL_DIVIDER          8

#define I2C_ID_ADXL345      ((uint8_t)0x53) // assumes ADXL345 alternate address pin is grounded
#define I2C_ID_L3G4200D     ((uint8_t)0x69)
#define I2C_ID_HMC5883L     ((uint8_t)0x1E)
#define I2C_ID_BMP085       ((uint8_t)0x77)

extern char gszBuf[80];


#define ABS(x)                 ((x) < 0 ? -(x) : (x))
#define CLAMP(x,min,max)       {if ((x) <= (min)) (x) = (min); else if ((x) >= (max)) (x) = (max);}

void util_MemSet(uint8_t* pBuffer, uint8_t val, int32_t nBytes);
void util_MemCpy(uint8_t* pDestination, uint8_t* pSource, int32_t nBytes);
int util_AverageSamples(short buf[], char numSamples);
int util_SigmaSamples(short buf[], char numSamples, int average);
int32_t util_WaitBtnPressTimeout(int32_t seconds);

#endif // GLOBAL_H_
