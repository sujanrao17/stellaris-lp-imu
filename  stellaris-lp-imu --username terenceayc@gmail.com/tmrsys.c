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
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"

volatile u32 gnSysTick = 0; // counts ticks since power up or reset
volatile u32 gnHours = 0; // elapsed time counter
volatile u32 gnMinutes = 0;
volatile u32 gnSeconds = 0;

volatile u32 gBtnState = 0;
volatile int gbBtnPressed = 0;
volatile int gbSysTickFlag = 0;
volatile u32 gnTick = 0; //  time since tmrsys_ResetElapsedTime() was called

#define BTN_PIN			3
#define BTN_PRESSED    	GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);

#define BUTTONS_GPIO_PERIPH     SYSCTL_PERIPH_GPIOF
#define BUTTONS_GPIO_BASE       GPIO_PORTF_BASE

#define NUM_BUTTONS             2
#define LEFT_BUTTON             GPIO_PIN_4
#define RIGHT_BUTTON            GPIO_PIN_0

#define ALL_BUTTONS             (LEFT_BUTTON | RIGHT_BUTTON)

///
/// System timer is used to generate system tick,
/// keep track of elapsed time.
/// Configure the system tick timer for a TMRSYS_TICK_MS interval
/// with interrupt enabled.

void tmrsys_Config(void) {
	//
	// Enable the GPIO port to which the pushbuttons are connected.
	//
	ROM_SysCtlPeripheralEnable (BUTTONS_GPIO_PERIPH);

	//
	// Unlock PF0 so we can change it to a GPIO input
	// Once we have enabled (unlocked) the commit register then re-lock it
	// to prevent further changes.  PF0 is muxed with NMI thus a special case.
	//
	HWREG(BUTTONS_GPIO_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
	HWREG(BUTTONS_GPIO_BASE + GPIO_O_CR) |= 0x01;
	HWREG(BUTTONS_GPIO_BASE + GPIO_O_LOCK) = 0;

	//
	// Set each of the button GPIO pins as an input with a pull-up.
	//
	ROM_GPIODirModeSet (BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_DIR_MODE_IN);
	ROM_GPIOPadConfigSet (BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_STRENGTH_2MA,
			GPIO_PIN_TYPE_STD_WPU);

	long clk = (ROM_SysCtlClockGet () / 100) - 1;
	SysTickPeriodSet((ROM_SysCtlClockGet () / 100) - 1); // 10mS tick
	SysTickEnable();
	SysTickIntEnable();
	IntMasterEnable();

	gnTick = 0;
}

///
/// Interrupt service routine for system tick timer
/// Increment the system tick, debounce the button,
/// increment the elapsed seconds, minutes and hours if needed
void SysTick_Handler(void) {
	gbSysTickFlag = 1;
	gBtnState = (gBtnState << 1) | (u32) BTN_PRESSED
	;
	if ((gBtnState & 63UL) == 32) {
		gbBtnPressed = 1;
	}

	gnSysTick++;
	gnTick++;
	if (gnTick == 100) {
		gnTick = 0;
		gnSeconds++;
		if (gnSeconds == 60) {
			gnSeconds = 0;
			gnMinutes++;
			if (gnMinutes == 60) {
				gnMinutes = 0;
				gnHours++;
			}
		}
	}
}

///
/// Reset the elapsed seconds, minutes and hours to zero
void tmrsys_ResetElapsedTime(void) {
	gnTick = 0;
	gnSeconds = 0;
	gnMinutes = 0;
	gnHours = 0;
}

/// Busy wait for specified time in milliseconds
/// @param delayMs number of milliseconds to wait
///         resolution cannot be less than TMRSYS_TICK_MS
void tmrsys_DelayMs(u32 delayMs) {
	u32 curTick, delayTicks;
	delayTicks = delayMs / TMRSYS_TICK_MS;
	curTick = gnSysTick;
	while ((gnSysTick - curTick) < delayTicks)
		;
}
