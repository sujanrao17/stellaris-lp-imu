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


#ifndef TMRSYS_H_
#define TMRSYS_H_

#define TMRSYS_TICK_MS  10


void tmrsys_Config(void);
void tmrsys_ResetElapsedTime(void);
void tmrsys_DelayMs(u32 delayMs);

extern volatile u32 gnSysTick;
extern volatile u32 gnHours;
extern volatile u32 gnMinutes;
extern volatile u32 gnSeconds;
extern volatile int gbBtnPressed;
extern volatile int gbSysTickFlag;
extern volatile u32 gBtnState;
extern volatile u32 gnTick;

#endif // TMRSYS_H_
