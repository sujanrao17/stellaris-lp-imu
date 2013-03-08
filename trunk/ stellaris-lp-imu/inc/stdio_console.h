//*****************************************************************************
//
// stdio_console.h - Setup code for UARTStdio console.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE AUTHOR SHALL NOT, UNDER
// ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// AUTHOR: JOERG QUINTEN
// E2E-NICKNAME: aBUGSworstnightmare
//
//*****************************************************************************

#ifndef STDIO_CONSOLE_H_
#define STDIO_CONSOLE_H_

//*****************************************************************************
//
// Additional Defines for the API.
//
//*****************************************************************************
#define ASCII_ESC 27

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//  This function sets up UART0 to be used for a console I/O
extern void InitConsole(void);


#endif /* STDIO_CONSOLE_H_ */
