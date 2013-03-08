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

#ifndef I2C_IMU_H_
#define I2C_IMU_H_

// setup code for all I2C modules present on the LM4F120E5QR
void i2c_Config(void);

// functions for writing/reading single bytes of data
uint8_t i2c_RcvByte( uint8_t devId, uint8_t addr);

int32_t i2c_XmtByte( uint8_t devId, uint8_t addr, uint8_t data);

// functions for writing/reading multiple bytes of data
int32_t i2c_RcvBuf(uint8_t devId, uint8_t addr, int32_t nBytes , uint8_t* pBuf );

int32_t i2c_XmtBuf( uint8_t devId, uint8_t addr, int32_t nBytes , uint8_t* pBuf);

#endif /* I2C_IMU_H_ */
