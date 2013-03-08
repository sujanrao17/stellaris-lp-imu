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

#ifndef IMU_H_
#define IMU_H_

#define M_PI 3.1415927f

#define DEG2RAD(d)   (((d)*M_PI)/180.0f)

#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.1f) // 2 * integral gain

void imu_Init(void);
void imu_GetQ(float * q);
void imu_GetEuler(float * angles);
void imu_GetYawPitchRoll(float * ypr);
void imu_UpdateData(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void imu_AHRSUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
float imu_InvSqrt(float number);
#endif
