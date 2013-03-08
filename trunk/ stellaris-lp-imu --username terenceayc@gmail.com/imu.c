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

#include <math.h>
#include "common.h"
#include "tmrsys.h"
#include "imu.h"
#include "hmc5883l.h"
#include "adxl345.h"
#include "l3g4200d.h"
#include  "bmp085.h"
#include "i2c_IMU.h"

float iq0, iq1, iq2, iq3;
float exInt, eyInt, ezInt;  // scaled integral error
volatile float twoKp;      // 2 * proportional gain (Kp)
volatile float twoKi;      // 2 * integral gain (Ki)
volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx, integralFBy, integralFBz;
unsigned long lastUpdate, now; // sample period expressed in milliseconds
float sampleFreq; // half the sample period expressed in seconds
int startLoopTime;

int gRawData[11]; // concatenate accelerometer, gyroscope, magnetometer, temperature and pressure data

float gRealData[11]; //

float imu_InvSqrt(float number);

void imu_Init(void) {
	adxl345_Config();
	hmc5883l_Config();
	l3g_Config();
	bmp085_Config();

	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	exInt = 0.0;
	eyInt = 0.0;
	ezInt = 0.0;
	twoKp = twoKpDef;
	twoKi = twoKiDef;
	integralFBx = 0.0f;
	integralFBy = 0.0f;
	integralFBz = 0.0f;
	lastUpdate = 0;
	now = 0;
}

void imu_GetRawValues(int axraw, int ayraw, int azraw, int gxraw, int gyraw,
		int gzraw, int mxraw, int myraw, int mzraw) {
	gRawData[0] = axraw;
	gRawData[1] = ayraw;
	gRawData[2] = azraw;
	gRawData[3] = gxraw;
	gRawData[4] = gyraw;
	gRawData[5] = gzraw;
	gRawData[6] = mxraw;
	gRawData[7] = myraw;
	gRawData[8] = mzraw;
}

void imu_UpdateData(float ax, float ay, float az, float gx, float gy, float gz,
		float mx, float my, float mz) {
	gRealData[0] = ax;
	gRealData[1] = ay;
	gRealData[2] = az;
	gRealData[3] = gx;
	gRealData[4] = gy;
	gRealData[5] = gz;
	gRealData[6] = mx;
	gRealData[7] = my;
	gRealData[8] = mz;
}

void imu_AHRSUpdate(float gx, float gy, float gz, float ax, float ay, float az,
		float mx, float my, float mz) {
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float halfex = 0.0f;
	float halfey = 0.0f;
	float halfez = 0.0f;
	float qa, qb, qc;

	// Auxiliary variables to avoid repeated arithmetic
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

	// Use magnetometer measurement only when valid (avoids NaN in magnetometer normalisation)
	if ((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) {
		float hx, hy, bx, bz;
		float halfwx, halfwy, halfwz;

		// Normalise magnetometer measurement
		recipNorm = imu_InvSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Reference direction of Earth's magnetic field
		hx = 2.0f
				* (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3)
						+ mz * (q1q3 + q0q2));
		hy = 2.0f
				* (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3)
						+ mz * (q2q3 - q0q1));
		bx = sqrt(hx * hx + hy * hy);
		bz = 2.0f
				* (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1)
						+ mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of magnetic field
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (my * halfwz - mz * halfwy);
		halfey = (mz * halfwx - mx * halfwz);
		halfez = (mx * halfwy - my * halfwx);
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if ((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
		float halfvx, halfvy, halfvz;

		// Normalise accelerometer measurement
		recipNorm = imu_InvSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex += (ay * halfvz - az * halfvy);
		halfey += (az * halfvx - ax * halfvz);
		halfez += (ax * halfvy - ay * halfvx);
	}

	// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	if (halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
		// Compute and apply integral feedback if enabled
		if (twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;  // apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f; // prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = imu_InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//#define DEBUG_PRINT(x)   uart_Printf("%f\r\n",x)
#define DEBUG_PRINT(x)

void imu_GetQ(float * q) {
	DEBUG_PRINT(DEG2RAD(gRealData[3])); DEBUG_PRINT(DEG2RAD(gRealData[4])); DEBUG_PRINT(DEG2RAD(gRealData[5])); DEBUG_PRINT(gRealData[0]); DEBUG_PRINT(gRealData[1]); DEBUG_PRINT(gRealData[2]); DEBUG_PRINT(gRealData[6]); DEBUG_PRINT(gRealData[7]); DEBUG_PRINT(gRealData[8]);

	now = gnSysTick; //10ms per tick
	sampleFreq = 100.0f / (float) (now - lastUpdate);
	lastUpdate = now;

	// gyro values are expressed in deg/sec, convert to radians/sec
	imu_AHRSUpdate(DEG2RAD(gRealData[3]), DEG2RAD(gRealData[4]),
			DEG2RAD(gRealData[5]), gRealData[0], gRealData[1], gRealData[2],
			gRealData[6], gRealData[7], gRealData[8]);

	q[0] = q0;
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;
}

// Returns the Euler angles in radians defined with the Aerospace sequence.
// See Sebastian O.H. Madwick report
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
void imu_GetEuler(float * angles) {
	float q[4]; // quaternion
	imu_GetQ(q);

	angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3],
			2 * q[0] * q[0] + 2 * q[1] * q[1] - 1) * 180 / M_PI; // psi
	angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180 / M_PI; // theta
	angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1],
			2 * q[0] * q[0] + 2 * q[3] * q[3] - 1) * 180 / M_PI; // phi
}

void imu_GetYawPitchRoll(float * ypr) {
	float q[4]; // quaternion
	float gx, gy, gz; // estimated gravity direction
	imu_GetQ(q);

	gx = 2 * (q[1] * q[3] - q[0] * q[2]);
	gy = 2 * (q[0] * q[1] + q[2] * q[3]);
	gz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

	ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3],
			2 * q[0] * q[0] + 2 * q[1] * q[1] - 1) * 180 / M_PI;
	ypr[1] = atan(gx / sqrt(gy * gy + gz * gz)) * 180 / M_PI;
	ypr[2] = atan(gy / sqrt(gx * gx + gz * gz)) * 180 / M_PI;
}

float imu_InvSqrt(float number) {
	volatile long i;
	volatile float x, y;
	volatile const float f = 1.5F;

	x = number * 0.5F;
	y = number;
	i = *((long *) &y);
	i = 0x5f375a86 - (i >> 1);
	y = *((float *) &i);
	y = y * (f - (x * y * y));
	return y;
}
