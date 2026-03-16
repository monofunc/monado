// Copyright 2026, Stanislav Aleksandrov <lightofmysoul@gmail.com>
// SPDX-License-Identifier: BSL-1.0
// Compatibility header replacing NXP framework dependencies.

#ifndef NXP_MAG_COMPAT_H
#define NXP_MAG_COMPAT_H

#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef int8_t int8;
typedef uint8_t uint8;
typedef int16_t int16;
typedef uint16_t uint16;
typedef int32_t int32;
typedef uint32_t uint32;

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define X 0
#define Y 1
#define Z 2

#define DEFAULTB 50.0F
#define ONETHIRD 0.33333333F
#define ONESIXTH 0.166666667F

// Minimal sensor structs with only the fields used by magnetic.c
struct AccelSensor
{
	int16 iGp[3]; // integer gravity vector (accelerometer, counts)
};

struct MagSensor
{
	int16 iBpFast[3];   // fast (unaveraged) magnetometer reading (counts)
	float fBp[3];       // float magnetometer reading (uT)
	float fBpFast[3];   // fast float magnetometer reading (uT)
	float fBc[3];       // calibrated magnetometer reading (uT)
	int16 iBc[3];       // calibrated magnetometer reading (counts)
	float fBcFast[3];   // fast calibrated magnetometer reading (uT)
	float fuTPerCount;   // conversion factor
	float fCountsPeruT;  // inverse conversion factor
};

// Forward declarations for matrix.h functions
void f3x3matrixAeqI(float A[][3]);
void fmatrixAeqI(float *A[], int16 rc);
void f3x3matrixAeqScalar(float A[][3], float Scalar);
void f3x3matrixAeqInvSymB(float A[][3], float B[][3]);
void f3x3matrixAeqAxScalar(float A[][3], float Scalar);
void f3x3matrixAeqMinusA(float A[][3]);
float f3x3matrixDetA(float A[][3]);
void eigencompute(float A[][10], float eigval[], float eigvec[][10], int8 n);
void fmatrixAeqInvA(float *A[], int8 iColInd[], int8 iRowInd[], int8 iPivot[], int8 isize);

#endif // NXP_MAG_COMPAT_H
