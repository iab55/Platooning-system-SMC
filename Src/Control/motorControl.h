/*
 * motorControl.h
 *
 *  Created on: 2 ago. 2020
 *      Author: isaac.ambit
 */

#ifndef CONTROL_MOTORCONTROL_H_
#define CONTROL_MOTORCONTROL_H_

#include "../SystemVariables.h"
#include "math.h"

#define KFF  0.11
#define KpMotor 0.25 					//Proportional gain for speed control
#define KiMotor 0.2/30/1000

float uSpeedIL;
float uSpeedIR;

extern float batteryV;
extern _Bool start;

//FUNCTIONS

float calculateError(const float *reference, const volatile float *signal);
void controlRightMotor(const float *error, volatile const float *reference, float *controlAction);
void controlLeftMotor(const float *error, volatile const float *reference, float *controlAction);
float slidingModeControlRight(float *errorRight);
float slidingModeControlLeft(float *errorLeft);

float calculateU1(float *sigma1, float *sigma2, float *velX, float *omega, float *d, float *d_diff, float *theta);
//float calculateU2(float *sigma1, float *theta, float *d);
float calculateU2(float *sigma1, float *velX, float *omega, float *d, float *d_diff, float *theta);
float calculateD_diff(float *d);
float calculateTheta(float *omega, float *d, float *d_diff, float *velX);
#endif /* CONTROL_MOTORCONTROL_H_ */
