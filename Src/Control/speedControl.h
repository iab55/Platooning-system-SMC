/*
 * speedControl.h
 *
 *  Created on: 16 jul. 2020
 *      Author: isaac.ambit
 */

#ifndef CONTROL_SPEEDCONTROL_H_
#define CONTROL_SPEEDCONTROL_H_

#include "math.h"

#define MAXIMUM_SPEED					0.4
#define MINIMUM_DISTANCE				0.15
#define SPEED_TO_DISTANCE				2
#define DISTANCE_TO_SPEED				1/(float)SPEED_TO_DISTANCE
#define MAXIMUM_OBJECTIVE_DISTANCE 		((float)MINIMUM_DISTANCE + (float)SPEED_TO_DISTANCE*(float)MAXIMUM_SPEED)
#define APROXIMETION_COEFICIENT			0.05
#define APROXIMETION_COEFICIENT_INV		1/APROXIMETION_COEFICIENT

#define VEHICLE_MASS					1
#define SAMPLING_TIME					0.008

unsigned char algorithm;

//FUNCTIONS

float controlLinearSpeed(const float *sigma);
float sigmaCalculation(const float *velocity, volatile const float *distanceBetweenVehicles);
float sigmaDerivativeCalculation(const float *sigma);

#endif /* CONTROL_SPEEDCONTROL_H_ */
