/*
 * lineControl.h
 *
 *  Created on: 19 jul. 2020
 *      Author: isaac.ambit
 */

#ifndef CONTROL_LINECONTROL_H_
#define CONTROL_LINECONTROL_H_

#include "stm32f4xx_hal.h"
#include "math.h"
#include "../SystemVariables.h"

#define INV_SENSOR_DISTANCE 14.28				// Inverse of the distance from wheel axes to the line sensor
#define PWM_FREQ 20000							//PWM desired frequency is 20 KHz
#define a_line 1
#define b_line 0.02
#define a_coef_aprox 0.5

float d_measured_filter[2];

//FUNCTIONS

float distanceToLine(volatile const uint32_t *uhADCxConvertedValue);
float controlAngularSpeed(const float *d_measured);

#endif /* CONTROL_LINECONTROL_H_ */
