/*
 * kinematics.c
 *
 *  Created on: 4 ago. 2020
 *      Author: isaac.ambit
 */


#include "kinematics.h"

forwardKinematics ForwardKinematics(const volatile float *leftWheelAngularVelocity, const volatile float *rightWheelAngularVelocity)
{
	vehicleVelocity.linearVelocity = (*rightWheelAngularVelocity + *leftWheelAngularVelocity) * 0.5 * WHEEL_RADIUS;
	vehicleVelocity.angularVelocity = (*rightWheelAngularVelocity - *leftWheelAngularVelocity) * WHEEL_RADIUS * WHEEL_DISTANCE_INV;
	return vehicleVelocity;
}

inverseKinematics InverseKinematics(const float *linearVelocity, const float *angularVelocity)
{
	wheelVelocity.wLeft = (*linearVelocity-WHEEL_DISTANCE*(*angularVelocity))*WHEEL_RADIUS_INV;
	wheelVelocity.wRight = 2*(*linearVelocity)*WHEEL_RADIUS_INV-wheelVelocity.wLeft;
	return wheelVelocity;
}
