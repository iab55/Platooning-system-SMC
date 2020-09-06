/*
 * kinematics.h
 *
 *  Created on: 4 ago. 2020
 *      Author: isaac.ambit
 */

#ifndef ODOMETRY_KINEMATICS_H_
#define ODOMETRY_KINEMATICS_H_

#define WHEEL_RADIUS			0.035
#define WHEEL_RADIUS_INV 		28.57
#define WHEEL_DISTANCE 			0.17
#define WHEEL_DISTANCE_INV 		1/WHEEL_DISTANCE

typedef struct forwardKinematics{
	float linearVelocity;
	float angularVelocity;
}forwardKinematics;
typedef struct inverseKinematics{
	float wRight;
	float wLeft;
}inverseKinematics;

forwardKinematics vehicleVelocity;
inverseKinematics wheelVelocity;

forwardKinematics ForwardKinematics(const volatile float *leftWheelAngularVelocity, volatile const float *rightWheelAngularVelocity);
inverseKinematics InverseKinematics(const float *linearVelocity, const float *angularVelocity);
#endif /* ODOMETRY_KINEMATICS_H_ */
