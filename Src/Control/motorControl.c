/*
 * motorControl.c
 *
 *  Created on: 2 ago. 2020
 *      Author: isaac.ambit
 */


#include "motorControl.h"

float calculateError(const float *reference, const volatile float *signal)
{
	return *reference - *signal;
}
void controlRightMotor(const float *error, volatile const float *reference, float *controlAction)
{
	float uSpeedP= *error*KpMotor;												//Propotional action
	float uSpeedFF= *reference*KFF;												//Feedforward
	if (*controlAction<batteryV) uSpeedIR = (uSpeedIR+(KiMotor*(*error)))*start;	//Integral + Anti wind-up
	*controlAction=(uSpeedP+uSpeedIR+uSpeedFF);									//Right wheel voltage
}
void controlLeftMotor(const float *error, volatile const float *reference, float *controlAction)
{
	float uSpeedP= *error*KpMotor;												//Propotional action
	float uSpeedFF= *reference*KFF;												//Feedforward
	if (*controlAction<batteryV) uSpeedIL = (uSpeedIL+(KiMotor*(*error)))*start;	//Integral + Anti wind-up
	*controlAction=(uSpeedP+uSpeedIL+uSpeedFF);									//Right wheel voltage
}

float slidingModeControlRight(float *errorRight)
{
//	if(*errorRight > 2) return 2.4;
//	else return 0;
	return 3*(*errorRight)/(fabs(*errorRight)+1.7);

}
float slidingModeControlLeft(float *errorLeft)
{
//	if(*errorLeft > 0) return 2.4;
//	else return 0;
	return 3*(*errorLeft)/(fabs(*errorLeft)+1.7);
}
//////////////////////////////
#define timeStepInv	10000

#define k1	3
#define k2	5

#define a	0.085
#define b	0.03
#define l	0.068
#define r	0.035
#define L	0.25
#define W	0.18
#define m	1.175
#define Iz	0.009292
#define Bf	0.0001543

float alpha = 1/(Iz + m*b*b);
float beta1 = 2*Bf/(r*r);
float beta2 = 2*Bf*a/(r*r);

float l_alpha = l/(Iz + m*b*b);
float a_alpha = a/(Iz + m*b*b);
float m_b = m*b;
float m_r = m*r;


float calculateD_diff(float *d)
{
	static float d_last = 0;
	float d_diff = (*d - d_last)*timeStepInv;
	d_last = *d;
	return d_diff;

}

float calculateTheta_diff(float *theta){
	static float theta_last = 0;
	float theta_diff = (*theta - theta_last)*timeStepInv;
	theta_last = *theta;
	return theta_diff;

}

float sign(float *value){
	if(*value > 0) return 1;
	else if(*value < 0) return -1;
	else return 0;
}

float calculateTheta(float *omega, float *d, float *d_diff, float *velX)
{
	return atan((l**omega + *d_diff)/(*omega**d - *velX));

}

float calculateF1(float *velX, float *omega, float *d, float *d_diff, float *theta, float *theta_diff){
	//float beta2Extended = m_b**omega**velX - beta2**omega;
	//return *d_diff - l_alpha*(beta2Extended) + (1 + pow(tan(*theta),2)**theta_diff*(*velX - *omega**d)) + tan(*theta)*(*omega**d_diff - beta1**velX - b**omega**omega - *d*alpha*(beta2Extended));
	return (1 + *omega*tan(*theta))**d_diff + alpha*(*d*tan(*theta) - l)*(-beta2**omega + m_b**omega**velX) - (1 + pow(tan(*theta),2))**theta_diff*(*velX - *omega**d) - tan(*theta)*(-beta1**velX - b**omega**omega);
}

float calculateU1(float *sigma1, float *sigma2, float *velX, float *omega, float *d, float *d_diff, float *theta)
{
//	float theta_diff = calculateTheta_diff(theta);
//
//	float f1 = calculateF1(d, d_diff, omega, velX, theta, &theta_diff);
//	float f2 = -(beta2**velX + b**omega**omega);
//
//	return m_r*(*sigma2*f2 + *sigma1*f1 + k1*sign(sigma1) + k2*sign(sigma2))/(*sigma2 + *sigma1*tan(*theta));
//	return -4.2*sign(sigma2);
	float sigma3 = *sigma2 - *sigma1*tan(*theta);
	//float sigma3 = *sigma2;
	float f2 = -beta1**velX -b**omega**omega;
	return m_r*(-100*sign(&sigma3) - *sigma2*f2);
//	return m_r*(-100*sign(&sigma3));

}

float calculateU2(float *sigma1, float *velX, float *omega, float *d, float *d_diff, float *theta)
{
	//return r*(-k3*sign(sigma1))/(a_alpha*(l - *d*tan(*theta)));
//	return -2.7*sign(sigma1);
	float theta_diff = calculateTheta_diff(theta);
	float f1 = calculateF1(velX, omega, d, d_diff, theta, &theta_diff);
	return r*(-45*sign(sigma1) - f1)/(alpha*a*(*d*tan(*theta) - l));
//	return r*(-90*sign(sigma1))/fabs((alpha*a*(*d + tan(*theta) - l)));

}
