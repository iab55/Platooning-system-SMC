/*
 * lineControl.c
 *
 *  Created on: 19 jul. 2020
 *      Author: isaac.ambit
 */


#include "lineControl.h"

const float kls[4] = {0.03334, 0.02381, 0.01429, 0.00477}; //{1,24/33.6,14.4/31.5,4.8/33.6};
const float COUNTS_TO_VOLTS = 0.000244; //0.0064;
const float k_s = 0.1;

float distanceToLine(volatile const uint32_t *uhADCxConvertedValue)	//mm
{
	return COUNTS_TO_VOLTS*(kls[0]*((float)uhADCxConvertedValue[7]-(float)uhADCxConvertedValue[0])+kls[1]*((float)uhADCxConvertedValue[6]-(float)uhADCxConvertedValue[1])+kls[2]*((float)uhADCxConvertedValue[5]-(float)uhADCxConvertedValue[2])+kls[3]*((float)uhADCxConvertedValue[4]-(float)uhADCxConvertedValue[3]));
}

//control angular file
float controlAngularSpeed(const float *d_measured)
{
	d_measured_filter[0] = d_measured_filter[1]*0.5 + *d_measured*0.5;
	float d_measured_dot=(d_measured_filter[0]-d_measured_filter[1])*(float)PWM_FREQ;
	d_measured_filter[1]=d_measured_filter[0];

	float sigma = a_line*(*d_measured) + b_line*d_measured_dot;

	float u_sliding = sigma / (fabs(sigma)+ a_coef_aprox);
	return -k_s*INV_SENSOR_DISTANCE*(u_sliding);
}
