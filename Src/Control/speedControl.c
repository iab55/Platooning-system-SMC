/*
 * speedControl.c
 *
 *  Created on: 16 jul. 2020
 *      Author: isaac.ambit
 */


#include "speedControl.h"

float lastSigma;
float u1_smc;
float u2_dot;
float u2_smc;

float sigmaDerivativeCalculation(const float *sigma)
{
	float sigmaDerivative = (*sigma - lastSigma);
	lastSigma = *sigma;
	return sigmaDerivative;
}

float sigmaCalculation(const float *velocity, volatile const float *distanceBetweenVehicles)
{
	float VelDes;
	float difDist = *distanceBetweenVehicles - MAXIMUM_OBJECTIVE_DISTANCE;
	if(difDist <= -APROXIMETION_COEFICIENT){
		VelDes = MAXIMUM_SPEED + (*distanceBetweenVehicles - MAXIMUM_OBJECTIVE_DISTANCE)*DISTANCE_TO_SPEED;
		//BSP_LED_On(LED5);
	}
	else if(fabs(difDist) < APROXIMETION_COEFICIENT){
		VelDes = MAXIMUM_SPEED -pow((difDist-APROXIMETION_COEFICIENT),2)/(4*APROXIMETION_COEFICIENT*SPEED_TO_DISTANCE);
		//BSP_LED_On(LED5);
	}
	else{
		VelDes = MAXIMUM_SPEED;
		//BSP_LED_On(LED5);
	}
	return VelDes;//0.998*(VelDes - velocity) - 0.002*sigmasDerivative;
}

float controlLinearSpeed(const float *sigma)
{
	float U;
	switch(algorithm)
	{
	case 1: //fosmc
		if(*sigma >= 0) U = *sigma;
		else U = 0;
		//U = 1.5*MAXIMUM_SPEED**sigma/(fabs(*sigma)+1);
		//if(*sigma < 2.5 || *sigma > 2.5) U = MAXIMUM_SPEED**sigma/(fabs(*sigma));
//		if(*sigma >= 0){
//			U = (MAXIMUM_SPEED+0.15)**sigma/(fabs(*sigma)+0.03);
//		}
//		else if(*sigma <= -0.12){
//			U = (MAXIMUM_SPEED+0.15)**sigma/(fabs(*sigma)+0.03);
//		}
//		else{
//			U = 0;
//		}
		break;
	case 2: //super-twisting algorithm
		u1_smc = 1.5*(MAXIMUM_SPEED)*sqrt(fabs(*sigma))*(*sigma)/fabs(*sigma);	//k1*sqrt(abs(*sigma))**sigma/abs(*sigma);
		u2_dot = 0.4*(*sigma)/fabs(*sigma);									//k2**sigma/abs(*sigma);
		u2_smc += u2_dot*SAMPLING_TIME;

		if(u2_smc>1){
			u2_smc = 1;
		}
		else if(u2_smc<-1){
			u2_smc = -1;
		}
		//U = ueq + m*(u1_smc+u2_smc);
		U = VEHICLE_MASS*(u1_smc+u2_smc);
		break;
   case 3: //linear correction algorithm
	   if(*sigma>0.005 || *sigma<-0.005){
			u1_smc = 3*sqrt(fabs(*sigma))*(*sigma)/fabs(*sigma) + 1*(*sigma);//k1*sqrt(abs(*sigma))**sigma/abs(*sigma);
			u2_dot = 9.375*(*sigma)/fabs(*sigma) + 118.5*(*sigma);//k2**sigma/abs(*sigma);
			u2_smc += u2_dot*SAMPLING_TIME;

			if(u2_smc>200){
				u2_smc = 200;
			}
			else if(u2_smc<-200){
				u2_smc = -200;
			}

			U = VEHICLE_MASS*(u1_smc+u2_smc);
		}
		else{
			U = 0;
		}
	   break;
   default:
	   U = 0;
	   break;
	}
	return U;
}
