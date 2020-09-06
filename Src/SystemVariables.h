/*
 * SystemVariables.h
 *
 *  Created on: 7 jul. 2020
 *      Author: isaac.ambit
 */

#ifndef SYSTEMVARIABLES_H_
#define SYSTEMVARIABLES_H_

#include "Communications/queueBuffer.h"
#include "stm32f4xx_hal.h"


_Bool start;
Queue bufferInputUART;
UART_HandleTypeDef UartHandle;


float Velocity;
float sigmaSMC;
float USd_kdef;
float referenceLinearVelocity;
float referenceAngularVelocity;

//ultrasounds sensors
float volatile USFilteredDistance[2];

volatile uint32_t uhADCxConvertedValue[9];

volatile float wLeft;
volatile float wRight;

float batteryV;

#endif /* SYSTEMVARIABLES_H_ */
