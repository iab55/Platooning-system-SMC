/*
 * serialization.h
 *
 *  Created on: 13 jul. 2020
 *      Author: isaac.ambit
 */

#ifndef SERIALIZATION_H_
#define SERIALIZATION_H_

//#include "queueBuffer.h"
#include <string.h>
//#include <stdlib.h>
//#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "../SystemVariables.h"

#define STATUS_SIZE 30

extern float Velocity;
extern float sigmaSMC;
extern float USd_kdef;
extern float referenceLinearVelocity;
extern volatile float wRight;
extern volatile float wLeft;

//void serializeStatus(Queue *bufferOutputUART);
void sendUART(UART_HandleTypeDef *UartHandle);

#endif /* SERIALIZATION_H_ */
