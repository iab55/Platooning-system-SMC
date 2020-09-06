/*
 * serialization.c
 *
 *  Created on: 13 jul. 2020
 *      Author: isaac.ambit
 */


#include "serialization.h"


void sendUART(UART_HandleTypeDef *UartHandle){
	uint8_t dadesPaquet[30] = {0};

	if(0){
		memcpy(dadesPaquet, "AT+CIPSEND=0,16\r\n", 17);
		HAL_UART_Transmit_IT(UartHandle, dadesPaquet, 17);
		HAL_Delay(2);

		memcpy(&dadesPaquet[0], &Velocity, sizeof(float));
		memcpy(&dadesPaquet[4], &sigmaSMC, sizeof(float));
		memcpy(&dadesPaquet[8], &USd_kdef, sizeof(float));
		memcpy(&dadesPaquet[12], &referenceLinearVelocity, sizeof(float));
		HAL_UART_Transmit_IT(UartHandle, dadesPaquet, 16);
		HAL_Delay(2);
	}
	else if(1){
		memcpy(dadesPaquet, "AT+CIPSEND=0,11\r\n", 17);
		HAL_UART_Transmit_IT(UartHandle, dadesPaquet, 17);
		HAL_Delay(2);

		memcpy(&dadesPaquet[0], "sigma", 5);
		memcpy(&dadesPaquet[5], "hEstm", 5);
		memcpy(&dadesPaquet[10], "U", 1);
		HAL_UART_Transmit_IT(UartHandle, dadesPaquet, 11);
		HAL_Delay(2);
	}

}

