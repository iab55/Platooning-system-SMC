/*
 * communicationManager.c
 *
 *  Created on: 14 jul. 2020
 *      Author: isaac.ambit
 */


#include "communicationManager.h"


void manageCommunications(UART_HandleTypeDef *UartHandle, Queue *bufferInputUART){
	static uint32_t tickEnviamentDadesUART = 0;

	processData(bufferInputUART);

	if((HAL_GetTick() - tickEnviamentDadesUART) > 10) {
		sendUART(UartHandle);
		tickEnviamentDadesUART = HAL_GetTick();
	}
}
