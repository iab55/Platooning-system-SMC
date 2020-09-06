/*
 * communicationManager.h
 *
 *  Created on: 14 jul. 2020
 *      Author: isaac.ambit
 */

#ifndef COMMUNICATIONMANAGER_H_
#define COMMUNICATIONMANAGER_H_

#include "deserialization.h"
#include "serialization.h"
#include "../SystemVariables.h"

#define PERIODE_ENVIAMENT_DADES_UART 40
#define PERIODE_ENVIAMENT_ESTAT		 30


void manageCommunications(UART_HandleTypeDef *UartHandle, Queue *bufferInputUART);

#endif /* COMMUNICATIONMANAGER_H_ */
