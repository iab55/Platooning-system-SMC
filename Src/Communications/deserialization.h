/*
 * dataProcess.h
 *
 *  Created on: 10 jul. 2020
 *      Author: isaac.ambit
 */

#ifndef DESERIALIZATION_H_
#define DESERIALIZATION_H_

#include "stdint.h"
#include "../SystemVariables.h"
#include "../LED/led.h"
#include "queueBuffer.h"

#define MAX_MESSAGE_SIZE 70

extern _Bool start;

_Bool processMessage(uint8_t *message);
_Bool processData(Queue *buffer);
_Bool deserializeMessage(Queue *buffer, uint8_t *data);

#endif /* DESERIALIZATION_H_ */
