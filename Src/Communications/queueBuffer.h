/*
 * communication.h
 *
 *  Created on: 7 jul. 2020
 *      Author: isaac.ambit
 */

#ifndef QUEUEBUFFER_H_
#define QUEUEBUFFER_H_

#include "stdint.h"

#define BUFFER_CAPACITY				1024

typedef struct Queue {
	uint16_t front;
	uint16_t rear;
	uint16_t ptrData;
	uint8_t data[BUFFER_CAPACITY];
}Queue;


void initializeBuffer(Queue *buffer);
_Bool enQueue(Queue *buffer, uint8_t value);
uint8_t deQueue(Queue *buffer);
uint8_t peekQueue(Queue *buffer);
uint16_t findValue(Queue *buffer, uint8_t value);

#endif /* QUEUEBUFFER_H_ */
