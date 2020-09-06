/*
 * communication.c
 *
 *  Created on: 7 jul. 2020
 *      Author: isaac.ambit
 */


#include "queueBuffer.h"

// circular Queue functions
void initializeBuffer(Queue *buffer){
	buffer->front = 0xFF;
	buffer->rear = 0xFF;
}

_Bool enQueue(Queue *buffer, uint8_t value){
	if((buffer->front == 0xFF) & (buffer->rear == 0xFF)){
		buffer->front = 0;
		buffer->rear = 0;
		buffer->data[buffer->rear] = value;
		return 1;
	}
	else if((buffer->rear < BUFFER_CAPACITY - 1) & (buffer->rear !=buffer->front - 1)){
		buffer->rear++;
		buffer->data[buffer->rear] = value;
		return 1;
	}
	  else if((buffer->rear == BUFFER_CAPACITY - 1) & (buffer->front != 0)){
		buffer->rear = 0;
		buffer->data[buffer->rear] = value;
		return 1;
	  }
	  else return 0;
}

uint8_t deQueue(Queue *buffer){
    if(buffer->front == 0xFF) return 0xFF;
    else if(buffer->front == buffer->rear){
        uint8_t tmp = buffer->data[buffer->front];
        buffer->front = 0xFF;
        buffer->rear = 0xFF;
        return tmp;
    }
    else{
        uint8_t tmp = buffer->data[buffer->front];
        if(buffer->front == BUFFER_CAPACITY-1) buffer->front = 0;
        else buffer->front++;
        return tmp;
    }
}

uint8_t peekQueue(Queue *buffer){
	return buffer->data[buffer->front];
}

uint16_t findValue(Queue *buffer, uint8_t value){
	buffer->ptrData = buffer->front;

	if(buffer->ptrData == 0xFF) return 0xFF;

	while(buffer->data[buffer->ptrData] != value){
		if(buffer->ptrData == 0xFF) return 0xFF;

		else if(buffer->ptrData == buffer->rear){
			buffer->ptrData = 0xFF;
		}

		else{
			if(buffer->ptrData == BUFFER_CAPACITY-1) buffer->ptrData = 0;
			else buffer->ptrData++;
		}
	}
	return buffer->ptrData;
}
