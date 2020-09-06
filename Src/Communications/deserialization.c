/*
 * dataProcess.c
 *
 *  Created on: 10 jul. 2020
 *      Author: isaac.ambit
 */


#include "deserialization.h"

_Bool processMessage(uint8_t *message){
	switch(message[0]){
	case 0:
		return 1;
	case 11:
		BSP_LED_On(LED4);
		start = 1;
		return 1;
	case 12:
		BSP_LED_Off(LED4);
		start = 0;
		return 1;
	case 13:
		return 1;
	case 14:
		return 1;
	case 15:
		return 1;
	case 16:
		return 1;
	case 17:
		return 1;
	case 18:
		return 1;
	case 20:
		return 1;
	case 21:
		return 1;
	default:
		return 0;
	}
}

_Bool processData(Queue *buffer){
	uint8_t data[MAX_MESSAGE_SIZE] = {0};
	if(deserializeMessage(buffer, data)){
		if(data[0] == '+'){
			uint8_t i = 1;
			while(data[i] != ':'){
				if(i == MAX_MESSAGE_SIZE) return 0;
				i++;
			}

			i++;
			uint8_t messageLength = 0;
			uint8_t message[MAX_MESSAGE_SIZE] = {0};

			while((messageLength < MAX_MESSAGE_SIZE - 1) && (i < MAX_MESSAGE_SIZE)){
				message[messageLength] = data[i];
				messageLength++;
				i++;
			}
			return processMessage(message);
		}
	}
	return 0;
}

_Bool deserializeMessage(Queue *buffer, uint8_t *data){
	if(findValue(buffer,'\n') != 0xFF){
		uint16_t index = 0;
		while(peekQueue(buffer) != '\n'){
			data[index] = deQueue(buffer);
			index++;
		}
		if(index >= MAX_MESSAGE_SIZE) return 0;
		deQueue(buffer);
		return 1;
	}
	else return 0;
}
