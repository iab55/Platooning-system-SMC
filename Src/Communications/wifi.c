/*
 * wifi.c
 *
 *  Created on: 10 jul. 2020
 *      Author: isaac.ambit
 */


#include "wifi.h"

void configureWifi(UART_HandleTypeDef *UartHandle){
	HAL_Delay(500);

	uint8_t ATECommand[] = "ATE0\r\n";
	HAL_UART_Transmit_IT(UartHandle, ATECommand, COUNTOF(ATECommand) -1);

	HAL_Delay(500);
	uint8_t CWMODECommand[] = "AT+CWMODE=1\r\n";
	HAL_UART_Transmit_IT(UartHandle, CWMODECommand, COUNTOF(CWMODECommand) -1);

	HAL_Delay(500);
	uint8_t CIPSTACommand[66] = "AT+CIPSTA=\"";
	strcat((char *)CIPSTACommand, IP_SERVER);
	strcat((char *)CIPSTACommand, "\",\"");
	strcat((char *)CIPSTACommand, GATEWAY);
	strcat((char *)CIPSTACommand, "\",\"");
	strcat((char *)CIPSTACommand, MASK);
	strcat((char *)CIPSTACommand, "\"\r\n");
	HAL_UART_Transmit_IT(UartHandle, CIPSTACommand, COUNTOF(CIPSTACommand) -1);

	HAL_Delay(500);
	uint8_t CWJAPCommand[65] = "AT+CWJAP=\"";
	strcat((char *)CWJAPCommand, SSID);
	strcat((char *)CWJAPCommand, "\",\"");
	strcat((char *)CWJAPCommand, PASSWORD);
	strcat((char *)CWJAPCommand, "\"\r\n");
	HAL_UART_Transmit_IT(UartHandle, CWJAPCommand, COUNTOF(CWJAPCommand) -1);

	HAL_Delay(7000);
}

void connect(UART_HandleTypeDef *UartHandle){
	HAL_Delay(500);

	uint8_t CIPMUXCommand[] = "AT+CIPMUX=1\r\n";
	HAL_UART_Transmit_IT(UartHandle, CIPMUXCommand, COUNTOF(CIPMUXCommand) -1);

	HAL_Delay(500);

	uint8_t UDPClose[] = "AT+CIPCLOSE=0\r\n";
	HAL_UART_Transmit_IT(UartHandle, UDPClose, COUNTOF(UDPClose) -1);

	HAL_Delay(500);

	/*uint8_t UDPOpen[53] = "AT+CIPSTART=0,\"UDP\",\"";
	strcat((char *)UDPOpen, IP_CLIENT);
	strcat((char *)UDPOpen, "\",");
	strcat((char *)UDPOpen, REMOTE_PORT);
	strcat((char *)UDPOpen, ",");
	strcat((char *)UDPOpen, LOCAL_PORT);
	strcat((char *)UDPOpen, ",");
	strcat((char *)UDPOpen, NUMBER_OF_PORTS);
	strcat((char *)UDPOpen, "\r\n");*/

	uint8_t UDPOpen[] = "AT+CIPSTART=0,\"UDP\",\"192.168.173.10\",6790,6789,2\r\n";
	HAL_UART_Transmit_IT(UartHandle, UDPOpen, COUNTOF(UDPOpen) -1);

	HAL_Delay(500);
}
