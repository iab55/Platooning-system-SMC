/*
 * wifi.h
 *
 *  Created on: 10 jul. 2020
 *      Author: isaac.ambit
 */

#ifndef WIFI_H_
#define WIFI_H_

#include "stm32f4xx_hal.h"
#include <string.h>
#include "stdint.h"

#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

#define IP_CLIENT			"192.168.173.010"
#define GATEWAY				"192.168.173.001"
#define IP_SERVER	 		"192.168.173.100"
#define MASK				"255.255.255.000"
#define SSID				"Redmi2"
#define PASSWORD			"Iab23++2"
#define NUMBER_OF_PORTS		"2"
#define REMOTE_PORT			"6790"
#define LOCAL_PORT			"6789"


void configureWifi(UART_HandleTypeDef *UartHandle);
void connect(UART_HandleTypeDef *UartHandle);

#endif /* WIFI_H_ */
