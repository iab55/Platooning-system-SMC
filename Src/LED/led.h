/*
 * led.h
 *
 *  Created on: 7 jul. 2020
 *      Author: isaac.ambit
 */

#ifndef LED_H_
#define LED_H_

#include "stm32f4xx_hal.h"

#define LEDn 4

#define LED3_GPIO_PORT GPIOD
#define LED4_GPIO_PORT GPIOD
#define LED5_GPIO_PORT GPIOD
#define LED6_GPIO_PORT GPIOD

#define LED4_PIN GPIO_PIN_12
#define LED3_PIN GPIO_PIN_13
#define LED5_PIN GPIO_PIN_14
#define LED6_PIN GPIO_PIN_15


typedef enum
{
  LED4 = 0,
  LED3 = 1,
  LED5 = 2,
  LED6 = 3
} Led_TypeDef;


void BSP_LED_Init(Led_TypeDef Led);
void BSP_LED_On(Led_TypeDef Led);
void BSP_LED_Off(Led_TypeDef Led);
void BSP_LED_Toggle(Led_TypeDef Led);

#endif /* LED_H_ */
