/*
 * led.c
 *
 *  Created on: 7 jul. 2020
 *      Author: isaac.ambit
 */

#include "led.h"

GPIO_TypeDef* GPIO_PORT[LEDn] = {LED4_GPIO_PORT,
                                 LED3_GPIO_PORT,
                                 LED5_GPIO_PORT,
                                 LED6_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED4_PIN,
                                 LED3_PIN,
                                 LED5_PIN,
                                 LED6_PIN};


void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}


void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}


void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}


void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}
