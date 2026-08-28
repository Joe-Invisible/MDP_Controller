/*
 * led3.c
 *
 *  Created on: 2026年8月28日
 *      Author: Joe
 */


#include "led3.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"

void LED_Toggle() {
	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
}

void LED_On() {
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}

void LED_Off() {
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
}
