/*
 * buzzer.c
 *
 *  Created on: 2026年9月17日
 *      Author: Joe
 */


#include "buzzer.h"
#include "gpio.h"

void Buzzer_On() {
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}

void Buzzer_Off() {
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

void Buzzer_BlockingBuzz(uint32_t ms) {
	Buzzer_On();

	HAL_Delay(ms);

	Buzzer_Off();
}
