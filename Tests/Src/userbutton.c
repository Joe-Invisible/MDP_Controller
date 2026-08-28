/*
 * userbutton.h
 *
 *  Created on: 2026年8月28日
 *      Author: Joe
 */


#include "userbutton.h"

#include "gpio.h"

void SW1_WhileNotPressed() {
	// escapes when SW1 reads low.
	while (HAL_GPIO_ReadPin(USERBTN_GPIO_Port, USERBTN_Pin));
}
