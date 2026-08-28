/*
 * userbutton.h
 *
 *  Created on: 2026年8月28日
 *      Author: Joe
 */

#ifndef INC_USERBUTTON_H_
#define INC_USERBUTTON_H_

#include "stm32f4xx_hal.h"

typedef enum SW1_State_t {
	SW1_Idle = GPIO_PIN_SET,
	SW1_Enabled = GPIO_PIN_RESET,
} SW1_State_t;

/**
 * SW delay loop that blocks when SW1 is not pressed
 */
void SW1_WhileNotPressed();

/**
 * Read GPIO state of SW1
 */
SW1_State_t SW1_ReadState();

#endif /* INC_USERBUTTON_H_ */
