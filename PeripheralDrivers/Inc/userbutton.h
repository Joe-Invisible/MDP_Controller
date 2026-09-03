/*
 * userbutton.h
 *
 *  Created on: 2026年8月28日
 *      Author: Joe
 */

#ifndef INC_USERBUTTON_H_
#define INC_USERBUTTON_H_

#include "stm32f4xx_hal.h"

typedef enum SW1_State_t
{
    SW1_Idle = GPIO_PIN_SET,
    SW1_Enabled = GPIO_PIN_RESET,
} SW1_State_t;


/**
 * @brief Read the instantaneous GPIO state of SW1.
 *
 * This function performs no debouncing and does not block.
 * Use it when SW1 must be polled from an existing loop.
 */
SW1_State_t SW1_ReadState(void);


/**
 * @brief Block until SW1 has been stably pressed.
 */
void SW1_WaitForPress(void);


/**
 * @brief Block until SW1 has been stably released.
 */
void SW1_WaitForRelease(void);


/**
 * @brief Block for one complete debounced button action:
 *        press followed by release.
 */
void SW1_WaitForPressAndRelease(void);


/**
 * @deprecated
 * @brief Compatibility wrapper for existing code. New code
 * 		  that intends to start some procedure based on SW1
 * 		  press should use SW1_WaitForPressAndRelease() in-
 * 		  stead, so that subsequent polls do not get trigg-
 * 		  ered unexpectedly.
 *
 * Blocks while SW1 is not pressed and returns once a stable
 * press has been detected.
 */
void SW1_WhileNotPressed(void);


#endif /* INC_USERBUTTON_H_ */
