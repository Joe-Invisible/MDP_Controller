/*
 * hal_callback.c
 *
 *  Created on: 2026年9月6日
 *      Author: Joe
 */


#include "RobotPeripherals.h"

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	HCSR04_HandleInputCapture(&hcsr04, htim);
}
