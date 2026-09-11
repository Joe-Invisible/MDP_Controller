/*
 * hal_callback.c
 *
 *  Created on: 2026年9月6日
 *      Author: Joe
 */


#include "RobotPeripherals.h"
#include "CommandLink.h"

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	HCSR04_HandleInputCapture(&hcsr04, htim);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	CommandLink_IsrRxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	CommandLink_IsrError(huart);
}
