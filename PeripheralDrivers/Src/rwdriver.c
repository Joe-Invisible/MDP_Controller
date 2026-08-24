/*
 * rwdriver.h
 *
 *  Created on: 2026年8月24日
 *      Author: Joe
 */


#include "rwdriver.h"

#define BRAKE_PWM_DUTY_CYCLE ((int8_t)100)

bool DCMotor_Init(
		DCMotor* rm,
		TIM_HandleTypeDef* pwmHtim,
		TIM_HandleTypeDef* encHtim,
		uint32_t encCPR,
		float wheelDiameter) {

	if (!rm || !pwmHtim || !encHtim) return false;
	if (encCPR <= 0 || wheelDiameter <= 0.0f) return false;

	rm->config.pwmHtim = pwmHtim;
	// hard-coded CH1 and 2 for now.
	rm->config.pwmChannel1 = TIM_CHANNEL_1;
	rm->config.pwmChannel2 = TIM_CHANNEL_2;
	rm->config.encHtim = encHtim;

	rm->intrin.encCPR = encCPR;
	rm->intrin.wheelDiameter = wheelDiameter;

	rm->state.encCount = 0;
	rm->state.activeDutyCycle = 0;
	rm->state.direction = -1;

	return true;
}

bool DCMotor_Enable(DCMotor* rm) {
	if (!rm) return false;


    // Make sure motor cannot move when PWM outputs are enabled
	DCMotor_SetPWM(rm, 0);

	if (HAL_TIM_PWM_Start(rm->config.pwmHtim,
						  rm->config.pwmChannel1) != HAL_OK)
		return false;

	if (HAL_TIM_PWM_Start(rm->config.pwmHtim,
						  rm->config.pwmChannel2) != HAL_OK)
		return false;


	__HAL_TIM_SET_COUNTER(rm->config.encHtim, 0);
	if(HAL_TIM_Encoder_Start(rm->config.encHtim,
			TIM_CHANNEL_ALL) != HAL_OK)
		return false;

	return true;
}

void DCMotor_Disable(DCMotor* rm) {
	if (!rm) return;

	DCMotor_SetPWM(rm, 0);

	if(HAL_TIM_Encoder_Stop(rm->config.encHtim,
			TIM_CHANNEL_ALL) != HAL_OK)
		return;

	HAL_TIM_PWM_Stop(
			rm->config.pwmHtim,
			rm->config.pwmChannel1
	);

	HAL_TIM_PWM_Stop(
			rm->config.pwmHtim,
			rm->config.pwmChannel2
	);
}

void DCMotor_SetPWM(DCMotor* rm, int8_t dutyCycle) {
	if (!rm) return;

	// get dutyCycle sign by convention
	int8_t direction = (dutyCycle >> sizeof(dutyCycle) * 8);
	dutyCycle = dutyCycle < 0 ? -dutyCycle : dutyCycle;

	// Calculate compare value
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(rm->config.pwmHtim);
	uint32_t compare = (dutyCycle * (arr + 1)) / 100;

	// Determine active PWM input
	uint32_t activeChannel = -1;
	uint32_t inactiveChannel = -1;
	if (direction == 1) {
		activeChannel = rm->config.pwmChannel1;
		inactiveChannel = rm->config.pwmChannel2;
	} else if (direction == 0) {
		activeChannel = rm->config.pwmChannel2;
		inactiveChannel = rm->config.pwmChannel1;
	} else {
		return;
	}

	rm->state.activeDutyCycle = dutyCycle;
	rm->state.direction = dutyCycle == 0 ? -1 : direction;

	// Set duty cycles
	__HAL_TIM_SET_COMPARE(
			rm->config.pwmHtim, activeChannel, compare
	);
	// Slow decay PWM, i.e., inactive=1 is also an option
	__HAL_TIM_SET_COMPARE(
			rm->config.pwmHtim, inactiveChannel, 0
	);
}

void DCMotor_Neutral(DCMotor* rm) {
	DCMotor_SetPWM(rm, 0);
}

void DCMotor_Brake(DCMotor *rm)
{
	if (!rm) return;

	rm->state.activeDutyCycle = BRAKE_PWM_DUTY_CYCLE;
	rm->state.direction = -1;

	uint32_t period =
			__HAL_TIM_GET_AUTORELOAD(rm->config.pwmHtim) + 1U;

	__HAL_TIM_SET_COMPARE(
			rm->config.pwmHtim,
			rm->config.pwmChannel1,
			period);

	__HAL_TIM_SET_COMPARE(
			rm->config.pwmHtim,
			rm->config.pwmChannel2,
			period);
}

uint16_t DCMotor_GetEncoderCount(DCMotor* rm) {
	rm->state.encCount = (uint16_t)__HAL_TIM_GET_COUNTER(rm->config.encHtim);
	return rm->state.encCount;
}
