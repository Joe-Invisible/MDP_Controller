/*
 * DCMotorTestBasic.c
 *
 *  Created on: 2026年8月24日
 *      Author: Joe
 */


#include "DCMotorTestBasic.h"

#define MOTORBPWMSRC htim9
#define MOTORCPWMSRC htim1

#define MOTORBENC htim3
#define MOTORCENC htim4

void DCMotorTestRun() {
	DCMotor lmotor = { 0 };
	DCMotor rmotor = { 0 };
	DCMotor_Init(&lmotor, &MOTORBPWMSRC, &MOTORBENC, MG512P30_QUAD_RESOLUTION, 65);
	DCMotor_Init(&rmotor, &MOTORCPWMSRC, &MOTORCENC, MG512P30_QUAD_RESOLUTION, 65);

	DCMotor_Enable(&lmotor);
	DCMotor_Enable(&rmotor);

	// 3 different speeds forward
	DCMotor_SetPWM(&lmotor, 25);
	DCMotor_SetPWM(&rmotor, 25);
	HAL_Delay(2000);

	DCMotor_SetPWM(&lmotor, 50);
	DCMotor_SetPWM(&rmotor, 50);
	HAL_Delay(2000);

	DCMotor_SetPWM(&lmotor, 75);
	DCMotor_SetPWM(&rmotor, 75);
	HAL_Delay(2000);

	// Brake
	DCMotor_Brake(&lmotor);
	DCMotor_Brake(&rmotor);
	HAL_Delay(2000);

	// 3 different speeds reverse
	DCMotor_SetPWM(&lmotor, -25);
	DCMotor_SetPWM(&rmotor, -25);
	HAL_Delay(2000);

	DCMotor_SetPWM(&lmotor, -50);
	DCMotor_SetPWM(&rmotor, -50);
	HAL_Delay(2000);

	DCMotor_SetPWM(&lmotor, -75);
	DCMotor_SetPWM(&rmotor, -75);
	HAL_Delay(2000);

	// Brake
	DCMotor_Brake(&lmotor);
	DCMotor_Brake(&rmotor);
	HAL_Delay(2000);

	DCMotor_GetEncoderCount(&lmotor);
	DCMotor_GetEncoderCount(&rmotor);
}
