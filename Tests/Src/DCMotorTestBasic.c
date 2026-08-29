/*
 * DCMotorTestBasic.c
 *
 *  Created on: 2026年8月24日
 *      Author: Joe
 */


#include "DCMotorTestBasic.h"
#include "oledutils.h"
#include "userbutton.h"
#include <stdlib.h>

#define MOTORBPWMSRC htim9
#define MOTORCPWMSRC htim1

#define MOTORBENC htim3
#define MOTORCENC htim4

void DCMotorTestRun() {
	OLED_Clear();
	OLED_Printf(0, 0, "Motor Test");
	OLED_Printf(0, 1, "Press SW1 to start");
	OLED_Refresh_Gram();
	SW1_WhileNotPressed();

	DCMotor lmotor = { 0 };
	DCMotor rmotor = { 0 };
	DCMotor_Init(&lmotor, &MOTORBPWMSRC, false, &MOTORBENC);
	DCMotor_Init(&rmotor, &MOTORCPWMSRC, true, &MOTORCENC);

	DCMotor_Enable(&lmotor);
	DCMotor_Enable(&rmotor);

	// 3 different speeds forward
	DCMotor_SetPWM(&lmotor, 60);
	DCMotor_SetPWM(&rmotor, 60);
	HAL_Delay(2000);

	DCMotor_SetPWM(&lmotor, 90);
	DCMotor_SetPWM(&rmotor, 90);
	HAL_Delay(2000);

	DCMotor_SetPWM(&lmotor, 100);
	DCMotor_SetPWM(&rmotor, 100);
	HAL_Delay(2000);

	// Brake
	DCMotor_Brake(&lmotor);
	DCMotor_Brake(&rmotor);
	HAL_Delay(2000);

	// 3 different speeds reverse
	DCMotor_SetPWM(&lmotor, -60);
	DCMotor_SetPWM(&rmotor, -60);
	HAL_Delay(2000);

	DCMotor_SetPWM(&lmotor, -90);
	DCMotor_SetPWM(&rmotor, -90);
	HAL_Delay(2000);

	DCMotor_SetPWM(&lmotor, -100);
	DCMotor_SetPWM(&rmotor, -100);
	HAL_Delay(2000);

	// Brake
	DCMotor_Brake(&lmotor);
	DCMotor_Brake(&rmotor);
	HAL_Delay(500);

	DCMotor_SetPWM(&lmotor, 0);
	DCMotor_SetPWM(&rmotor, 0);

	uint16_t lcount = DCMotor_GetEncoderCount(&lmotor);
	uint16_t rcount = DCMotor_GetEncoderCount(&rmotor);

	OLED_Clear();
	OLED_Printf(0, 0, "L:%hu", lcount);
	OLED_Printf(0, 1, "R:%hu", rcount);
	OLED_Printf(0, 2, "D:%d", (int)lcount - (int)rcount);
	OLED_Refresh_Gram();

}

// Minimum encoder counts to pass non-stationary test
#define MIN_TEST_COUNTS	5

void DCMotorTestMinimumPWM() {
	OLED_Clear();
	OLED_Printf(0, 0, "Minimum PWM Test");
	OLED_Printf(0, 1, "Press SW1 to start");
	OLED_Refresh_Gram();
	SW1_WhileNotPressed();

	OLED_Printf(0, 1, "Testing forward turn...");
	OLED_Refresh_Gram();

	DCMotor lmotor = { 0 };
	DCMotor rmotor = { 0 };
	DCMotor_Init(&lmotor, &MOTORBPWMSRC, false, &MOTORBENC);
	DCMotor_Init(&rmotor, &MOTORCPWMSRC, true, &MOTORCENC);

	DCMotor_Enable(&lmotor);
	DCMotor_Enable(&rmotor);

	DCMotor* targetMotor = &lmotor;

	for (int j = 0; j < 2; j++) {
		int16_t countOld = 0;

		int16_t count = DCMotor_GetEncoderCount(targetMotor);

		int16_t delta = 0;
		int8_t minPWMVal = 0;

		for (int8_t i = 40; i <= 100; i += 5) {
			DCMotor_SetPWM(targetMotor, 0);
			// Wait for motor to become stationary
			do {
				countOld = count;
				HAL_Delay(100U);
				count = DCMotor_GetEncoderCount(targetMotor);
			} while (countOld != count);

			int16_t startCount = DCMotor_GetEncoderCount(targetMotor);
			// test target
			DCMotor_SetPWM(targetMotor, i);
			HAL_Delay(1000U);

			int16_t endCount = DCMotor_GetEncoderCount(targetMotor);

			delta = endCount - startCount;

			OLED_Printf(0, j + 2, "PWM %d%% d %d", i, delta);
			OLED_Refresh_Gram();
			if (abs(delta) > MIN_TEST_COUNTS) {
				minPWMVal = i;
				break;
			}
		}

		// Wait for motor to become stationary
		DCMotor_SetPWM(targetMotor, 0);
		do {
			countOld = count;
			HAL_Delay(100U);
			count = DCMotor_GetEncoderCount(targetMotor);
		} while (countOld != count);

		if (j == 0) {
			OLED_Printf(0, j + 2, "L PWM %d%% d %d", minPWMVal, delta);

		} else {
			OLED_Printf(0, j + 2, "R PWM %d%% d %d", minPWMVal, delta);
		}
		OLED_Refresh_Gram();

		targetMotor = &rmotor;
	}

	OLED_Printf(0, 4, "Press SW1 to Continue");
	OLED_Refresh_Gram();
	SW1_WhileNotPressed();


	OLED_Clear();
	OLED_Printf(0, 0, "Minimum PWM Test");
	OLED_Printf(0, 1, "Testing reverse turn...");
	OLED_Refresh_Gram();

	targetMotor = &lmotor;

	for (int j = 0; j < 2; j++) {
		int16_t countOld = 0;

		int16_t count = DCMotor_GetEncoderCount(targetMotor);

		int16_t delta = 0;
		int8_t minPWMVal = 0;

		for (int8_t i = -40; i >= -100; i -= 5) {
			DCMotor_SetPWM(targetMotor, 0);
			// Wait for motor to become stationary
			do {
				countOld = count;
				HAL_Delay(100U);
				count = DCMotor_GetEncoderCount(targetMotor);
			} while (countOld != count);

			uint16_t startCount = DCMotor_GetEncoderCount(targetMotor);
			// test target
			DCMotor_SetPWM(targetMotor, i);
			HAL_Delay(1000U);

			int16_t endCount = DCMotor_GetEncoderCount(targetMotor);

			delta = endCount - startCount;

			OLED_Printf(0, j + 2, "PWM %d%% d %d", i, delta);
			OLED_Refresh_Gram();
			if (abs(delta) > MIN_TEST_COUNTS) {
				minPWMVal = i;
				break;
			}
		}

		// Wait for motor to become stationary
		DCMotor_SetPWM(targetMotor, 0);
		do {
			countOld = count;
			HAL_Delay(100U);
			count = DCMotor_GetEncoderCount(targetMotor);
		} while (countOld != count);

		if (j == 0) {
			OLED_Printf(0, j + 2, "L PWM %d%% d %d", minPWMVal, delta);
		} else {
			OLED_Printf(0, j + 2, "R PWM %d%% d %d", minPWMVal, delta);
		}
		OLED_Refresh_Gram();

		targetMotor = &rmotor;
	}

}









