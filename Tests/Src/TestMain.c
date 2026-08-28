/*
 * TestMain.c
 *
 *  Created on: 2026年8月24日
 *      Author: Joe
 */

#include "DCMotorTestBasic.h"
#include "ServoTestBasic.h"
#include "OLEDTest.h"
#include "oled.h"

#include "userbutton.h"

void InvokeTest() {
	OLED_Init();
	OLED_Clear();
	OLED_ShowString5x7(0, 0, (const char*)"BTT: Press SW1 to start test.");
	OLED_Refresh_Gram();
	SW1_WhileNotPressed();

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);

	// DCMotorTestRun();
	// SW1_WhileNotPressed();

	ServoTestRun();
	SW1_WhileNotPressed();
}
