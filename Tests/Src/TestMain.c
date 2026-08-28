/*
 * TestMain.c
 *
 *  Created on: 2026年8月24日
 *      Author: Joe
 */

#include <userbutton.h>
#include "DCMotorTestBasic.h"
#include "ServoTestBasic.h"
#include "ICM20948Test.h"
#include "OLEDTest.h"
#include "oled.h"
#include "led3.h"

void InvokeTest() {
	OLED_Init();
	OLED_Clear();
//	OLED_ShowString5x7(0, 0, (const char*)"BTT: Press SW1 to start test.");
//	OLED_Refresh_Gram();
//	SW1_WhileNotPressed();

	LED_On();

	HAL_Delay(200); // Because Init is too fast

	ICM20948TestRun();

	SW1_WhileNotPressed();

}
