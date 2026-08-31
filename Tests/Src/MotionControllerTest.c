/*
 * MotionControllerTest.c
 *
 *  Created on: 2026年8月31日
 *      Author: Joe
 */


#include "MotionControllerTest.h"
#include "oledutils.h"
#include "userbutton.h"
#include "RobotTestFixture.h"

bool MotionControllerTest_Init(RobotTestFixture *fixture) {
	OLED_Init();
	OLED_Clear();
	OLED_Refresh_Gram();

	if (!RobotTestFixture_InitMotionController(fixture, 0.0f, 0.0f, 0.0f, 0.0f))
			return false;

	return true;
}



void MotionControllerTestRun() {
	RobotTestFixture fixture = { 0 };

	if (!MotionControllerTest_Init(&fixture)) {
		OLED_Printf(0, 0, "Motion Ctrl Init Failed");
		OLED_Refresh_Gram();
		SW1_WhileNotPressed();
		return;
	}

	OLED_Printf(0, 0, "Motion Ctrl Test");
	OLED_Printf(0, 1, "Start: SW1");

	OLED_Refresh_Gram();

	SW1_WhileNotPressed();

	OLED_Printf(0, 1, "straight motion");
	OLED_Printf(0, 2, "heading correction=none");

	OLED_Refresh_Gram();
}
