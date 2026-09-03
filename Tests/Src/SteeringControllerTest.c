/*
 * SteeringControllerTest.c
 *
 *  Created on: 2026年9月3日
 *      Author: Joe
 */


#include "SteeringController.h"
#include "SteeringControllerConfig.h"

#include "RobotTestFixture.h"
#include "userbutton.h"
#include "oled.h"
#include "oledutils.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

void SteeringControllerTestRun() {
	RobotTestFixture fixture = { 0 };

	if (!RobotTestFixture_InitFrontWheels(&fixture)) {
		OLED_Printf(0, 1, "Servo Init failed");
		OLED_Refresh_Gram();
	}

	if (!RobotTestFixture_InitSteeringController(&fixture)) {
		OLED_Printf(0, 1, "Controller Init failed");
		OLED_Refresh_Gram();
	}

	OLED_Clear();
	OLED_Printf(0, 0, "Steering Ctrl Test");
	OLED_Printf(0, 1, "SW1: Start");
	OLED_Refresh_Gram();

	SW1_WaitForPressAndRelease();

	SteeringController_SetCommand(&fixture.steeringController, -11.0f);

	float command = SteeringController_GetCommand(&fixture.steeringController);
	float effectiveAngleRad = SteeringController_GetEffectiveAngleRad(&fixture.steeringController);

	OLED_Printf(0, 1, "Cmd: %+.1f", command);
	OLED_Printf(0, 2, "Rad: %+.3f", effectiveAngleRad);
	OLED_Refresh_Gram();

	HAL_Delay(2000U);

	SteeringController_Centre(&fixture.steeringController);

	SteeringController_SetCommand(&fixture.steeringController, 11.0f);

	command = SteeringController_GetCommand(&fixture.steeringController);
	effectiveAngleRad = SteeringController_GetEffectiveAngleRad(&fixture.steeringController);
	OLED_Printf(0, 1, "Cmd: %+.1f", command);
	OLED_Printf(0, 2, "Rad: %+.3f", effectiveAngleRad);
	OLED_Refresh_Gram();

	HAL_Delay(2000U);

	OLED_Clear();
	OLED_Printf(0, 0, "TEST COMPLETE");
	OLED_Printf(0, 1, "SW1: Exit");
	OLED_Refresh_Gram();

}
