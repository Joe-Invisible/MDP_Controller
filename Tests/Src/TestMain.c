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
#include "WheelSpeedControllerTest.h"
#include "MotionControllerTest.h"
#include "SteeringGeometryCalibrationTest.h"
#include "oled.h"
#include "led3.h"

void InvokeTest() {
	OLED_Init();
	OLED_Clear();

	LED_On();

	SteeringGeometryCalibrationTestRun();


	while (1) {
		// Test ended. Loop forever.
	}
}
