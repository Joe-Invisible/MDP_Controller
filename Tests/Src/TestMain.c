/*
 * TestMain.c
 *
 * Test branch entry point.
 *
 *  Created on: 2026年8月24日
 *      Author: Joe
 */

#include <userbutton.h>
#include "DCMotorTestBasic.h"
#include "ServoTestBasic.h"
#include "ICM20948Test.h"
#include "HCSR04Test.h"
#include "GP2Y0A21YKTest.h"
#include "OLEDTest.h"
#include "WheelSpeedControllerTest.h"
#include "SteeringControllerTest.h"
#include "DynamicBrakeMapTest.h"
#include "MotionProfileTest.h"
#include "RobotKinematicsTest.h"
#include "MotionControllerArcTest.h"
#include "MotionControllerSequenceTest.h"
#include "MotionControllerStraightFeedforwardTest.h"
#include "MotionControllerTest.h"
#include "oled.h"
#include "led3.h"

void InvokeTest() {
	OLED_Init();
	OLED_Clear();

	LED_On();

	/* Select this separate harness for straight feedforward calibration:
	 * MotionControllerStraightFeedforwardTestRun();
	 * The original arc harness retains the pending raw +95 setup.
	 * Stationary ultrasonic bring-up (no button needed): HCSR04TestRun(); */
	MotionControllerSequenceTestRun();

	while (1) {
		// Test ended. Loop forever.
	}
}
