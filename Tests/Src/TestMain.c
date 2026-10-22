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

void InvokeTest()
{
    OLED_Init();
    OLED_Clear();

    LED_On();

    /* Distance-terminated, open-loop loose-arc feedforward calibration. */
    MotionControllerArcTestRun();

    while (1)
    {
        // Test ended. Loop forever.
    }
}
