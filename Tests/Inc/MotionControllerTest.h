/*
 * MotionControllerTest.h
 *
 *  Created on: 2026年8月31日
 *      Author: Joe
 */

#ifndef INC_MOTIONCONTROLLERTEST_H_
#define INC_MOTIONCONTROLLERTEST_H_

#include "MotionController.h"
#include "MotionControllerConfig.h"

typedef struct {
    uint32_t timeMs;
    uint32_t state;

    /*
     * Motion profile / progress
     */
    float profileTargetSpeedCps;
    float travelledDistanceMm;

    /*
     * Wheel speed control
     */
    float leftTargetCps;
    float rightTargetCps;

    float leftMeasuredCps;
    float rightMeasuredCps;

    float leftPwm;
    float rightPwm;

    /*
     * Wheel synchronisation
     */
    float desiredWheelTravelDifferenceMm;
    float wheelSyncErrorMm;
    float wheelSyncCorrectionCps;

    /*
     * Heading / steering
     */
    float yawDeg;
    float targetSteeringAngleRad;
    float effectiveSteeringAngleRad;

    /*
     * Dynamic braking
     */
    WheelSpeedActuatorMode leftActuatorMode;
    WheelSpeedActuatorMode rightActuatorMode;

    float leftBrakeDemand;
    float rightBrakeDemand;

    float leftBrakePWM;
    float rightBrakePWM;
} MotionControllerTestLogSample;
void MotionControllerTestRun();

#endif /* INC_MOTIONCONTROLLERTEST_H_ */
