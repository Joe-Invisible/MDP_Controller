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

    float leftTargetCps;
    float rightTargetCps;

    float leftMeasuredCps;
    float rightMeasuredCps;

    float leftPwm;
    float rightPwm;

    float leftDistanceMm;
    float rightDistanceMm;

    float speedDifferenceCps;
    float distanceDifferenceMm;

    float yawDeg;

    float controllerDistanceDifferenceMm;
    float desiredWheelTravelDifferenceMm;

    float wheelSyncErrorMm;
    float wheelSyncCorrectionCps;

    float steeringCommand;
    float effectiveSteeringAngleRad;
    bool steeringBacklashActive;
    int8_t steeringMovementDirection;

    float targetSteeringAngleRad;
    float headingErrorRad;

    bool steeringReversalPending;
} MotionControllerTestLogSample;

void MotionControllerTestRun();

#endif /* INC_MOTIONCONTROLLERTEST_H_ */
