/*
 * MotionControllerArcTest.h
 *
 * Feedforward constant-curvature arc motion test.
 *
 * Created on: 2026年9月9日
 * Author: Joe
 */

#ifndef INC_MOTIONCONTROLLERARCTEST_H_
#define INC_MOTIONCONTROLLERARCTEST_H_

#include <stdint.h>

#include "WheelSpeedController.h"

typedef struct
{
    uint32_t timeMs;
    uint32_t state;

    /* Motion profile */
    float profileTargetSpeedCps;
    float travelledDistanceMm;

    /* Geometric path command */
    float targetCurvaturePerMm;
    float arcCommandedCurvaturePerMm;

    float feedforwardSteeringAngleRad;

    /* Steering controller */
    float steeringTargetAngleRad;
    float effectiveSteeringAngleRad;

    /*
     * Phase 2B: outer heading loop
     */
    float arcDesiredYawRad;
    float arcHeadingErrorRad;

    float arcFeedforwardYawRateRadPerSec;
    float arcHeadingYawRateCorrectionRadPerSec;

    /*
     * Phase 2A: inner yaw-rate loop
     */
    float yawRateDps;
    float filteredYawRateDps;

    float arcTargetYawRateRadPerSec;
    float arcYawRateErrorRadPerSec;

    float arcSteeringCorrectionCommand;
    float arcSteeringTargetCommand;

    float measuredYawRateRadPerSec;

    float steeringCommand;

    /* Wheel-speed controllers */
    float leftTargetCps;
    float rightTargetCps;

    float leftMeasuredCps;
    float rightMeasuredCps;

    float leftPwm;
    float rightPwm;

    float leftDistanceMm;
    float rightDistanceMm;

    /* Rear-wheel geometry */
    float wheelReferenceCurvaturePerMm;
    float leftBaseTargetCps;
    float rightBaseTargetCps;

    /* Synchronisation */
    float desiredWheelTravelDifferenceMm;
    float wheelSyncErrorMm;
    float wheelSyncCorrectionCps;

    /* Heading */
    float yawDeg;
    float idealYawDeg;

    /* Dynamic braking */
    WheelSpeedActuatorMode leftActuatorMode;
    WheelSpeedActuatorMode rightActuatorMode;

    float leftBrakeDemand;
    float rightBrakeDemand;

    float leftBrakePWM;
    float rightBrakePWM;

} MotionControllerArcTestLogSample;

void MotionControllerArcTestRun(void);

#endif /* INC_MOTIONCONTROLLERARCTEST_H_ */
