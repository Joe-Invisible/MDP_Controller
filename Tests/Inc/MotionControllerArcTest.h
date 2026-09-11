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

    /*
     * Motion profile / progress
     */
    float profileTargetSpeedCps;
    float travelledDistanceMm;

    /*
     * Arc command
     */
    float targetCurvaturePerMm;
    float feedforwardSteeringAngleRad;

    /*
     * Steering
     */
    float steeringTargetAngleRad;
    float effectiveSteeringAngleRad;

    /*
     * Phase 2A yaw-rate / raw-steering control
     */
    float yawRateDps;
    float filteredYawRateDps;

    float arcTargetYawRateRadPerSec;
    float arcYawRateErrorRadPerSec;

    float arcSteeringCorrectionCommand;
    float arcSteeringTargetCommand;

    float measuredYawRateRadPerSec;

    /*
     * Actual raw command after SteeringController
     * rate limiting.
     */
    float steeringCommand;

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
     * Independently reconstructed wheel travel
     */
    float leftDistanceMm;
    float rightDistanceMm;

    /*
     * Wheel synchronisation
     */
    float wheelReferenceCurvaturePerMm;
    float leftBaseTargetCps;
    float rightBaseTargetCps;

    float desiredWheelTravelDifferenceMm;
    float wheelSyncErrorMm;
    float wheelSyncCorrectionCps;

    /*
     * Heading
     *
     * idealYawDeg is the geometric prediction:
     *
     *     psi = s * curvature
     *
     * It is most meaningful while state == MOTIONCONTROLLER_ARC.
     */
    float yawDeg;
    float idealYawDeg;

    /*
     * Braking
     */
    WheelSpeedActuatorMode leftActuatorMode;
    WheelSpeedActuatorMode rightActuatorMode;

    float leftBrakeDemand;
    float rightBrakeDemand;

    float leftBrakePWM;
    float rightBrakePWM;

} MotionControllerArcTestLogSample;


void MotionControllerArcTestRun(void);

#endif /* INC_MOTIONCONTROLLERARCTEST_H_ */
