/*
 * MotionControllerSequenceTest.h
 *
 * Mixed straight + arc command sequence test for MotionController.
 *
 * Created on: 2026-09-17
 * Author: Joe
 */

#ifndef INC_MOTIONCONTROLLERSEQUENCETEST_H_
#define INC_MOTIONCONTROLLERSEQUENCETEST_H_

#include <stdbool.h>
#include <stdint.h>


typedef enum
{
    MOTION_SEQUENCE_TEST_STRAIGHT = 0,
    MOTION_SEQUENCE_TEST_ARC

} MotionControllerSequenceTestCommandType;


typedef struct
{
    MotionControllerSequenceTestCommandType type;

    /*
     * Signed rear-axle-centre path length.
     */
    float distanceMm;

    /*
     * ARC only. Ignored for STRAIGHT.
     */
    float radiusMm;

    /*
     * Unsigned requested cruise speed.
     */
    float speedCps;

} MotionControllerSequenceTestCommand;


typedef struct
{
    uint32_t sequenceTimeMs;
    uint32_t commandTimeMs;

    uint32_t commandIndex;
    uint32_t commandType;

    uint32_t state;

    /*
     * Sequence-level pose bookkeeping.
     *
     * MotionController resets relative yaw/odometry for each command,
     * so these are accumulated by the test harness.
     */
    float sequenceYawDeg;
    float sequenceIdealYawDeg;
    float sequenceTravelledDistanceMm;

    /*
     * Current MotionController command-local state.
     */
    float profileTargetSpeedCps;
    float travelledDistanceMm;

    float targetCurvaturePerMm;
    float arcCommandedCurvaturePerMm;

    float feedforwardSteeringAngleRad;
    float steeringTargetAngleRad;
    float effectiveSteeringAngleRad;

    float arcDesiredYawRad;
    float arcHeadingErrorRad;

    float arcFeedforwardYawRateRadPerSec;
    float arcHeadingYawRateCorrectionRadPerSec;

    float yawRateDps;
    float filteredYawRateDps;

    float arcTargetYawRateRadPerSec;
    float arcYawRateErrorRadPerSec;

    float arcSteeringCorrectionCommand;
    float arcSteeringTargetCommand;

    float measuredYawRateRadPerSec;
    float steeringCommand;

    float leftTargetCps;
    float rightTargetCps;

    float leftMeasuredCps;
    float rightMeasuredCps;

    float leftPwm;
    float rightPwm;

    /*
     * Independent sequence-level wheel-distance integration.
     */
    float leftDistanceMm;
    float rightDistanceMm;

    float wheelReferenceCurvaturePerMm;

    float leftBaseTargetCps;
    float rightBaseTargetCps;

    float desiredWheelTravelDifferenceMm;
    float wheelSyncErrorMm;
    float wheelSyncCorrectionCps;

    float yawDeg;
    float idealYawDeg;

    uint32_t leftActuatorMode;
    uint32_t rightActuatorMode;

    float leftBrakeDemand;
    float rightBrakeDemand;

    float leftBrakePWM;
    float rightBrakePWM;

} MotionControllerSequenceTestLogSample;


typedef struct
{
    bool commandAccepted;
    bool timedOut;
    bool motionExitCaptured;

    uint32_t commandIndex;
    uint32_t commandType;

    uint32_t startTimeMs;
    uint32_t motionExitTimeMs;
    uint32_t endTimeMs;

    float requestedDistanceMm;
    float requestedRadiusMm;
    float requestedSpeedCps;

    /*
     * State at STRAIGHT/ARC -> BRAKING.
     *
     * This is the cleanest measurement of the commanded path before
     * braking/centering changes the steering state.
     */
    float motionExitDistanceMm;
    float motionExitYawDeg;
    float motionExitIdealYawDeg;

    /*
     * Fully stopped state. This is the physically relevant heading
     * from which the next sequence command begins.
     */
    float finalDistanceMm;
    float finalYawDeg;

    float sequenceYawDegAfter;
    float sequenceIdealYawDegAfter;
    float sequenceTravelledDistanceMmAfter;

} MotionControllerSequenceTestResult;


void MotionControllerSequenceTestRun(void);


#endif /* INC_MOTIONCONTROLLERSEQUENCETEST_H_ */
