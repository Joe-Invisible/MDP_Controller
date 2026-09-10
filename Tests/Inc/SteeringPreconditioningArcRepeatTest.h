/*
 * SteeringPreconditioningArcRepeatTest.h
 *
 * Self-propelled repeated-arc diagnostic for steering preconditioning /
 * centering history.
 *
 * Three identical constant-curvature arcs are executed inside one
 * RobotTestFixture / MotionController lifetime.  The robot is manually
 * repositioned between runs, but motion during each measurement is entirely
 * self-propelled.
 *
 * Created on: 2026年9月9日
 * Author: Joe
 */

#ifndef INC_STEERINGPRECONDITIONINGARCREPEATTEST_H_
#define INC_STEERINGPRECONDITIONINGARCREPEATTEST_H_

#include <stdbool.h>
#include <stdint.h>


#define STEERING_PRECOND_ARC_REPEAT_COUNT        (3U)
#define STEERING_PRECOND_ARC_LOG_CAPACITY        (384U)


typedef struct
{
    uint8_t runIndex;
    uint8_t state;
    uint16_t reserved;

    uint32_t timeMs;

    float travelledDistanceMm;

    float yawDeg;
    float idealYawDeg;

    float steeringCommand;
    float steeringTargetAngleRad;
    float effectiveSteeringAngleRad;

    float leftTargetCps;
    float rightTargetCps;

    float leftMeasuredCps;
    float rightMeasuredCps;

    float leftTravelMm;
    float rightTravelMm;

    float desiredWheelTravelDifferenceMm;
    float wheelSyncErrorMm;
    float wheelSyncCorrectionCps;

} SteeringPreconditioningArcRepeatLogSample;


typedef struct
{
    uint8_t runIndex;

    uint8_t commandAccepted;
    uint8_t timedOut;
    uint8_t updateFailed;

    uint8_t arcExitCaptured;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t reserved2;

    uint32_t arcExitTimeMs;

    /*
     * Steering state immediately before MotionController_MoveArc().
     */
    float startSteeringCommand;
    float startSteeringTargetAngleRad;
    float startEffectiveSteeringAngleRad;

    /*
     * State at ARC -> non-ARC transition.
     */
    float arcExitDistanceMm;
    float arcExitYawDeg;
    float arcExitIdealYawDeg;

    float arcExitLeftTravelMm;
    float arcExitRightTravelMm;

    float arcExitWheelTravelDifferenceMm;
    float arcExitDesiredWheelTravelDifferenceMm;
    float arcExitWheelSyncErrorMm;

    /*
     * Final stationary state after braking.
     */
    float finalDistanceMm;
    float finalYawDeg;

} SteeringPreconditioningArcRepeatSummary;


extern volatile SteeringPreconditioningArcRepeatLogSample
    g_steeringPreconditioningArcRepeatLog[
        STEERING_PRECOND_ARC_LOG_CAPACITY];

extern volatile uint32_t
    g_steeringPreconditioningArcRepeatLogCount;


extern volatile SteeringPreconditioningArcRepeatSummary
    g_steeringPreconditioningArcRepeatSummary[
        STEERING_PRECOND_ARC_REPEAT_COUNT];


extern volatile char
    g_steeringPreconditioningArcRepeatInfo[];


void SteeringPreconditioningArcRepeatTestRun(void);


#endif /* INC_STEERINGPRECONDITIONINGARCREPEATTEST_H_ */
