#ifndef MOTION_RESPONSE_TEST_H
#define MOTION_RESPONSE_TEST_H
#include "MotionResponsePlan.h"
#include "MotionController.h"
#define MOTION_RESPONSE_LOG_CAPACITY 700U
/* 96 bytes; one run/export. Pose inferred from encoders+gyro is not an
 * independent ground-position measurement. */
typedef struct
{
    uint32_t timeMs;
    uint8_t mode, phase;
    uint16_t reserved;
    float progressMm, rawReference, rawCommand, bodyCurvaturePerMm, wheelCurvaturePerMm;
    float desiredYawDeg, yawDeg, yawRateDps, filteredYawRateDps;
    float expectedXmm, expectedYmm, estimatedXmm, estimatedYmm;
    float leftTargetCps, rightTargetCps, leftMeasuredCps, rightMeasuredCps;
    float wheelSyncErrorMm, wheelSyncCorrectionCps;
    float measuredCentreSpeedMmps, geometricYawRateDps, rawCommandRatePerSec;
} MotionResponseTestSample;
typedef enum
{
    RESPONSE_NOT_RUN,
    RESPONSE_COMPLETED,
    RESPONSE_PREPARATION_ERROR,
    RESPONSE_CANCELLED,
    RESPONSE_TIMED_OUT,
    RESPONSE_LOG_FULL,
    RESPONSE_CONTROLLER_ERROR,
    RESPONSE_BRAKE_TIMED_OUT
} MotionResponseOutcome;
typedef struct
{
    MotionResponseOutcome outcome;
    MotionControllerStatus status;
    uint32_t elapsedMs, maxUpdateIntervalMs;
    float minXmm, maxXmm, minYmm, maxYmm;
    MotionResponsePose plannedEnd, estimatedEnd;
    float measuredDistanceMm;
} MotionResponseTestResult;
extern MotionResponsePlan motionResponsePlan;
extern MotionControllerConfig motionResponseConfig;
extern MotionResponseTestSample motionResponseLog[MOTION_RESPONSE_LOG_CAPACITY];
extern MotionResponseTestResult motionResponseResult;
extern uint32_t motionResponseLogCount;
extern bool motionResponseFinished;
void MotionResponseTestRun(void);
void MotionResponseTestFinished(void);
#endif
