#ifndef TASK2_MOTION_TEST_H
#define TASK2_MOTION_TEST_H

#include "MotionController.h"
#include "Task2Route.h"

#define TASK2_MOTION_TEST_LOG_CAPACITY 650U

typedef struct
{
    bool measureObstacle1AtStartup;
    float sensorAheadOfRearAxleMm;
    uint32_t controlPeriodMs;
    uint32_t logPeriodMs;
    uint32_t startClearanceMs;
    uint32_t timeoutMs;
    uint32_t brakeTimeoutMs;
} Task2MotionTestConfig;

typedef enum
{
    TASK2_TEST_IDLE,
    TASK2_TEST_READY,
    TASK2_TEST_RUNNING,
    TASK2_TEST_BRAKING,
    TASK2_TEST_COMPLETE,
    TASK2_TEST_CANCELLED,
    TASK2_TEST_FAILED,
} Task2MotionTestState;

typedef struct
{
    bool attempted;
    bool valid;
    float sensorToObstacleMm;
    float parkToObstacle1Mm;
} Task2MotionTestStartupRange;

typedef struct
{
    bool passed;
    bool cancelled;
    bool timedOut;
    bool logTruncated;
    MotionControllerStatus status;
    uint32_t elapsedMs;
    uint32_t completedSteps;
    float measuredTravelMm;
    float measuredYawRad;
    bool arrow1Recognised;
    uint32_t recognitionElapsedMs;
    Task2RoutePose recognitionPoseEstimate;
    bool obstacle1ExitPoseEstimateValid;
    Task2RoutePose obstacle1ExitPoseEstimate;
    Task2RoutePose finalPoseEstimate;
} Task2MotionTestResult;

typedef struct
{
    float measuredDistanceMm;
    float measuredYawRad;
    bool yawPredictedReached;
    bool distanceLimitReached;
} Task2MotionTestStepResult;

typedef struct
{
    uint32_t timeMs;
    uint32_t stepIndex;
    uint32_t mode;
    float stepTravelMm;
    float routeTravelMm;
    float routeYawRad;
    float targetSpeedCps;
    float measuredCentreSpeedMmps;
    float targetYawRateRadPerSec;
    float yawRateDps;
    float headingErrorRad;
    float steeringFeedforward;
    float steeringCorrection;
    float steeringCommand;
    float leftTargetCps;
    float rightTargetCps;
    float leftMeasuredCps;
    float rightMeasuredCps;
    float accelerationLimitMmps2;
    float decelerationLimitMmps2;
} Task2MotionTestSample;

extern Task2MotionTestConfig task2MotionTestConfig;
extern Task2MotionTestState task2MotionTestState;
extern Task2Route task2MotionTestRoute;
extern MotionControllerConfig task2MotionTestControllerConfig;
extern Task2MotionTestStartupRange task2MotionTestStartupRange;
extern Task2MotionTestResult task2MotionTestResult;
extern Task2MotionTestStepResult task2MotionTestStepResults[TASK2_ROUTE_STEP_COUNT];
extern Task2MotionTestSample task2MotionTestLog[TASK2_MOTION_TEST_LOG_CAPACITY];
extern uint32_t task2MotionTestLogCount;
extern uint32_t task2MotionTestStepIndex;

void Task2MotionTestRun(void);
/* Break here for export; all results and the final sample are ready. */
void Task2MotionTestFinished(void);
#endif
