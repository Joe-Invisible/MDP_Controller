#ifndef MOTION_SEQUENCE_FUSION_TEST_H
#define MOTION_SEQUENCE_FUSION_TEST_H

#include "MotionSequence.h"

#define MOTION_SEQUENCE_FUSION_TEST_RUN_COUNT 2U

typedef enum
{
    MOTION_SEQUENCE_FUSION_TEST_TASK2_TIMING = 0,
    MOTION_SEQUENCE_FUSION_TEST_REFERENCE_FILTER,
} MotionSequenceFusionTestExperiment;

/* Set before MotionSequenceFusionTestRun(). Default: short filter A/B. */
extern MotionSequenceFusionTestExperiment motionSequenceFusionTestExperiment;
extern uint32_t motionSequenceFusionTestLogPeriodMs;

typedef struct
{
    bool stopAfterEachSegment;
    bool useMatchedYawRateReferenceFilter;
    bool passed;
    bool timedOut;
    bool cancelled;
    bool logTruncated;
    MotionControllerStatus status;
    uint32_t elapsedMs;
    uint32_t completedRuns;
    float measuredTravelMm;
    float measuredYawRad;
    float nominalTravelMm;
    float nominalYawRad;
} MotionSequenceFusionTestResult;

typedef struct
{
    uint32_t timeMs;
    uint32_t comparisonRunIndex; /* See experiment selector and per-run result. */
    uint32_t mode;
    uint32_t sequenceState;
    uint32_t firstSegment;
    uint32_t lastSegment;
    float runDistanceMm;
    float sequenceTravelMm;
    float sequenceYawRad;
    float desiredYawRad;
    float curvaturePerMm;
    float commandedCurvaturePerMm;
    float localSpeedLimitCps;
    float targetSpeedCps;
    float measuredCentreSpeedMmps;
    float filteredMeasuredCentreSpeedMmps;
    float yawRateDps;
    float filteredYawRateDps;
    float targetYawRateRadPerSec;
    float unfilteredGeometricYawRateRadPerSec;
    float filteredGeometricYawRateRadPerSec;
    float headingErrorRad;
    float steeringFeedforward;
    float steeringCorrection;
    float steeringTarget;
    float steeringCommand;
    float leftTargetCps;
    float rightTargetCps;
    float leftMeasuredCps;
    float rightMeasuredCps;
    float wheelSyncErrorMm;
    float wheelSyncCorrectionCps;
    float straightTuningWeight;
    float accelerationLimitMmps2;
    float decelerationLimitMmps2;
    float brakingSpeedLimitCps;
} MotionSequenceFusionTestSample;

extern MotionSequenceFusionTestResult
    motionSequenceFusionTestResults[MOTION_SEQUENCE_FUSION_TEST_RUN_COUNT];
extern uint32_t motionSequenceFusionTestResultCount;
extern bool motionSequenceFusionTestComparisonValid;
extern int32_t motionSequenceFusionTestSavedTimeMs; /* stopped minus fused */
extern float motionSequenceFusionTestSavedPercent; /* relative to stopped */

void MotionSequenceFusionTestRun(void);
/* Debugger breakpoint: result globals and final log sample are ready. */
void MotionSequenceFusionTestFinished(void);

#endif
