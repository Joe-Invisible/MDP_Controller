#ifndef MOTION_SEQUENCE_FUSION_TEST_H
#define MOTION_SEQUENCE_FUSION_TEST_H

#include "MotionSequence.h"
#include "Task2Route.h"
#include "Obstacle1Route.h"

/* Result capacity for optional A/B modes; single tuning uses only index 0. */
#define MOTION_SEQUENCE_FUSION_TEST_RUN_COUNT 2U

typedef enum
{
    MOTION_SEQUENCE_FUSION_TEST_TASK2_TIMING = 0,
    MOTION_SEQUENCE_FUSION_TEST_REFERENCE_FILTER,
    MOTION_SEQUENCE_FUSION_TEST_TASK1_GAINS,
    MOTION_SEQUENCE_FUSION_TEST_SINGLE_TUNING,
    MOTION_SEQUENCE_FUSION_TEST_TASK2_PARAMETERISED,
    MOTION_SEQUENCE_FUSION_TEST_OBSTACLE1_BYPASS,
} MotionSequenceFusionTestExperiment;

/* Set before Run(). Default: isolated 4-primitive O1 A/B.
 * TASK2_TIMING retains the historical fixed 5-primitive experiment. */
extern MotionSequenceFusionTestExperiment motionSequenceFusionTestExperiment;
extern uint32_t motionSequenceFusionTestLogPeriodMs;
/* Task 2 A-only controls; B remains standalone. Set before Run(). */
extern float motionSequenceFusionTestTask2BlendLengthMm;
extern bool motionSequenceFusionTestTask2AllowBlendOverlap;

/* Eight-primitive mode only. Disable to retain the manual O1 distance.
 * The offset is longitudinal, measured from rear axle to transducer face. */
typedef struct
{
    bool enabled;
    float sensorAheadOfRearAxleMm;
} MotionSequenceFusionTestStartupRangeConfig;

typedef struct
{
    bool attempted;
    bool valid;
    float sensorToObstacleMm;
    float parkToObstacle1Mm;
} MotionSequenceFusionTestStartupRange;

extern MotionSequenceFusionTestStartupRangeConfig motionSequenceFusionTestStartupRangeConfig;
extern MotionSequenceFusionTestStartupRange motionSequenceFusionTestStartupRange;

typedef struct
{
    float yawRateKp;
    float yawRateKi;
    float yawRateKd;
    float headingKpPerSec;
} MotionSequenceFusionTestTuning;

/* Single-run settings, independent of production config; set before Run(). */
extern MotionSequenceFusionTestTuning motionSequenceFusionTestStraightTuning;
extern MotionSequenceFusionTestTuning motionSequenceFusionTestArcTuning;

typedef struct
{
    bool stopAfterEachSegment;
    bool usesStandalonePrimitives;
    bool yawPriorityArcExits;
    bool useMatchedYawRateReferenceFilter;
    bool allowBlendOverlap;
    float blendLengthMm;
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
    /* Actual per-run configuration; final controller config belongs to last run. */
    MotionSequenceFusionTestTuning straightTuning;
    MotionSequenceFusionTestTuning arcTuning;
    bool approachRecognitionReceived;
    uint32_t approachRecognitionElapsedMs;
    Task2RoutePose approachRecognitionPoseEstimate;
    bool obstacle1ExitPoseEstimateValid;
    Task2RoutePose obstacle1ExitPoseEstimate; /* Encoder/gyro estimate, no error bound. */
} MotionSequenceFusionTestResult;

/* Side B exits, including braking drift. A distance guard alone does not
 * demonstrate a successful heading exit. Indices are zero-based. */
typedef struct
{
    uint32_t segmentIndex;
    float nominalDistanceMm;
    float measuredDistanceMm;
    float nominalYawRad;
    float measuredYawRad;
    bool yawPredictedReached;
    bool distanceLimitReached;
} MotionSequenceFusionTestArcExit;

extern MotionSequenceFusionTestArcExit motionSequenceFusionTestArcExits[MOTION_SEQUENCE_CAPACITY];
extern uint32_t motionSequenceFusionTestArcExitCount;
/* Preserve A's prepared blend lengths/speed caps after B replaces the route. */
extern MotionSequencePlan motionSequenceFusionTestFusedPlan;
/* Frozen route/config/preflight for both sides, preserved after B replaces A. */
extern Task2Route motionSequenceFusionTestTask2Routes[MOTION_SEQUENCE_FUSION_TEST_RUN_COUNT];
extern Obstacle1Route motionSequenceFusionTestObstacle1Routes[MOTION_SEQUENCE_FUSION_TEST_RUN_COUNT];

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
