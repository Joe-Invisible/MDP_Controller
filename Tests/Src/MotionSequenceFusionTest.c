#include "MotionSequenceFusionTest.h"
#include "RobotTestFixture.h"
#include "oledutils.h"
#include "userbutton.h"
#include <math.h>
#include <stddef.h>

#define FUSION_CONTROL_PERIOD_MS 10U
#define FUSION_TIMING_LOG_PERIOD_MS 100U
#define FUSION_FILTER_LOG_PERIOD_MS 20U
#define FUSION_LOG_CAPACITY 650U
#define FUSION_TIMEOUT_MS 60000U
#define FUSION_BRAKE_TIMEOUT_MS 3000U
#define FUSION_INITIAL_CLEARANCE_MS 400U
#define FUSION_SPEED_CPS 5000.0f
#define FUSION_PI 3.14159265358979323846f

typedef struct
{
    float signedDistanceMm;
    float radiusMm; /* Zero selects straight motion. */
    float speedCps;
} MotionSequenceFusionTestStep;

/* Short accuracy experiment: 300 straight, +R525 semicircle, 600 straight.
 * All short comparisons fuse these three segments with identical geometry. */
static const MotionSequenceFusionTestStep filterTestSteps[] = {
    {300.0f, 0.0f, FUSION_SPEED_CPS},
    {525.0f * FUSION_PI, 525.0f, FUSION_SPEED_CPS},
    {600.0f, 0.0f, FUSION_SPEED_CPS},
};
/* Longer transition experiment: each neighbour is at least 1000 mm, so
 * the existing quarter-segment bound permits two complete 500 mm blends.
 * Keep the original short geometry for the optional historical A/B modes. */
#define FUSION_TUNING_BLEND_LENGTH_MM 500.0f
static const MotionSequenceFusionTestStep tuningTestSteps[] = {
    {1000.0f, 0.0f, FUSION_SPEED_CPS},
    {525.0f * FUSION_PI, 525.0f, FUSION_SPEED_CPS},
    {1000.0f, 0.0f, FUSION_SPEED_CPS},
};
MotionSequenceFusionTestExperiment motionSequenceFusionTestExperiment =
    MOTION_SEQUENCE_FUSION_TEST_OBSTACLE1_BYPASS;
MotionSequenceFusionTestTuning motionSequenceFusionTestStraightTuning = {
    .yawRateKp = 150.0f,
    .yawRateKi = 50.0f,
    .yawRateKd = 0.0f,
    .headingKpPerSec = 0.5f,
};
MotionSequenceFusionTestTuning motionSequenceFusionTestArcTuning = {
    .yawRateKp = 150.0f,
    .yawRateKi = 100.0f,
    .yawRateKd = 0.0f,
    .headingKpPerSec = 0.5f,
};
uint32_t motionSequenceFusionTestLogPeriodMs;
float motionSequenceFusionTestTask2BlendLengthMm = 500.0f;
bool motionSequenceFusionTestTask2AllowBlendOverlap = true;

/* Stationary O1 range before route preflight (eight-primitive mode only).
 * Measured transducer face to rear axle midpoint: 210 mm.
 * Start with chassis tail 213 mm from park back wall: axle is then 250 mm
 * from that wall (x=-250 from the mouth). Tail itself is not the pose origin. */
MotionSequenceFusionTestStartupRangeConfig motionSequenceFusionTestStartupRangeConfig = {
    .enabled = true,
    .sensorAheadOfRearAxleMm = 210.0f,
};
MotionSequenceFusionTestStartupRange motionSequenceFusionTestStartupRange;

static bool MotionSequenceFusionTest_IsObstacle1(void)
{
    return motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_OBSTACLE1_BYPASS;
}

static bool MotionSequenceFusionTest_IsTask2(void)
{
    return motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK2_TIMING ||
        motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK2_PARAMETERISED ||
        MotionSequenceFusionTest_IsObstacle1();
}

static bool MotionSequenceFusionTest_IsSingleRun(void)
{
    return motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_SINGLE_TUNING;
}

static uint32_t MotionSequenceFusionTest_RunCount(void)
{
    return MotionSequenceFusionTest_IsSingleRun() ? 1U : MOTION_SEQUENCE_FUSION_TEST_RUN_COUNT;
}

static uint32_t MotionSequenceFusionTest_RunLogCapacity(void)
{
    return FUSION_LOG_CAPACITY / MotionSequenceFusionTest_RunCount();
}

static bool MotionSequenceFusionTest_IsAccuracyExperiment(void)
{
    return motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_REFERENCE_FILTER ||
           motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK1_GAINS ||
           MotionSequenceFusionTest_IsSingleRun();
}

static const char *MotionSequenceFusionTest_RunLabel(uint32_t index)
{
    if (MotionSequenceFusionTest_IsSingleRun())
        return "Tuning";
    if (motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK1_GAINS)
        return index == 0U ? "A reduced" : "B Task1";
    return index == 0U ? "A existing" : "B matched";
}

static RobotTestFixture fixture;
/* Experiment-local settings: leave the reusable production defaults intact. */
static MotionControllerConfig fusionMotionConfig;
/* Measured near-centre entries used locally in longblend runs 10-16.
 * Freeze them here with the original positive branch; leave shared defaults
 * intact. The finite-curvature points used by B are unchanged. */
static const MotionControllerArcFeedforwardPoint task2PositiveFeedforwardPoints[] = {
    {0.000054f, -5.625f},
    {0.000183f, -11.25f},
    {0.000436f, -16.875f},
    {1.0f / 1500.0f, -22.5f},
    {1.0f / 1000.0f, -27.7f},
    {1.0f / 500.0f, -47.5f},
    {1.0f / 300.0f, -75.0f},
    {1.0f / 275.0f, -81.7f},
};
static MotionControllerArcConfig task2ArcConfig;
/* Test-local endpoint for very gentle negative return arcs. It lies exactly
 * on the existing profile interpolation from centre to the nearest measured
 * negative point; this is a derived value, not a new calibration measurement. */
static MotionControllerArcFeedforwardPoint task2NegativeFeedforwardPoints[8];
#define FUSION_WHEEL_KP 0.03f
MotionSequence motionSequenceFusionTestSequence;
MotionSequenceFusionTestSample motionSequenceFusionTestLog[FUSION_LOG_CAPACITY];
uint32_t motionSequenceFusionTestLogCount;
uint32_t motionSequenceFusionTestElapsedMs;
MotionControllerStatus motionSequenceFusionTestStatus;
bool motionSequenceFusionTestPassed;
bool motionSequenceFusionTestTimedOut;
bool motionSequenceFusionTestCancelled;
bool motionSequenceFusionTestLogTruncated;

MotionSequenceFusionTestResult
    motionSequenceFusionTestResults[MOTION_SEQUENCE_FUSION_TEST_RUN_COUNT];
uint32_t motionSequenceFusionTestResultCount;
bool motionSequenceFusionTestComparisonValid;
int32_t motionSequenceFusionTestSavedTimeMs;
float motionSequenceFusionTestSavedPercent;
static uint32_t comparisonRunIndex;
static bool runLogTruncated;
static uint32_t runLogStartCount;
static bool standaloneStoppedRun;
MotionSequenceFusionTestArcExit motionSequenceFusionTestArcExits[MOTION_SEQUENCE_CAPACITY];
uint32_t motionSequenceFusionTestArcExitCount;
MotionSequencePlan motionSequenceFusionTestFusedPlan;
Task2Route motionSequenceFusionTestTask2Routes[MOTION_SEQUENCE_FUSION_TEST_RUN_COUNT];
Obstacle1Route motionSequenceFusionTestObstacle1Routes[MOTION_SEQUENCE_FUSION_TEST_RUN_COUNT];
static Task2RoutePose estimatedPose;
static float previousPoseTravelMm;
static float previousPoseYawRad;

/* Keep one encoder/gyro pose frame through primitive resets and braking.
 * This is an estimate for diagnostics, not an absolute-position guarantee. */
static void MotionSequenceFusionTest_UpdatePose(void)
{
    const MotionSequence *sequence = &motionSequenceFusionTestSequence;
    const MotionController *controller = sequence->controller;
    bool included = sequence->state == MOTION_SEQUENCE_COMPLETE ||
                    sequence->state == MOTION_SEQUENCE_ABORTED;
    float travel = sequence->completedMeasuredTravelMm +
        (included ? 0.0f : fabsf(controller->travelledDistanceMm));
    float yaw = sequence->completedMeasuredYawRad +
        (included ? 0.0f : controller->yawDeg * FUSION_PI / 180.0f);
    float distance = travel - previousPoseTravelMm;
    float middleHeading = 0.5f * (previousPoseYawRad + yaw);
    estimatedPose.xMm += distance * cosf(middleHeading);
    estimatedPose.yMm += distance * sinf(middleHeading);
    estimatedPose.yawRad = yaw;
    previousPoseTravelMm = travel;
    previousPoseYawRad = yaw;

    if (MotionSequenceFusionTest_IsObstacle1())
    {
        if (sequence->state == MOTION_SEQUENCE_COMPLETE)
        {
            MotionSequenceFusionTestResult *result = &motionSequenceFusionTestResults[comparisonRunIndex];
            result->obstacle1ExitPoseEstimateValid = true;
            result->obstacle1ExitPoseEstimate = estimatedPose;
        }
        return;
    }
    if (motionSequenceFusionTestExperiment != MOTION_SEQUENCE_FUSION_TEST_TASK2_PARAMETERISED)
        return;
    MotionSequenceFusionTestResult *result = &motionSequenceFusionTestResults[comparisonRunIndex];
    float exitProgress = sequence->plan.junctionHalfLengthMm[3];
    for (unsigned i = 0U; i < 4U; ++i)
        exitProgress += sequence->plan.segments[i].signedDistanceMm;
    bool afterFirst = standaloneStoppedRun ? sequence->completedRuns >= 4U : travel >= exitProgress;
    if (afterFirst && !result->obstacle1ExitPoseEstimateValid)
    {
        result->obstacle1ExitPoseEstimateValid = true;
        result->obstacle1ExitPoseEstimate = estimatedPose;
    }
}

/* Test-local stopped executor: use the established standalone APIs, including
 * yaw-priority arc termination and their normal steering preparation/rate.
 * Sequence storage describes the route and aggregates logs only on side B. */
static MotionControllerStatus MotionSequenceFusionTest_LaunchStandalone(uint32_t index)
{
    MotionSequence *sequence = &motionSequenceFusionTestSequence;
    const MotionSequenceSegment *step = &sequence->plan.segments[index];
    sequence->run = (MotionSequenceRun){&sequence->plan, index, index};
    sequence->expectedBraking = false;
    MotionControllerStatus status = step->curvaturePerMm == 0.0f
        ? MotionController_MoveStraight(sequence->controller, step->signedDistanceMm, step->speedCps)
        : MotionController_MoveArc(sequence->controller, step->signedDistanceMm,
                                   1.0f / step->curvaturePerMm, step->speedCps);
    if (status == MOTIONCONTROLLER_STATUS_OK && index == 0U && MotionSequenceFusionTest_IsObstacle1())
        status = MotionController_RequestCruiseSpeed(sequence->controller,
            motionSequenceFusionTestObstacle1Routes[comparisonRunIndex].config.approachSpeedCps);
    sequence->lastStatus = status;
    sequence->state = status == MOTIONCONTROLLER_STATUS_OK
        ? MOTION_SEQUENCE_RUNNING : MOTION_SEQUENCE_FAILED;
    return status;
}

static MotionControllerStatus MotionSequenceFusionTest_Execute(void)
{
    MotionSequence *sequence = &motionSequenceFusionTestSequence;
    if (!standaloneStoppedRun)
    {
        MotionControllerStatus status = MotionSequence_Execute(sequence);
        if (MotionSequenceFusionTest_IsTask2())
            motionSequenceFusionTestFusedPlan = sequence->plan;
        if (status == MOTIONCONTROLLER_STATUS_OK && MotionSequenceFusionTest_IsObstacle1())
            status = MotionController_RequestCruiseSpeed(sequence->controller,
                motionSequenceFusionTestObstacle1Routes[comparisonRunIndex].config.approachSpeedCps);
        return status;
    }
    const MotionControllerConfig *config = sequence->controller->config;
    const float mmPerCount = FUSION_PI * config->kinematics->rearWheelDiameterMm /
        config->kinematics->rearEncoderCountsPerRev;
    if (!MotionSequencePlan_Prepare(&sequence->plan, &sequence->config, mmPerCount,
            config->straightAccelerationMmps2, config->straightDecelerationMmps2,
            config->arcAccelerationMmps2, config->arcDecelerationMmps2, 0.0f,
            sequence->controller->steering->calibration->maxCommandRatePerSec))
        return MOTIONCONTROLLER_STATUS_PROFILE_ERROR;
    return MotionSequenceFusionTest_LaunchStandalone(0U);
}

static MotionControllerStatus MotionSequenceFusionTest_Update(float dt)
{
    MotionSequence *sequence = &motionSequenceFusionTestSequence;
    if (!standaloneStoppedRun)
        return MotionSequence_Update(sequence, dt);
    MotionController *controller = sequence->controller;
    sequence->lastStatus = MotionController_Update(controller, dt);
    if (sequence->lastStatus != MOTIONCONTROLLER_STATUS_OK)
    {
        sequence->state = MOTION_SEQUENCE_FAILED;
        return sequence->lastStatus;
    }
    if (MotionController_IsBusy(controller))
        return sequence->lastStatus;

    sequence->completedMeasuredTravelMm += fabsf(controller->travelledDistanceMm);
    sequence->completedMeasuredYawRad += controller->yawDeg * FUSION_PI / 180.0f;
    if (sequence->state == MOTION_SEQUENCE_ABORTING)
    {
        sequence->state = MOTION_SEQUENCE_ABORTED;
        return sequence->lastStatus;
    }
    const MotionSequenceSegment *step = &sequence->plan.segments[sequence->run.first];
    if (step->curvaturePerMm != 0.0f)
    {
        motionSequenceFusionTestArcExits[motionSequenceFusionTestArcExitCount++] =
            (MotionSequenceFusionTestArcExit){
                .segmentIndex = sequence->run.first,
                .nominalDistanceMm = fabsf(step->signedDistanceMm),
                .measuredDistanceMm = fabsf(controller->travelledDistanceMm),
                .nominalYawRad = step->signedDistanceMm * step->curvaturePerMm,
                .measuredYawRad = controller->yawDeg * FUSION_PI / 180.0f,
                .yawPredictedReached = controller->terminalYawPredictedReached,
                .distanceLimitReached = controller->terminalDistanceLimitReached,
            };
    }
    ++sequence->completedRuns;
    const uint32_t next = sequence->run.last + 1U;
    if (next == sequence->plan.count)
    {
        sequence->state = MOTION_SEQUENCE_COMPLETE;
        return sequence->lastStatus;
    }
    return MotionSequenceFusionTest_LaunchStandalone(next);
}

static void MotionSequenceFusionTest_LogSample(uint32_t elapsedMs)
{
    if (motionSequenceFusionTestLogCount >= FUSION_LOG_CAPACITY ||
        (MotionSequenceFusionTest_IsAccuracyExperiment() &&
         motionSequenceFusionTestLogCount - runLogStartCount >= MotionSequenceFusionTest_RunLogCapacity()))
    {
        motionSequenceFusionTestLogTruncated = true;
        runLogTruncated = true;
        return;
    }

    const MotionSequence *sequence = &motionSequenceFusionTestSequence;
    const MotionController *controller = &fixture.motionController;
    MotionSequenceFusionTestSample *sample =
        &motionSequenceFusionTestLog[motionSequenceFusionTestLogCount++];
    *sample = (MotionSequenceFusionTestSample){0};
    sample->timeMs = elapsedMs;
    sample->comparisonRunIndex = comparisonRunIndex;
    sample->mode = controller->mode;
    sample->sequenceState = sequence->state;
    sample->firstSegment = sequence->run.first;
    sample->lastSegment = sequence->run.last;
    sample->runDistanceMm = controller->travelledDistanceMm;
    bool currentRunIncluded = sequence->state == MOTION_SEQUENCE_COMPLETE ||
                              sequence->state == MOTION_SEQUENCE_ABORTED;
    sample->sequenceTravelMm =
        sequence->completedMeasuredTravelMm +
        (currentRunIncluded ? 0.0f : fabsf(controller->travelledDistanceMm));
    sample->sequenceYawRad =
        sequence->completedMeasuredYawRad +
        (currentRunIncluded ? 0.0f
                            : controller->yawDeg * (3.14159265358979323846f / 180.0f));
    sample->desiredYawRad = standaloneStoppedRun
        ? controller->arcDesiredYawRad : controller->pathSample.desiredYawRad;
    sample->curvaturePerMm = controller->targetCurvaturePerMm;
    sample->commandedCurvaturePerMm = controller->arcCommandedCurvaturePerMm;
    sample->localSpeedLimitCps = standaloneStoppedRun
        ? sequence->plan.segments[sequence->run.first].speedCps : controller->pathSample.speedLimitCps;
    sample->targetSpeedCps = controller->targetSpeedCps;
    sample->measuredCentreSpeedMmps = controller->measuredCentreSpeedMmps;
    sample->filteredMeasuredCentreSpeedMmps =
        controller->filteredMeasuredCentreSpeedMmps;
    sample->yawRateDps = controller->yawRateDps;
    sample->filteredYawRateDps = controller->filteredYawRateDps;
    sample->targetYawRateRadPerSec = controller->arcTargetYawRateRadPerSec;
    sample->unfilteredGeometricYawRateRadPerSec =
        controller->arcUnfilteredFeedforwardYawRateRadPerSec;
    sample->filteredGeometricYawRateRadPerSec =
        controller->arcFilteredFeedforwardYawRateRadPerSec;
    sample->headingErrorRad = controller->arcHeadingErrorRad;
    sample->steeringFeedforward = controller->arcSteeringFeedforwardCommand;
    sample->steeringCorrection = controller->arcSteeringCorrectionCommand;
    sample->steeringTarget = controller->arcSteeringTargetCommand;
    sample->steeringCommand = fixture.steeringController.command;
    sample->leftTargetCps = fixture.leftWheelController.targetSpeedCps;
    sample->rightTargetCps = fixture.rightWheelController.targetSpeedCps;
    sample->leftMeasuredCps = fixture.leftWheelController.measuredSpeedCps;
    sample->rightMeasuredCps = fixture.rightWheelController.measuredSpeedCps;
    sample->wheelSyncErrorMm = controller->wheelSyncErrorMm;
    sample->wheelSyncCorrectionCps = controller->wheelSyncCorrectionCps;
    sample->straightTuningWeight = standaloneStoppedRun
        ? (sequence->plan.segments[sequence->run.first].curvaturePerMm == 0.0f ? 1.0f : 0.0f)
        : controller->pathSample.straightTuningWeight;
    sample->accelerationLimitMmps2 = controller->motionProfile.accelerationMmps2;
    sample->decelerationLimitMmps2 = controller->motionProfile.decelerationMmps2;
    sample->brakingSpeedLimitCps = controller->pathSample.brakingSpeedLimitCps;

}

__attribute__((noinline)) void MotionSequenceFusionTestFinished(void)
{
    const MotionSequenceFusionTestResult *fused = &motionSequenceFusionTestResults[0];
    const MotionSequenceFusionTestResult *stopped = &motionSequenceFusionTestResults[1];
    if (MotionSequenceFusionTest_IsSingleRun())
    {
        OLED_Clear();
        OLED_Printf(0, 0, "Tuning %s", motionSequenceFusionTestPassed ? "DONE" : "INVALID");
        OLED_Printf(0, 1, "Time: %lu ms", (unsigned long)fused->elapsedMs);
        OLED_Printf(0, 2, "Yaw: %+.2f deg", (double)((fused->measuredYawRad - fused->nominalYawRad) * 180.0f / FUSION_PI));
        OLED_Printf(0, 3, "Status: %u", (unsigned)motionSequenceFusionTestStatus);
        OLED_Printf(0, 4, "%s", motionSequenceFusionTestTimedOut ? "TIMEOUT" :
                    motionSequenceFusionTestCancelled ? "CANCELLED" :
                    fused->passed ? "COMPLETE" : "INCOMPLETE");
        OLED_Printf(0, 5, "%s", motionSequenceFusionTestLogTruncated ? "LOG TRUNCATED" : "20ms; export once");
        OLED_Refresh_Gram();
        return;
    }
    if (MotionSequenceFusionTest_IsAccuracyExperiment())
    {
        OLED_Clear();
        OLED_Printf(0, 0, "%s A/B %s",
                    motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK1_GAINS
                        ? "Gains" : "Filter",
                    motionSequenceFusionTestComparisonValid ? "DONE" : "INVALID");
        OLED_Printf(0, 1, "%s: %s", MotionSequenceFusionTest_RunLabel(0U), fused->passed ? "OK" : "INCOMPLETE");
        OLED_Printf(0, 2, "%s: %s", MotionSequenceFusionTest_RunLabel(1U), stopped->passed ? "OK" : "INCOMPLETE");
        OLED_Printf(0, 3, "A yaw: %+.2f deg", (double)((fused->measuredYawRad - fused->nominalYawRad) * 180.0f / FUSION_PI));
        OLED_Printf(0, 4, "B yaw: %+.2f deg", (double)((stopped->measuredYawRad - stopped->nominalYawRad) * 180.0f / FUSION_PI));
        OLED_Printf(0, 5, "%s", motionSequenceFusionTestLogTruncated ? "LOG TRUNCATED" : "20ms; export once");
        OLED_Refresh_Gram();
        return;
    }
    OLED_Clear();
    OLED_Printf(0, 0, "Pattern A/B %s", motionSequenceFusionTestComparisonValid ? "DONE" : "INVALID");
    OLED_Printf(0, 1, "A fused: %lu ms", (unsigned long)fused->elapsedMs);
    OLED_Printf(0, 2, "B stopped: %lu ms", (unsigned long)stopped->elapsedMs);
    if (motionSequenceFusionTestComparisonValid)
    {
        OLED_Printf(0, 3, "Saved: %ld ms", (long)motionSequenceFusionTestSavedTimeMs);
        OLED_Printf(0, 4, "Reduction: %.1f%%", motionSequenceFusionTestSavedPercent);
    }
    else
    {
        OLED_Printf(0, 3, "A:%s B:%s", fused->passed ? "OK" : "INCOMPLETE",
                    stopped->passed ? "OK" : "INCOMPLETE");
        OLED_Printf(0, 4, "Status: %u", (unsigned)motionSequenceFusionTestStatus);
        OLED_Printf(0, 5, "%s", motionSequenceFusionTestTimedOut ? "TIMEOUT" :
                    motionSequenceFusionTestCancelled ? "CANCELLED" : "NO COMPARISON");
    }
    OLED_Refresh_Gram();
}

static MotionControllerStatus MotionSequenceFusionTest_BuildPattern(bool stopAfter)
{
    /* Generated by Tests/Host/plan_task2_route.py. B is the planning target.
     * 200 mm body clearance; only the side gaps at obstacle 2 are 500 mm.
     * There is no minimum primitive length or back-boundary assumption. */
    const MotionSequenceFusionTestStep fusionTestSteps[] = {
        {1177.644340f, 2102.575115f, FUSION_SPEED_CPS},
        {711.288258f, -429.686391f, FUSION_SPEED_CPS},
        {474.840304f, 0.000000f, FUSION_SPEED_CPS},
        {2561.468011f, 585.845538f, FUSION_SPEED_CPS},
        {2460.422929f, 11919.893421f, FUSION_SPEED_CPS},
    };
    _Static_assert(sizeof(fusionTestSteps) / sizeof(fusionTestSteps[0]) <=
                       MOTION_SEQUENCE_CAPACITY, "Task 2 course exceeds sequence capacity");
    const MotionSequenceFusionTestStep *steps = MotionSequenceFusionTest_IsSingleRun()
        ? tuningTestSteps : MotionSequenceFusionTest_IsAccuracyExperiment()
        ? filterTestSteps : fusionTestSteps;
    const size_t stepCount = MotionSequenceFusionTest_IsSingleRun()
        ? sizeof(tuningTestSteps) / sizeof(tuningTestSteps[0])
        : MotionSequenceFusionTest_IsAccuracyExperiment()
        ? sizeof(filterTestSteps) / sizeof(filterTestSteps[0])
        : sizeof(fusionTestSteps) / sizeof(fusionTestSteps[0]);
    MotionSequenceConfig sequenceConfig = motionSequenceConfig;
    if (MotionSequenceFusionTest_IsSingleRun() ||
        motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK2_TIMING)
        sequenceConfig.blendLengthMm = FUSION_TUNING_BLEND_LENGTH_MM;
    if (motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK2_TIMING && !stopAfter)
    {
        sequenceConfig.blendLengthMm = motionSequenceFusionTestTask2BlendLengthMm;
        sequenceConfig.allowBlendOverlap = motionSequenceFusionTestTask2AllowBlendOverlap;
    }
    if (motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK2_PARAMETERISED)
    {
        sequenceConfig.blendLengthMm = motionSequenceFusionTestTask2BlendLengthMm;
        sequenceConfig.allowBlendOverlap = !stopAfter && motionSequenceFusionTestTask2AllowBlendOverlap;
    }
    if (MotionSequenceFusionTest_IsObstacle1())
    {
        const Obstacle1RouteConfig *config = &motionSequenceFusionTestObstacle1Routes[comparisonRunIndex].config;
        sequenceConfig.blendLengthMm = fmaxf(config->entryBlendMm, config->bypassBlendMm);
        sequenceConfig.allowBlendOverlap = !stopAfter;
    }
    MotionSequence *sequence = &motionSequenceFusionTestSequence;
    MotionControllerStatus status = MotionSequence_Begin(
        sequence, &fixture.motionController, &sequenceConfig);
    if (MotionSequenceFusionTest_IsObstacle1())
    {
        const Obstacle1Route *route = &motionSequenceFusionTestObstacle1Routes[comparisonRunIndex];
        for (unsigned i = 0U; i < OBSTACLE1_ROUTE_STEP_COUNT && status == MOTIONCONTROLLER_STATUS_OK; ++i)
        {
            const MotionSequenceSegment *step = &route->steps[i];
            status = step->curvaturePerMm == 0.0f
                ? MotionSequence_AddStraight(sequence, step->signedDistanceMm, step->speedCps, stopAfter)
                : MotionSequence_AddArc(sequence, step->signedDistanceMm,
                                        1.0f / step->curvaturePerMm, step->speedCps, stopAfter);
            sequence->plan.junctionBlendLimitMm[i] = route->blendLimitMm[i];
        }
        return status;
    }
    if (motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK2_PARAMETERISED)
    {
        const Task2Route *route = &motionSequenceFusionTestTask2Routes[comparisonRunIndex];
        for (unsigned i = 0U; i < TASK2_ROUTE_STEP_COUNT && status == MOTIONCONTROLLER_STATUS_OK; ++i)
        {
            const MotionSequenceSegment *step = &route->steps[i];
            status = step->curvaturePerMm == 0.0f
                ? MotionSequence_AddStraight(sequence, step->signedDistanceMm, step->speedCps, stopAfter)
                : MotionSequence_AddArc(sequence, step->signedDistanceMm,
                                        1.0f / step->curvaturePerMm, step->speedCps, stopAfter);
            sequence->plan.junctionBlendLimitMm[i] = route->blendLimitMm[i];
        }
        return status;
    }
    for (size_t stepIndex = 0U;
         stepIndex < stepCount &&
         status == MOTIONCONTROLLER_STATUS_OK; ++stepIndex)
    {
        const MotionSequenceFusionTestStep *step = &steps[stepIndex];
        if (step->radiusMm == 0.0f)
        {
            status = MotionSequence_AddStraight(
                sequence, step->signedDistanceMm, step->speedCps, stopAfter);
        }
        else
        {
            status = MotionSequence_AddArc(
                sequence, step->signedDistanceMm, step->radiusMm, step->speedCps, stopAfter);
        }
    }
    return status;
}

static bool MotionSequenceFusionTest_RunPattern(bool stopAfter)
{
    const bool task2 = MotionSequenceFusionTest_IsTask2();
    standaloneStoppedRun = task2 && stopAfter;
    MotionSequenceFusionTestResult *result =
        &motionSequenceFusionTestResults[comparisonRunIndex];
    /* Reset this branch's defaults for the timing mode. The Task 1 operating
     * point below is user-confirmed and independent of these branch defaults. */
    fusionMotionConfig.straightYawRateKp = motionControllerConfig.straightYawRateKp;
    fusionMotionConfig.straightYawRateKi = motionControllerConfig.straightYawRateKi;
    fusionMotionConfig.straightYawRateKd = motionControllerConfig.straightYawRateKd;
    fusionMotionConfig.straightHeadingKpPerSec = motionControllerConfig.straightHeadingKpPerSec;
    fusionMotionConfig.arcYawRateKp = motionControllerConfig.arcYawRateKp;
    fusionMotionConfig.arcYawRateKi = motionControllerConfig.arcYawRateKi;
    fusionMotionConfig.arcYawRateKd = motionControllerConfig.arcYawRateKd;
    fusionMotionConfig.arcHeadingKpPerSec = motionControllerConfig.arcHeadingKpPerSec;
    if (task2)
    {
        /* A: last logged pre-response-planner tuning, longblend runs 15/16.
         * B: confirmed stopped production operating point (Task 1).
         * Shared speed, feedforward, wheel tuning and motion limits isolate
         * the complete execution strategies, not just a gain change. */
        fusionMotionConfig.straightYawRateKp = stopAfter ? 270.0f : 100.0f;
        fusionMotionConfig.straightYawRateKi = stopAfter ? 150.0f : 50.0f;
        fusionMotionConfig.straightYawRateKd = 0.0f;
        fusionMotionConfig.straightHeadingKpPerSec = stopAfter ? 0.0f : 0.5f;
        fusionMotionConfig.arcYawRateKp = stopAfter ? 270.0f : 150.0f;
        fusionMotionConfig.arcYawRateKi = stopAfter ? 150.0f : 75.0f;
        fusionMotionConfig.arcYawRateKd = 0.0f;
        fusionMotionConfig.arcHeadingKpPerSec = stopAfter ? 0.0f : 0.5f;
    }
    if (motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK1_GAINS &&
        comparisonRunIndex == 1U)
    {
        fusionMotionConfig.straightYawRateKp = fusionMotionConfig.arcYawRateKp = 270.0f;
        fusionMotionConfig.straightYawRateKi = fusionMotionConfig.arcYawRateKi = 150.0f;
        fusionMotionConfig.straightYawRateKd = fusionMotionConfig.arcYawRateKd = 0.0f;
        fusionMotionConfig.straightHeadingKpPerSec = fusionMotionConfig.arcHeadingKpPerSec = 0.0f;
    }
    if (MotionSequenceFusionTest_IsAccuracyExperiment() &&
        (motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_REFERENCE_FILTER ||
         (comparisonRunIndex == 0U && !MotionSequenceFusionTest_IsSingleRun())))
    {
        fusionMotionConfig.straightYawRateKp = fusionMotionConfig.arcYawRateKp = 100.0f;
        fusionMotionConfig.straightYawRateKi = fusionMotionConfig.arcYawRateKi = 0.0f;
        fusionMotionConfig.straightYawRateKd = fusionMotionConfig.arcYawRateKd = 0.0f;
        fusionMotionConfig.straightHeadingKpPerSec = fusionMotionConfig.arcHeadingKpPerSec = 0.0f;
    }
    if (MotionSequenceFusionTest_IsSingleRun())
    {
        const MotionSequenceFusionTestTuning *straight = &motionSequenceFusionTestStraightTuning;
        const MotionSequenceFusionTestTuning *arc = &motionSequenceFusionTestArcTuning;
        fusionMotionConfig.straightYawRateKp = straight->yawRateKp;
        fusionMotionConfig.straightYawRateKi = straight->yawRateKi;
        fusionMotionConfig.straightYawRateKd = straight->yawRateKd;
        fusionMotionConfig.straightHeadingKpPerSec = straight->headingKpPerSec;
        fusionMotionConfig.arcYawRateKp = arc->yawRateKp;
        fusionMotionConfig.arcYawRateKi = arc->yawRateKi;
        fusionMotionConfig.arcYawRateKd = arc->yawRateKd;
        fusionMotionConfig.arcHeadingKpPerSec = arc->headingKpPerSec;
    }
    fusionMotionConfig.useMatchedYawRateReferenceFilter =
        (task2 && !stopAfter) || MotionSequenceFusionTest_IsSingleRun() ||
        motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK1_GAINS ||
        (motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_REFERENCE_FILTER &&
         comparisonRunIndex == 1U);
    result->stopAfterEachSegment = stopAfter;
    result->usesStandalonePrimitives = standaloneStoppedRun;
    result->yawPriorityArcExits = standaloneStoppedRun;
    result->useMatchedYawRateReferenceFilter = fusionMotionConfig.useMatchedYawRateReferenceFilter;
    result->straightTuning = (MotionSequenceFusionTestTuning){
        fusionMotionConfig.straightYawRateKp, fusionMotionConfig.straightYawRateKi,
        fusionMotionConfig.straightYawRateKd, fusionMotionConfig.straightHeadingKpPerSec};
    result->arcTuning = (MotionSequenceFusionTestTuning){
        fusionMotionConfig.arcYawRateKp, fusionMotionConfig.arcYawRateKi,
        fusionMotionConfig.arcYawRateKd, fusionMotionConfig.arcHeadingKpPerSec};
    runLogStartCount = motionSequenceFusionTestLogCount;
    runLogTruncated = false;
    motionSequenceFusionTestElapsedMs = 0U;
    motionSequenceFusionTestTimedOut = false;
    motionSequenceFusionTestCancelled = false;
    estimatedPose = (Task2RoutePose){TASK2_ROUTE_START_X_MM, 0.0f, 0.0f};
    previousPoseTravelMm = 0.0f;
    previousPoseYawRad = 0.0f;
    /* Validate each run's actual config and initialize PI gains/history before
     * its ready screen. Reuse the initialized hardware and IMU calibration. */
    motionSequenceFusionTestStatus = MotionController_Init(
        &fixture.motionController, &fixture.leftWheelController, &fixture.rightWheelController,
        &fixture.steeringController, &fixture.imu, &fusionMotionConfig);
    if (motionSequenceFusionTestStatus == MOTIONCONTROLLER_STATUS_OK)
        motionSequenceFusionTestStatus = MotionSequenceFusionTest_BuildPattern(stopAfter);
    if (motionSequenceFusionTestStatus != MOTIONCONTROLLER_STATUS_OK)
    {
        result->status = motionSequenceFusionTestStatus;
        ++motionSequenceFusionTestResultCount;
        return false;
    }

    float estimatedTravelMm = 0.0f;
    for (uint32_t i = 0U; i < motionSequenceFusionTestSequence.plan.count; ++i)
        estimatedTravelMm += fabsf(motionSequenceFusionTestSequence.plan.segments[i].signedDistanceMm);
    OLED_Clear();
    if (MotionSequenceFusionTest_IsAccuracyExperiment())
    {
        OLED_Printf(0, 0, "%s ready", MotionSequenceFusionTest_RunLabel(comparisonRunIndex));
        OLED_Printf(0, 1, "R525 180: %.2fm", (double)(estimatedTravelMm / 1000.0f));
    }
    else
    {
        OLED_Printf(0, 0, "%s pattern ready", stopAfter ? "B stopped" : "A fused");
        OLED_Printf(0, 1, MotionSequenceFusionTest_IsObstacle1() ? "O1: %.2fm" : "Task2: %.2fm", (double)(estimatedTravelMm / 1000.0f));
    }
    if (MotionSequenceFusionTest_IsObstacle1())
        OLED_Printf(0, 2, "%s:%gmm", stopAfter ? "Stopped" : "Overlap",
            (double)motionSequenceFusionTestSequence.config.blendLengthMm);
    else if (MotionSequenceFusionTest_IsSingleRun())
        OLED_Printf(0, 2, "5000 CPS; B:%gmm", (double)motionSequenceFusionTestSequence.config.blendLengthMm);
    else if (task2 && !stopAfter)
        OLED_Printf(0, 2, "%s:%gmm", motionSequenceFusionTestTask2AllowBlendOverlap ?
                    "Overlap" : "Bounded", (double)motionSequenceFusionTestTask2BlendLengthMm);
    else
        OLED_Printf(0, 2, "%s", standaloneStoppedRun ? "Arc exits: yaw first" :
                    task2 ? "5000 CPS; B:500mm" : "5000 CPS requested");
    OLED_Printf(0, 3, "%s", MotionSequenceFusionTest_IsSingleRun()
        ? "Path: 2.0 x 1.6m" : MotionSequenceFusionTest_IsAccuracyExperiment()
        ? "Path: 1.6 x 1.5m" : "A/D S:3000 C:2000");
    if (motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK2_PARAMETERISED)
    {
        const Task2RouteConfig *config = &motionSequenceFusionTestTask2Routes[comparisonRunIndex].config;
        OLED_Printf(0, 3, "O1:%c O2:%c W:%g", config->firstArrow == TASK2_ARROW_LEFT ? 'L' : 'R',
                    config->secondArrow == TASK2_ARROW_LEFT ? 'L' : 'R', (double)config->obstacle2WidthMm);
    }
    if (MotionSequenceFusionTest_IsObstacle1())
    {
        const Obstacle1RouteConfig *config = &motionSequenceFusionTestObstacle1Routes[comparisonRunIndex].config;
        OLED_Printf(0, 3, "O1:%c front:%gmm", config->arrow == TASK2_ARROW_LEFT ? 'L' : 'R',
                    (double)config->parkToObstacleMm);
    }
    OLED_Printf(0, 4, "%s", MotionSequenceFusionTest_IsAccuracyExperiment()
        ? "Reset +x; SW1 start" : "Park +x; SW1 start");
    OLED_Printf(0, 5, "SW1 running: cancel");
    if (MotionSequenceFusionTest_IsAccuracyExperiment() || task2)
    {
        const MotionControllerConfig *config = fixture.motionController.config;
        OLED_Printf(0, 6, "S P%g I%g D%g H%g",
                    (double)config->straightYawRateKp, (double)config->straightYawRateKi,
                    (double)config->straightYawRateKd, (double)config->straightHeadingKpPerSec);
        OLED_Printf(0, 7, "C P%g I%g D%g H%g",
                    (double)config->arcYawRateKp, (double)config->arcYawRateKi,
                    (double)config->arcYawRateKd, (double)config->arcHeadingKpPerSec);
    }
    OLED_Refresh_Gram();
    SW1_WaitForPressAndRelease();
    HAL_Delay(FUSION_INITIAL_CLEARANCE_MS);

    /* Start before Execute: controller preparation and final braking count.
     * The button wait, manual pose reset and hand clearance do not count. */
    MotionSequence *sequence = &motionSequenceFusionTestSequence;
    uint32_t startMs = HAL_GetTick();
    motionSequenceFusionTestStatus = MotionSequenceFusionTest_Execute();
    uint32_t previousUpdateMs = startMs;
    uint32_t lastLogMs = startMs;
    uint32_t brakeStartMs = startMs;
    bool aborting = false;
    MotionSequenceFusionTest_LogSample(0U);

    while (motionSequenceFusionTestStatus == MOTIONCONTROLLER_STATUS_OK &&
           MotionSequence_IsBusy(sequence))
    {
        uint32_t nowMs = HAL_GetTick();
        if (nowMs - previousUpdateMs < FUSION_CONTROL_PERIOD_MS)
        {
            HAL_Delay(1U);
            continue;
        }

        if (!aborting &&
            ((nowMs - startMs >= FUSION_TIMEOUT_MS) || SW1_ReadState() == SW1_Enabled))
        {
            motionSequenceFusionTestTimedOut = nowMs - startMs >= FUSION_TIMEOUT_MS;
            motionSequenceFusionTestCancelled = !motionSequenceFusionTestTimedOut;
            motionSequenceFusionTestStatus = MotionSequence_Brake(sequence);
            brakeStartMs = nowMs;
            aborting = true;
        }

        if (aborting && nowMs - brakeStartMs >= FUSION_BRAKE_TIMEOUT_MS)
        {
            MotionController_Stop(&fixture.motionController);
            motionSequenceFusionTestStatus = MOTIONCONTROLLER_STATUS_INVALID_STATE;
            sequence->lastStatus = motionSequenceFusionTestStatus;
            sequence->state = MOTION_SEQUENCE_FAILED;
            break;
        }

        if (motionSequenceFusionTestStatus == MOTIONCONTROLLER_STATUS_OK)
        {
            motionSequenceFusionTestStatus =
                MotionSequenceFusionTest_Update((nowMs - previousUpdateMs) / 1000.0f);
            MotionSequenceFusionTest_UpdatePose();
            if (MotionSequenceFusionTest_IsObstacle1() && !aborting &&
                !result->approachRecognitionReceived && MotionController_IsBusy(sequence->controller) &&
                Obstacle1Route_CameraVisible(&motionSequenceFusionTestObstacle1Routes[comparisonRunIndex].config,
                                             &estimatedPose))
            {
                /* Model an immediate camera/RPi reply. The active profile and
                 * endpoint survive this speed request; acceleration is bounded. */
                motionSequenceFusionTestStatus = MotionController_RequestCruiseSpeed(sequence->controller,
                    motionSequenceFusionTestObstacle1Routes[comparisonRunIndex].config.cruiseSpeedCps);
                if (motionSequenceFusionTestStatus == MOTIONCONTROLLER_STATUS_OK)
                {
                    result->approachRecognitionReceived = true;
                    result->approachRecognitionElapsedMs = nowMs - startMs;
                    result->approachRecognitionPoseEstimate = estimatedPose;
                }
            }
        }
        previousUpdateMs = nowMs;
        if (nowMs - lastLogMs >= motionSequenceFusionTestLogPeriodMs)
        {
            MotionSequenceFusionTest_LogSample(nowMs - startMs);
            lastLogMs = nowMs;
        }
    }

    motionSequenceFusionTestElapsedMs = HAL_GetTick() - startMs;
    result->passed =
        motionSequenceFusionTestStatus == MOTIONCONTROLLER_STATUS_OK &&
        sequence->state == MOTION_SEQUENCE_COMPLETE &&
        (!MotionSequenceFusionTest_IsObstacle1() || result->approachRecognitionReceived);

    /* Reserve/replace the last slot so a truncated log still has a final sample. */
    if (motionSequenceFusionTestLogCount == FUSION_LOG_CAPACITY ||
        (MotionSequenceFusionTest_IsAccuracyExperiment() &&
         motionSequenceFusionTestLogCount - runLogStartCount == MotionSequenceFusionTest_RunLogCapacity()))
    {
        --motionSequenceFusionTestLogCount;
        motionSequenceFusionTestLogTruncated = true;
        runLogTruncated = true;
    }
    MotionSequenceFusionTest_LogSample(motionSequenceFusionTestElapsedMs);
    result->status = motionSequenceFusionTestStatus;
    result->elapsedMs = motionSequenceFusionTestElapsedMs;
    result->timedOut = motionSequenceFusionTestTimedOut;
    result->cancelled = motionSequenceFusionTestCancelled;
    result->logTruncated = runLogTruncated;
    result->completedRuns = sequence->completedRuns;
    result->measuredTravelMm = sequence->completedMeasuredTravelMm;
    result->measuredYawRad = sequence->completedMeasuredYawRad;
    result->nominalTravelMm = sequence->plan.totalTravelMm;
    result->nominalYawRad = sequence->plan.nominalFinalYawRad;
    result->allowBlendOverlap = sequence->config.allowBlendOverlap;
    result->blendLengthMm = sequence->config.blendLengthMm;
    ++motionSequenceFusionTestResultCount;
    return result->passed;
}

void MotionSequenceFusionTestRun(void)
{
    motionSequenceFusionTestStartupRange = (MotionSequenceFusionTestStartupRange){0};
    motionSequenceFusionTestLogCount = 0U;
    motionSequenceFusionTestElapsedMs = 0U;
    motionSequenceFusionTestPassed = false;
    motionSequenceFusionTestTimedOut = false;
    motionSequenceFusionTestCancelled = false;
    motionSequenceFusionTestLogTruncated = false;
    motionSequenceFusionTestResultCount = 0U;
    motionSequenceFusionTestComparisonValid = false;
    motionSequenceFusionTestSavedTimeMs = 0;
    motionSequenceFusionTestSavedPercent = 0.0f;
    motionSequenceFusionTestArcExitCount = 0U;
    motionSequenceFusionTestFusedPlan = (MotionSequencePlan){0};
    for (unsigned i = 0U; i < MOTION_SEQUENCE_FUSION_TEST_RUN_COUNT; ++i)
    {
        motionSequenceFusionTestTask2Routes[i] = (Task2Route){0};
        motionSequenceFusionTestObstacle1Routes[i] = (Obstacle1Route){0};
    }
    standaloneStoppedRun = false;
    comparisonRunIndex = 0U;
    for (uint32_t i = 0U; i < MOTION_SEQUENCE_FUSION_TEST_RUN_COUNT; ++i)
    {
        motionSequenceFusionTestResults[i] = (MotionSequenceFusionTestResult){0};
    }
    motionSequenceFusionTestStatus = MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;
    if (motionSequenceFusionTestExperiment != MOTION_SEQUENCE_FUSION_TEST_TASK2_TIMING &&
        motionSequenceFusionTestExperiment != MOTION_SEQUENCE_FUSION_TEST_TASK2_PARAMETERISED &&
        motionSequenceFusionTestExperiment != MOTION_SEQUENCE_FUSION_TEST_REFERENCE_FILTER &&
        motionSequenceFusionTestExperiment != MOTION_SEQUENCE_FUSION_TEST_TASK1_GAINS &&
        !MotionSequenceFusionTest_IsSingleRun() && !MotionSequenceFusionTest_IsObstacle1())
    {
        motionSequenceFusionTestStatus = MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
        MotionSequenceFusionTestFinished();
        return;
    }
    motionSequenceFusionTestLogPeriodMs = MotionSequenceFusionTest_IsAccuracyExperiment()
        ? FUSION_FILTER_LOG_PERIOD_MS : FUSION_TIMING_LOG_PERIOD_MS;
    fusionMotionConfig = motionControllerConfig;
    if (MotionSequenceFusionTest_IsTask2())
    {
        task2ArcConfig = arcMotionConfig;
        task2ArcConfig.positivePoints = task2PositiveFeedforwardPoints;
        task2ArcConfig.positivePointCount = (uint32_t)(sizeof(task2PositiveFeedforwardPoints) /
            sizeof(task2PositiveFeedforwardPoints[0]));
        fusionMotionConfig.arcConfig = &task2ArcConfig;
    }
    fusionMotionConfig.useMatchedYawRateReferenceFilter = false;
    fusionMotionConfig.arcYawRateFilterTauSec = 0.10f;
    fusionMotionConfig.straightAccelerationMmps2 = 3000.0f;
    fusionMotionConfig.straightDecelerationMmps2 = 3000.0f;
    fusionMotionConfig.arcAccelerationMmps2 = 2000.0f;
    fusionMotionConfig.arcDecelerationMmps2 = 2000.0f;
    if (MotionSequenceFusionTest_IsObstacle1())
    {
        MotionSequenceConfig sequenceConfig = motionSequenceConfig;
        sequenceConfig.blendLengthMm = fmaxf(obstacle1RouteConfig.entryBlendMm, obstacle1RouteConfig.bypassBlendMm);
        sequenceConfig.allowBlendOverlap = true;
        float mmPerCount = FUSION_PI * fusionMotionConfig.kinematics->rearWheelDiameterMm /
            fusionMotionConfig.kinematics->rearEncoderCountsPerRev;
        for (unsigned i = 0U; i < MOTION_SEQUENCE_FUSION_TEST_RUN_COUNT; ++i)
        {
            if (!Obstacle1Route_Create(&obstacle1RouteConfig, &sequenceConfig, mmPerCount,
                                      i == 0U, &motionSequenceFusionTestObstacle1Routes[i]))
            {
                motionSequenceFusionTestStatus = MOTIONCONTROLLER_STATUS_PROFILE_ERROR;
                MotionSequenceFusionTestFinished();
                OLED_Printf(0, 5, "%s", motionSequenceFusionTestObstacle1Routes[i].report.reason);
                OLED_Refresh_Gram();
                return;
            }
        }
    }
    if (motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK2_PARAMETERISED)
    {
        if (motionSequenceFusionTestStartupRangeConfig.enabled)
        {
            float offset = motionSequenceFusionTestStartupRangeConfig.sensorAheadOfRearAxleMm;
            if (!isfinite(offset))
            {
                motionSequenceFusionTestStatus = MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
                MotionSequenceFusionTestFinished();
                return;
            }
            OLED_Clear();
            OLED_Printf(0, 0, "Measure O1 range");
            OLED_Printf(0, 2, "Park +x, stationary");
            OLED_Printf(0, 4, "SW1: measure once");
            OLED_Refresh_Gram();
            SW1_WaitForPressAndRelease();
            HAL_Delay(FUSION_INITIAL_CLEARANCE_MS);
            motionSequenceFusionTestStartupRange.attempted = true;
            float range;
            if (!RobotTestFixture_ReadUltrasonicMm(&range) || !isfinite(range) ||
                range < 20.0f || range > 4000.0f)
            {
                motionSequenceFusionTestStatus = MOTIONCONTROLLER_STATUS_INVALID_STATE;
                MotionSequenceFusionTestFinished();
                OLED_Printf(0, 5, "O1 range failed");
                OLED_Refresh_Gram();
                return;
            }
            /* Sensor x = starting axle x + mounting offset. The reading
             * measures sensor-to-front-face, not park-mouth-to-front-face. */
            float parkToFace = range + TASK2_ROUTE_START_X_MM + offset;
            motionSequenceFusionTestStartupRange.sensorToObstacleMm = range;
            motionSequenceFusionTestStartupRange.parkToObstacle1Mm = parkToFace;
            motionSequenceFusionTestStartupRange.valid = true;
            task2RouteConfig.parkToObstacle1Mm = parkToFace;
        }
        /* Freeze the same measured/manual layout into both sides before A.
         * Motors stay uninitialised when measurement or preflight fails. */
        MotionSequenceConfig sequenceConfig = motionSequenceConfig;
        sequenceConfig.blendLengthMm = motionSequenceFusionTestTask2BlendLengthMm;
        sequenceConfig.allowBlendOverlap = motionSequenceFusionTestTask2AllowBlendOverlap;
        float mmPerCount = FUSION_PI * fusionMotionConfig.kinematics->rearWheelDiameterMm /
            fusionMotionConfig.kinematics->rearEncoderCountsPerRev;
        for (unsigned i = 0U; i < MOTION_SEQUENCE_FUSION_TEST_RUN_COUNT; ++i)
        {
            if (!Task2Route_Create(&task2RouteConfig, &sequenceConfig, mmPerCount,
                                  i == 0U, &motionSequenceFusionTestTask2Routes[i]))
            {
                motionSequenceFusionTestStatus = MOTIONCONTROLLER_STATUS_PROFILE_ERROR;
                MotionSequenceFusionTestFinished();
                OLED_Printf(0, 5, "%s", motionSequenceFusionTestTask2Routes[i].report.reason);
                OLED_Refresh_Gram();
                return;
            }
        }
    }
    if (motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK2_PARAMETERISED)
    {
        float returnCurvature = motionSequenceFusionTestTask2Routes[1].steps[7].curvaturePerMm;
        uint32_t count = arcMotionConfig.negativePointCount;
        const MotionControllerArcFeedforwardPoint *nearCentre =
            &arcMotionConfig.negativePoints[count - 1U];
        if (returnCurvature < 0.0f && returnCurvature > nearCentre->curvaturePerMm)
        {
            if (count >= sizeof(task2NegativeFeedforwardPoints) / sizeof(task2NegativeFeedforwardPoints[0]))
            {
                motionSequenceFusionTestStatus = MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
                MotionSequenceFusionTestFinished();
                return;
            }
            for (uint32_t i = 0U; i < count; ++i)
                task2NegativeFeedforwardPoints[i] = arcMotionConfig.negativePoints[i];
            task2NegativeFeedforwardPoints[count] = (MotionControllerArcFeedforwardPoint){
                nextafterf(returnCurvature, 0.0f),
                nearCentre->rawSteeringCommand * nextafterf(returnCurvature, 0.0f) / nearCentre->curvaturePerMm,
            };
            task2ArcConfig.negativePoints = task2NegativeFeedforwardPoints;
            task2ArcConfig.negativePointCount = count + 1U;
        }
    }
    if (!RobotTestFixture_InitMotionController(&fixture, &fusionMotionConfig))
    {
        MotionSequenceFusionTestFinished();
        return;
    }
    fixture.leftWheelController.pid.kp = FUSION_WHEEL_KP;
    fixture.rightWheelController.pid.kp = FUSION_WHEEL_KP;
    fixture.leftWheelController.pid.ki = fixture.rightWheelController.pid.ki = 0.0f;
    fixture.leftWheelController.pid.kd = fixture.rightWheelController.pid.kd = 0.0f;

    for (comparisonRunIndex = 0U;
         comparisonRunIndex < MotionSequenceFusionTest_RunCount(); ++comparisonRunIndex)
    {
        if (!MotionSequenceFusionTest_RunPattern(
                !MotionSequenceFusionTest_IsAccuracyExperiment() && comparisonRunIndex == 1U))
        {
            break; /* A failed/cancelled trace never launches another trace. */
        }
    }

    motionSequenceFusionTestComparisonValid =
        !MotionSequenceFusionTest_IsSingleRun() &&
        motionSequenceFusionTestResultCount == MOTION_SEQUENCE_FUSION_TEST_RUN_COUNT &&
        motionSequenceFusionTestResults[0].passed &&
        motionSequenceFusionTestResults[1].passed &&
        motionSequenceFusionTestResults[1].elapsedMs > 0U &&
        (!MotionSequenceFusionTest_IsAccuracyExperiment() || !motionSequenceFusionTestLogTruncated);
    motionSequenceFusionTestPassed = MotionSequenceFusionTest_IsSingleRun()
        ? motionSequenceFusionTestResultCount == 1U &&
          motionSequenceFusionTestResults[0].passed && !motionSequenceFusionTestLogTruncated
        : motionSequenceFusionTestComparisonValid;
    if (motionSequenceFusionTestComparisonValid && !MotionSequenceFusionTest_IsAccuracyExperiment())
    {
        motionSequenceFusionTestSavedTimeMs =
            (int32_t)motionSequenceFusionTestResults[1].elapsedMs -
            (int32_t)motionSequenceFusionTestResults[0].elapsedMs;
        motionSequenceFusionTestSavedPercent =
            100.0f * (float)motionSequenceFusionTestSavedTimeMs /
            (float)motionSequenceFusionTestResults[1].elapsedMs;
    }
    MotionSequenceFusionTestFinished();
}
