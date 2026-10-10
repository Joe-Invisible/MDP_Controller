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
#define FUSION_RADIUS_MM 275.0f
#define FUSION_SIXTH_TURN_MM (FUSION_RADIUS_MM * FUSION_PI / 3.0f)
#define FUSION_SQRT_3 1.73205080756887729353f
/* Mouth x=0, corrected park interior x=-500..0, y=-300..300.
 * Start at its centre (-250,0), initially heading +x. */
#define FUSION_PARK_START_X_MM (-250.0f)
#define FUSION_PARK_STEM_MM (100.0f - FUSION_PARK_START_X_MM)
#define FUSION_FACE_GAP_MM 1050.0f
#define FUSION_OBSTACLE_THICKNESS_MM 100.0f
#define FUSION_OBSTACLE_1_X_MM (FUSION_FACE_GAP_MM + 50.0f)
#define FUSION_OBSTACLE_2_X_MM \
    (FUSION_OBSTACLE_1_X_MM + FUSION_OBSTACLE_THICKNESS_MM + FUSION_FACE_GAP_MM)
/* Compact 500 mm second obstacle. Preserve 225 mm centre-to-face
 * outbound clearance: second lane is -475 mm; first remains +275 mm. */
#define FUSION_FIRST_LANE_Y_MM FUSION_RADIUS_MM
#define FUSION_OBSTACLE_2_LENGTH_MM 500.0f
#define FUSION_SECOND_LANE_Y_MM (-(FUSION_OBSTACLE_2_LENGTH_MM / 2.0f + 225.0f))
#define FUSION_ENTRY_END_X_MM (100.0f + FUSION_RADIUS_MM * FUSION_SQRT_3)
#define FUSION_CROSS_DIAGONAL_MM \
    (2.0f * (FUSION_FIRST_LANE_Y_MM - FUSION_SECOND_LANE_Y_MM - FUSION_RADIUS_MM) / FUSION_SQRT_3)
#define FUSION_CROSS_ADVANCE_MM \
    (FUSION_RADIUS_MM * FUSION_SQRT_3 + FUSION_CROSS_DIAGONAL_MM / 2.0f)
#define FUSION_CROSS_START_X_MM \
    ((FUSION_OBSTACLE_1_X_MM + FUSION_OBSTACLE_2_X_MM - FUSION_CROSS_ADVANCE_MM) / 2.0f)
#define FUSION_CROSS_END_X_MM (FUSION_CROSS_START_X_MM + FUSION_CROSS_ADVANCE_MM)
#define FUSION_FAR_TURN_X_MM (FUSION_OBSTACLE_2_X_MM + 275.0f)
#define FUSION_FIRST_LANE_MM (FUSION_CROSS_START_X_MM - FUSION_ENTRY_END_X_MM)
#define FUSION_SECOND_LANE_MM (FUSION_FAR_TURN_X_MM - FUSION_CROSS_END_X_MM)
#define FUSION_RETURN_RADIUS_MM (-FUSION_SECOND_LANE_Y_MM + 50.0f)

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
MotionSequenceFusionTestExperiment motionSequenceFusionTestExperiment =
    MOTION_SEQUENCE_FUSION_TEST_SINGLE_TUNING;
MotionSequenceFusionTestTuning motionSequenceFusionTestStraightTuning = {
    .yawRateKp = 150.0f,
    .yawRateKi = 0.0f,
    .yawRateKd = 0.0f,
    .headingKpPerSec = 0.0f,
};
MotionSequenceFusionTestTuning motionSequenceFusionTestArcTuning = {
    .yawRateKp = 150.0f,
    .yawRateKi = 0.0f,
    .yawRateKd = 0.0f,
    .headingKpPerSec = 0.0f,
};
uint32_t motionSequenceFusionTestLogPeriodMs;

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
    sample->desiredYawRad = controller->pathSample.desiredYawRad;
    sample->curvaturePerMm = controller->targetCurvaturePerMm;
    sample->commandedCurvaturePerMm = controller->arcCommandedCurvaturePerMm;
    sample->localSpeedLimitCps = controller->pathSample.speedLimitCps;
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
    sample->straightTuningWeight = controller->pathSample.straightTuningWeight;
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
    /* Circle centre is (farTurnX, secondLaneY + returnRadius). Choose
     * its upper tangent through the park start: one diagonal straight return
     * clears both obstacles and needs no extra lane change near the park. */
    const float returnDx = FUSION_FAR_TURN_X_MM - FUSION_PARK_START_X_MM;
    const float returnDy = FUSION_SECOND_LANE_Y_MM + FUSION_RETURN_RADIUS_MM;
    const float returnCentreDistance = hypotf(returnDx, returnDy);
    const float returnYawRad = FUSION_PI + atan2f(returnDy, returnDx) +
        asinf(FUSION_RETURN_RADIUS_MM / returnCentreDistance);
    const float returnDistanceMm = sqrtf(returnCentreDistance * returnCentreDistance -
        FUSION_RETURN_RADIUS_MM * FUSION_RETURN_RADIUS_MM);
    /* Task 2 outward S: first left, second right; tangent straight back.
     * Only stopAfter differs between A and B. No reversing/vision/IR stops.
     * Blends preserve distance/yaw, not exact XY closure; see route checks. */
    const MotionSequenceFusionTestStep fusionTestSteps[] = {
        /* Distance [mm], radius [mm], requested speed [CPS] */
        {FUSION_PARK_STEM_MM, 0.0f, FUSION_SPEED_CPS},
        {FUSION_SIXTH_TURN_MM, +FUSION_RADIUS_MM, FUSION_SPEED_CPS},
        {FUSION_SIXTH_TURN_MM, -FUSION_RADIUS_MM, FUSION_SPEED_CPS},
        {FUSION_FIRST_LANE_MM, 0.0f, FUSION_SPEED_CPS},
        {FUSION_SIXTH_TURN_MM, -FUSION_RADIUS_MM, FUSION_SPEED_CPS},
        {FUSION_CROSS_DIAGONAL_MM, 0.0f, FUSION_SPEED_CPS},
        {FUSION_SIXTH_TURN_MM, +FUSION_RADIUS_MM, FUSION_SPEED_CPS},
        {FUSION_SECOND_LANE_MM, 0.0f, FUSION_SPEED_CPS},
        {FUSION_RETURN_RADIUS_MM * returnYawRad, +FUSION_RETURN_RADIUS_MM, FUSION_SPEED_CPS},
        {returnDistanceMm, 0.0f, FUSION_SPEED_CPS},
    };
    _Static_assert(sizeof(fusionTestSteps) / sizeof(fusionTestSteps[0]) <=
                       MOTION_SEQUENCE_CAPACITY, "Task 2 course exceeds sequence capacity");
    const MotionSequenceFusionTestStep *steps = MotionSequenceFusionTest_IsAccuracyExperiment()
        ? filterTestSteps : fusionTestSteps;
    const size_t stepCount = MotionSequenceFusionTest_IsAccuracyExperiment()
        ? sizeof(filterTestSteps) / sizeof(filterTestSteps[0])
        : sizeof(fusionTestSteps) / sizeof(fusionTestSteps[0]);
    MotionSequence *sequence = &motionSequenceFusionTestSequence;
    MotionControllerStatus status = MotionSequence_Begin(
        sequence, &fixture.motionController, &motionSequenceConfig);
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
        MotionSequenceFusionTest_IsSingleRun() ||
        motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK1_GAINS ||
        (motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_REFERENCE_FILTER &&
         comparisonRunIndex == 1U);
    result->stopAfterEachSegment = stopAfter;
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
        OLED_Printf(0, 1, "Task2 S: %.2fm", (double)(estimatedTravelMm / 1000.0f));
    }
    OLED_Printf(0, 2, "5000 CPS requested");
    OLED_Printf(0, 3, "%s", MotionSequenceFusionTest_IsAccuracyExperiment()
        ? "Path: 1.6 x 1.5m" : "A/D S:3000 C:2000");
    OLED_Printf(0, 4, "%s", MotionSequenceFusionTest_IsAccuracyExperiment()
        ? "Reset +x; SW1 start" : "Park +x; SW1 start");
    OLED_Printf(0, 5, "SW1 running: cancel");
    if (MotionSequenceFusionTest_IsAccuracyExperiment())
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
    motionSequenceFusionTestStatus = MotionSequence_Execute(sequence);
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
                MotionSequence_Update(sequence, (nowMs - previousUpdateMs) / 1000.0f);
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
        sequence->state == MOTION_SEQUENCE_COMPLETE;

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
    ++motionSequenceFusionTestResultCount;
    return result->passed;
}

void MotionSequenceFusionTestRun(void)
{
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
    comparisonRunIndex = 0U;
    for (uint32_t i = 0U; i < MOTION_SEQUENCE_FUSION_TEST_RUN_COUNT; ++i)
    {
        motionSequenceFusionTestResults[i] = (MotionSequenceFusionTestResult){0};
    }
    motionSequenceFusionTestStatus = MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;
    if (motionSequenceFusionTestExperiment != MOTION_SEQUENCE_FUSION_TEST_TASK2_TIMING &&
        motionSequenceFusionTestExperiment != MOTION_SEQUENCE_FUSION_TEST_REFERENCE_FILTER &&
        motionSequenceFusionTestExperiment != MOTION_SEQUENCE_FUSION_TEST_TASK1_GAINS &&
        !MotionSequenceFusionTest_IsSingleRun())
    {
        motionSequenceFusionTestStatus = MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
        MotionSequenceFusionTestFinished();
        return;
    }
    motionSequenceFusionTestLogPeriodMs = MotionSequenceFusionTest_IsAccuracyExperiment()
        ? FUSION_FILTER_LOG_PERIOD_MS : FUSION_TIMING_LOG_PERIOD_MS;
    fusionMotionConfig = motionControllerConfig;
    fusionMotionConfig.useMatchedYawRateReferenceFilter = false;
    if (MotionSequenceFusionTest_IsAccuracyExperiment())
        fusionMotionConfig.arcYawRateFilterTauSec = 0.10f;
    fusionMotionConfig.straightAccelerationMmps2 = 3000.0f;
    fusionMotionConfig.straightDecelerationMmps2 = 3000.0f;
    fusionMotionConfig.arcAccelerationMmps2 = 2000.0f;
    fusionMotionConfig.arcDecelerationMmps2 = 2000.0f;
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
