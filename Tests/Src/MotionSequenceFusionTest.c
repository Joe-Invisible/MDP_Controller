#include "MotionSequenceFusionTest.h"
#include "RobotTestFixture.h"
#include "oledutils.h"
#include "userbutton.h"
#include <math.h>
#include <stddef.h>

#define FUSION_CONTROL_PERIOD_MS 10U
#define FUSION_LOG_PERIOD_MS 100U
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
/* First bypass +275 mm; wider second bypass -550 mm. Two 60-degree
 * arcs supply 275 mm of the 825 mm crossover, the diagonal supplies 550. */
#define FUSION_FIRST_LANE_Y_MM FUSION_RADIUS_MM
#define FUSION_SECOND_LANE_Y_MM (-550.0f)
#define FUSION_ENTRY_END_X_MM (100.0f + FUSION_RADIUS_MM * FUSION_SQRT_3)
#define FUSION_CROSS_DIAGONAL_MM \
    (2.0f * (FUSION_FIRST_LANE_Y_MM - FUSION_SECOND_LANE_Y_MM - FUSION_RADIUS_MM) / FUSION_SQRT_3)
#define FUSION_CROSS_ADVANCE_MM \
    (FUSION_RADIUS_MM * FUSION_SQRT_3 + FUSION_CROSS_DIAGONAL_MM / 2.0f)
#define FUSION_CROSS_START_X_MM \
    ((FUSION_OBSTACLE_1_X_MM + FUSION_OBSTACLE_2_X_MM - FUSION_CROSS_ADVANCE_MM) / 2.0f)
#define FUSION_CROSS_END_X_MM (FUSION_CROSS_START_X_MM + FUSION_CROSS_ADVANCE_MM)
#define FUSION_FAR_TURN_X_MM (FUSION_OBSTACLE_2_X_MM + 350.0f)
#define FUSION_FIRST_LANE_MM (FUSION_CROSS_START_X_MM - FUSION_ENTRY_END_X_MM)
#define FUSION_SECOND_LANE_MM (FUSION_FAR_TURN_X_MM - FUSION_CROSS_END_X_MM)
#define FUSION_RETURN_RADIUS_MM 600.0f

typedef struct
{
    float signedDistanceMm;
    float radiusMm; /* Zero selects straight motion. */
    float speedCps;
} MotionSequenceFusionTestStep;

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

static void MotionSequenceFusionTest_LogSample(uint32_t elapsedMs)
{
    if (motionSequenceFusionTestLogCount >= FUSION_LOG_CAPACITY)
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
    MotionSequence *sequence = &motionSequenceFusionTestSequence;
    MotionControllerStatus status = MotionSequence_Begin(
        sequence, &fixture.motionController, &motionSequenceConfig);
    for (size_t stepIndex = 0U;
         stepIndex < sizeof(fusionTestSteps) / sizeof(fusionTestSteps[0]) &&
         status == MOTIONCONTROLLER_STATUS_OK; ++stepIndex)
    {
        const MotionSequenceFusionTestStep *step = &fusionTestSteps[stepIndex];
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
    result->stopAfterEachSegment = stopAfter;
    runLogTruncated = false;
    motionSequenceFusionTestElapsedMs = 0U;
    motionSequenceFusionTestTimedOut = false;
    motionSequenceFusionTestCancelled = false;
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
    OLED_Printf(0, 0, "%s pattern ready", stopAfter ? "B stopped" : "A fused");
    OLED_Printf(0, 1, "Task2 S: %.2fm", (double)(estimatedTravelMm / 1000.0f));
    OLED_Printf(0, 2, "5000 CPS requested");
    OLED_Printf(0, 3, "A/D S:3000 C:2000");
    OLED_Printf(0, 4, "Park +x; SW1 start");
    OLED_Printf(0, 5, "SW1 running: cancel");
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
        if (nowMs - lastLogMs >= FUSION_LOG_PERIOD_MS)
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
    if (motionSequenceFusionTestLogCount == FUSION_LOG_CAPACITY)
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
    fusionMotionConfig = motionControllerConfig;
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

    for (comparisonRunIndex = 0U;
         comparisonRunIndex < MOTION_SEQUENCE_FUSION_TEST_RUN_COUNT; ++comparisonRunIndex)
    {
        if (!MotionSequenceFusionTest_RunPattern(comparisonRunIndex == 1U))
        {
            break; /* A failed/cancelled trace never launches the next trace. */
        }
    }

    motionSequenceFusionTestComparisonValid =
        motionSequenceFusionTestResultCount == MOTION_SEQUENCE_FUSION_TEST_RUN_COUNT &&
        motionSequenceFusionTestResults[0].passed &&
        motionSequenceFusionTestResults[1].passed &&
        motionSequenceFusionTestResults[1].elapsedMs > 0U;
    motionSequenceFusionTestPassed = motionSequenceFusionTestComparisonValid;
    if (motionSequenceFusionTestComparisonValid)
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
