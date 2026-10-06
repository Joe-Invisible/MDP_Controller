#include "MotionSequenceFusionTest.h"
#include "RobotTestFixture.h"
#include "oledutils.h"
#include "userbutton.h"
#include <math.h>
#include <stddef.h>

#define FUSION_CONTROL_PERIOD_MS 10U
#define FUSION_LOG_PERIOD_MS 20U
#define FUSION_LOG_CAPACITY 500U
#define FUSION_TIMEOUT_MS 20000U
#define FUSION_BRAKE_TIMEOUT_MS 3000U
#define FUSION_SPEED_CPS 2000.0f
/* Set to true for a stop-at-each-waypoint PROFILE comparison. This uses the
 * same profile endpoint policy on both sides of the experiment. */
#define FUSION_STOP_EACH_SEGMENT false

typedef struct
{
    float signedDistanceMm;
    float radiusMm; /* Zero selects straight motion. */
    float speedCps;
    bool stopAfter;
} MotionSequenceFusionTestStep;

/* Edit this table to change the test sequence. Individual stopAfter flags
 * remain available when FUSION_STOP_EACH_SEGMENT is false. */
static const MotionSequenceFusionTestStep fusionTestSteps[] = {
    /* Distance [mm], radius [mm], speed [CPS], stop after */
    {300.0f,    0.0f, FUSION_SPEED_CPS, false},
    {500.0f, -500.0f, FUSION_SPEED_CPS, false},
    {300.0f,    0.0f, FUSION_SPEED_CPS, false},
};

static RobotTestFixture fixture;
MotionSequence motionSequenceFusionTestSequence;
MotionSequenceFusionTestSample motionSequenceFusionTestLog[FUSION_LOG_CAPACITY];
uint32_t motionSequenceFusionTestLogCount;
uint32_t motionSequenceFusionTestElapsedMs;
MotionControllerStatus motionSequenceFusionTestStatus;
bool motionSequenceFusionTestPassed;
bool motionSequenceFusionTestTimedOut;
bool motionSequenceFusionTestCancelled;
bool motionSequenceFusionTestLogTruncated;

static void MotionSequenceFusionTest_LogSample(uint32_t elapsedMs)
{
    if (motionSequenceFusionTestLogCount >= FUSION_LOG_CAPACITY)
    {
        motionSequenceFusionTestLogTruncated = true;
        return;
    }

    const MotionSequence *sequence = &motionSequenceFusionTestSequence;
    const MotionController *controller = &fixture.motionController;
    MotionSequenceFusionTestSample *sample =
        &motionSequenceFusionTestLog[motionSequenceFusionTestLogCount++];
    *sample = (MotionSequenceFusionTestSample){0};
    sample->timeMs = elapsedMs;
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
}

__attribute__((noinline)) void MotionSequenceFusionTestFinished(void)
{
    OLED_Clear();
    OLED_Printf(0, 0, "Fusion %s", motionSequenceFusionTestPassed ? "DONE" : "ENDED");
    OLED_Printf(0, 16, "Status: %u", (unsigned)motionSequenceFusionTestStatus);
    OLED_Printf(
        0, 32, "Time: %lu ms", (unsigned long)motionSequenceFusionTestElapsedMs);
    OLED_Refresh_Gram();
}

void MotionSequenceFusionTestRun(void)
{
    motionSequenceFusionTestLogCount = 0U;
    motionSequenceFusionTestElapsedMs = 0U;
    motionSequenceFusionTestPassed = false;
    motionSequenceFusionTestTimedOut = false;
    motionSequenceFusionTestCancelled = false;
    motionSequenceFusionTestLogTruncated = false;
    motionSequenceFusionTestStatus = MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;
    if (!RobotTestFixture_InitMotionController(&fixture, &motionControllerConfig))
    {
        MotionSequenceFusionTestFinished();
        return;
    }

    /* Build the sequence from the test table, stopping at the first error. */
    MotionSequence *sequence = &motionSequenceFusionTestSequence;
    motionSequenceFusionTestStatus = MotionSequence_Begin(
        sequence, &fixture.motionController, &motionSequenceConfig);
    for (size_t stepIndex = 0U;
         stepIndex < sizeof(fusionTestSteps) / sizeof(fusionTestSteps[0]) &&
             motionSequenceFusionTestStatus == MOTIONCONTROLLER_STATUS_OK;
         ++stepIndex)
    {
        const MotionSequenceFusionTestStep *step = &fusionTestSteps[stepIndex];
        bool stopAfter = FUSION_STOP_EACH_SEGMENT || step->stopAfter;

        if (step->radiusMm == 0.0f)
        {
            motionSequenceFusionTestStatus = MotionSequence_AddStraight(
                sequence,
                step->signedDistanceMm,
                step->speedCps,
                stopAfter);
        }
        else
        {
            motionSequenceFusionTestStatus = MotionSequence_AddArc(
                sequence,
                step->signedDistanceMm,
                step->radiusMm,
                step->speedCps,
                stopAfter);
        }
    }
    if (motionSequenceFusionTestStatus != MOTIONCONTROLLER_STATUS_OK)
    {
        MotionSequenceFusionTestFinished();
        return;
    }

    OLED_Clear();
    OLED_Printf(0, 0, "Fusion ready");
    OLED_Printf(0, 16, "SW1 start / stop");
    OLED_Refresh_Gram();
    SW1_WaitForPressAndRelease();
    motionSequenceFusionTestStatus = MotionSequence_Execute(sequence);
    uint32_t startMs = HAL_GetTick();
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
    motionSequenceFusionTestPassed =
        motionSequenceFusionTestStatus == MOTIONCONTROLLER_STATUS_OK &&
        sequence->state == MOTION_SEQUENCE_COMPLETE;

    /* Reserve/replace the last slot so a truncated log still has a final sample. */
    if (motionSequenceFusionTestLogCount == FUSION_LOG_CAPACITY)
    {
        --motionSequenceFusionTestLogCount;
        motionSequenceFusionTestLogTruncated = true;
    }
    MotionSequenceFusionTest_LogSample(motionSequenceFusionTestElapsedMs);
    MotionSequenceFusionTestFinished();
}
