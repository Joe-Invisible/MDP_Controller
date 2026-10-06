#include "MotionSequenceFusionTest.h"
#include "RobotTestFixture.h"
#include "oledutils.h"
#include "userbutton.h"
#include <math.h>

#define FUSION_CONTROL_PERIOD_MS 10U
#define FUSION_LOG_PERIOD_MS 20U
#define FUSION_LOG_CAPACITY 500U
#define FUSION_TIMEOUT_MS 20000U
#define FUSION_BRAKE_TIMEOUT_MS 3000U
#define FUSION_SPEED_CPS 2000.0f
/* Set to true for a stop-at-each-waypoint PROFILE comparison. This uses the
 * same distance-based endpoint policy on both sides of the experiment. */
#define FUSION_STOP_EACH_SEGMENT false

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

static void logSample(uint32_t elapsed)
{
    if (motionSequenceFusionTestLogCount >= FUSION_LOG_CAPACITY) {
        motionSequenceFusionTestLogTruncated = true;
        return;
    }
    const MotionSequence *s = &motionSequenceFusionTestSequence;
    const MotionController *m = &fixture.motionController;
    MotionSequenceFusionTestSample *v =
        &motionSequenceFusionTestLog[motionSequenceFusionTestLogCount++];
    *v = (MotionSequenceFusionTestSample){0};
    v->timeMs = elapsed;
    v->mode = m->mode;
    v->sequenceState = s->state;
    v->firstSegment = s->run.first;
    v->lastSegment = s->run.last;
    v->runDistanceMm = m->travelledDistanceMm;
    bool currentRunIncluded = s->state == MOTION_SEQUENCE_COMPLETE ||
                              s->state == MOTION_SEQUENCE_ABORTED;
    v->sequenceTravelMm = s->completedMeasuredTravelMm +
        (currentRunIncluded ? 0.0f : fabsf(m->travelledDistanceMm));
    v->sequenceYawRad = s->completedMeasuredYawRad +
        (currentRunIncluded ? 0.0f : m->yawDeg * (3.14159265358979323846f / 180.0f));
    v->desiredYawRad = m->pathSample.desiredYawRad;
    v->curvaturePerMm = m->targetCurvaturePerMm;
    v->commandedCurvaturePerMm = m->arcCommandedCurvaturePerMm;
    v->localSpeedLimitCps = m->pathSample.speedLimitCps;
    v->targetSpeedCps = m->targetSpeedCps;
    v->measuredCentreSpeedMmps = m->measuredCentreSpeedMmps;
    v->filteredMeasuredCentreSpeedMmps = m->filteredMeasuredCentreSpeedMmps;
    v->yawRateDps = m->yawRateDps;
    v->filteredYawRateDps = m->filteredYawRateDps;
    v->targetYawRateRadPerSec = m->arcTargetYawRateRadPerSec;
    v->headingErrorRad = m->arcHeadingErrorRad;
    v->steeringFeedforward = m->arcSteeringFeedforwardCommand;
    v->steeringCorrection = m->arcSteeringCorrectionCommand;
    v->steeringTarget = m->arcSteeringTargetCommand;
    v->steeringCommand = fixture.steeringController.command;
    v->leftTargetCps = fixture.leftWheelController.targetSpeedCps;
    v->rightTargetCps = fixture.rightWheelController.targetSpeedCps;
    v->leftMeasuredCps = fixture.leftWheelController.measuredSpeedCps;
    v->rightMeasuredCps = fixture.rightWheelController.measuredSpeedCps;
    v->wheelSyncErrorMm = m->wheelSyncErrorMm;
    v->wheelSyncCorrectionCps = m->wheelSyncCorrectionCps;
    v->straightTuningWeight = m->pathSample.straightTuningWeight;
}

__attribute__((noinline)) void MotionSequenceFusionTestFinished(void)
{
    OLED_Clear();
    OLED_Printf(0, 0, "Fusion %s", motionSequenceFusionTestPassed ? "DONE" : "ENDED");
    OLED_Printf(0, 16, "Status: %u", (unsigned)motionSequenceFusionTestStatus);
    OLED_Printf(0, 32, "Time: %lu ms", (unsigned long)motionSequenceFusionTestElapsedMs);
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
    if (!RobotTestFixture_InitMotionController(&fixture, &motionControllerConfig)) {
        MotionSequenceFusionTestFinished();
        return;
    }
    MotionSequence *s = &motionSequenceFusionTestSequence;
    motionSequenceFusionTestStatus = MotionSequence_Begin(s,
        &fixture.motionController, &motionSequenceConfig);
    if (motionSequenceFusionTestStatus == MOTIONCONTROLLER_STATUS_OK)
        motionSequenceFusionTestStatus = MotionSequence_AddStraight(s, 300.0f,
            FUSION_SPEED_CPS, FUSION_STOP_EACH_SEGMENT);
    if (motionSequenceFusionTestStatus == MOTIONCONTROLLER_STATUS_OK)
        motionSequenceFusionTestStatus = MotionSequence_AddArc(s, 500.0f, -500.0f,
            FUSION_SPEED_CPS, FUSION_STOP_EACH_SEGMENT);
    if (motionSequenceFusionTestStatus == MOTIONCONTROLLER_STATUS_OK)
        motionSequenceFusionTestStatus = MotionSequence_AddStraight(s, 300.0f,
            FUSION_SPEED_CPS, FUSION_STOP_EACH_SEGMENT);
    if (motionSequenceFusionTestStatus != MOTIONCONTROLLER_STATUS_OK) {
        MotionSequenceFusionTestFinished();
        return;
    }
    OLED_Clear();
    OLED_Printf(0, 0, "Fusion ready");
    OLED_Printf(0, 16, "SW1 start / stop");
    OLED_Refresh_Gram();
    SW1_WaitForPressAndRelease();
    motionSequenceFusionTestStatus = MotionSequence_Execute(s);
    uint32_t start = HAL_GetTick(), previous = start, lastLog = start;
    uint32_t brakeStart = start;
    bool aborting = false;
    logSample(0U);
    while (motionSequenceFusionTestStatus == MOTIONCONTROLLER_STATUS_OK &&
           MotionSequence_IsBusy(s)) {
        uint32_t now = HAL_GetTick();
        if (now - previous < FUSION_CONTROL_PERIOD_MS) {
            HAL_Delay(1U);
            continue;
        }
        if (!aborting && ((now - start >= FUSION_TIMEOUT_MS) ||
                          SW1_ReadState() == SW1_Enabled)) {
            motionSequenceFusionTestTimedOut = now - start >= FUSION_TIMEOUT_MS;
            motionSequenceFusionTestCancelled = !motionSequenceFusionTestTimedOut;
            motionSequenceFusionTestStatus = MotionSequence_Brake(s);
            brakeStart = now;
            aborting = true;
        }
        if (aborting && now - brakeStart >= FUSION_BRAKE_TIMEOUT_MS) {
            MotionController_Stop(&fixture.motionController);
            motionSequenceFusionTestStatus = MOTIONCONTROLLER_STATUS_INVALID_STATE;
            s->lastStatus = motionSequenceFusionTestStatus;
            s->state = MOTION_SEQUENCE_FAILED;
            break;
        }
        if (motionSequenceFusionTestStatus == MOTIONCONTROLLER_STATUS_OK)
            motionSequenceFusionTestStatus = MotionSequence_Update(s, (now - previous) / 1000.0f);
        previous = now;
        if (now - lastLog >= FUSION_LOG_PERIOD_MS) {
            logSample(now - start);
            lastLog = now;
        }
    }
    motionSequenceFusionTestElapsedMs = HAL_GetTick() - start;
    motionSequenceFusionTestPassed = motionSequenceFusionTestStatus == MOTIONCONTROLLER_STATUS_OK &&
        s->state == MOTION_SEQUENCE_COMPLETE;
    /* Reserve/replace the last slot so a truncated log still has a final sample. */
    if (motionSequenceFusionTestLogCount == FUSION_LOG_CAPACITY) {
        --motionSequenceFusionTestLogCount;
        motionSequenceFusionTestLogTruncated = true;
    }
    logSample(motionSequenceFusionTestElapsedMs);
    MotionSequenceFusionTestFinished();
}
