/*
 * SteeringPreconditioningArcRepeatTest.c
 *
 * Purpose
 * -------
 * Test whether repeated deterministic steering centering / preconditioning
 * inside ONE live controller instance produces a systematic change in
 * physical curvature.
 *
 * The previous hand-pushed steering calibration showed a strong repeat-order
 * trend, but manual pushing is itself a yaw disturbance.  This test removes
 * that confounder by using the normal self-propelled MotionController arc.
 *
 * Experimental structure
 * ----------------------
 *
 * One RobotTestFixture is initialized once.
 *
 * For run 1, 2 and 3:
 *
 *     user repositions robot
 *         ->
 *     deterministic SteeringController centering
 *         ->
 *     100 ms centre settle
 *         ->
 *     pre-position to arc feedforward effective angle for 500 ms
 *         ->
 *     hold another 500 ms
 *         ->
 *     resynchronise wheel-speed encoder references
 *         ->
 *     execute identical +10 m radius / 1 m arc at 2000 CPS
 *         ->
 *     brake to IDLE
 *
 * The fixture, SteeringController object and MotionController object are NOT
 * reinitialized between runs.
 *
 * Compare this three-run batch against the earlier run04/run05/run06 batch,
 * where each invocation created a fresh RobotTestFixture.
 *
 * Created on: 2026年9月9日
 * Author: Joe
 */

#include "SteeringPreconditioningArcRepeatTest.h"

#include "RobotTestFixture.h"
#include "RobotKinematics.h"

#include "oledutils.h"
#include "userbutton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>


/* -------------------------------------------------------------------------- */
/* Test configuration                                                         */
/* -------------------------------------------------------------------------- */

#define TEST_DISTANCE_MM                  (1000.0f)
#define TEST_RADIUS_MM                    (10000.0f)
#define TEST_SPEED_CPS                    (2000.0f)

#define TEST_CONTROL_PERIOD_MS            (10U)
#define TEST_CONTROL_PERIOD_S             (0.010f)

#define TEST_LOG_INTERVAL_MS              (50U)

#define TEST_TIMEOUT_MS                   (10000U)
#define TEST_BRAKE_TIMEOUT_MS             (3000U)

#define TEST_CENTRE_SETTLE_MS             (100U)

#define TEST_STEERING_PREPOSITION_MS      (500U)
#define TEST_STEERING_HOLD_MS             (500U)

#define TEST_HANDS_OFF_MS                 (400U)


/*
 * Keep the motion-control settings identical to the recent feedforward arc
 * batch.
 */
#define TEST_SYNC_KP_CPS_PER_MM           (10.0f)
#define TEST_SYNC_MAX_CORRECTION_CPS      (100.0f)

#define TEST_HEADING_KP                   (0.0f)
#define TEST_HEADING_KI                   (0.0f)
#define TEST_HEADING_KD                   (0.0f)
#define TEST_HEADING_LIMIT_RAD            (0.010f)

#define TEST_ACCELERATION_MMPS2           (500.0f)
#define TEST_DECELERATION_MMPS2           (250.0f)

static const MotionControllerConfig testMotionConfig =
{
    .kinematics = &kinematics,
    .arcConfig = &arcMotionConfig,

    .headingKp = TEST_HEADING_KP,
    .headingKi = TEST_HEADING_KI,
    .headingKd = TEST_HEADING_KD,
    .maxHeadingSteeringAngleRad = TEST_HEADING_LIMIT_RAD,

    .arcYawRateKp = 10.0f,
    .arcYawRateKi = 0.0f,
    .arcYawRateKd = 0.0f,
    .maxArcSteeringCommandCorrection = 5.0f,

    .arcHeadingKpPerSec = 1.0f,

    .wheelSyncKpCpsPerMm = TEST_SYNC_KP_CPS_PER_MM,
    .maxWheelSyncCorrectionCps = TEST_SYNC_MAX_CORRECTION_CPS,

    .motionAccelerationMmps2 = TEST_ACCELERATION_MMPS2,
    .motionDecelerationMmps2 = TEST_DECELERATION_MMPS2,
    .motionCompletionToleranceMm = 0.5f,

    .arcYawRateFilterTauSec = 0.10f,

    .stopStableSampleCount = 3U,
};


#define TEST_PI                           (3.14159265358979323846f)
#define TEST_RAD_TO_DEG                   (180.0f / TEST_PI)

#define TEST_CURVATURE_PER_MM             (1.0f / TEST_RADIUS_MM)

#define TEST_NOMINAL_YAW_DEG \
    ((TEST_DISTANCE_MM / TEST_RADIUS_MM) * TEST_RAD_TO_DEG)


/* -------------------------------------------------------------------------- */
/* Debugger-exportable state                                                  */
/* -------------------------------------------------------------------------- */

volatile SteeringPreconditioningArcRepeatLogSample
    g_steeringPreconditioningArcRepeatLog[
        STEERING_PRECOND_ARC_LOG_CAPACITY];

volatile uint32_t
    g_steeringPreconditioningArcRepeatLogCount = 0U;


volatile SteeringPreconditioningArcRepeatSummary
    g_steeringPreconditioningArcRepeatSummary[
        STEERING_PRECOND_ARC_REPEAT_COUNT];


volatile char g_steeringPreconditioningArcRepeatInfo[] =
    "3 self-propelled arcs in ONE fixture lifetime; "
    "R=+10000mm, S=1000mm, 2000CPS, "
    "accel=500, decel=250, Ksync=10, heading feedback disabled, "
    "deterministic centre + 500ms preposition + 500ms hold";


/* -------------------------------------------------------------------------- */
/* Utility                                                                    */
/* -------------------------------------------------------------------------- */

static float Test_GetIdealYawDeg(
    float travelledDistanceMm)
{
    return
        travelledDistanceMm *
        TEST_CURVATURE_PER_MM *
        TEST_RAD_TO_DEG;
}


static void Test_ResetExportState(void)
{
    g_steeringPreconditioningArcRepeatLogCount = 0U;

    for (uint32_t i = 0U;
         i < STEERING_PRECOND_ARC_REPEAT_COUNT;
         i++)
    {
        g_steeringPreconditioningArcRepeatSummary[i] =
            (SteeringPreconditioningArcRepeatSummary){0};

        g_steeringPreconditioningArcRepeatSummary[i].runIndex =
            (uint8_t)i;
    }
}


/* -------------------------------------------------------------------------- */
/* Steering preparation                                                       */
/* -------------------------------------------------------------------------- */

static void Test_DeterministicCentre(
    RobotTestFixture *fixture)
{
    SteeringController_StartCentre(
        &fixture->steeringController);

    while (!SteeringController_UpdateCentre(
        &fixture->steeringController,
        TEST_CONTROL_PERIOD_S))
    {
        HAL_Delay(
            TEST_CONTROL_PERIOD_MS);
    }

    /*
     * Match the startup procedure used by the recent arc experiment.
     */
    HAL_Delay(
        TEST_CENTRE_SETTLE_MS);
}


static void Test_PrepositionArcSteering(
    RobotTestFixture *fixture)
{
    const float targetSteeringAngleRad =
        RobotKinematics_GetSteeringAngleRad(
            fixture->motionController.config->kinematics,
            TEST_CURVATURE_PER_MM);

    const uint32_t updateCount =
        TEST_STEERING_PREPOSITION_MS /
        TEST_CONTROL_PERIOD_MS;

    /*
     * Drive SteeringController through the same 100 Hz interface used
     * during normal motion.  This preserves the configured raw-command
     * slew limit and backlash handling.
     */
    for (uint32_t i = 0U;
         i < updateCount;
         i++)
    {
        SteeringController_SetEffectiveAngleRad(
            &fixture->steeringController,
            targetSteeringAngleRad,
            TEST_CONTROL_PERIOD_S);

        HAL_Delay(
            TEST_CONTROL_PERIOD_MS);
    }

    /*
     * Separate mechanical hold after the commanded position has settled.
     */
    HAL_Delay(
        TEST_STEERING_HOLD_MS);
}


/*
 * The user physically repositions the robot between runs.  MotionController
 * resets its own odometry when MoveArc() begins, but the two low-level
 * WheelSpeedControllers also keep an encoder reference for speed estimation.
 *
 * Resynchronise those references immediately before each new motion so wheel
 * rotation during manual repositioning cannot appear as the first speed
 * sample of the next run.
 */
static void Test_ResynchroniseWheelSpeedControllers(
    RobotTestFixture *fixture)
{
    WheelSpeedController_Stop(
        fixture->motionController.leftWheel);

    WheelSpeedController_Stop(
        fixture->motionController.rightWheel);
}


/* -------------------------------------------------------------------------- */
/* Logging                                                                    */
/* -------------------------------------------------------------------------- */

static void Test_LogSample(
    RobotTestFixture *fixture,
    uint8_t runIndex,
    uint32_t elapsedMs)
{
    if (g_steeringPreconditioningArcRepeatLogCount >=
        STEERING_PRECOND_ARC_LOG_CAPACITY)
    {
        return;
    }

    MotionController *controller =
        &fixture->motionController;

    WheelSpeedController *leftWheel =
        controller->leftWheel;

    WheelSpeedController *rightWheel =
        controller->rightWheel;

    uint32_t i =
        g_steeringPreconditioningArcRepeatLogCount;

    volatile SteeringPreconditioningArcRepeatLogSample *sample =
        &g_steeringPreconditioningArcRepeatLog[i];

    sample->runIndex =
        runIndex;

    sample->state =
        (uint8_t)controller->mode;

    sample->reserved =
        0U;

    sample->timeMs =
        elapsedMs;

    sample->travelledDistanceMm =
        controller->travelledDistanceMm;

    sample->yawDeg =
        controller->yawDeg;

    sample->idealYawDeg =
        Test_GetIdealYawDeg(
            controller->travelledDistanceMm);

    sample->steeringCommand =
        SteeringController_GetCommand(
            controller->steering);

    sample->steeringTargetAngleRad =
        SteeringController_GetTargetEffectiveAngleRad(
            controller->steering);

    sample->effectiveSteeringAngleRad =
        SteeringController_GetEffectiveAngleRad(
            controller->steering);

    sample->leftTargetCps =
        leftWheel->targetSpeedCps;

    sample->rightTargetCps =
        rightWheel->targetSpeedCps;

    sample->leftMeasuredCps =
        leftWheel->measuredSpeedCps;

    sample->rightMeasuredCps =
        rightWheel->measuredSpeedCps;

    sample->leftTravelMm =
        controller->leftTravelMm;

    sample->rightTravelMm =
        controller->rightTravelMm;

    sample->desiredWheelTravelDifferenceMm =
        controller->desiredWheelTravelDifferenceMm;

    sample->wheelSyncErrorMm =
        controller->wheelSyncErrorMm;

    sample->wheelSyncCorrectionCps =
        controller->wheelSyncCorrectionCps;

    g_steeringPreconditioningArcRepeatLogCount++;
}


static void Test_CaptureStartSteering(
    RobotTestFixture *fixture,
    volatile SteeringPreconditioningArcRepeatSummary *summary)
{
    summary->startSteeringCommand =
        SteeringController_GetCommand(
            &fixture->steeringController);

    summary->startSteeringTargetAngleRad =
        SteeringController_GetTargetEffectiveAngleRad(
            &fixture->steeringController);

    summary->startEffectiveSteeringAngleRad =
        SteeringController_GetEffectiveAngleRad(
            &fixture->steeringController);
}


static void Test_CaptureArcExit(
    RobotTestFixture *fixture,
    uint32_t elapsedMs,
    volatile SteeringPreconditioningArcRepeatSummary *summary)
{
    if (summary->arcExitCaptured)
    {
        return;
    }

    MotionController *controller =
        &fixture->motionController;

    summary->arcExitCaptured =
        1U;

    summary->arcExitTimeMs =
        elapsedMs;

    summary->arcExitDistanceMm =
        controller->travelledDistanceMm;

    summary->arcExitYawDeg =
        controller->yawDeg;

    summary->arcExitIdealYawDeg =
        Test_GetIdealYawDeg(
            controller->travelledDistanceMm);

    summary->arcExitLeftTravelMm =
        controller->leftTravelMm;

    summary->arcExitRightTravelMm =
        controller->rightTravelMm;

    summary->arcExitWheelTravelDifferenceMm =
        controller->rightTravelMm -
        controller->leftTravelMm;

    summary->arcExitDesiredWheelTravelDifferenceMm =
        controller->desiredWheelTravelDifferenceMm;

    summary->arcExitWheelSyncErrorMm =
        controller->wheelSyncErrorMm;
}


/* -------------------------------------------------------------------------- */
/* One run                                                                    */
/* -------------------------------------------------------------------------- */

static void Test_ShowReady(
    uint32_t runIndex)
{
    OLED_Clear();

    OLED_Printf(
        0, 0,
        "ARC REPEAT %lu/%u",
        (unsigned long)(runIndex + 1U),
        (unsigned int)STEERING_PRECOND_ARC_REPEAT_COUNT);

    OLED_Printf(
        0, 1,
        "R:%+.0f S:%.0f",
        TEST_RADIUS_MM,
        TEST_DISTANCE_MM);

    OLED_Printf(
        0, 2,
        "%.0f CPS",
        TEST_SPEED_CPS);

    OLED_Printf(
        0, 3,
        "REPOSITION ROBOT");

    OLED_Printf(
        0, 4,
        "PRESS SW1");

    OLED_Refresh_Gram();
}


static void Test_ShowRunResult(
    const volatile SteeringPreconditioningArcRepeatSummary *summary)
{
    OLED_Clear();

    OLED_Printf(
        0, 0,
        "RUN %u COMPLETE",
        (unsigned int)(summary->runIndex + 1U));

    OLED_Printf(
        0, 1,
        "Exit Y:%+.2f",
        summary->arcExitYawDeg);

    OLED_Printf(
        0, 2,
        "Ideal :%+.2f",
        summary->arcExitIdealYawDeg);

    OLED_Printf(
        0, 3,
        "Final :%+.2f",
        summary->finalYawDeg);

    OLED_Printf(
        0, 4,
        "Dist:%6.1f",
        summary->finalDistanceMm);

    OLED_Printf(
        0, 5,
        "%s%s",
        summary->timedOut ? "TIMEOUT " : "",
        summary->updateFailed ? "UPDATEFAIL" : "");

    OLED_Refresh_Gram();
}


static bool Test_RunOneArc(
    RobotTestFixture *fixture,
    uint8_t runIndex)
{
    volatile SteeringPreconditioningArcRepeatSummary *summary =
        &g_steeringPreconditioningArcRepeatSummary[runIndex];

    summary->runIndex =
        runIndex;


    Test_ShowReady(
        runIndex);

    SW1_WaitForPressAndRelease();


    /*
     * Give the user time to remove their hand from the chassis.
     */
    HAL_Delay(
        TEST_HANDS_OFF_MS);


    OLED_Clear();
    OLED_Printf(
        0, 0,
        "RUN %u CENTER",
        (unsigned int)(runIndex + 1U));
    OLED_Refresh_Gram();


    /*
     * This is the sequence under investigation.
     *
     * It is deliberately repeated before every arc WITHOUT reinitializing
     * SteeringController or RobotTestFixture.
     */
    Test_DeterministicCentre(
        fixture);


    OLED_Printf(
        0, 1,
        "PREPOSITION");
    OLED_Refresh_Gram();


    Test_PrepositionArcSteering(
        fixture);


    /*
     * Exclude wheel motion caused by manual repositioning from the first
     * WheelSpeedController velocity sample.
     */
    Test_ResynchroniseWheelSpeedControllers(
        fixture);


    /*
     * Capture the exact software steering state BEFORE MoveArc().
     * MoveArc() must not centre the steering.
     */
    Test_CaptureStartSteering(
        fixture,
        summary);


    summary->commandAccepted =
        MotionController_MoveArc(
            &fixture->motionController,
            TEST_DISTANCE_MM,
            TEST_RADIUS_MM,
            TEST_SPEED_CPS) ==
        MOTIONCONTROLLER_STATUS_OK;


    if (!summary->commandAccepted)
    {
        OLED_Clear();
        OLED_Printf(
            0, 0,
            "RUN %u REJECTED",
            (unsigned int)(runIndex + 1U));
        OLED_Refresh_Gram();

        return false;
    }


    /*
     * t = 0 should already show the final feedforward steering position.
     */
    Test_LogSample(
        fixture,
        runIndex,
        0U);


    uint32_t startTick =
        HAL_GetTick();

    uint32_t lastControlTick =
        startTick;

    uint32_t lastLogTick =
        startTick;


    while (fixture->motionController.mode !=
           MOTIONCONTROLLER_IDLE)
    {
        uint32_t now =
            HAL_GetTick();


        if ((now - startTick) >=
            TEST_TIMEOUT_MS)
        {
            summary->timedOut =
                1U;

            MotionController_Brake(
                &fixture->motionController);

            break;
        }


        if ((now - lastControlTick) >=
            TEST_CONTROL_PERIOD_MS)
        {
            lastControlTick +=
                TEST_CONTROL_PERIOD_MS;


            MotionControllerMode previousMode =
                fixture->motionController.mode;


            if (MotionController_Update(
                    &fixture->motionController,
                    TEST_CONTROL_PERIOD_S) !=
                MOTIONCONTROLLER_STATUS_OK)
            {
                summary->updateFailed =
                    1U;

                /*
                 * MotionController currently stops itself on important
                 * update failures such as IMU acquisition failure.
                 */
                break;
            }


            if ((previousMode == MOTIONCONTROLLER_ARC) &&
                (fixture->motionController.mode !=
                 MOTIONCONTROLLER_ARC))
            {
                Test_CaptureArcExit(
                    fixture,
                    now - startTick,
                    summary);
            }
        }


        if ((now - lastLogTick) >=
            TEST_LOG_INTERVAL_MS)
        {
            lastLogTick +=
                TEST_LOG_INTERVAL_MS;

            Test_LogSample(
                fixture,
                runIndex,
                now - startTick);
        }
    }


    /*
     * A hard timeout enters BRAKING. Continue stepping until stationary so
     * final yaw/distance remain comparable with the normal runs.
     */
    if (summary->timedOut &&
        fixture->motionController.mode !=
            MOTIONCONTROLLER_IDLE)
    {
        uint32_t brakeStartTick =
            HAL_GetTick();

        while ((fixture->motionController.mode !=
                MOTIONCONTROLLER_IDLE) &&
               ((HAL_GetTick() - brakeStartTick) <
                TEST_BRAKE_TIMEOUT_MS))
        {
            uint32_t now =
                HAL_GetTick();

            if ((now - lastControlTick) >=
                TEST_CONTROL_PERIOD_MS)
            {
                lastControlTick +=
                    TEST_CONTROL_PERIOD_MS;

                if (MotionController_Update(
                        &fixture->motionController,
                        TEST_CONTROL_PERIOD_S) !=
                    MOTIONCONTROLLER_STATUS_OK)
                {
                    summary->updateFailed =
                        1U;

                    break;
                }
            }

            if ((now - lastLogTick) >=
                TEST_LOG_INTERVAL_MS)
            {
                lastLogTick +=
                    TEST_LOG_INTERVAL_MS;

                Test_LogSample(
                    fixture,
                    runIndex,
                    now - startTick);
            }
        }


        if (fixture->motionController.mode !=
            MOTIONCONTROLLER_IDLE)
        {
            MotionController_Stop(
                &fixture->motionController);
        }
    }


    /*
     * If MotionController left ARC for any reason without the normal
     * transition capture, retain the best available endpoint diagnostic.
     */
    if (!summary->arcExitCaptured)
    {
        Test_CaptureArcExit(
            fixture,
            HAL_GetTick() - startTick,
            summary);
    }


    summary->finalDistanceMm =
        fixture->motionController.travelledDistanceMm;

    summary->finalYawDeg =
        fixture->motionController.yawDeg;


    /*
     * Guaranteed final sample for this run.
     */
    Test_LogSample(
        fixture,
        runIndex,
        HAL_GetTick() - startTick);


    Test_ShowRunResult(
        summary);


    /*
     * The next iteration displays the Ready screen and waits for SW1.
     * Reposition the robot while that screen is displayed.
     */
    return true;
}


/* -------------------------------------------------------------------------- */
/* Initialisation                                                              */
/* -------------------------------------------------------------------------- */

static bool Test_Init(
    RobotTestFixture *fixture)
{
    OLED_Init();

    OLED_Clear();
    OLED_Refresh_Gram();


    if (!RobotTestFixture_InitMotionController(
        fixture,
        &testMotionConfig))
    {
        return false;
    }


    return true;
}


/* -------------------------------------------------------------------------- */
/* Public test                                                                */
/* -------------------------------------------------------------------------- */

void SteeringPreconditioningArcRepeatTestRun(void)
{
    RobotTestFixture fixture = {0};


    /*
     * Real volatile reference so the experiment-description string remains
     * reachable in the linked image for GDB inspection.
     */
    volatile char infoKeepAlive =
        g_steeringPreconditioningArcRepeatInfo[0];

    (void)infoKeepAlive;


    Test_ResetExportState();


    if (!Test_Init(
            &fixture))
    {
        OLED_Printf(
            0, 0,
            "REPEAT INIT FAIL");

        OLED_Refresh_Gram();

        SW1_WhileNotPressed();

        return;
    }


    OLED_Clear();

    OLED_Printf(
        0, 0,
        "STEER PRECOND TEST");

    OLED_Printf(
        0, 1,
        "3x SAME FIXTURE");

    OLED_Printf(
        0, 2,
        "R:%+.0f S:%.0f",
        TEST_RADIUS_MM,
        TEST_DISTANCE_MM);

    OLED_Printf(
        0, 3,
        "Ideal:%+.2f deg",
        TEST_NOMINAL_YAW_DEG);

    OLED_Printf(
        0, 4,
        "RUN 1 NEXT");

    OLED_Refresh_Gram();


    HAL_Delay(700U);


    for (uint8_t runIndex = 0U;
         runIndex < STEERING_PRECOND_ARC_REPEAT_COUNT;
         runIndex++)
    {
        if (!Test_RunOneArc(
                &fixture,
                runIndex))
        {
            break;
        }
    }


    OLED_Clear();

    OLED_Printf(
        0, 0,
        "PRECOND DONE");

    OLED_Printf(
        0, 1,
        "Y1:%+.2f",
        g_steeringPreconditioningArcRepeatSummary[0]
            .arcExitYawDeg);

    OLED_Printf(
        0, 2,
        "Y2:%+.2f",
        g_steeringPreconditioningArcRepeatSummary[1]
            .arcExitYawDeg);

    OLED_Printf(
        0, 3,
        "Y3:%+.2f",
        g_steeringPreconditioningArcRepeatSummary[2]
            .arcExitYawDeg);

    OLED_Printf(
        0, 4,
        "N:%lu",
        (unsigned long)
            g_steeringPreconditioningArcRepeatLogCount);

    OLED_Printf(
        0, 5,
        "EXPORT WITH GDB");

    OLED_Refresh_Gram();


    /*
     * Preserve all exported data in RAM.
     */
    SW1_WhileNotPressed();
}
