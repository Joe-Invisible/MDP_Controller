/*
 * MotionControllerArcTest.c
 *
 * First physical constant-curvature arc test.
 *
 * Feedforward steering only:
 *
 *     radius     = +4000 mm
 *     arc length = 1000 mm
 *     speed      = 2000 CPS
 *
 * Nominal full-arc geometry:
 *
 *     curvature = 1 / 4000 = 0.00025 /mm
 *     yaw        = 1000 / 4000
 *                = 0.25 rad
 *                = 14.32 deg
 *
 * Created on: 2026年9月9日
 * Author: Joe
 */

#include "MotionControllerArcTest.h"

#include "oledutils.h"
#include "userbutton.h"
#include "RobotTestFixture.h"
#include "RobotKinematics.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* -------------------------------------------------------------------------- */
/* Test configuration                                                         */
/* -------------------------------------------------------------------------- */

#define ARC_TEST_DISTANCE_MM              (1000.0f)
#define ARC_TEST_RADIUS_MM                (10000.0f)
#define ARC_TEST_SPEED_CPS                (2000.0f)

#define ARC_TEST_STEERING_PREPOSITION_MS  (500U)
#define ARC_TEST_STEERING_HOLD_MS         (500U)

#define ARC_TEST_CONTROL_PERIOD_MS        (10U)
#define ARC_TEST_CONTROL_PERIOD_S         (0.010f)

#define ARC_TEST_LOG_INTERVAL_MS          (20U)
#define ARC_TEST_TIMEOUT_MS               (10000U)

#define ARC_TEST_LOG_CAPACITY             (500U)

#define ARC_TEST_COMMAND_RATE_PER_SEC     (60.0f)

#define ARC_TEST_RAW_MIN_COMMAND          (-12.0f)
#define ARC_TEST_RAW_MAX_COMMAND          (+12.0f)

#define ARC_TEST_PRELOAD_HOLD_MS          (500U)

/*
 * Rear-wheel calibration used by the diagnostic distance integrator.
 */
#define ARC_TEST_ENCODER_CPR              (1560.0f)
#define ARC_TEST_WHEEL_DIAMETER_MM        (65.5f)


/*
 * Synchronisation controller.
 */
#define ARC_TEST_SYNC_KP_CPS_PER_MM       (10.0f)
#define ARC_TEST_SYNC_MAX_CORRECTION_CPS  (100.0f)


/*
 * Heading PID is deliberately not used during ARC mode.
 *
 * Set Kp = 0 here as an additional indication that this is
 * a pure feedforward curved-motion experiment.
 *
 * maxHeadingSteeringAngleRad remains non-zero because it must
 * still be a valid MotionController configuration parameter.
 */
#define ARC_TEST_HEADING_KP               (0.0f)
#define ARC_TEST_HEADING_KI               (0.0f)
#define ARC_TEST_HEADING_KD               (0.0f)
#define ARC_TEST_HEADING_LIMIT_RAD        (0.010f)


/*
 * Current motion-profile parameters.
 */
#define ARC_TEST_ACCELERATION_MMPS2       (500.0f)
#define ARC_TEST_DECELERATION_MMPS2       (250.0f)

static const MotionControllerConfig arcTestMotionConfig =
{
    .kinematics = &kinematics,
    .arcConfig = &arcMotionConfig,

    .headingKp = ARC_TEST_HEADING_KP,
    .headingKi = ARC_TEST_HEADING_KI,
    .headingKd = ARC_TEST_HEADING_KD,
    .maxHeadingSteeringAngleRad = ARC_TEST_HEADING_LIMIT_RAD,

    .arcYawRateKp = 10.0f,
    .arcYawRateKi = 0.0f,
    .arcYawRateKd = 0.0f,
    .maxArcSteeringCommandCorrection = 5.0f,

    .arcHeadingKpPerSec = 1.0f,

    .wheelSyncKpCpsPerMm = ARC_TEST_SYNC_KP_CPS_PER_MM,
    .maxWheelSyncCorrectionCps = ARC_TEST_SYNC_MAX_CORRECTION_CPS,

    .motionAccelerationMmps2 = ARC_TEST_ACCELERATION_MMPS2,
    .motionDecelerationMmps2 = ARC_TEST_DECELERATION_MMPS2,
    .motionCompletionToleranceMm = 0.5f,

    .arcYawRateFilterTauSec = 0.10f,

    .stopStableSampleCount = 3U,
};


#define ARC_TEST_PI                       (3.14159265358979323846f)

#define ARC_TEST_RAD_TO_DEG               (180.0f / ARC_TEST_PI)

#define ARC_TEST_MM_PER_COUNT \
    (ARC_TEST_PI * ARC_TEST_WHEEL_DIAMETER_MM \
    / ARC_TEST_ENCODER_CPR)

#define ARC_TEST_CURVATURE_PER_MM \
    (1.0f / ARC_TEST_RADIUS_MM)

#define ARC_TEST_NOMINAL_YAW_DEG \
    ((ARC_TEST_DISTANCE_MM / ARC_TEST_RADIUS_MM) \
    * ARC_TEST_RAD_TO_DEG)


/* -------------------------------------------------------------------------- */
/* Diagnostic state                                                           */
/* -------------------------------------------------------------------------- */

static float arcTestLeftDistanceMm = 0.0f;
static float arcTestRightDistanceMm = 0.0f;


/*
 * Global debugger-visible log.
 */
volatile MotionControllerArcTestLogSample
motionControllerArcTestLog[ARC_TEST_LOG_CAPACITY];

volatile uint32_t motionControllerArcTestLogCount = 0U;


/*
 * Useful top-level experiment results.
 */
volatile bool motionControllerArcTestCommandAccepted = false;
volatile bool motionControllerArcTestTimedOut = false;

volatile float motionControllerArcTestPreparedRawCommand = 0.0f;
volatile float motionControllerArcTestPreparedEffectiveAngleRad = 0.0f;

/*
 * State captured at the instant ARC mode ends.
 *
 * This is especially useful because the current controller centres
 * steering during braking. Therefore the yaw at ARC -> BRAKING is
 * a cleaner measure of constant-curvature feedforward accuracy than
 * final yaw after the entire braking manoeuvre.
 */
volatile bool motionControllerArcTestArcExitCaptured = false;

volatile uint32_t motionControllerArcTestArcExitTimeMs = 0U;

volatile float motionControllerArcTestArcExitDistanceMm = 0.0f;
volatile float motionControllerArcTestArcExitYawDeg = 0.0f;
volatile float motionControllerArcTestArcExitIdealYawDeg = 0.0f;

volatile float motionControllerArcTestArcExitLeftDistanceMm = 0.0f;
volatile float motionControllerArcTestArcExitRightDistanceMm = 0.0f;


/*
 * Final stopped state.
 */
volatile float motionControllerArcTestFinalYawDeg = 0.0f;
volatile float motionControllerArcTestFinalDistanceMm = 0.0f;

volatile float motionControllerArcTestFinalLeftDistanceMm = 0.0f;
volatile float motionControllerArcTestFinalRightDistanceMm = 0.0f;


volatile const char *motionControllerArcTestInfo =
    "FF arc major-branch diagnostic: R=+10000mm, S=1000mm, "
    "2000CPS, accel=500, decel=250, Ksync=10, "
    "heading feedback disabled; steering precondition "
    "+12 -> -12 -> target, 60 raw/s, 500ms holds/settle";


/* -------------------------------------------------------------------------- */
/* Logging helpers                                                            */
/* -------------------------------------------------------------------------- */

static void MotionControllerArcTest_ResetLog(void)
{
    motionControllerArcTestLogCount = 0U;

    arcTestLeftDistanceMm = 0.0f;
    arcTestRightDistanceMm = 0.0f;

    motionControllerArcTestCommandAccepted = false;
    motionControllerArcTestTimedOut = false;

    motionControllerArcTestArcExitCaptured = false;
    motionControllerArcTestArcExitTimeMs = 0U;

    motionControllerArcTestArcExitDistanceMm = 0.0f;
    motionControllerArcTestArcExitYawDeg = 0.0f;
    motionControllerArcTestArcExitIdealYawDeg = 0.0f;

    motionControllerArcTestArcExitLeftDistanceMm = 0.0f;
    motionControllerArcTestArcExitRightDistanceMm = 0.0f;

    motionControllerArcTestFinalYawDeg = 0.0f;
    motionControllerArcTestFinalDistanceMm = 0.0f;

    motionControllerArcTestFinalLeftDistanceMm = 0.0f;
    motionControllerArcTestFinalRightDistanceMm = 0.0f;
}


static void MotionControllerArcTest_UpdateMeasurements(
    RobotTestFixture *fixture)
{
    MotionController *motionController =
        &fixture->motionController;

    WheelSpeedController *leftWheel =
        motionController->leftWheel;

    WheelSpeedController *rightWheel =
        motionController->rightWheel;


    /*
     * CPS * seconds = encoder counts
     * counts * mm/count = wheel travel
     */
    arcTestLeftDistanceMm +=
        leftWheel->measuredSpeedCps *
        ARC_TEST_CONTROL_PERIOD_S *
        ARC_TEST_MM_PER_COUNT;

    arcTestRightDistanceMm +=
        rightWheel->measuredSpeedCps *
        ARC_TEST_CONTROL_PERIOD_S *
        ARC_TEST_MM_PER_COUNT;
}


static void MotionControllerArcTest_LogSample(
    RobotTestFixture *fixture,
    uint32_t elapsedMs)
{
    if (motionControllerArcTestLogCount >=
        ARC_TEST_LOG_CAPACITY)
    {
        return;
    }


    MotionController *motionController =
        &fixture->motionController;

    WheelSpeedController *leftWheel =
        motionController->leftWheel;

    WheelSpeedController *rightWheel =
        motionController->rightWheel;


    uint32_t i =
        motionControllerArcTestLogCount;


    MotionControllerArcTestLogSample *sample =
        (MotionControllerArcTestLogSample *)
        &motionControllerArcTestLog[i];


    sample->timeMs =
        elapsedMs;

    sample->state =
        (uint32_t)motionController->mode;


    /* Motion profile */
    sample->profileTargetSpeedCps =
        motionController->targetSpeedCps;

    sample->travelledDistanceMm =
        motionController->travelledDistanceMm;


    /* Geometric command */
    sample->targetCurvaturePerMm =
        motionController->targetCurvaturePerMm;

    sample->feedforwardSteeringAngleRad =
        motionController->targetSteeringAngleRad;


    /* Steering controller */
    sample->steeringTargetAngleRad =
        SteeringController_GetTargetEffectiveAngleRad(
            motionController->steering);

    sample->effectiveSteeringAngleRad =
        SteeringController_GetEffectiveAngleRad(
            motionController->steering);


    /* Wheel-speed controllers */
    sample->leftTargetCps =
        leftWheel->targetSpeedCps;

    sample->rightTargetCps =
        rightWheel->targetSpeedCps;

    sample->leftMeasuredCps =
        leftWheel->measuredSpeedCps;

    sample->rightMeasuredCps =
        rightWheel->measuredSpeedCps;

    sample->leftPwm =
        leftWheel->outputPWM;

    sample->rightPwm =
        rightWheel->outputPWM;


    /* Independent wheel-distance integration */
    sample->leftDistanceMm =
        arcTestLeftDistanceMm;

    sample->rightDistanceMm =
        arcTestRightDistanceMm;


    /* Synchronisation */
    sample->desiredWheelTravelDifferenceMm =
        motionController->desiredWheelTravelDifferenceMm;

    sample->wheelSyncErrorMm =
        motionController->wheelSyncErrorMm;

    sample->wheelSyncCorrectionCps =
        motionController->wheelSyncCorrectionCps;


    /* Heading */
    sample->yawDeg =
        motionController->yawDeg;

    sample->idealYawDeg =
        motionController->travelledDistanceMm *
        motionController->targetCurvaturePerMm *
        ARC_TEST_RAD_TO_DEG;


    /* Dynamic braking */
    sample->leftActuatorMode =
        leftWheel->actuatorMode;

    sample->rightActuatorMode =
        rightWheel->actuatorMode;

    sample->leftBrakeDemand =
        leftWheel->brakeDemand;

    sample->rightBrakeDemand =
        rightWheel->brakeDemand;

    sample->leftBrakePWM =
        leftWheel->brakePWM;

    sample->rightBrakePWM =
        rightWheel->brakePWM;


    motionControllerArcTestLogCount++;
}


static void MotionControllerArcTest_CaptureArcExit(
    RobotTestFixture *fixture,
    uint32_t elapsedMs)
{
    if (motionControllerArcTestArcExitCaptured)
    {
        return;
    }


    MotionController *motionController =
        &fixture->motionController;


    motionControllerArcTestArcExitCaptured = true;

    motionControllerArcTestArcExitTimeMs =
        elapsedMs;

    motionControllerArcTestArcExitDistanceMm =
        motionController->travelledDistanceMm;

    motionControllerArcTestArcExitYawDeg =
        motionController->yawDeg;

    motionControllerArcTestArcExitIdealYawDeg =
        motionController->travelledDistanceMm *
        motionController->targetCurvaturePerMm *
        ARC_TEST_RAD_TO_DEG;

    motionControllerArcTestArcExitLeftDistanceMm =
        arcTestLeftDistanceMm;

    motionControllerArcTestArcExitRightDistanceMm =
        arcTestRightDistanceMm;
}

static float MotionControllerArcTest_Clamp(
    float value,
    float minValue,
    float maxValue)
{
    if (value > maxValue)
    {
        return maxValue;
    }

    if (value < minValue)
    {
        return minValue;
    }

    return value;
}


static void MotionControllerArcTest_SlewRawCommand(
    RobotTestFixture *fixture,
    float targetCommand)
{
    float currentCommand =
        SteeringController_GetCommand(
            &fixture->steeringController);


    const float maxDelta =
        ARC_TEST_COMMAND_RATE_PER_SEC *
        ARC_TEST_CONTROL_PERIOD_S;


    while (fabsf(
        targetCommand -
        currentCommand) > 0.0001f)
    {
        float delta =
            targetCommand -
            currentCommand;


        delta =
            MotionControllerArcTest_Clamp(
                delta,
                -maxDelta,
                +maxDelta);


        currentCommand +=
            delta;


        SteeringController_SetCommand(
            &fixture->steeringController,
            currentCommand);


        HAL_Delay(
            ARC_TEST_CONTROL_PERIOD_MS);
    }


    SteeringController_SetCommand(
        &fixture->steeringController,
        targetCommand);
}


static void MotionControllerArcTest_PreconditionIncreasingBranch(
    RobotTestFixture *fixture)
{
    /*
     * Exact major increasing-branch preparation:
     *
     *     current -> +12
     *             -> -12
     *             -> target
     */

    MotionControllerArcTest_SlewRawCommand(
        fixture,
        ARC_TEST_RAW_MAX_COMMAND);

    HAL_Delay(
        ARC_TEST_PRELOAD_HOLD_MS);


    MotionControllerArcTest_SlewRawCommand(
        fixture,
        ARC_TEST_RAW_MIN_COMMAND);

    HAL_Delay(
        ARC_TEST_PRELOAD_HOLD_MS);
}

static void MotionControllerArcTest_PrepositionSteering(
    RobotTestFixture *fixture)
{
    float curvaturePerMm =
        1.0f / ARC_TEST_RADIUS_MM;

    float targetSteeringAngleRad =
        RobotKinematics_GetSteeringAngleRad(
            fixture->motionController.config->kinematics,
            curvaturePerMm);

    uint32_t updateCount =
        ARC_TEST_STEERING_PREPOSITION_MS /
        ARC_TEST_CONTROL_PERIOD_MS;

    /*
     * Advance the rate-limited steering command at the same
     * 100 Hz interval used during normal motion control.
     */
    for (uint32_t i = 0U;
         i < updateCount;
         i++)
    {
        SteeringController_SetEffectiveAngleRad(
            &fixture->steeringController,
            targetSteeringAngleRad,
            ARC_TEST_CONTROL_PERIOD_S);

        HAL_Delay(
            ARC_TEST_CONTROL_PERIOD_MS);
    }

    /*
     * Steering should now be at the requested feedforward
     * angle. Hold it there so the servo/linkage can settle
     * mechanically before the chassis starts moving.
     */
    HAL_Delay(
        ARC_TEST_STEERING_HOLD_MS);
}

/* -------------------------------------------------------------------------- */
/* Initialisation                                                              */
/* -------------------------------------------------------------------------- */

static bool MotionControllerArcTest_Init(
    RobotTestFixture *fixture)
{
    OLED_Init();

    OLED_Clear();
    OLED_Refresh_Gram();


    if (!RobotTestFixture_InitMotionController(
        fixture,
        &arcTestMotionConfig))
    {
        return false;
    }


    return true;
}


/* -------------------------------------------------------------------------- */
/* Test                                                                        */
/* -------------------------------------------------------------------------- */

void MotionControllerArcTestRun(void)
{
    RobotTestFixture fixture = { 0 };


    if (!MotionControllerArcTest_Init(&fixture))
    {
        OLED_Printf(0, 0, "Arc Init Failed");
        OLED_Refresh_Gram();

        SW1_WhileNotPressed();
        return;
    }


    OLED_Printf(
        0, 0,
        "Arc Motion Test");

    OLED_Printf(
        0, 1,
        "R:%+.0f S:%.0f",
        ARC_TEST_RADIUS_MM,
        ARC_TEST_DISTANCE_MM);

    OLED_Printf(
        0, 2,
        "Speed: %.0f CPS",
        ARC_TEST_SPEED_CPS);

    OLED_Printf(
        0, 3,
        "Ideal yaw:%+.2f",
        ARC_TEST_NOMINAL_YAW_DEG);

    OLED_Printf(
        0, 4,
        "Start: SW1");

    OLED_Refresh_Gram();


    SW1_WaitForPressAndRelease();


    OLED_Clear();


    /*
     * Allow finger to clear the robot.
     */
    HAL_Delay(400U);


    /*
     * Diagnostic steering preparation.
     *
     * Reproduce the same major increasing hysteresis branch
     * used during the self-propelled steering calibration:
     *
     *     +12 -> -12 -> target
     *
     * Do NOT centre first. The extrema themselves establish
     * the known branch history.
     */
    OLED_Printf(
        0, 0,
        "INC preload...");

    OLED_Printf(
        0, 1,
        "+12 -> -12");

    OLED_Refresh_Gram();


    MotionControllerArcTest_PreconditionIncreasingBranch(
        &fixture);


    /*
     * Now approach the requested +10 m-radius feedforward angle
     * from raw -12. This is an increasing-command approach,
     * matching the calibration branch.
     */
    OLED_Clear();

    OLED_Printf(
        0, 0,
        "Pre-steering...");

    OLED_Printf(
        0, 1,
        "-12 -> target");

    OLED_Refresh_Gram();


    MotionControllerArcTest_PrepositionSteering(
        &fixture);

    motionControllerArcTestPreparedRawCommand =
        SteeringController_GetCommand(
            &fixture.steeringController);

    motionControllerArcTestPreparedEffectiveAngleRad =
        SteeringController_GetEffectiveAngleRad(
            &fixture.steeringController);

    /*
     * Only reset the motion-test log after pre-steering.
     *
     * Therefore t = 0 in the exported dataset remains
     * the instant immediately before motion begins.
     */
    MotionControllerArcTest_ResetLog();


    /*
     * Start the constant-curvature motion.
     */
    motionControllerArcTestCommandAccepted =
        MotionController_MoveArc(
            &fixture.motionController,
            ARC_TEST_DISTANCE_MM,
            ARC_TEST_RADIUS_MM,
            ARC_TEST_SPEED_CPS) ==
        MOTIONCONTROLLER_STATUS_OK;


    if (!motionControllerArcTestCommandAccepted)
    {
        OLED_Printf(
            0, 0,
            "MoveArc rejected");

        OLED_Printf(
            0, 1,
            "R:%+.0f mm",
            ARC_TEST_RADIUS_MM);

        OLED_Refresh_Gram();

        SW1_WhileNotPressed();
        return;
    }


    /*
     * Capture state before the first control update.
     */
    MotionControllerArcTest_LogSample(
        &fixture,
        0U);


    uint32_t startTick =
        HAL_GetTick();

    uint32_t lastControlTick =
        startTick;

    uint32_t lastLogTick =
        startTick;


    while (fixture.motionController.mode !=
           MOTIONCONTROLLER_IDLE)
    {
        uint32_t now =
            HAL_GetTick();


        /*
         * Hard timeout.
         */
        if ((now - startTick) >=
            ARC_TEST_TIMEOUT_MS)
        {
            motionControllerArcTestTimedOut = true;

            MotionController_Brake(
                &fixture.motionController);

            break;
        }


        /*
         * 100 Hz controller.
         */
        if ((now - lastControlTick) >=
            ARC_TEST_CONTROL_PERIOD_MS)
        {
            lastControlTick +=
                ARC_TEST_CONTROL_PERIOD_MS;


            MotionControllerMode previousMode =
                fixture.motionController.mode;


            MotionController_Update(
                &fixture.motionController,
                ARC_TEST_CONTROL_PERIOD_S);


            MotionControllerArcTest_UpdateMeasurements(
                &fixture);


            /*
             * Capture the first transition out of ARC mode.
             *
             * Normally this will be ARC -> BRAKING.
             */
            if ((previousMode == MOTIONCONTROLLER_ARC) &&
                (fixture.motionController.mode !=
                 MOTIONCONTROLLER_ARC))
            {
                MotionControllerArcTest_CaptureArcExit(
                    &fixture,
                    now - startTick);
            }
        }


        /*
         * 50 Hz debugger log.
         */
        if ((now - lastLogTick) >=
            ARC_TEST_LOG_INTERVAL_MS)
        {
            lastLogTick +=
                ARC_TEST_LOG_INTERVAL_MS;


            MotionControllerArcTest_LogSample(
                &fixture,
                now - startTick);
        }
    }


    /*
     * If timeout initiated braking, continue stepping the
     * controller so active braking can complete.
     */
    if (motionControllerArcTestTimedOut)
    {
        uint32_t brakeStartTick =
            HAL_GetTick();


        while ((fixture.motionController.mode !=
                MOTIONCONTROLLER_IDLE) &&
               ((HAL_GetTick() - brakeStartTick) <
                3000U))
        {
            uint32_t now =
                HAL_GetTick();


            if ((now - lastControlTick) >=
                ARC_TEST_CONTROL_PERIOD_MS)
            {
                lastControlTick +=
                    ARC_TEST_CONTROL_PERIOD_MS;


                MotionController_Update(
                    &fixture.motionController,
                    ARC_TEST_CONTROL_PERIOD_S);


                MotionControllerArcTest_UpdateMeasurements(
                    &fixture);
            }


            if ((now - lastLogTick) >=
                ARC_TEST_LOG_INTERVAL_MS)
            {
                lastLogTick +=
                    ARC_TEST_LOG_INTERVAL_MS;


                MotionControllerArcTest_LogSample(
                    &fixture,
                    now - startTick);
            }
        }


        /*
         * Last-resort stop.
         */
        if (fixture.motionController.mode !=
            MOTIONCONTROLLER_IDLE)
        {
            MotionController_Stop(
                &fixture.motionController);
        }
    }


    /*
     * Guaranteed final sample.
     */
    MotionControllerArcTest_LogSample(
        &fixture,
        HAL_GetTick() - startTick);


    motionControllerArcTestFinalYawDeg =
        fixture.motionController.yawDeg;

    motionControllerArcTestFinalDistanceMm =
        fixture.motionController.travelledDistanceMm;

    motionControllerArcTestFinalLeftDistanceMm =
        arcTestLeftDistanceMm;

    motionControllerArcTestFinalRightDistanceMm =
        arcTestRightDistanceMm;


    /*
     * Results.
     */
    OLED_Clear();

    OLED_Printf(
        0, 0,
        "Arc Exit:%+.2f",
        motionControllerArcTestArcExitYawDeg);

    OLED_Printf(
        0, 1,
        "Ideal:%+.2f",
        motionControllerArcTestArcExitIdealYawDeg);

    OLED_Printf(
        0, 2,
        "Final:%+.2f",
        motionControllerArcTestFinalYawDeg);

    OLED_Printf(
        0, 3,
        "Dist:%6.1f",
        motionControllerArcTestFinalDistanceMm);

    OLED_Printf(
        0, 4,
        "N:%lu%s",
        motionControllerArcTestLogCount,
        motionControllerArcTestTimedOut
            ? " TIMEOUT"
            : "");

    OLED_Refresh_Gram();


    /*
     * Preserve all data in RAM for debugger export.
     */
    SW1_WhileNotPressed();
}
