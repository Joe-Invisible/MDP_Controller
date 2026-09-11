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


/* -------------------------------------------------------------------------- */
/* Test configuration                                                         */
/* -------------------------------------------------------------------------- */

#define ARC_TEST_DISTANCE_MM              (1000.0f)
#define ARC_TEST_RADIUS_MM                (-5000.0f)
#define ARC_TEST_SPEED_CPS                (2000.0f)

#define ARC_TEST_STEERING_SETTLE_MS       (500U)

#define ARC_TEST_CONTROL_PERIOD_MS        (10U)
#define ARC_TEST_CONTROL_PERIOD_S         (0.010f)

#define ARC_TEST_LOG_INTERVAL_MS          (20U)
#define ARC_TEST_TIMEOUT_MS               (10000U)

#define ARC_TEST_LOG_CAPACITY             (500U)


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
#define ARC_TEST_HEADING_KP               (1.2f)
#define ARC_TEST_HEADING_KI               (0.05f)
#define ARC_TEST_HEADING_KD               (0.0f)
#define ARC_TEST_HEADING_LIMIT_RAD        (0.015f)

#define ARC_TEST_YAWRATE_KP				 (200.0f)
#define ARC_TEST_YAWRATE_KI				 (200.0f)
#define ARC_TEST_YAWRATE_KD				 (0.0f)
#define ARC_TEST_YAWRATE_LIMIT_UNIT		 (20.0f)

#define ARC_TEST_HEADING_OUTER_KP_PER_SEC   (1.0f)

/*
 * Current motion-profile parameters.
 */
#define ARC_TEST_ACCELERATION_MMPS2       (500.0f)
#define ARC_TEST_DECELERATION_MMPS2       (250.0f)


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
    "Closed-loop arc: R=-10000mm, S=1000mm, 2000CPS, "
    "accel=500, decel=250, Ksync=10, heading Kp=1.00, "
    "steering pre-positioned 500ms";


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

    /* Geometric path command */
    sample->targetCurvaturePerMm =
        motionController->targetCurvaturePerMm;

    sample->arcCommandedCurvaturePerMm =
        motionController->arcCommandedCurvaturePerMm;

    sample->feedforwardSteeringAngleRad =
        motionController->targetSteeringAngleRad;


    /* Steering controller */
    sample->steeringTargetAngleRad =
        SteeringController_GetTargetEffectiveAngleRad(
            motionController->steering);

    sample->effectiveSteeringAngleRad =
        SteeringController_GetEffectiveAngleRad(
            motionController->steering);

    /* Steering controller */
    sample->steeringTargetAngleRad =
        SteeringController_GetTargetEffectiveAngleRad(
            motionController->steering);

    sample->effectiveSteeringAngleRad =
        SteeringController_GetEffectiveAngleRad(
            motionController->steering);


    /*
     * Phase 2B: outer heading loop
     */
    sample->arcDesiredYawRad =
        motionController->arcDesiredYawRad;

    sample->arcHeadingErrorRad =
        motionController->arcHeadingErrorRad;

    sample->arcFeedforwardYawRateRadPerSec =
        motionController->arcFeedforwardYawRateRadPerSec;

    sample->arcHeadingYawRateCorrectionRadPerSec =
        motionController->arcHeadingYawRateCorrectionRadPerSec;


    /*
     * Phase 2A: inner yaw-rate loop
     */
    sample->yawRateDps =
        motionController->yawRateDps;

    sample->filteredYawRateDps =
        motionController->filteredYawRateDps;

    sample->arcTargetYawRateRadPerSec =
        motionController->arcTargetYawRateRadPerSec;

    sample->arcYawRateErrorRadPerSec =
        motionController->arcYawRateErrorRadPerSec;

    sample->arcSteeringCorrectionCommand =
        motionController->arcSteeringCorrectionCommand;

    sample->arcSteeringTargetCommand =
        motionController->arcSteeringTargetCommand;

    /*
     * Explicitly log the actual feedback quantity seen
     * by the inner PI.
     */
    sample->measuredYawRateRadPerSec =
        motionController->filteredYawRateDps *
        (ARC_TEST_PI / 180.0f);

    sample->steeringCommand =
        SteeringController_GetCommand(
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
    sample->wheelReferenceCurvaturePerMm =
        motionController->wheelReferenceCurvaturePerMm;

    sample->leftBaseTargetCps =
        motionController->leftBaseTargetCps;

    sample->rightBaseTargetCps =
        motionController->rightBaseTargetCps;

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


//static void MotionControllerArcTest_PrepositionSteering(
//    RobotTestFixture *fixture)
//{
//    float curvaturePerMm =
//        1.0f / ARC_TEST_RADIUS_MM;
//
//    float targetSteeringAngleRad =
//        RobotKinematics_GetSteeringAngleRad(
//            fixture->motionController.kinematics,
//            curvaturePerMm);
//
//
//    uint32_t startTick =
//        HAL_GetTick();
//
//    uint32_t lastUpdateTick =
//        startTick;
//
//
//    /*
//     * Repeatedly command the same target because
//     * SteeringController_SetEffectiveAngleRad() applies
//     * the configured steering slew-rate limit.
//     *
//     * After the target has been reached, continuing to call
//     * it simply holds the steering there for the remainder
//     * of the settling interval.
//     */
//    while ((HAL_GetTick() - startTick) <
//           ARC_TEST_STEERING_SETTLE_MS)
//    {
//        uint32_t now =
//            HAL_GetTick();
//
//        if ((now - lastUpdateTick) >=
//            ARC_TEST_CONTROL_PERIOD_MS)
//        {
//            lastUpdateTick +=
//                ARC_TEST_CONTROL_PERIOD_MS;
//
//            SteeringController_SetEffectiveAngleRad(
//                &fixture->steeringController,
//                targetSteeringAngleRad,
//                ARC_TEST_CONTROL_PERIOD_S);
//        }
//    }
//}


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

        ARC_TEST_HEADING_KP,
        ARC_TEST_HEADING_KI,
        ARC_TEST_HEADING_KD,
        ARC_TEST_HEADING_LIMIT_RAD,

		ARC_TEST_YAWRATE_KP,
		ARC_TEST_YAWRATE_KI,
		ARC_TEST_YAWRATE_KD,
		ARC_TEST_YAWRATE_LIMIT_UNIT,

		ARC_TEST_HEADING_OUTER_KP_PER_SEC,

        ARC_TEST_SYNC_KP_CPS_PER_MM,
        ARC_TEST_SYNC_MAX_CORRECTION_CPS,

        ARC_TEST_ACCELERATION_MMPS2,
        ARC_TEST_DECELERATION_MMPS2))
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
     * Deterministically centre steering before every run.
     */
    SteeringController_StartCentre(
        &fixture.steeringController);

    while (!SteeringController_UpdateCentre(
        &fixture.steeringController,
        ARC_TEST_CONTROL_PERIOD_S))
    {
        HAL_Delay(
            ARC_TEST_CONTROL_PERIOD_MS);
    }


    /*
     * Allow centre position to settle mechanically.
     */
    HAL_Delay(100U);


    /*
     * Pre-position the front steering to the required
     * constant-curvature feedforward angle BEFORE the
     * robot starts moving.
     *
     * This removes steering slew / initial mechanical
     * settling as a variable in this experiment.
     */
//    OLED_Printf(
//        0, 0,
//        "Pre-steering...");

    OLED_Refresh_Gram();

//    MotionControllerArcTest_PrepositionSteering(
//        &fixture);


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
            ARC_TEST_SPEED_CPS);


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
