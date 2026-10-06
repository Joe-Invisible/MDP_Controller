/*
 * MotionControllerStraightFeedforwardTest.c
 *
 * Isolated zero-curvature feedforward calibration. Derived from the original
 * arc harness; that file is preserved for the pending raw +95 calibration.
 * Edit only this test's distance, unsigned speed and candidate raw command.
 * FF_EXP selects either open-loop feedforward or the current straight-path
 * feedback tuning. Rear-wheel reference curvature stays zero; wheel-speed
 * and synchronization feedback remain enabled.
 */

#include "MotionControllerStraightFeedforwardTest.h"

#include "oledutils.h"
#include "userbutton.h"
#include "buzzer.h"
#include "RobotTestFixture.h"
#include "RobotKinematics.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>


/* -------------------------------------------------------------------------- */
/* Test configuration                                                         */
/* -------------------------------------------------------------------------- */

#define STRAIGHT_FF_TEST_DISTANCE_MM              (-500.0f)
#define STRAIGHT_FF_TEST_RAW_COMMAND              (0.0f)
#define STRAIGHT_FF_TEST_SPEED_CPS                (2000.0f)

#define STRAIGHT_FF_TEST_CONTROL_PERIOD_MS        (10U)
#define STRAIGHT_FF_TEST_CONTROL_PERIOD_S         (0.010f)

#define STRAIGHT_FF_TEST_LOG_INTERVAL_MS          (20U)
#define STRAIGHT_FF_TEST_TIMEOUT_MS               (10000U)

#define STRAIGHT_FF_TEST_LOG_CAPACITY             (500U)


/*
 * Rear-wheel calibration used by the diagnostic distance integrator.
 */
#define STRAIGHT_FF_TEST_ENCODER_CPR              (1560.0f)
#define STRAIGHT_FF_TEST_WHEEL_DIAMETER_MM        (65.5f)


/*
 * Synchronisation controller.
 */
#define STRAIGHT_FF_TEST_SYNC_KP_CPS_PER_MM       (10.0f)
#define STRAIGHT_FF_TEST_SYNC_MAX_CORRECTION_CPS  (100.0f)

#define FF_EXP	(0)
#define STRAIGHT_FF_TEST_HEADING_KP               (0.0f)
#define STRAIGHT_FF_TEST_HEADING_KI               (0.0f)
#define STRAIGHT_FF_TEST_HEADING_KD               (0.0f)
#define STRAIGHT_FF_TEST_HEADING_LIMIT_RAD        (0.015f)
#if FF_EXP == 1
#define STRAIGHT_FF_TEST_YAWRATE_KP               (0.0f)
#define STRAIGHT_FF_TEST_YAWRATE_KI               (0.0f)
#define STRAIGHT_FF_TEST_YAWRATE_KD               (0.0f)
#define STRAIGHT_FF_TEST_YAWRATE_LIMIT_UNIT       (5.0f)
#define STRAIGHT_FF_TEST_HEADING_OUTER_KP_PER_SEC (0.0f)
#elif FF_EXP == 0
#define STRAIGHT_FF_TEST_YAWRATE_KP               (370.0f)
#define STRAIGHT_FF_TEST_YAWRATE_KI               (450.0f)
#define STRAIGHT_FF_TEST_YAWRATE_KD               (0.0f)
#define STRAIGHT_FF_TEST_YAWRATE_LIMIT_UNIT       (30.0f)
#define STRAIGHT_FF_TEST_HEADING_OUTER_KP_PER_SEC (2.0f)
#endif

/*
 * Current motion-profile parameters.
 */
#define STRAIGHT_FF_TEST_ACCELERATION_MMPS2       (500.0f)
#define STRAIGHT_FF_TEST_DECELERATION_MMPS2       (250.0f)

static const MotionControllerConfig straightFeedforwardTestMotionConfig =
{
    .kinematics = &kinematics,
    .arcConfig = &arcMotionConfig,

    .straightSteeringSettlingTimeSec = 0.5f,
    .straightSteeringFeedforwardCommand = STRAIGHT_FF_TEST_RAW_COMMAND,
    .useLegacyStraightSteering = false,
    .maxPathCorrectionCurvaturePerMm = 1.0f / 1000.0f,

    .headingKp = STRAIGHT_FF_TEST_HEADING_KP,
    .headingKi = STRAIGHT_FF_TEST_HEADING_KI,
    .headingKd = STRAIGHT_FF_TEST_HEADING_KD,
    .maxHeadingSteeringAngleRad = STRAIGHT_FF_TEST_HEADING_LIMIT_RAD,

    .straightYawRateKp = STRAIGHT_FF_TEST_YAWRATE_KP,
    .straightYawRateKi = STRAIGHT_FF_TEST_YAWRATE_KI,
    .straightYawRateKd = STRAIGHT_FF_TEST_YAWRATE_KD,
    .straightHeadingKpPerSec = STRAIGHT_FF_TEST_HEADING_OUTER_KP_PER_SEC,

    /* Explicitly initialize the arc regime too, although this harness is straight-only. */
    .arcYawRateKp = STRAIGHT_FF_TEST_YAWRATE_KP,
    .arcYawRateKi = STRAIGHT_FF_TEST_YAWRATE_KI,
    .arcYawRateKd = STRAIGHT_FF_TEST_YAWRATE_KD,
    .maxArcSteeringCommandCorrection = STRAIGHT_FF_TEST_YAWRATE_LIMIT_UNIT,

    .arcHeadingKpPerSec = STRAIGHT_FF_TEST_HEADING_OUTER_KP_PER_SEC,

    .wheelSyncKpCpsPerMm = STRAIGHT_FF_TEST_SYNC_KP_CPS_PER_MM,
    .maxWheelSyncCorrectionCps = STRAIGHT_FF_TEST_SYNC_MAX_CORRECTION_CPS,

    .motionAccelerationMmps2 = STRAIGHT_FF_TEST_ACCELERATION_MMPS2,
    .motionDecelerationMmps2 = STRAIGHT_FF_TEST_DECELERATION_MMPS2,
    .motionCompletionToleranceMm = 0.5f,

    .arcYawRateFilterTauSec = 0.10f,

    .stopStableSampleCount = 3U,
};


#define STRAIGHT_FF_TEST_PI                       (3.14159265358979323846f)

#define STRAIGHT_FF_TEST_RAD_TO_DEG               (180.0f / STRAIGHT_FF_TEST_PI)

#define STRAIGHT_FF_TEST_MM_PER_COUNT \
    (STRAIGHT_FF_TEST_PI * STRAIGHT_FF_TEST_WHEEL_DIAMETER_MM \
    / STRAIGHT_FF_TEST_ENCODER_CPR)

#define STRAIGHT_FF_TEST_CURVATURE_PER_MM \
    (0.0f)

#define STRAIGHT_FF_TEST_NOMINAL_YAW_DEG \
    (0.0f)


/* -------------------------------------------------------------------------- */
/* Diagnostic state                                                           */
/* -------------------------------------------------------------------------- */

static float straightFeedforwardTestLeftDistanceMm = 0.0f;
static float straightFeedforwardTestRightDistanceMm = 0.0f;


/* Exported settings describe this test rather than production defaults. */
volatile const float motionControllerStraightFeedforwardTestRequestedDistanceMm =
    STRAIGHT_FF_TEST_DISTANCE_MM;
volatile const float motionControllerStraightFeedforwardTestRequestedSpeedCps =
    STRAIGHT_FF_TEST_SPEED_CPS;
volatile const float motionControllerStraightFeedforwardTestRawCommand =
    STRAIGHT_FF_TEST_RAW_COMMAND;
volatile MotionControllerStatus motionControllerStraightFeedforwardTestUpdateStatus =
    MOTIONCONTROLLER_STATUS_OK;

/*
 * Global debugger-visible log.
 */
volatile MotionControllerStraightFeedforwardTestLogSample
motionControllerStraightFeedforwardTestLog[STRAIGHT_FF_TEST_LOG_CAPACITY];

volatile uint32_t motionControllerStraightFeedforwardTestLogCount = 0U;


/*
 * Useful top-level experiment results.
 */
volatile bool motionControllerStraightFeedforwardTestCommandAccepted = false;
volatile bool motionControllerStraightFeedforwardTestTimedOut = false;
/*
 * State preserved at the instant the hard timeout is detected,
 * before timeout handling changes MotionController state.
 */
volatile MotionControllerMode
motionControllerStraightFeedforwardTestTimeoutMode =
    MOTIONCONTROLLER_IDLE;

volatile float
motionControllerStraightFeedforwardTestTimeoutDistanceMm = 0.0f;

volatile float
motionControllerStraightFeedforwardTestTimeoutTargetDistanceMm = 0.0f;

volatile bool
motionControllerStraightFeedforwardTestTimeoutProfileActive = false;

volatile uint32_t
motionControllerStraightFeedforwardTestTimeoutStationarySamples = 0U;

volatile float
motionControllerStraightFeedforwardTestTimeoutLeftCps = 0.0f;

volatile float
motionControllerStraightFeedforwardTestTimeoutRightCps = 0.0f;

volatile float
motionControllerStraightFeedforwardTestTimeoutYawDeg = 0.0f;

volatile bool
motionControllerStraightFeedforwardTestTimeoutMotionExitCaptured = false;


/*
 * State captured at the instant STRAIGHT mode ends.
 *
 * This is especially useful because the current controller centres
 * steering during braking. Therefore the yaw at STRAIGHT -> BRAKING is
 * a cleaner measure of zero-curvature feedforward accuracy than
 * final yaw after the entire braking manoeuvre.
 */
volatile bool motionControllerStraightFeedforwardTestMotionExitCaptured = false;

volatile uint32_t motionControllerStraightFeedforwardTestMotionExitTimeMs = 0U;

volatile float motionControllerStraightFeedforwardTestMotionExitDistanceMm = 0.0f;
volatile float motionControllerStraightFeedforwardTestMotionExitYawDeg = 0.0f;
volatile float motionControllerStraightFeedforwardTestMotionExitIdealYawDeg = 0.0f;

volatile float motionControllerStraightFeedforwardTestMotionExitLeftDistanceMm = 0.0f;
volatile float motionControllerStraightFeedforwardTestMotionExitRightDistanceMm = 0.0f;


/*
 * Final stopped state.
 */
volatile float motionControllerStraightFeedforwardTestFinalYawDeg = 0.0f;
volatile float motionControllerStraightFeedforwardTestFinalDistanceMm = 0.0f;

volatile float motionControllerStraightFeedforwardTestFinalLeftDistanceMm = 0.0f;
volatile float motionControllerStraightFeedforwardTestFinalRightDistanceMm = 0.0f;


/* -------------------------------------------------------------------------- */
/* Logging helpers                                                            */
/* -------------------------------------------------------------------------- */

static void MotionControllerStraightFeedforwardTest_ResetLog(void)
{
    motionControllerStraightFeedforwardTestLogCount = 0U;

    straightFeedforwardTestLeftDistanceMm = 0.0f;
    straightFeedforwardTestRightDistanceMm = 0.0f;

    motionControllerStraightFeedforwardTestCommandAccepted = false;
    motionControllerStraightFeedforwardTestTimedOut = false;
    motionControllerStraightFeedforwardTestUpdateStatus = MOTIONCONTROLLER_STATUS_OK;

    motionControllerStraightFeedforwardTestMotionExitCaptured = false;
    motionControllerStraightFeedforwardTestMotionExitTimeMs = 0U;

    motionControllerStraightFeedforwardTestMotionExitDistanceMm = 0.0f;
    motionControllerStraightFeedforwardTestMotionExitYawDeg = 0.0f;
    motionControllerStraightFeedforwardTestMotionExitIdealYawDeg = 0.0f;

    motionControllerStraightFeedforwardTestMotionExitLeftDistanceMm = 0.0f;
    motionControllerStraightFeedforwardTestMotionExitRightDistanceMm = 0.0f;

    motionControllerStraightFeedforwardTestFinalYawDeg = 0.0f;
    motionControllerStraightFeedforwardTestFinalDistanceMm = 0.0f;

    motionControllerStraightFeedforwardTestFinalLeftDistanceMm = 0.0f;
    motionControllerStraightFeedforwardTestFinalRightDistanceMm = 0.0f;

    motionControllerStraightFeedforwardTestTimeoutMode =
        MOTIONCONTROLLER_IDLE;

    motionControllerStraightFeedforwardTestTimeoutDistanceMm = 0.0f;
    motionControllerStraightFeedforwardTestTimeoutTargetDistanceMm = 0.0f;
    motionControllerStraightFeedforwardTestTimeoutProfileActive = false;
    motionControllerStraightFeedforwardTestTimeoutStationarySamples = 0U;
    motionControllerStraightFeedforwardTestTimeoutLeftCps = 0.0f;
    motionControllerStraightFeedforwardTestTimeoutRightCps = 0.0f;
    motionControllerStraightFeedforwardTestTimeoutYawDeg = 0.0f;
    motionControllerStraightFeedforwardTestTimeoutMotionExitCaptured = false;
}


static void MotionControllerStraightFeedforwardTest_UpdateMeasurements(
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
    straightFeedforwardTestLeftDistanceMm +=
        leftWheel->measuredSpeedCps *
        STRAIGHT_FF_TEST_CONTROL_PERIOD_S *
        STRAIGHT_FF_TEST_MM_PER_COUNT;

    straightFeedforwardTestRightDistanceMm +=
        rightWheel->measuredSpeedCps *
        STRAIGHT_FF_TEST_CONTROL_PERIOD_S *
        STRAIGHT_FF_TEST_MM_PER_COUNT;
}


static void MotionControllerStraightFeedforwardTest_LogSample(
    RobotTestFixture *fixture,
    uint32_t elapsedMs)
{
    if (motionControllerStraightFeedforwardTestLogCount >=
        STRAIGHT_FF_TEST_LOG_CAPACITY)
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
        motionControllerStraightFeedforwardTestLogCount;


    MotionControllerStraightFeedforwardTestLogSample *sample =
        (MotionControllerStraightFeedforwardTestLogSample *)
        &motionControllerStraightFeedforwardTestLog[i];


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


    /* Legacy angle estimates are stale during raw control; see validity flag. */
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
        (STRAIGHT_FF_TEST_PI / 180.0f);

    sample->steeringFeedforwardCommand =
        motionController->arcSteeringFeedforwardCommand;
    sample->effectiveAngleModelValid =
        fixture->steeringController.effectiveAngleModelValid;

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
        straightFeedforwardTestLeftDistanceMm;

    sample->rightDistanceMm =
        straightFeedforwardTestRightDistanceMm;


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
        STRAIGHT_FF_TEST_RAD_TO_DEG;


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


    motionControllerStraightFeedforwardTestLogCount++;
}

static void MotionControllerStraightFeedforwardTest_CaptureTimeout(
    RobotTestFixture *fixture)
{
    MotionController *motionController =
        &fixture->motionController;

    motionControllerStraightFeedforwardTestTimeoutMode =
        motionController->mode;

    motionControllerStraightFeedforwardTestTimeoutDistanceMm =
        motionController->travelledDistanceMm;

    motionControllerStraightFeedforwardTestTimeoutTargetDistanceMm =
        motionController->targetDistanceMm;

    motionControllerStraightFeedforwardTestTimeoutProfileActive =
        MotionProfile_IsActive(
            &motionController->motionProfile);

    motionControllerStraightFeedforwardTestTimeoutStationarySamples =
        motionController->stationarySamples;

    motionControllerStraightFeedforwardTestTimeoutLeftCps =
        motionController->leftWheel->measuredSpeedCps;

    motionControllerStraightFeedforwardTestTimeoutRightCps =
        motionController->rightWheel->measuredSpeedCps;

    motionControllerStraightFeedforwardTestTimeoutYawDeg =
        motionController->yawDeg;

    motionControllerStraightFeedforwardTestTimeoutMotionExitCaptured =
        motionControllerStraightFeedforwardTestMotionExitCaptured;
}

static void MotionControllerStraightFeedforwardTest_CaptureMotionExit(
    RobotTestFixture *fixture,
    uint32_t elapsedMs)
{
    if (motionControllerStraightFeedforwardTestMotionExitCaptured)
    {
        return;
    }


    MotionController *motionController =
        &fixture->motionController;


    motionControllerStraightFeedforwardTestMotionExitCaptured = true;

    motionControllerStraightFeedforwardTestMotionExitTimeMs =
        elapsedMs;

    motionControllerStraightFeedforwardTestMotionExitDistanceMm =
        motionController->travelledDistanceMm;

    motionControllerStraightFeedforwardTestMotionExitYawDeg =
        motionController->yawDeg;

    motionControllerStraightFeedforwardTestMotionExitIdealYawDeg =
        motionController->travelledDistanceMm *
        motionController->targetCurvaturePerMm *
        STRAIGHT_FF_TEST_RAD_TO_DEG;

    motionControllerStraightFeedforwardTestMotionExitLeftDistanceMm =
        straightFeedforwardTestLeftDistanceMm;

    motionControllerStraightFeedforwardTestMotionExitRightDistanceMm =
        straightFeedforwardTestRightDistanceMm;
}


/* -------------------------------------------------------------------------- */
/* Initialisation                                                              */
/* -------------------------------------------------------------------------- */

static bool MotionControllerStraightFeedforwardTest_Init(
    RobotTestFixture *fixture)
{
    OLED_Init();

    OLED_Clear();
    OLED_Refresh_Gram();


    if (!RobotTestFixture_InitMotionController(
        fixture,
        &straightFeedforwardTestMotionConfig))
    {
        return false;
    }


    return true;
}


/* -------------------------------------------------------------------------- */
/* Test                                                                        */
/* -------------------------------------------------------------------------- */

/* All result globals are finalized before this completion/export hook. */
static void MotionControllerStraightFeedforwardTest_ShowFinal(void)
{
    /*
     * Results.
     */
    OLED_Clear();

    if (!motionControllerStraightFeedforwardTestCommandAccepted)
    {
        OLED_Printf(0, 0, "Command rejected");
    }
    else if (motionControllerStraightFeedforwardTestUpdateStatus !=
             MOTIONCONTROLLER_STATUS_OK)
    {
        OLED_Printf(0, 0, "Control error:%u",
            (unsigned)motionControllerStraightFeedforwardTestUpdateStatus);
    }
    else if (motionControllerStraightFeedforwardTestTimedOut)
    {
        /*
         * Diagnostic timeout screen.
         *
         * Mode values:
         *   0 = IDLE
         *   1 = STRAIGHT
         *   2 = ARC
         *   3 = BRAKING
         *   4 = ARC_PREPARING
         *   5 = STRAIGHT_PREPARING
         *
         * P    = MotionProfile active
         * Stat = consecutive stationary samples
         * Exit = whether the normal STRAIGHT-exit transition
         *        had already been observed
         */
        OLED_Printf(
            0, 0,
            "TIMEOUT M:%lu P:%u",
            (unsigned long)
                motionControllerStraightFeedforwardTestTimeoutMode,
            motionControllerStraightFeedforwardTestTimeoutProfileActive
                ? 1U : 0U);

        OLED_Printf(
            0, 1,
            "D:%.1f/%.1f",
            motionControllerStraightFeedforwardTestTimeoutDistanceMm,
            motionControllerStraightFeedforwardTestTimeoutTargetDistanceMm);

        OLED_Printf(
            0, 2,
            "Stat:%lu Exit:%u",
            (unsigned long)
                motionControllerStraightFeedforwardTestTimeoutStationarySamples,
            motionControllerStraightFeedforwardTestTimeoutMotionExitCaptured
                ? 1U : 0U);

        OLED_Printf(
            0, 3,
            "L:%+.0f R:%+.0f",
            motionControllerStraightFeedforwardTestTimeoutLeftCps,
            motionControllerStraightFeedforwardTestTimeoutRightCps);

        OLED_Printf(
            0, 4,
            "Yaw:%+.2f",
            motionControllerStraightFeedforwardTestTimeoutYawDeg);
    }
    else
    {
        /*
         * Normal successful result screen.
         */
        OLED_Printf(
            0, 0,
            "Move Exit:%+.2f",
            motionControllerStraightFeedforwardTestMotionExitYawDeg);

        OLED_Printf(
            0, 1,
            "Ideal:%+.2f",
            motionControllerStraightFeedforwardTestMotionExitIdealYawDeg);

        OLED_Printf(
            0, 2,
            "Final:%+.2f",
            motionControllerStraightFeedforwardTestFinalYawDeg);

        OLED_Printf(
            0, 3,
            "Dist:%6.1f",
            motionControllerStraightFeedforwardTestFinalDistanceMm);

        OLED_Printf(
            0, 4,
            "N:%lu",
            motionControllerStraightFeedforwardTestLogCount);
    }

    OLED_Refresh_Gram();


}

void MotionControllerStraightFeedforwardTestRun(void)
{
    RobotTestFixture fixture = { 0 };
    /* Read exported volatile settings so section GC retains their symbols. */
    const float requestedDistanceMm =
        motionControllerStraightFeedforwardTestRequestedDistanceMm;
    const float requestedSpeedCps =
        motionControllerStraightFeedforwardTestRequestedSpeedCps;
    const float rawCommand = motionControllerStraightFeedforwardTestRawCommand;


    if (!MotionControllerStraightFeedforwardTest_Init(&fixture))
    {
        OLED_Printf(0, 0, "Straight Init Failed");
        OLED_Refresh_Gram();

        SW1_WhileNotPressed();
        return;
    }


    OLED_Printf(
        0, 0,
        "Straight FF Test");

    OLED_Printf(
        0, 1,
        "Raw:%+.3f S:%.0f",
        rawCommand,
        requestedDistanceMm);

    OLED_Printf(
        0, 2,
        "Speed: %.0f CPS",
        requestedSpeedCps);

    OLED_Printf(
        0, 3,
        "Ideal yaw:%+.2f",
        STRAIGHT_FF_TEST_NOMINAL_YAW_DEG);

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
        STRAIGHT_FF_TEST_CONTROL_PERIOD_S))
    {
        HAL_Delay(
            STRAIGHT_FF_TEST_CONTROL_PERIOD_MS);
    }


    /*
     * Allow centre position to settle mechanically.
     */
    HAL_Delay(100U);


    OLED_Refresh_Gram();


    /*
     * Only reset the motion-test log after pre-steering.
     *
     * Therefore t = 0 in the exported dataset remains
     * the instant immediately before motion begins.
     */
    MotionControllerStraightFeedforwardTest_ResetLog();


    /* Do not let Servo_SetSteering silently clamp a calibration candidate. */
    if (!isfinite(rawCommand) ||
        rawCommand < SERVO_STEER_MIN ||
        rawCommand > SERVO_STEER_MAX)
    {
        MotionControllerStraightFeedforwardTest_ShowFinal();
        SW1_WhileNotPressed();
        return;
    }

    /*
     * Start the zero-curvature motion. MoveStraight() applies the configured
     * raw feedforward before the 500 ms preparation interval.
     */
    motionControllerStraightFeedforwardTestCommandAccepted =
        MotionController_MoveStraight(
            &fixture.motionController,
            requestedDistanceMm,
            requestedSpeedCps) ==
        MOTIONCONTROLLER_STATUS_OK;


    if (!motionControllerStraightFeedforwardTestCommandAccepted)
    {
        MotionControllerStraightFeedforwardTest_ShowFinal();
        SW1_WhileNotPressed();
        return;
    }

    /*
     * Capture state before the first control update.
     */
    MotionControllerStraightFeedforwardTest_LogSample(
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
            STRAIGHT_FF_TEST_TIMEOUT_MS)
        {
            /*
             * Preserve the state that caused the timeout before
             * MotionController_Brake() changes the controller mode
             * and motion-profile state.
             */
            MotionControllerStraightFeedforwardTest_CaptureTimeout(
                &fixture);

            motionControllerStraightFeedforwardTestTimedOut = true;

            MotionController_Brake(
                &fixture.motionController);

            break;
        }


        /*
         * 100 Hz controller.
         */
        if ((now - lastControlTick) >=
            STRAIGHT_FF_TEST_CONTROL_PERIOD_MS)
        {
            lastControlTick +=
                STRAIGHT_FF_TEST_CONTROL_PERIOD_MS;


            MotionControllerMode previousMode =
                fixture.motionController.mode;


            MotionControllerStatus updateStatus = MotionController_Update(
                &fixture.motionController,
                STRAIGHT_FF_TEST_CONTROL_PERIOD_S);
            if (updateStatus != MOTIONCONTROLLER_STATUS_OK)
            {
                motionControllerStraightFeedforwardTestUpdateStatus = updateStatus;
            }


            MotionControllerStraightFeedforwardTest_UpdateMeasurements(
                &fixture);


            /*
             * Capture the first transition out of STRAIGHT mode.
             *
             * Normally this will be STRAIGHT -> BRAKING.
             */
            if ((previousMode == MOTIONCONTROLLER_STRAIGHT) &&
                (fixture.motionController.mode !=
                 MOTIONCONTROLLER_STRAIGHT))
            {
                MotionControllerStraightFeedforwardTest_CaptureMotionExit(
                    &fixture,
                    now - startTick);
            }
        }


        /*
         * 50 Hz debugger log.
         */
        if ((now - lastLogTick) >=
            STRAIGHT_FF_TEST_LOG_INTERVAL_MS)
        {
            lastLogTick +=
                STRAIGHT_FF_TEST_LOG_INTERVAL_MS;


            MotionControllerStraightFeedforwardTest_LogSample(
                &fixture,
                now - startTick);
        }
    }


    /*
     * If timeout initiated braking, continue stepping the
     * controller so active braking can complete.
     */
    if (motionControllerStraightFeedforwardTestTimedOut)
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
                STRAIGHT_FF_TEST_CONTROL_PERIOD_MS)
            {
                lastControlTick +=
                    STRAIGHT_FF_TEST_CONTROL_PERIOD_MS;


                MotionController_Update(
                    &fixture.motionController,
                    STRAIGHT_FF_TEST_CONTROL_PERIOD_S);


                MotionControllerStraightFeedforwardTest_UpdateMeasurements(
                    &fixture);
            }


            if ((now - lastLogTick) >=
                STRAIGHT_FF_TEST_LOG_INTERVAL_MS)
            {
                lastLogTick +=
                    STRAIGHT_FF_TEST_LOG_INTERVAL_MS;


                MotionControllerStraightFeedforwardTest_LogSample(
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

    // Buzzer_BlockingBuzz(100);

    /*
     * Guaranteed final sample.
     */
    MotionControllerStraightFeedforwardTest_LogSample(
        &fixture,
        HAL_GetTick() - startTick);


    motionControllerStraightFeedforwardTestFinalYawDeg =
        fixture.motionController.yawDeg;

    motionControllerStraightFeedforwardTestFinalDistanceMm =
        fixture.motionController.travelledDistanceMm;

    motionControllerStraightFeedforwardTestFinalLeftDistanceMm =
        straightFeedforwardTestLeftDistanceMm;

    motionControllerStraightFeedforwardTestFinalRightDistanceMm =
        straightFeedforwardTestRightDistanceMm;


    MotionControllerStraightFeedforwardTest_ShowFinal();

    /*
     * Preserve all data in RAM for debugger export.
     */
    SW1_WhileNotPressed();
}
