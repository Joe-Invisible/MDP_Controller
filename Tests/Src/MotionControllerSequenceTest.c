/*
 * MotionControllerSequenceTest.c
 *
 * Mixed straight + constant-curvature arc command sequence test.
 *
 * The purpose of this test is different from MotionControllerArcTest:
 *
 *   - MotionControllerArcTest remains useful for isolated arc calibration
 *     and detailed feedforward validation.
 *
 *   - This test exercises the production-facing motion API as a sequence:
 *
 *         MoveStraight(...)
 *         MoveArc(...)
 *         MoveStraight(...)
 *         ...
 *
 * Each command is allowed to complete all the way to IDLE before the next
 * command is issued. Therefore this tests command composition and repeatable
 * stop/start transitions; it is NOT trajectory blending.
 *
 * MotionController currently resets yaw and odometry at the start of every
 * command. This harness therefore maintains separate sequence-level yaw and
 * distance bookkeeping while retaining the command-local controller values
 * in the debugger log.
 *
 * Created on: 2026-09-17
 * Author: Joe
 */

#include "MotionControllerSequenceTest.h"

#include "buzzer.h"
#include "oledutils.h"
#include "RobotTestFixture.h"
#include "userbutton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>


/* -------------------------------------------------------------------------- */
/* Test configuration                                                         */
/* -------------------------------------------------------------------------- */

#define SEQ_TEST_CONTROL_PERIOD_MS             (10U)
#define SEQ_TEST_CONTROL_PERIOD_S              (0.010f)

/*
 * 40 ms gives 25 Hz diagnostics and 20 s of logging with 500 samples,
 * without increasing the already-large debugger log RAM footprint.
 *
 * Change back to 20 ms when analysing a short sequence in more detail.
 */
#define SEQ_TEST_LOG_INTERVAL_MS               (40U)
#define SEQ_TEST_LOG_CAPACITY                  (500U)

#define SEQ_TEST_COMMAND_TIMEOUT_MS            (20000U)
#define SEQ_TEST_BRAKE_TIMEOUT_MS              (3000U)

#define SEQ_TEST_INITIAL_CLEARANCE_MS           (400U)
#define SEQ_TEST_INITIAL_CENTRE_SETTLE_MS       (100U)

#define SEQ_TEST_STRAIGHT_SETTLE_MS             (100U)

#define SEQ_TEST_BETWEEN_COMMANDS_MS            (200U)


/*
 * Rear-wheel calibration used only by the independent diagnostic
 * wheel-distance integrator.
 */
#define SEQ_TEST_ENCODER_CPR                    (1560.0f)
#define SEQ_TEST_WHEEL_DIAMETER_MM              (65.5f)


/*
 * Current production-candidate controller tuning.
 */

#define SEQ_TEST_PI                             (3.14159265358979323846f)
#define SEQ_TEST_RAD_TO_DEG                     (180.0f / SEQ_TEST_PI)

#define SEQ_TEST_MM_PER_COUNT \
    (SEQ_TEST_PI * SEQ_TEST_WHEEL_DIAMETER_MM / SEQ_TEST_ENCODER_CPR)

#define SEQ_TEST_ARRAY_COUNT(a) \
    ((uint32_t)(sizeof(a) / sizeof((a)[0])))


/* -------------------------------------------------------------------------- */
/* Sequence definition                                                        */
/* -------------------------------------------------------------------------- */

/*
 * The macros make the test path read like a small command script.
 *
 * For STRAIGHT, radiusMm is unused.
 *
 * Example below:
 *
 *   1. forward straight 300 mm
 *   2. forward right/negative-radius 90-degree arc at R = -350 mm
 *   3. forward straight 300 mm
 *   4. forward left/positive-radius 90-degree arc at R = +350 mm
 *
 * Replace only this array for most sequence experiments.
 */

#define SEQ_STRAIGHT(distanceMm_, speedCps_) \
    { \
        MOTION_SEQUENCE_TEST_STRAIGHT, \
        (distanceMm_), \
        0.0f, \
        (speedCps_) \
    }

#define SEQ_ARC(distanceMm_, radiusMm_, speedCps_) \
    { \
        MOTION_SEQUENCE_TEST_ARC, \
        (distanceMm_), \
        (radiusMm_), \
        (speedCps_) \
    }


static const MotionControllerSequenceTestCommand
motionControllerSequenceTestCommands[] = {
	SEQ_STRAIGHT(
		2000.0f,
		6000.0f),
	SEQ_ARC(
		350.0f * SEQ_TEST_PI / 2.0f,
		-350.0f,
		2000.0f),
	SEQ_STRAIGHT(
		800.0f,
		6000.0f),
};
//motionControllerSequenceTestCommands[] =
//{
//	// BL
//    SEQ_STRAIGHT(
//        500.0f,
//        6000.0f),
//
//    /*
//     * Quarter-circle:
//     *     s = |R| * pi / 2
//     *
//     * Positive distance + negative radius gives negative yaw.
//     */
//	// TL
//    SEQ_ARC(
//        350.0f * SEQ_TEST_PI / 2.0f,
//        -350.0f,
//        2000.0f),
//
//	// TL
//    SEQ_STRAIGHT(
//        500.0f,
//        6000.0f),
//
//	// TR
//    SEQ_ARC(
//        350.0f * SEQ_TEST_PI / 2.0f,
//        -350.0f,
//        2000.0f),
//
//	// TR
//	SEQ_STRAIGHT(
//		500.0f,
//		6000.0f),
//
//	// BR
//	SEQ_ARC(
//		350.0f * SEQ_TEST_PI / 2.0f,
//		-350.0f,
//		2000.0f),
//
//	// BR
//	SEQ_STRAIGHT(
//		500.0f,
//		6000.0f),
//
//	// BL
//	SEQ_ARC(
//		350.0f * SEQ_TEST_PI / 2.0f,
//		-350.0f,
//		2000.0f),
//};


static const uint32_t
motionControllerSequenceTestCommandCount =
    SEQ_TEST_ARRAY_COUNT(
        motionControllerSequenceTestCommands);


/* -------------------------------------------------------------------------- */
/* Debugger-visible exports                                                   */
/* -------------------------------------------------------------------------- */

volatile MotionControllerSequenceTestLogSample
motionControllerSequenceTestLog[SEQ_TEST_LOG_CAPACITY];

volatile uint32_t
motionControllerSequenceTestLogCount = 0U;


volatile MotionControllerSequenceTestResult
motionControllerSequenceTestResults[
    SEQ_TEST_ARRAY_COUNT(
        motionControllerSequenceTestCommands)];

volatile uint32_t
motionControllerSequenceTestResultCount = 0U;


volatile bool
motionControllerSequenceTestPassed = false;

volatile bool
motionControllerSequenceTestTimedOut = false;

volatile bool
motionControllerSequenceTestCommandRejected = false;

volatile uint32_t
motionControllerSequenceTestFailureCommandIndex = UINT32_MAX;


/*
 * Final whole-sequence bookkeeping.
 */
volatile float
motionControllerSequenceTestFinalYawDeg = 0.0f;

volatile float
motionControllerSequenceTestFinalIdealYawDeg = 0.0f;

volatile float
motionControllerSequenceTestFinalTravelledDistanceMm = 0.0f;


volatile const char *motionControllerSequenceTestInfo =
    "Mixed straight/arc sequence; each command completes to IDLE; "
    "MotionController owns arc raw preposition and settling";


/* -------------------------------------------------------------------------- */
/* Local sequence state                                                       */
/* -------------------------------------------------------------------------- */

typedef struct
{
    float leftDistanceMm;
    float rightDistanceMm;

    float completedYawDeg;
    float completedIdealYawDeg;
    float completedTravelledDistanceMm;

} MotionControllerSequenceTestState;


/* -------------------------------------------------------------------------- */
/* Utility helpers                                                            */
/* -------------------------------------------------------------------------- */

static float MotionControllerSequenceTest_CommandIdealYawDeg(
    const MotionControllerSequenceTestCommand *command)
{
    if (command->type !=
        MOTION_SEQUENCE_TEST_ARC)
    {
        return 0.0f;
    }

    if (command->radiusMm == 0.0f)
    {
        return 0.0f;
    }

    return
        (command->distanceMm /
         command->radiusMm) *
        SEQ_TEST_RAD_TO_DEG;
}


static float MotionControllerSequenceTest_SignedCommandDistance(
    const MotionControllerSequenceTestCommand *command,
    float controllerDistanceMm)
{
    if (command->distanceMm < 0.0f)
    {
        return -fabsf(controllerDistanceMm);
    }

    return fabsf(controllerDistanceMm);
}


static float MotionControllerSequenceTest_CurrentIdealYawDeg(
    const MotionControllerSequenceTestCommand *command,
    const MotionController *motionController)
{
    if (command->type !=
        MOTION_SEQUENCE_TEST_ARC)
    {
        return 0.0f;
    }

    float requestedMagnitudeMm =
        fabsf(command->distanceMm);

    if (requestedMagnitudeMm <= 0.0f)
    {
        return 0.0f;
    }

    float progress =
        motionController->travelledDistanceMm /
        requestedMagnitudeMm;

    if (progress < 0.0f)
    {
        progress = 0.0f;
    }

    if (progress > 1.0f)
    {
        progress = 1.0f;
    }

    return
        progress *
        MotionControllerSequenceTest_CommandIdealYawDeg(
            command);
}


/* -------------------------------------------------------------------------- */
/* Reset                                                                      */
/* -------------------------------------------------------------------------- */

static void MotionControllerSequenceTest_ResetExports(void)
{
    motionControllerSequenceTestLogCount = 0U;
    motionControllerSequenceTestResultCount = 0U;

    motionControllerSequenceTestPassed = false;
    motionControllerSequenceTestTimedOut = false;
    motionControllerSequenceTestCommandRejected = false;

    motionControllerSequenceTestFailureCommandIndex =
        UINT32_MAX;

    motionControllerSequenceTestFinalYawDeg = 0.0f;
    motionControllerSequenceTestFinalIdealYawDeg = 0.0f;
    motionControllerSequenceTestFinalTravelledDistanceMm = 0.0f;


    for (uint32_t i = 0U;
         i < motionControllerSequenceTestCommandCount;
         i++)
    {
        motionControllerSequenceTestResults[i] =
            (MotionControllerSequenceTestResult){0};
    }
}


/* -------------------------------------------------------------------------- */
/* Independent diagnostic measurements                                        */
/* -------------------------------------------------------------------------- */

static void MotionControllerSequenceTest_UpdateMeasurements(
    RobotTestFixture *fixture,
    MotionControllerSequenceTestState *state)
{
    MotionController *motionController =
        &fixture->motionController;

    WheelSpeedController *leftWheel =
        motionController->leftWheel;

    WheelSpeedController *rightWheel =
        motionController->rightWheel;


    state->leftDistanceMm +=
        leftWheel->measuredSpeedCps *
        SEQ_TEST_CONTROL_PERIOD_S *
        SEQ_TEST_MM_PER_COUNT;

    state->rightDistanceMm +=
        rightWheel->measuredSpeedCps *
        SEQ_TEST_CONTROL_PERIOD_S *
        SEQ_TEST_MM_PER_COUNT;
}


/* -------------------------------------------------------------------------- */
/* Logging                                                                    */
/* -------------------------------------------------------------------------- */

static void MotionControllerSequenceTest_LogSample(
    RobotTestFixture *fixture,
    const MotionControllerSequenceTestState *state,
    const MotionControllerSequenceTestCommand *command,
    uint32_t commandIndex,
    uint32_t sequenceElapsedMs,
    uint32_t commandElapsedMs)
{
    if (motionControllerSequenceTestLogCount >=
        SEQ_TEST_LOG_CAPACITY)
    {
        return;
    }


    MotionController *motionController =
        &fixture->motionController;

    WheelSpeedController *leftWheel =
        motionController->leftWheel;

    WheelSpeedController *rightWheel =
        motionController->rightWheel;


    MotionControllerSequenceTestLogSample *sample =
        (MotionControllerSequenceTestLogSample *)
        &motionControllerSequenceTestLog[
            motionControllerSequenceTestLogCount];


    *sample =
        (MotionControllerSequenceTestLogSample){0};


    sample->sequenceTimeMs =
        sequenceElapsedMs;

    sample->commandTimeMs =
        commandElapsedMs;

    sample->commandIndex =
        commandIndex;

    sample->commandType =
        (uint32_t)command->type;

    sample->state =
        (uint32_t)motionController->mode;


    /*
     * Whole-sequence bookkeeping.
     */
    sample->sequenceYawDeg =
        state->completedYawDeg +
        motionController->yawDeg;

    sample->sequenceIdealYawDeg =
        state->completedIdealYawDeg +
        MotionControllerSequenceTest_CurrentIdealYawDeg(
            command,
            motionController);

    sample->sequenceTravelledDistanceMm =
        state->completedTravelledDistanceMm +
        MotionControllerSequenceTest_SignedCommandDistance(
            command,
            motionController->travelledDistanceMm);


    /*
     * Motion profile / local command state.
     */
    sample->profileTargetSpeedCps =
        motionController->targetSpeedCps;

    sample->travelledDistanceMm =
        motionController->travelledDistanceMm;

    sample->targetCurvaturePerMm =
        motionController->targetCurvaturePerMm;

    sample->arcCommandedCurvaturePerMm =
        motionController->arcCommandedCurvaturePerMm;


    /*
     * Steering.
     */
    sample->feedforwardSteeringAngleRad =
        motionController->targetSteeringAngleRad;

    sample->steeringTargetAngleRad =
        SteeringController_GetTargetEffectiveAngleRad(
            motionController->steering);

    sample->effectiveSteeringAngleRad =
        SteeringController_GetEffectiveAngleRad(
            motionController->steering);

    sample->steeringCommand =
        SteeringController_GetCommand(
            motionController->steering);


    /*
     * ARC outer heading loop.
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
     * ARC inner yaw-rate loop.
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

    sample->measuredYawRateRadPerSec =
        motionController->filteredYawRateDps *
        (SEQ_TEST_PI / 180.0f);


    /*
     * Wheel-speed controllers.
     */
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


    /*
     * Independent sequence-level wheel integration.
     */
    sample->leftDistanceMm =
        state->leftDistanceMm;

    sample->rightDistanceMm =
        state->rightDistanceMm;


    /*
     * Rear-wheel geometric reference / synchroniser.
     */
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


    /*
     * Command-local heading.
     */
    sample->yawDeg =
        motionController->yawDeg;

    sample->idealYawDeg =
        MotionControllerSequenceTest_CurrentIdealYawDeg(
            command,
            motionController);


    /*
     * Dynamic braking.
     */
    sample->leftActuatorMode =
        (uint32_t)leftWheel->actuatorMode;

    sample->rightActuatorMode =
        (uint32_t)rightWheel->actuatorMode;

    sample->leftBrakeDemand =
        leftWheel->brakeDemand;

    sample->rightBrakeDemand =
        rightWheel->brakeDemand;

    sample->leftBrakePWM =
        leftWheel->brakePWM;

    sample->rightBrakePWM =
        rightWheel->brakePWM;


    motionControllerSequenceTestLogCount++;
}


/* -------------------------------------------------------------------------- */
/* Initialisation                                                              */
/* -------------------------------------------------------------------------- */

static bool MotionControllerSequenceTest_Init(
    RobotTestFixture *fixture)
{
    OLED_Init();

    OLED_Clear();
    OLED_Refresh_Gram();


    return RobotTestFixture_InitMotionController(
        fixture,
        &motionControllerConfig);
}


static void MotionControllerSequenceTest_CentreSteering(
    RobotTestFixture *fixture)
{
    SteeringController_StartCentre(
        &fixture->steeringController);


    while (!SteeringController_UpdateCentre(
        &fixture->steeringController,
        SEQ_TEST_CONTROL_PERIOD_S))
    {
        HAL_Delay(
            SEQ_TEST_CONTROL_PERIOD_MS);
    }


    HAL_Delay(
        SEQ_TEST_INITIAL_CENTRE_SETTLE_MS);
}


/* -------------------------------------------------------------------------- */
/* Command preparation                                                        */
/* -------------------------------------------------------------------------- */

static bool MotionControllerSequenceTest_BeginCommand(
    RobotTestFixture *fixture,
    const MotionControllerSequenceTestCommand *command)
{
    MotionControllerStatus status =
        MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;


    switch (command->type)
    {
        case MOTION_SEQUENCE_TEST_STRAIGHT:
        {
            status =
                MotionController_MoveStraight(
                    &fixture->motionController,
                    command->distanceMm,
                    command->speedCps);

            if (status == MOTIONCONTROLLER_STATUS_OK)
            {
                /*
                 * MoveStraight() centres steering immediately.
                 * Let the front axle settle before profile updates begin.
                 */
                HAL_Delay(
                    SEQ_TEST_STRAIGHT_SETTLE_MS);
            }

            break;
        }


        case MOTION_SEQUENCE_TEST_ARC:
        {
            status =
                MotionController_MoveArc(
                    &fixture->motionController,
                    command->distanceMm,
                    command->radiusMm,
                    command->speedCps);

            break;
        }


        default:
        {
            status =
                MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
            break;
        }
    }


    return status == MOTIONCONTROLLER_STATUS_OK;
}


/* -------------------------------------------------------------------------- */
/* Result capture                                                             */
/* -------------------------------------------------------------------------- */

static void MotionControllerSequenceTest_CaptureMotionExit(
    RobotTestFixture *fixture,
    const MotionControllerSequenceTestCommand *command,
    MotionControllerSequenceTestResult *result,
    uint32_t commandElapsedMs)
{
    if (result->motionExitCaptured)
    {
        return;
    }


    result->motionExitCaptured = true;

    result->motionExitTimeMs =
        commandElapsedMs;

    result->motionExitDistanceMm =
        fixture->motionController.travelledDistanceMm;

    result->motionExitYawDeg =
        fixture->motionController.yawDeg;

    result->motionExitIdealYawDeg =
        MotionControllerSequenceTest_CurrentIdealYawDeg(
            command,
            &fixture->motionController);
}


/* -------------------------------------------------------------------------- */
/* One command                                                                */
/* -------------------------------------------------------------------------- */

static bool MotionControllerSequenceTest_RunCommand(
    RobotTestFixture *fixture,
    MotionControllerSequenceTestState *state,
    const MotionControllerSequenceTestCommand *command,
    uint32_t commandIndex,
    uint32_t sequenceStartTick)
{
    MotionControllerSequenceTestResult *result =
        (MotionControllerSequenceTestResult *)
        &motionControllerSequenceTestResults[
            commandIndex];


    *result =
        (MotionControllerSequenceTestResult){0};


    result->commandIndex =
        commandIndex;

    result->commandType =
        (uint32_t)command->type;

    result->requestedDistanceMm =
        command->distanceMm;

    result->requestedRadiusMm =
        command->radiusMm;

    result->requestedSpeedCps =
        command->speedCps;


    if (!MotionControllerSequenceTest_BeginCommand(
            fixture,
            command))
    {
        result->commandAccepted = false;

        motionControllerSequenceTestCommandRejected =
            true;

        motionControllerSequenceTestFailureCommandIndex =
            commandIndex;

        return false;
    }


    result->commandAccepted = true;


    /*
     * Start command timing only after steering preparation/settling.
     * Thus commandTimeMs = 0 corresponds to immediately before the
     * first controller update, matching the useful convention from
     * the original arc test.
     */
    uint32_t commandStartTick =
        HAL_GetTick();

    result->startTimeMs =
        commandStartTick -
        sequenceStartTick;


    uint32_t lastControlTick =
        commandStartTick;

    uint32_t lastLogTick =
        commandStartTick;


    MotionControllerSequenceTest_LogSample(
        fixture,
        state,
        command,
        commandIndex,
        commandStartTick - sequenceStartTick,
        0U);


    while (fixture->motionController.mode !=
           MOTIONCONTROLLER_IDLE)
    {
        uint32_t now =
            HAL_GetTick();


        if ((now - commandStartTick) >=
            SEQ_TEST_COMMAND_TIMEOUT_MS)
        {
            result->timedOut = true;

            motionControllerSequenceTestTimedOut =
                true;

            motionControllerSequenceTestFailureCommandIndex =
                commandIndex;

            MotionController_Brake(
                &fixture->motionController);

            break;
        }


        if ((now - lastControlTick) >=
            SEQ_TEST_CONTROL_PERIOD_MS)
        {
            lastControlTick +=
                SEQ_TEST_CONTROL_PERIOD_MS;


            MotionControllerMode previousMode =
                fixture->motionController.mode;


            MotionController_Update(
                &fixture->motionController,
                SEQ_TEST_CONTROL_PERIOD_S);


            MotionControllerSequenceTest_UpdateMeasurements(
                fixture,
                state);


            /*
             * Capture the first transition out of the commanded
             * motion mode. Normally:
             *
             *     STRAIGHT -> BRAKING
             *     ARC      -> BRAKING
             */
            MotionControllerMode commandedMode =
                (command->type ==
                 MOTION_SEQUENCE_TEST_ARC)
                    ? MOTIONCONTROLLER_ARC
                    : MOTIONCONTROLLER_STRAIGHT;


            if ((previousMode == commandedMode) &&
                (fixture->motionController.mode !=
                 commandedMode))
            {
                MotionControllerSequenceTest_CaptureMotionExit(
                    fixture,
                    command,
                    result,
                    now - commandStartTick);
            }
        }


        if ((now - lastLogTick) >=
            SEQ_TEST_LOG_INTERVAL_MS)
        {
            lastLogTick +=
                SEQ_TEST_LOG_INTERVAL_MS;


            MotionControllerSequenceTest_LogSample(
                fixture,
                state,
                command,
                commandIndex,
                now - sequenceStartTick,
                now - commandStartTick);
        }
    }


    /*
     * If timeout requested braking, allow braking to finish.
     */
    if (result->timedOut)
    {
        uint32_t brakeStartTick =
            HAL_GetTick();


        while ((fixture->motionController.mode !=
                MOTIONCONTROLLER_IDLE) &&
               ((HAL_GetTick() - brakeStartTick) <
                SEQ_TEST_BRAKE_TIMEOUT_MS))
        {
            uint32_t now =
                HAL_GetTick();


            if ((now - lastControlTick) >=
                SEQ_TEST_CONTROL_PERIOD_MS)
            {
                lastControlTick +=
                    SEQ_TEST_CONTROL_PERIOD_MS;


                MotionController_Update(
                    &fixture->motionController,
                    SEQ_TEST_CONTROL_PERIOD_S);


                MotionControllerSequenceTest_UpdateMeasurements(
                    fixture,
                    state);
            }


            if ((now - lastLogTick) >=
                SEQ_TEST_LOG_INTERVAL_MS)
            {
                lastLogTick +=
                    SEQ_TEST_LOG_INTERVAL_MS;


                MotionControllerSequenceTest_LogSample(
                    fixture,
                    state,
                    command,
                    commandIndex,
                    now - sequenceStartTick,
                    now - commandStartTick);
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
     * Guaranteed final sample for this command.
     */
    uint32_t commandEndTick =
        HAL_GetTick();


    MotionControllerSequenceTest_LogSample(
        fixture,
        state,
        command,
        commandIndex,
        commandEndTick - sequenceStartTick,
        commandEndTick - commandStartTick);


    result->endTimeMs =
        commandEndTick -
        sequenceStartTick;

    result->finalDistanceMm =
        fixture->motionController.travelledDistanceMm;

    result->finalYawDeg =
        fixture->motionController.yawDeg;


    /*
     * Whole-sequence pose update.
     *
     * Use FINAL yaw rather than motion-exit yaw because the robot's
     * physical heading after braking is the heading from which the
     * next command starts.
     */
    state->completedYawDeg +=
        result->finalYawDeg;

    state->completedIdealYawDeg +=
        MotionControllerSequenceTest_CommandIdealYawDeg(
            command);

    state->completedTravelledDistanceMm +=
        MotionControllerSequenceTest_SignedCommandDistance(
            command,
            result->finalDistanceMm);


    result->sequenceYawDegAfter =
        state->completedYawDeg;

    result->sequenceIdealYawDegAfter =
        state->completedIdealYawDeg;

    result->sequenceTravelledDistanceMmAfter =
        state->completedTravelledDistanceMm;


    motionControllerSequenceTestResultCount =
        commandIndex + 1U;


    return !result->timedOut;
}


/* -------------------------------------------------------------------------- */
/* OLED helpers                                                               */
/* -------------------------------------------------------------------------- */

static void MotionControllerSequenceTest_ShowReady(void)
{
    OLED_Clear();

    OLED_Printf(
        0, 0,
        "Motion Seq Test");

    OLED_Printf(
        0, 1,
        "Commands:%lu",
        (unsigned long)
            motionControllerSequenceTestCommandCount);

    OLED_Printf(
        0, 2,
        "Ctrl:%lums Log:%lums",
        (unsigned long)SEQ_TEST_CONTROL_PERIOD_MS,
        (unsigned long)SEQ_TEST_LOG_INTERVAL_MS);

    OLED_Printf(
        0, 4,
        "Start: SW1");

    OLED_Refresh_Gram();
}


static void MotionControllerSequenceTest_ShowCommand(
    const MotionControllerSequenceTestCommand *command,
    uint32_t commandIndex)
{
    OLED_Clear();

    OLED_Printf(
        0, 0,
        "Cmd %lu/%lu",
        (unsigned long)(commandIndex + 1U),
        (unsigned long)
            motionControllerSequenceTestCommandCount);


    if (command->type ==
        MOTION_SEQUENCE_TEST_STRAIGHT)
    {
        OLED_Printf(
            0, 1,
            "STRAIGHT");

        OLED_Printf(
            0, 2,
            "S:%+.0f mm",
            command->distanceMm);
    }
    else
    {
        OLED_Printf(
            0, 1,
            "ARC");

        OLED_Printf(
            0, 2,
            "S:%+.0f R:%+.0f",
            command->distanceMm,
            command->radiusMm);
    }


    OLED_Printf(
        0, 3,
        "%.0f CPS",
        command->speedCps);

    OLED_Refresh_Gram();
}


static void MotionControllerSequenceTest_ShowFinal(void)
{
    OLED_Clear();


    if (motionControllerSequenceTestPassed)
    {
        OLED_Printf(
            0, 0,
            "SEQ COMPLETE");
    }
    else if (motionControllerSequenceTestCommandRejected)
    {
        OLED_Printf(
            0, 0,
            "CMD REJECTED");
    }
    else if (motionControllerSequenceTestTimedOut)
    {
        OLED_Printf(
            0, 0,
            "SEQ TIMEOUT");
    }
    else
    {
        OLED_Printf(
            0, 0,
            "SEQ FAILED");
    }


    OLED_Printf(
        0, 1,
        "Done:%lu/%lu",
        (unsigned long)
            motionControllerSequenceTestResultCount,
        (unsigned long)
            motionControllerSequenceTestCommandCount);

    OLED_Printf(
        0, 2,
        "Yaw:%+.2f",
        motionControllerSequenceTestFinalYawDeg);

    OLED_Printf(
        0, 3,
        "Ideal:%+.2f",
        motionControllerSequenceTestFinalIdealYawDeg);

    OLED_Printf(
        0, 4,
        "S:%+.1f",
        motionControllerSequenceTestFinalTravelledDistanceMm);

    OLED_Printf(
        0, 5,
        "Log:%lu",
        (unsigned long)
            motionControllerSequenceTestLogCount);


    OLED_Refresh_Gram();
}


/* -------------------------------------------------------------------------- */
/* Public test                                                                */
/* -------------------------------------------------------------------------- */

void MotionControllerSequenceTestRun(void)
{
    RobotTestFixture fixture =
        {0};

    MotionControllerSequenceTestState state =
        {0};


    MotionControllerSequenceTest_ResetExports();


    if (!MotionControllerSequenceTest_Init(
            &fixture))
    {
        OLED_Printf(
            0, 0,
            "Seq Init Failed");

        OLED_Refresh_Gram();

        SW1_WhileNotPressed();

        return;
    }


    MotionControllerSequenceTest_ShowReady();


    SW1_WaitForPressAndRelease();


    OLED_Clear();
    OLED_Refresh_Gram();


    /*
     * Allow the user's hand to clear the robot.
     */
    HAL_Delay(
        SEQ_TEST_INITIAL_CLEARANCE_MS);


    /*
     * Establish a deterministic steering starting state once,
     * before the whole sequence.
     */
    MotionControllerSequenceTest_CentreSteering(
        &fixture);


    uint32_t sequenceStartTick =
        HAL_GetTick();


    for (uint32_t commandIndex = 0U;
         commandIndex <
            motionControllerSequenceTestCommandCount;
         commandIndex++)
    {
        const MotionControllerSequenceTestCommand *command =
            &motionControllerSequenceTestCommands[
                commandIndex];


        MotionControllerSequenceTest_ShowCommand(
            command,
            commandIndex);


        if (!MotionControllerSequenceTest_RunCommand(
                &fixture,
                &state,
                command,
                commandIndex,
                sequenceStartTick))
        {
            break;
        }


        if ((commandIndex + 1U) <
            motionControllerSequenceTestCommandCount)
        {
            HAL_Delay(
                SEQ_TEST_BETWEEN_COMMANDS_MS);
        }
    }


    motionControllerSequenceTestPassed =
        (motionControllerSequenceTestResultCount ==
         motionControllerSequenceTestCommandCount) &&
        !motionControllerSequenceTestTimedOut &&
        !motionControllerSequenceTestCommandRejected;


    motionControllerSequenceTestFinalYawDeg =
        state.completedYawDeg;

    motionControllerSequenceTestFinalIdealYawDeg =
        state.completedIdealYawDeg;

    motionControllerSequenceTestFinalTravelledDistanceMm =
        state.completedTravelledDistanceMm;


    Buzzer_BlockingBuzz(
        500);


    MotionControllerSequenceTest_ShowFinal();


    /*
     * Preserve logs/results in RAM for debugger export.
     */
    SW1_WhileNotPressed();
}
