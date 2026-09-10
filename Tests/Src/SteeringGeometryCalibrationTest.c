/*
 * SteeringGeometryCalibrationTest.c
 *
 * Focused calibration of:
 *
 *     raw steering command
 *
 * against the effective bicycle-model steering angle.
 *
 * This version is intentionally matched to the startup history used by
 * the feedforward constant-curvature arc test:
 *
 *   1. deterministic SteeringController centering,
 *   2. 100 ms centre settling,
 *   3. rate-limited approach to the raw test command at 60 command/s,
 *   4. 500 ms preposition phase,
 *   5. 500 ms additional mechanical hold,
 *   6. manual push while gyro-Z and rear encoders are recorded.
 *
 * The focused command set brackets the raw command expected to produce
 * approximately +0.0145 rad effective steering in the current model.
 */

#include "SteeringGeometryCalibrationTest.h"

#include "RobotTestFixture.h"
#include "MotionControllerConfig.h"

#include "userbutton.h"
#include "oled.h"
#include "oledutils.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>


/* ------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------ */

#define PI_F                        3.14159265358979323846f
#define DEG_TO_RAD_F                (PI_F / 180.0f)
#define RAD_TO_DEG_F                (180.0f / PI_F)


/*
 * Gyro ODR is currently approximately 112.5 Hz.
 */
#define CAL_SAMPLE_PERIOD_MS        10U


/*
 * Steering timing chosen to match the arc-test setup.
 */
#define CAL_CONTROL_PERIOD_MS       10U
#define CAL_CONTROL_PERIOD_S        0.010f

#define CAL_CENTRE_SETTLE_MS        100U

#define CAL_PREPOSITION_MS          500U
#define CAL_PREPOSITION_HOLD_MS     500U

/*
 * Match SteeringController calibration setting currently used
 * by the motion controller.
 */
#define CAL_COMMAND_RATE_PER_SEC    60.0f


/*
 * After the user presses/releases SW1 to start a run, give them
 * time to remove their hand before acquisition begins.
 */
#define CAL_HANDS_OFF_DELAY_MS      500U


/*
 * Stop automatically after either:
 *
 *  - 600 mm rear-axle-centre travel, or
 *  - 20 degrees heading change after at least 100 mm travel.
 *
 * The focused commands are weak enough that the 600 mm criterion
 * should normally terminate the run.
 */
#define CAL_TARGET_TRAVEL_MM                600.0f
#define CAL_TARGET_YAW_DEG                  20.0f
#define CAL_MIN_TRAVEL_FOR_YAW_STOP_MM      100.0f


/*
 * Minimum travel required for a valid calibration result.
 */
#define CAL_MIN_VALID_TRAVEL_MM             100.0f


/*
 * Protection against an abandoned measurement.
 */
#define CAL_RUN_TIMEOUT_MS                  45000U


/*
 * Focused raw-command region.
 *
 * Current positive-curvature arc operation is near the decreasing-
 * command branch close to -12.  The set below brackets the region
 * expected to correspond roughly to +0.010 ... +0.016 rad.
 */
static const float calCommands[] =
{
    -12.00f,
    -11.75f,
    -11.50f,
    -11.25f,
    -11.00f,
};

#define CAL_COMMAND_COUNT \
    ((uint32_t)(sizeof(calCommands) / sizeof(calCommands[0])))

#define CAL_REPEAT_COUNT            3U

/*
 * After deterministic centering, positive curvature is reached by
 * moving toward lower raw commands.
 */
#define CAL_APPROACH_DIRECTION      (-1)


/* ------------------------------------------------------------
 * Debugger-exportable results
 * ------------------------------------------------------------ */

volatile SteeringGeometryCalibrationResult
    g_steeringGeometryCalibrationResults[
        STEERING_GEOMETRY_CAL_RESULT_COUNT];

volatile uint32_t
    g_steeringGeometryCalibrationResultCount = 0U;


/* ------------------------------------------------------------
 * Display
 * ------------------------------------------------------------ */

static void SteeringCal_ShowMessage(
    const char *message)
{
    OLED_Clear();

    OLED_Printf(
        0,
        0,
        "STEER GEO CAL");

    OLED_Printf(
        0,
        2,
        "%s",
        message);

    OLED_Refresh_Gram();
}


static void SteeringCal_ShowReady(
    float command,
    uint32_t commandIndex,
    uint32_t repeatIndex)
{
    OLED_Clear();

    OLED_Printf(
        0,
        0,
        "ARC STEER CAL");

    OLED_Printf(
        0,
        1,
        "CMD %lu/%lu R%lu/%lu",
        (unsigned long)(commandIndex + 1U),
        (unsigned long)CAL_COMMAND_COUNT,
        (unsigned long)(repeatIndex + 1U),
        (unsigned long)CAL_REPEAT_COUNT);

    OLED_Printf(
        0,
        2,
        "STEER %+.2f",
        command);

    OLED_Printf(
        0,
        3,
        "POSITION ROBOT");

    OLED_Printf(
        0,
        4,
        "PRESS TO ARM");

    OLED_Printf(
        0,
        5,
        "THEN PUSH");

    OLED_Refresh_Gram();
}


static void SteeringCal_ShowMeasuring(
    float command,
    uint32_t repeatIndex)
{
    OLED_Clear();

    OLED_Printf(
        0,
        0,
        "U%+.2f R%lu",
        command,
        (unsigned long)(repeatIndex + 1U));

    OLED_Printf(
        0,
        2,
        "MEASURING");

    OLED_Printf(
        0,
        3,
        "PUSH FORWARD");

    OLED_Printf(
        0,
        5,
        "BTN = ABORT");

    OLED_Refresh_Gram();
}


static void SteeringCal_ShowResult(
    const SteeringGeometryCalibrationResult *result)
{
    OLED_Clear();

    if (result->valid)
    {
        OLED_Printf(
            0,
            0,
            "RUN COMPLETE");
    }
    else if (result->imuReadFailed)
    {
        OLED_Printf(
            0,
            0,
            "IMU READ FAIL");
    }
    else if (result->aborted)
    {
        OLED_Printf(
            0,
            0,
            "RUN ABORTED");
    }
    else
    {
        OLED_Printf(
            0,
            0,
            "INVALID RUN");
    }

    OLED_Printf(
        0,
        1,
        "U%+.2f R%u",
        result->steeringCommand,
        (unsigned int)(result->repeatIndex + 1U));

    OLED_Printf(
        0,
        2,
        "S %.1f mm",
        result->centreTravelMm);

    OLED_Printf(
        0,
        3,
        "YAW %+.2f",
        result->yawGyroDeg);

    OLED_Printf(
        0,
        4,
        "dG %+.3f",
        result->effectiveAngleGyroRad);

    OLED_Printf(
        0,
        5,
        "dE %+.3f",
        result->effectiveAngleEncoderRad);

    OLED_Refresh_Gram();
}


/* ------------------------------------------------------------
 * Steering preparation
 * ------------------------------------------------------------ */

static void SteeringCal_DeterministicCentre(
    RobotTestFixture *fixture)
{
    SteeringController_StartCentre(
        &fixture->steeringController);

    while (!SteeringController_UpdateCentre(
        &fixture->steeringController,
        CAL_CONTROL_PERIOD_S))
    {
        HAL_Delay(
            CAL_CONTROL_PERIOD_MS);
    }

    /*
     * Same explicit post-centering mechanical settle used by
     * the arc test.
     */
    HAL_Delay(
        CAL_CENTRE_SETTLE_MS);
}


static void SteeringCal_PrepositionRawCommand(
    RobotTestFixture *fixture,
    float targetCommand)
{
    float currentCommand =
        SteeringController_GetCommand(
            &fixture->steeringController);

    const float maxDeltaCommand =
        CAL_COMMAND_RATE_PER_SEC *
        CAL_CONTROL_PERIOD_S;

    const uint32_t updateCount =
        CAL_PREPOSITION_MS /
        CAL_CONTROL_PERIOD_MS;

    /*
     * Run for a fixed 500 ms, exactly like the arc-test
     * preposition phase.  If the target is reached early, the
     * remaining updates simply continue commanding the same raw
     * position.
     */
    for (uint32_t i = 0U;
         i < updateCount;
         i++)
    {
        float deltaCommand =
            targetCommand -
            currentCommand;

        if (deltaCommand > maxDeltaCommand)
        {
            deltaCommand =
                maxDeltaCommand;
        }
        else if (deltaCommand < -maxDeltaCommand)
        {
            deltaCommand =
                -maxDeltaCommand;
        }

        currentCommand +=
            deltaCommand;

        SteeringController_SetCommand(
            &fixture->steeringController,
            currentCommand);

        HAL_Delay(
            CAL_CONTROL_PERIOD_MS);
    }

    /*
     * Guard against floating-point accumulation leaving us
     * microscopically short of the requested command.
     */
    SteeringController_SetCommand(
        &fixture->steeringController,
        targetCommand);

    /*
     * Additional mechanical hold used by the modified arc test.
     */
    HAL_Delay(
        CAL_PREPOSITION_HOLD_MS);
}


/* ------------------------------------------------------------
 * Measurement
 * ------------------------------------------------------------ */

static void SteeringCal_Measure(
    RobotTestFixture *fixture,
    float command,
    uint32_t commandIndex,
    uint32_t repeatIndex,
    volatile SteeringGeometryCalibrationResult *destination)
{
    SteeringGeometryCalibrationResult result = {0};

    result.steeringCommand =
        command;

    result.sweepDirection =
        CAL_APPROACH_DIRECTION;

    result.commandIndex =
        (uint8_t)commandIndex;

    result.repeatIndex =
        (uint8_t)repeatIndex;


    /*
     * Rear motor PWM outputs remain enabled so that the encoder
     * timers stay active, but zero PWM leaves the motors neutral.
     */
    DCMotor_Neutral(
        &fixture->leftRearWheel);

    DCMotor_Neutral(
        &fixture->rightRearWheel);


    SteeringCal_ShowMeasuring(
        command,
        repeatIndex);


    /*
     * User has just released SW1. Give them time to remove
     * their hand before defining the measurement origin.
     */
    HAL_Delay(
        CAL_HANDS_OFF_DELAY_MS);


    /*
     * Capture starting encoder positions only after the
     * hands-off delay.
     */
    int16_t previousLeft =
        DCMotor_GetEncoderCount(
            &fixture->leftRearWheel);

    int16_t previousRight =
        DCMotor_GetEncoderCount(
            &fixture->rightRearWheel);


    const float mmPerCount =
        (PI_F *
         kinematics.rearWheelDiameterMm) /
        (float)kinematics.rearEncoderCountsPerRev;


    float leftTravelMm = 0.0f;
    float rightTravelMm = 0.0f;
    float yawDeg = 0.0f;


    ICM20948Measurement measurement = {0};

    /*
     * Prime the sensor read before timing begins.
     */
    if (!ICM20948_ReadMeasurement(
            &fixture->imu,
            &measurement))
    {
        result.imuReadFailed = 1U;
        result.valid = 0U;

        *destination = result;
        return;
    }


    uint32_t startTick =
        HAL_GetTick();

    uint32_t previousSampleTick =
        startTick;

    result.startTickMs =
        startTick;


    bool finished = false;

    while (!finished)
    {
        /*
         * SW1_ReadState() deliberately remains raw/non-blocking
         * because acquisition must continue while checking abort.
         */
        if (SW1_ReadState() ==
            SW1_Enabled)
        {
            result.aborted = 1U;

            SW1_WaitForRelease();

            break;
        }


        HAL_Delay(
            CAL_SAMPLE_PERIOD_MS);


        uint32_t now =
            HAL_GetTick();


        float dt =
            (float)(
                now -
                previousSampleTick) *
            0.001f;

        previousSampleTick =
            now;


        if (!ICM20948_ReadMeasurement(
                &fixture->imu,
                &measurement))
        {
            result.imuReadFailed = 1U;
            break;
        }


        /*
         * Gyro-Z is already bias corrected by the ICM20948
         * driver.
         */
        yawDeg +=
            measurement.gyroDps.z *
            dt;


        /*
         * Wrap-safe encoder subtraction, consistent with the
         * controller implementation.
         */
        int16_t currentLeft =
            DCMotor_GetEncoderCount(
                &fixture->leftRearWheel);

        int16_t currentRight =
            DCMotor_GetEncoderCount(
                &fixture->rightRearWheel);


        int16_t deltaLeft =
            (int16_t)(
                (uint16_t)currentLeft -
                (uint16_t)previousLeft);

        int16_t deltaRight =
            (int16_t)(
                (uint16_t)currentRight -
                (uint16_t)previousRight);


        previousLeft =
            currentLeft;

        previousRight =
            currentRight;


        leftTravelMm +=
            (float)deltaLeft *
            mmPerCount;

        rightTravelMm +=
            (float)deltaRight *
            mmPerCount;


        float centreTravelMm =
            0.5f *
            (leftTravelMm +
             rightTravelMm);


        result.sampleCount++;


        if (fabsf(centreTravelMm) >=
            CAL_TARGET_TRAVEL_MM)
        {
            finished = true;
        }


        if ((fabsf(centreTravelMm) >=
             CAL_MIN_TRAVEL_FOR_YAW_STOP_MM) &&
            (fabsf(yawDeg) >=
             CAL_TARGET_YAW_DEG))
        {
            finished = true;
        }


        if ((now - startTick) >=
            CAL_RUN_TIMEOUT_MS)
        {
            result.timedOut = 1U;
            finished = true;
        }
    }


    uint32_t finishTick =
        HAL_GetTick();


    result.durationMs =
        finishTick -
        startTick;


    result.leftTravelMm =
        leftTravelMm;

    result.rightTravelMm =
        rightTravelMm;

    result.centreTravelMm =
        0.5f *
        (leftTravelMm +
         rightTravelMm);


    result.distanceDifferenceMm =
        rightTravelMm -
        leftTravelMm;


    result.yawGyroDeg =
        yawDeg;


    /*
     * Rear-wheel kinematics:
     *
     *     dR - dL = W * deltaPsi
     */
    float yawEncoderRad =
        result.distanceDifferenceMm /
        kinematics.rearTrackWidthMm;


    result.yawEncoderDeg =
        yawEncoderRad *
        RAD_TO_DEG_F;


    if (fabsf(result.centreTravelMm) >=
        CAL_MIN_VALID_TRAVEL_MM)
    {
        float yawGyroRad =
            result.yawGyroDeg *
            DEG_TO_RAD_F;


        /*
         *     kappa = deltaPsi / s
         */
        result.curvatureGyroPerMm =
            yawGyroRad /
            result.centreTravelMm;

        result.curvatureEncoderPerMm =
            yawEncoderRad /
            result.centreTravelMm;


        /*
         * Bicycle model:
         *
         *     kappa = tan(delta) / L
         *
         * therefore:
         *
         *     delta = atan(L * kappa)
         */
        result.effectiveAngleGyroRad =
            atanf(
                kinematics.wheelbaseMm *
                result.curvatureGyroPerMm);

        result.effectiveAngleEncoderRad =
            atanf(
                kinematics.wheelbaseMm *
                result.curvatureEncoderPerMm);


        if (!result.aborted &&
            !result.imuReadFailed)
        {
            result.valid = 1U;
        }
    }


    *destination =
        result;
}


/* ------------------------------------------------------------
 * Focused repeated calibration
 * ------------------------------------------------------------ */

static void SteeringCal_RunFocusedTrials(
    RobotTestFixture *fixture)
{
    for (uint32_t commandIndex = 0U;
         commandIndex < CAL_COMMAND_COUNT;
         commandIndex++)
    {
        const float command =
            calCommands[commandIndex];


        for (uint32_t repeatIndex = 0U;
             repeatIndex < CAL_REPEAT_COUNT;
             repeatIndex++)
        {
            uint32_t resultIndex =
                g_steeringGeometryCalibrationResultCount;

            if (resultIndex >=
                STEERING_GEOMETRY_CAL_RESULT_COUNT)
            {
                return;
            }


            /*
             * Recreate the same mechanical history before EVERY
             * measurement instead of carrying history from the
             * previous calibration point.
             */
            SteeringCal_ShowMessage(
                "CENTERING");

            SteeringCal_DeterministicCentre(
                fixture);


            SteeringCal_ShowMessage(
                "PREPOSITION");

            SteeringCal_PrepositionRawCommand(
                fixture,
                command);


            SteeringCal_ShowReady(
                command,
                commandIndex,
                repeatIndex);


            SW1_WaitForPressAndRelease();


            SteeringCal_Measure(
                fixture,
                command,
                commandIndex,
                repeatIndex,
                &g_steeringGeometryCalibrationResults[
                    resultIndex]);


            g_steeringGeometryCalibrationResultCount++;


            SteeringGeometryCalibrationResult displayResult =
                g_steeringGeometryCalibrationResults[
                    resultIndex];


            SteeringCal_ShowResult(
                &displayResult);


            /*
             * User repositions the robot before the next trial.
             * The next trial will then deterministically centre
             * the steering again before applying its command.
             */
            SW1_WaitForPressAndRelease();
        }
    }
}


/* ------------------------------------------------------------
 * Public entry point
 * ------------------------------------------------------------ */

void SteeringGeometryCalibrationTestRun(void)
{
    RobotTestFixture fixture = {0};


    g_steeringGeometryCalibrationResultCount =
        0U;


    for (uint32_t i = 0U;
         i < STEERING_GEOMETRY_CAL_RESULT_COUNT;
         i++)
    {
        g_steeringGeometryCalibrationResults[i] =
            (SteeringGeometryCalibrationResult){0};
    }


    /*
     * ICM20948_Init() estimates gyro bias during
     * initialization. Robot must therefore remain stationary.
     */
    SteeringCal_ShowMessage(
        "KEEP ROBOT STILL");

    HAL_Delay(
        1000U);


    if (!RobotTestFixture_InitIMU(
            &fixture))
    {
        SteeringCal_ShowMessage(
            "IMU INIT FAIL");

        return;
    }


    /*
     * Initialize/start rear encoder timers.
     */
    if (!RobotTestFixture_InitRearWheels(
            &fixture))
    {
        SteeringCal_ShowMessage(
            "REAR INIT FAIL");

        return;
    }


    DCMotor_Neutral(
        &fixture.leftRearWheel);

    DCMotor_Neutral(
        &fixture.rightRearWheel);


    if (!RobotTestFixture_InitFrontWheels(
            &fixture))
    {
        SteeringCal_ShowMessage(
            "SERVO INIT FAIL");

        DCMotor_Disable(
            &fixture.leftRearWheel);

        DCMotor_Disable(
            &fixture.rightRearWheel);

        return;
    }


    /*
     * Unlike the old geometry sweep, this focused experiment
     * initializes SteeringController so its deterministic centering
     * sequence is exactly the one used by MotionControllerArcTest.
     */
    if (!RobotTestFixture_InitSteeringController(
            &fixture))
    {
        SteeringCal_ShowMessage(
            "STEER CTRL FAIL");

        Servo_Disable(
            &fixture.steeringServo);

        DCMotor_Disable(
            &fixture.leftRearWheel);

        DCMotor_Disable(
            &fixture.rightRearWheel);

        return;
    }


    SteeringCal_RunFocusedTrials(
        &fixture);


    /*
     * Safe shutdown.
     */
    SteeringController_Centre(
        &fixture.steeringController);

    HAL_Delay(
        300U);


    Servo_Disable(
        &fixture.steeringServo);


    DCMotor_Neutral(
        &fixture.leftRearWheel);

    DCMotor_Neutral(
        &fixture.rightRearWheel);


    DCMotor_Disable(
        &fixture.leftRearWheel);

    DCMotor_Disable(
        &fixture.rightRearWheel);


    OLED_Clear();

    OLED_Printf(
        0,
        0,
        "STEER CAL DONE");

    OLED_Printf(
        0,
        2,
        "RESULTS %lu",
        (unsigned long)
            g_steeringGeometryCalibrationResultCount);

    OLED_Printf(
        0,
        4,
        "EXPORT WITH GDB");

    OLED_Refresh_Gram();
}
