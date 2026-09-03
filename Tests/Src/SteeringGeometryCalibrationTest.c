/*
 * SteeringGeometryCalibrationTest.c
 *
 * Interactive calibration of:
 *
 *     Servo_SetSteering() command
 *
 * against the effective bicycle-model steering angle.
 *
 * The robot is pushed manually while the rear motors remain
 * neutral. Rear-wheel encoder travel and gyro-Z yaw are
 * recorded simultaneously.
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
 * Allow the steering linkage to settle after changing command.
 */
#define CAL_SERVO_SETTLE_MS         700U


/*
 * After the user presses/release SW1 to start a run, give them
 * time to remove their hand before acquisition begins.
 */
#define CAL_HANDS_OFF_DELAY_MS      500U


/*
 * Stop automatically after either:
 *
 *  - 600 mm rear-axle-centre travel, or
 *  - 20 degrees heading change after at least 100 mm travel.
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
 * Commands concentrated around centre, where the heading
 * controller normally operates and where linkage hysteresis
 * matters most.
 */
static const float calCommands[] =
{
    -12.0f,
    -10.0f,
     -8.0f,
     -6.0f,
     -4.0f,
     -2.0f,
      0.0f,
      2.0f,
      4.0f,
      6.0f,
      8.0f,
     10.0f,
     12.0f,
};


#define CAL_COMMAND_COUNT \
    ((uint32_t)(sizeof(calCommands) / sizeof(calCommands[0])))


/*
 * Approach the first measurement point from outside the
 * measured range so that the two sweeps exercise opposite
 * linkage histories.
 */
#define CAL_ASCENDING_PRECONDITION      (-15.0f)
#define CAL_DESCENDING_PRECONDITION     ( 15.0f)


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

static void SteeringCal_ShowMessage(const char *message)
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
    int8_t sweepDirection,
    uint32_t sweepIndex)
{
    OLED_Clear();

    OLED_Printf(
        0,
        0,
        "STEER GEO CAL");

    OLED_Printf(
        0,
        1,
        "%s %lu/%lu",
        (sweepDirection > 0) ? "UP" : "DOWN",
        (unsigned long)(sweepIndex + 1U),
        (unsigned long)CAL_COMMAND_COUNT);

    OLED_Printf(
        0,
        2,
        "STEER %+.1f",
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
    int8_t sweepDirection)
{
    OLED_Clear();

    OLED_Printf(
        0,
        0,
        "%s U%+.1f",
        (sweepDirection > 0) ? "UP" : "DN",
        command);

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
        "%s U%+.1f",
        (result->sweepDirection > 0) ? "UP" : "DN",
        result->steeringCommand);

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
        "dG %+.2f deg",
        result->effectiveAngleGyroRad *
            RAD_TO_DEG_F);

    OLED_Printf(
        0,
        5,
        "dE %+.2f deg",
        result->effectiveAngleEncoderRad *
            RAD_TO_DEG_F);

    OLED_Refresh_Gram();
}


/* ------------------------------------------------------------
 * Measurement
 * ------------------------------------------------------------ */

static void SteeringCal_Measure(
    RobotTestFixture *fixture,
    float command,
    int8_t sweepDirection,
    volatile SteeringGeometryCalibrationResult *destination)
{
    SteeringGeometryCalibrationResult result = {0};

    result.steeringCommand = command;
    result.sweepDirection = sweepDirection;

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
        sweepDirection);

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
         * SW1_ReadState() deliberately remains the raw,
         * non-blocking read here because acquisition must
         * continue while checking for an abort request.
         */
        if (SW1_ReadState() == SW1_Enabled)
        {
            result.aborted = 1U;

            /*
             * Consume/debounce the release centrally in the
             * user-button driver.
             */
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

        /*
         * Weak steering runs terminate by distance.
         */
        if (fabsf(centreTravelMm) >=
            CAL_TARGET_TRAVEL_MM)
        {
            finished = true;
        }

        /*
         * Strong steering runs can terminate earlier once
         * sufficient angular displacement has accumulated.
         */
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

    /*
     * A useful curvature estimate requires enough travelled
     * distance to avoid dividing by a very small number.
     */
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
 * Sweep
 * ------------------------------------------------------------ */

static void SteeringCal_RunSweep(
    RobotTestFixture *fixture,
    int8_t sweepDirection)
{
    float preconditionCommand =
        (sweepDirection > 0)
            ? CAL_ASCENDING_PRECONDITION
            : CAL_DESCENDING_PRECONDITION;

    /*
     * Establish the linkage approach direction.
     */
    Servo_SetSteering(
        &fixture->steeringServo,
        preconditionCommand);

    OLED_Clear();

    OLED_Printf(
        0,
        0,
        "%s SWEEP",
        (sweepDirection > 0)
            ? "UP"
            : "DOWN");

    OLED_Printf(
        0,
        2,
        "PRECOND %+.0f",
        preconditionCommand);

    OLED_Refresh_Gram();

    HAL_Delay(
        CAL_SERVO_SETTLE_MS);

    for (uint32_t i = 0U;
         i < CAL_COMMAND_COUNT;
         i++)
    {
        uint32_t commandIndex =
            (sweepDirection > 0)
                ? i
                : (CAL_COMMAND_COUNT -
                   1U -
                   i);

        float command =
            calCommands[
                commandIndex];

        /*
         * Move directly from the preceding sweep point.
         *
         * Do not centre between measurements: preserving the
         * approach direction is the reason for running both
         * sweeps.
         */
        Servo_SetSteering(
            &fixture->steeringServo,
            command);

        HAL_Delay(
            CAL_SERVO_SETTLE_MS);

        SteeringCal_ShowReady(
            command,
            sweepDirection,
            i);

        /*
         * Complete debounced action handled by userbutton.
         */
        SW1_WaitForPressAndRelease();

        uint32_t resultIndex =
            g_steeringGeometryCalibrationResultCount;

        if (resultIndex >=
            STEERING_GEOMETRY_CAL_RESULT_COUNT)
        {
            return;
        }

        SteeringCal_Measure(
            fixture,
            command,
            sweepDirection,
            &g_steeringGeometryCalibrationResults[
                resultIndex]);

        g_steeringGeometryCalibrationResultCount++;

        SteeringGeometryCalibrationResult displayResult =
            g_steeringGeometryCalibrationResults[
                resultIndex];

        SteeringCal_ShowResult(
            &displayResult);

        /*
         * Reposition the robot before proceeding.
         */
        SW1_WaitForPressAndRelease();
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
     * This initializes and starts both rear encoder timers.
     */
    if (!RobotTestFixture_InitRearWheels(
            &fixture))
    {
        SteeringCal_ShowMessage(
            "REAR INIT FAIL");

        return;
    }

    /*
     * Leave the motors unpowered but keep the driver/encoder
     * peripherals enabled.
     */
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

    Servo_Centre(&fixture.steeringServo);
    HAL_Delay(CAL_SERVO_SETTLE_MS);

    /*
     * Ascending:
     *
     *     -40 precondition
     *     -30 -> ... -> +30
     *
     * Descending:
     *
     *     +40 precondition
     *     +30 -> ... -> -30
     */
    SteeringCal_RunSweep(
        &fixture,
        +1);

    Servo_Centre(&fixture.steeringServo);
    HAL_Delay(CAL_SERVO_SETTLE_MS);

    SteeringCal_RunSweep(
        &fixture,
        -1);

    /*
     * Safe shutdown.
     */
    Servo_Centre(
        &fixture.steeringServo);

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
