/*
 * SteeringGeometryCalibrationTest.c
 *
 * Full-range self-propelled steering-geometry calibration.
 *
 * --------------------------------------------------------------------------
 * WHY THIS TEST EXISTS
 * --------------------------------------------------------------------------
 *
 * The earlier steering-geometry calibration used a hand-pushed robot.
 * Testing showed that the externally applied force could materially alter
 * final yaw, while a self-propelled repeated-arc experiment gave highly
 * repeatable curvature.
 *
 * This redesign therefore calibrates the physical steering geometry from
 * autonomous trajectory motion.
 *
 *
 * --------------------------------------------------------------------------
 * WHAT IS BEING MEASURED
 * --------------------------------------------------------------------------
 *
 * Independent variable:
 *
 *     raw steering command u
 *
 * Primary measured quantity:
 *
 *     kappa_gyro = deltaPsi_gyro / s
 *
 * Candidate SteeringController calibration value:
 *
 *     delta_eff = atan(L * kappa_gyro)
 *
 *
 * --------------------------------------------------------------------------
 * IMPORTANT: OLD STEERING TABLE IS NOT USED TO COMMAND THE TEST ANGLE
 * --------------------------------------------------------------------------
 *
 * SteeringController_SetCommand() is used directly.
 *
 * The existing effective-angle -> raw-command inverse calibration is NOT
 * used to position the steering during a measurement.
 *
 * RobotTestFixture_InitMotionController() is still used as a convenient,
 * already-tested initializer for the IMU, motors, wheel-speed controllers
 * and SteeringController object.  The MotionController motion path itself
 * is NOT used by this calibration.
 *
 *
 * --------------------------------------------------------------------------
 * TWO-PHASE METHOD
 * --------------------------------------------------------------------------
 *
 * Each of the 13 raw commands (-12, -10, ..., +12) is measured on both:
 *
 *     increasing-command branch
 *     decreasing-command branch
 *
 * Phase 1: SCOUT
 * ----------------
 *
 * Rear-wheel desired differential is zero.
 *
 * This deliberately avoids using any previous steering geometry.
 * The resulting gyro curvature is a first physical estimate.
 *
 *
 * Phase 2: REFINED
 * ----------------
 *
 * The corresponding SCOUT gyro curvature is used ONLY to generate the
 * rear-wheel relationship:
 *
 *     vL = v * (1 - W*kappa/2)
 *     vR = v * (1 + W*kappa/2)
 *
 * and:
 *
 *     (sR - sL)_desired = W * kappa * s
 *
 * The steering remains the same directly commanded raw value.
 *
 * Therefore the refined measurement is independent of the OLD steering
 * calibration table while reducing tyre scrub caused by equal rear-wheel
 * travel during a real turn.
 *
 *
 * --------------------------------------------------------------------------
 * BRANCH / BACKLASH HANDLING
 * --------------------------------------------------------------------------
 *
 * The current SteeringController supports separate increasing and
 * decreasing calibration tables.  This test therefore preserves the same
 * distinction.
 *
 * At the start of each sweep, steering is preconditioned from the opposite
 * extreme.  Points are then approached monotonically in the intended raw
 * command direction.
 *
 * The robot should be LIFTED and repositioned between runs rather than
 * pushed/rolled back, so manual repositioning does not preload the tyres
 * or steering linkage.
 *
 *
 * Created on: 2026年9月2日
 * Redesigned on: 2026年9月9日
 * Author: Joe
 */

#include "SteeringGeometryCalibrationTest_selfpropelled.h"

#include "RobotTestFixture.h"
#include "RobotKinematics.h"
#include "MotionProfile.h"

#include "userbutton.h"
#include "oled.h"
#include "oledutils.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>


/* -------------------------------------------------------------------------- */
/* Constants                                                                  */
/* -------------------------------------------------------------------------- */

#define CAL_PI_F                          (3.14159265358979323846f)
#define CAL_DEG_TO_RAD_F                  (CAL_PI_F / 180.0f)
#define CAL_RAD_TO_DEG_F                  (180.0f / CAL_PI_F)


#define CAL_CONTROL_PERIOD_MS             (10U)
#define CAL_CONTROL_PERIOD_S              (0.010f)


/*
 * Match the steering command slew limit currently used in normal motion.
 */
#define CAL_COMMAND_RATE_PER_SEC          (60.0f)

#define CAL_COMMAND_SETTLE_MS             (500U)
#define CAL_EXTREME_PRELOAD_HOLD_MS       (500U)

#define CAL_HANDS_OFF_MS                  (400U)


/*
 * Keep the physical motion comparable to the successful arc tests.
 *
 * 1000 mm gives useful yaw signal even around the steering centre.
 */
#define CAL_TARGET_TRAVEL_MM              (1000.0f)
#define CAL_MAX_CENTRE_SPEED_CPS          (2000.0f)

#define CAL_ACCELERATION_MMPS2            (500.0f)
#define CAL_DECELERATION_MMPS2            (250.0f)


/*
 * Local rear-wheel relationship controller.
 *
 * It intentionally mirrors the MotionController synchronisation law, but
 * its desired relationship comes from the calibration's own curvature
 * reference rather than SteeringController_GetEffectiveAngleRad().
 */
#define CAL_SYNC_KP_CPS_PER_MM            (10.0f)
#define CAL_SYNC_MAX_CORRECTION_CPS       (100.0f)


#define CAL_RUN_TIMEOUT_MS                (12000U)
#define CAL_BRAKE_TIMEOUT_MS              (3000U)
#define CAL_STOP_STABLE_SAMPLES           (3U)

#define CAL_MIN_VALID_TRAVEL_MM           (900.0f)


/*
 * Full existing raw-command range.
 *
 * This intentionally matches the current 13-point calibration-table size.
 */
static const float calCommands[
    STEERING_GEOMETRY_CAL_POINT_COUNT] =
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
     12.0f
};


/* -------------------------------------------------------------------------- */
/* Debugger-visible exports                                                   */
/* -------------------------------------------------------------------------- */

volatile SteeringGeometryCalibrationResult
    g_steeringGeometryCalibrationResults[
        STEERING_GEOMETRY_CAL_RESULT_COUNT];

volatile uint32_t
    g_steeringGeometryCalibrationResultCount = 0U;


volatile float
    g_steeringGeometryCalibrationScoutCurvaturePerMm[
        STEERING_GEOMETRY_CAL_SWEEP_COUNT]
        [STEERING_GEOMETRY_CAL_POINT_COUNT];

volatile float
    g_steeringGeometryCalibrationRefinedAngleRad[
        STEERING_GEOMETRY_CAL_SWEEP_COUNT]
        [STEERING_GEOMETRY_CAL_POINT_COUNT];


volatile char g_steeringGeometryCalibrationInfo[] =
    "Full-range self-propelled steering calibration; "
    "raw commands -12..+12 step 2; increasing + decreasing branches; "
    "SCOUT equal-path rear-wheel relationship, then REFINED using "
    "SCOUT gyro curvature; 1000mm @ max 2000CPS; "
    "accel=500, decel=250, Ksync=10; old steering inverse table "
    "not used to command calibration angle";


/* -------------------------------------------------------------------------- */
/* Small helpers                                                              */
/* -------------------------------------------------------------------------- */

static float SteeringCal_Clamp(
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


static float SteeringCal_GetMmPerCount(
    const RobotTestFixture *fixture)
{
    const RobotKinematics *kinematics =
        fixture->motionController.kinematics;

    return
        CAL_PI_F *
        kinematics->rearWheelDiameterMm /
        (float)kinematics->rearEncoderCountsPerRev;
}


static uint8_t SteeringCal_GetSweepSlot(
    int8_t sweepDirection)
{
    return
        (sweepDirection ==
         STEERING_GEOMETRY_CAL_INCREASING)
            ? 0U
            : 1U;
}


static const char *SteeringCal_GetPhaseName(
    SteeringGeometryCalibrationPhase phase)
{
    return
        (phase == STEERING_GEOMETRY_CAL_SCOUT)
            ? "SCOUT"
            : "REFINE";
}


static const char *SteeringCal_GetSweepName(
    int8_t sweepDirection)
{
    return
        (sweepDirection ==
         STEERING_GEOMETRY_CAL_INCREASING)
            ? "INC"
            : "DEC";
}


/* -------------------------------------------------------------------------- */
/* Display                                                                    */
/* -------------------------------------------------------------------------- */

static void SteeringCal_ShowMessage(
    const char *line1,
    const char *line2)
{
    OLED_Clear();

    OLED_Printf(
        0, 0,
        "STEER GEO CAL");

    if (line1 != NULL)
    {
        OLED_Printf(
            0, 2,
            "%s",
            line1);
    }

    if (line2 != NULL)
    {
        OLED_Printf(
            0, 3,
            "%s",
            line2);
    }

    OLED_Refresh_Gram();
}


static void SteeringCal_ShowReady(
    SteeringGeometryCalibrationPhase phase,
    int8_t sweepDirection,
    uint32_t commandIndex,
    uint32_t runOrdinal)
{
    OLED_Clear();

    OLED_Printf(
        0, 0,
        "%s %s %lu/52",
        SteeringCal_GetPhaseName(phase),
        SteeringCal_GetSweepName(sweepDirection),
        (unsigned long)runOrdinal);

    OLED_Printf(
        0, 1,
        "CMD %+.1f",
        calCommands[commandIndex]);

    OLED_Printf(
        0, 2,
        "LIFT + REPOSITION");

    OLED_Printf(
        0, 3,
        "DO NOT PUSH BACK");

    OLED_Printf(
        0, 5,
        "SW1 = RUN");

    OLED_Refresh_Gram();
}


static void SteeringCal_ShowResult(
    const SteeringGeometryCalibrationResult *result)
{
    OLED_Clear();

    OLED_Printf(
        0, 0,
        "%s %s U%+.0f",
        SteeringCal_GetPhaseName(
            (SteeringGeometryCalibrationPhase)
                result->phase),
        SteeringCal_GetSweepName(
            result->sweepDirection),
        result->steeringCommand);

    OLED_Printf(
        0, 1,
        "Y:%+.2f deg",
        result->yawGyroDeg);

    OLED_Printf(
        0, 2,
        "d:%+.4f rad",
        result->effectiveAngleGyroRad);

    OLED_Printf(
        0, 3,
        "S:%6.1f mm",
        result->travelledDistanceMm);

    OLED_Printf(
        0, 4,
        "sync:%+.2f",
        result->wheelSyncErrorMm);

    OLED_Printf(
        0, 5,
        "%s",
        result->valid ?
            "OK - reposition" :
            "INVALID");

    OLED_Refresh_Gram();
}


/* -------------------------------------------------------------------------- */
/* Raw steering positioning                                                   */
/* -------------------------------------------------------------------------- */

static void SteeringCal_SlewRawCommand(
    RobotTestFixture *fixture,
    float targetCommand)
{
    float currentCommand =
        SteeringController_GetCommand(
            &fixture->steeringController);

    const float maxDelta =
        CAL_COMMAND_RATE_PER_SEC *
        CAL_CONTROL_PERIOD_S;


    while (fabsf(
        targetCommand -
        currentCommand) > 0.0001f)
    {
        float delta =
            targetCommand -
            currentCommand;

        delta =
            SteeringCal_Clamp(
                delta,
                -maxDelta,
                maxDelta);

        currentCommand +=
            delta;

        SteeringController_SetCommand(
            &fixture->steeringController,
            currentCommand);

        HAL_Delay(
            CAL_CONTROL_PERIOD_MS);
    }


    SteeringController_SetCommand(
        &fixture->steeringController,
        targetCommand);
}


static void SteeringCal_ApproachAndSettle(
    RobotTestFixture *fixture,
    float targetCommand)
{
    SteeringCal_SlewRawCommand(
        fixture,
        targetCommand);

    HAL_Delay(
        CAL_COMMAND_SETTLE_MS);
}


/*
 * Establish a known approach-history state before beginning a sweep.
 *
 * Increasing sweep:
 *
 *     +12 -> -12 -> -10 -> ... -> +12
 *
 * Decreasing sweep:
 *
 *     -12 -> +12 -> +10 -> ... -> -12
 *
 * The first endpoint itself is mechanically saturated and therefore is not
 * a useful place to infer backlash magnitude, but the subsequent points
 * have the intended monotonic command history.
 */
static void SteeringCal_PreconditionSweep(
    RobotTestFixture *fixture,
    int8_t sweepDirection)
{
    if (sweepDirection ==
        STEERING_GEOMETRY_CAL_INCREASING)
    {
        SteeringCal_SlewRawCommand(
            fixture,
            +12.0f);

        HAL_Delay(
            CAL_EXTREME_PRELOAD_HOLD_MS);

        SteeringCal_SlewRawCommand(
            fixture,
            -12.0f);

        HAL_Delay(
            CAL_EXTREME_PRELOAD_HOLD_MS);
    }
    else
    {
        SteeringCal_SlewRawCommand(
            fixture,
            -12.0f);

        HAL_Delay(
            CAL_EXTREME_PRELOAD_HOLD_MS);

        SteeringCal_SlewRawCommand(
            fixture,
            +12.0f);

        HAL_Delay(
            CAL_EXTREME_PRELOAD_HOLD_MS);
    }
}


/* -------------------------------------------------------------------------- */
/* Odometry + IMU acquisition                                                 */
/* -------------------------------------------------------------------------- */

typedef struct
{
    int16_t previousLeftEncoder;
    int16_t previousRightEncoder;

    float leftTravelMm;
    float rightTravelMm;

    float centreTravelMm;
    float yawDeg;

} SteeringCalMeasurementState;


static void SteeringCal_ResetMeasurementState(
    RobotTestFixture *fixture,
    SteeringCalMeasurementState *state)
{
    *state =
        (SteeringCalMeasurementState){0};

    state->previousLeftEncoder =
        DCMotor_GetEncoderCount(
            fixture->motionController.leftWheel->motor);

    state->previousRightEncoder =
        DCMotor_GetEncoderCount(
            fixture->motionController.rightWheel->motor);
}


static void SteeringCal_UpdateOdometry(
    RobotTestFixture *fixture,
    SteeringCalMeasurementState *state,
    float mmPerCount)
{
    int16_t currentLeft =
        DCMotor_GetEncoderCount(
            fixture->motionController.leftWheel->motor);

    int16_t currentRight =
        DCMotor_GetEncoderCount(
            fixture->motionController.rightWheel->motor);


    int16_t deltaLeft =
        (int16_t)(
            (uint16_t)currentLeft -
            (uint16_t)state->previousLeftEncoder);

    int16_t deltaRight =
        (int16_t)(
            (uint16_t)currentRight -
            (uint16_t)state->previousRightEncoder);


    state->previousLeftEncoder =
        currentLeft;

    state->previousRightEncoder =
        currentRight;


    state->leftTravelMm +=
        (float)deltaLeft *
        mmPerCount;

    state->rightTravelMm +=
        (float)deltaRight *
        mmPerCount;


    state->centreTravelMm =
        0.5f *
        (state->leftTravelMm +
         state->rightTravelMm);
}


static bool SteeringCal_UpdateYaw(
    RobotTestFixture *fixture,
    SteeringCalMeasurementState *state,
    float dt)
{
    ICM20948Measurement measurement = {0};

    if (!ICM20948_ReadMeasurement(
            fixture->motionController.imu,
            &measurement))
    {
        return false;
    }


    state->yawDeg +=
        measurement.gyroDps.z *
        dt;

    return true;
}


/* -------------------------------------------------------------------------- */
/* Local rear-wheel relationship controller                                   */
/* -------------------------------------------------------------------------- */

static void SteeringCal_SetRearWheelTargets(
    RobotTestFixture *fixture,
    float centreSpeedCps,
    float referenceCurvaturePerMm,
    float desiredWheelTravelDifferenceMm,
    float actualWheelTravelDifferenceMm,
    float *wheelSyncErrorMm,
    float *wheelSyncCorrectionCps)
{
    const RobotKinematics *kinematics =
        fixture->motionController.kinematics;

    WheelSpeedController *leftWheel =
        fixture->motionController.leftWheel;

    WheelSpeedController *rightWheel =
        fixture->motionController.rightWheel;


    float leftBaseTargetCps;
    float rightBaseTargetCps;


    RobotKinematics_GetRearWheelSpeedTargets(
        kinematics,
        centreSpeedCps,
        referenceCurvaturePerMm,
        &leftBaseTargetCps,
        &rightBaseTargetCps);


    float errorMm =
        actualWheelTravelDifferenceMm -
        desiredWheelTravelDifferenceMm;


    float correctionCps =
        CAL_SYNC_KP_CPS_PER_MM *
        errorMm;


    float smallestBaseMagnitudeCps =
        fminf(
            fabsf(leftBaseTargetCps),
            fabsf(rightBaseTargetCps));


    float correctionLimitCps =
        fminf(
            CAL_SYNC_MAX_CORRECTION_CPS,
            smallestBaseMagnitudeCps);


    correctionCps =
        SteeringCal_Clamp(
            correctionCps,
            -correctionLimitCps,
            correctionLimitCps);


    WheelSpeedController_SetTarget(
        leftWheel,
        leftBaseTargetCps +
        correctionCps);

    WheelSpeedController_SetTarget(
        rightWheel,
        rightBaseTargetCps -
        correctionCps);


    *wheelSyncErrorMm =
        errorMm;

    *wheelSyncCorrectionCps =
        correctionCps;
}


/* -------------------------------------------------------------------------- */
/* Result computation                                                         */
/* -------------------------------------------------------------------------- */

static void SteeringCal_ComputeGeometry(
    RobotTestFixture *fixture,
    SteeringGeometryCalibrationResult *result)
{
    const RobotKinematics *kinematics =
        fixture->motionController.kinematics;


    result->wheelTravelDifferenceMm =
        result->rightTravelMm -
        result->leftTravelMm;


    if (fabsf(result->travelledDistanceMm) <
        CAL_MIN_VALID_TRAVEL_MM)
    {
        return;
    }


    float yawGyroRad =
        result->yawGyroDeg *
        CAL_DEG_TO_RAD_F;


    result->curvatureGyroPerMm =
        yawGyroRad /
        result->travelledDistanceMm;


    result->effectiveAngleGyroRad =
        atanf(
            kinematics->wheelbaseMm *
            result->curvatureGyroPerMm);


    float yawEncoderRad =
        result->wheelTravelDifferenceMm /
        kinematics->rearTrackWidthMm;


    result->yawEncoderDeg =
        yawEncoderRad *
        CAL_RAD_TO_DEG_F;


    result->curvatureEncoderPerMm =
        yawEncoderRad /
        result->travelledDistanceMm;


    result->effectiveAngleEncoderRad =
        atanf(
            kinematics->wheelbaseMm *
            result->curvatureEncoderPerMm);
}


/* -------------------------------------------------------------------------- */
/* One autonomous calibration run                                             */
/* -------------------------------------------------------------------------- */

static void SteeringCal_RunOne(
    RobotTestFixture *fixture,
    SteeringGeometryCalibrationPhase phase,
    int8_t sweepDirection,
    uint32_t commandIndex,
    float propulsionCurvaturePerMm,
    volatile SteeringGeometryCalibrationResult *destination)
{
    SteeringGeometryCalibrationResult result = {0};


    result.steeringCommand =
        calCommands[commandIndex];

    result.sweepDirection =
        sweepDirection;

    result.phase =
        (uint8_t)phase;

    result.commandIndex =
        (uint8_t)commandIndex;

    result.propulsionCurvaturePerMm =
        propulsionCurvaturePerMm;


    /*
     * Approach the point in the existing monotonic sweep direction.
     */
    SteeringCal_ApproachAndSettle(
        fixture,
        result.steeringCommand);


    /*
     * Resynchronise encoder references after manual repositioning.
     */
    WheelSpeedController_Stop(
        fixture->motionController.leftWheel);

    WheelSpeedController_Stop(
        fixture->motionController.rightWheel);


    /*
     * Keep the robot still briefly after the user's start press.
     */
    HAL_Delay(
        CAL_HANDS_OFF_MS);


    SteeringCalMeasurementState measurement = {0};

    SteeringCal_ResetMeasurementState(
        fixture,
        &measurement);


    /*
     * Prime IMU read before defining t = 0.
     */
    ICM20948Measurement prime = {0};

    if (!ICM20948_ReadMeasurement(
            fixture->motionController.imu,
            &prime))
    {
        result.imuReadFailed = 1U;
        *destination = result;
        return;
    }


    MotionProfile profile = {0};


    if (!MotionProfile_Init(
            &profile,
            CAL_ACCELERATION_MMPS2,
            CAL_DECELERATION_MMPS2))
    {
        result.profileFailed = 1U;
        *destination = result;
        return;
    }


    const float mmPerCount =
        SteeringCal_GetMmPerCount(
            fixture);


    const float maxSpeedMmps =
        CAL_MAX_CENTRE_SPEED_CPS *
        mmPerCount;


    if (!MotionProfile_Start(
            &profile,
            CAL_TARGET_TRAVEL_MM,
            maxSpeedMmps))
    {
        result.profileFailed = 1U;
        *destination = result;
        return;
    }


    uint32_t startTick =
        HAL_GetTick();

    uint32_t lastControlTick =
        startTick;


    result.startTickMs =
        startTick;


    float desiredWheelTravelDifferenceMm =
        0.0f;

    float wheelSyncErrorMm =
        0.0f;

    float wheelSyncCorrectionCps =
        0.0f;

    float maxAbsWheelSyncErrorMm =
        0.0f;


    bool measurementCaptured =
        false;

    bool braking =
        false;

    uint32_t stationarySamples =
        0U;

    uint32_t brakeStartTick =
        0U;


    while (true)
    {
        uint32_t now =
            HAL_GetTick();


        if (!braking &&
            ((now - startTick) >=
             CAL_RUN_TIMEOUT_MS))
        {
            result.timedOut = 1U;
            braking = true;
            brakeStartTick = now;

            WheelSpeedController_SetTarget(
                fixture->motionController.leftWheel,
                0.0f);

            WheelSpeedController_SetTarget(
                fixture->motionController.rightWheel,
                0.0f);
        }


        if (braking &&
            ((now - brakeStartTick) >=
             CAL_BRAKE_TIMEOUT_MS))
        {
            break;
        }


        /*
         * Optional emergency abort during an autonomous run.
         */
        if (!braking &&
            SW1_ReadState() ==
                SW1_Enabled)
        {
            result.aborted = 1U;
            braking = true;
            brakeStartTick = now;

            WheelSpeedController_SetTarget(
                fixture->motionController.leftWheel,
                0.0f);

            WheelSpeedController_SetTarget(
                fixture->motionController.rightWheel,
                0.0f);
        }


        if ((now - lastControlTick) <
            CAL_CONTROL_PERIOD_MS)
        {
            continue;
        }


        lastControlTick +=
            CAL_CONTROL_PERIOD_MS;


        /*
         * Record physical motion first.
         */
        float previousCentreTravelMm =
            measurement.centreTravelMm;


        SteeringCal_UpdateOdometry(
            fixture,
            &measurement,
            mmPerCount);


        float deltaCentreMm =
            measurement.centreTravelMm -
            previousCentreTravelMm;


        if (!SteeringCal_UpdateYaw(
                fixture,
                &measurement,
                CAL_CONTROL_PERIOD_S))
        {
            result.imuReadFailed = 1U;

            if (!braking)
            {
                braking = true;
                brakeStartTick = now;

                WheelSpeedController_SetTarget(
                    fixture->motionController.leftWheel,
                    0.0f);

                WheelSpeedController_SetTarget(
                    fixture->motionController.rightWheel,
                    0.0f);
            }
        }


        result.sampleCount++;


        /*
         * Accumulate the rear-wheel relationship required by the
         * curvature reference used for PROPULSION.
         *
         * This is zero throughout the scout phase.
         */
        desiredWheelTravelDifferenceMm +=
            fixture->motionController.kinematics->
                rearTrackWidthMm *
            propulsionCurvaturePerMm *
            deltaCentreMm;


        float actualWheelTravelDifferenceMm =
            measurement.rightTravelMm -
            measurement.leftTravelMm;


        if (!braking)
        {
            float targetSpeedMmps =
                MotionProfile_Update(
                    &profile,
                    fabsf(
                        measurement.centreTravelMm),
                    CAL_CONTROL_PERIOD_S);


            if (!MotionProfile_IsActive(
                    &profile))
            {
                /*
                 * Capture the calibration point BEFORE the braking tail.
                 */
                result.travelledDistanceMm =
                    measurement.centreTravelMm;

                result.leftTravelMm =
                    measurement.leftTravelMm;

                result.rightTravelMm =
                    measurement.rightTravelMm;

                result.yawGyroDeg =
                    measurement.yawDeg;

                result.desiredWheelTravelDifferenceMm =
                    desiredWheelTravelDifferenceMm;

                result.wheelSyncErrorMm =
                    actualWheelTravelDifferenceMm -
                    desiredWheelTravelDifferenceMm;

                result.maxAbsWheelSyncErrorMm =
                    maxAbsWheelSyncErrorMm;


                SteeringCal_ComputeGeometry(
                    fixture,
                    &result);


                measurementCaptured =
                    true;


                braking =
                    true;

                brakeStartTick =
                    now;


                WheelSpeedController_SetTarget(
                    fixture->motionController.leftWheel,
                    0.0f);

                WheelSpeedController_SetTarget(
                    fixture->motionController.rightWheel,
                    0.0f);
            }
            else
            {
                float centreSpeedCps =
                    targetSpeedMmps /
                    mmPerCount;


                SteeringCal_SetRearWheelTargets(
                    fixture,
                    centreSpeedCps,
                    propulsionCurvaturePerMm,
                    desiredWheelTravelDifferenceMm,
                    actualWheelTravelDifferenceMm,
                    &wheelSyncErrorMm,
                    &wheelSyncCorrectionCps);


                float absSyncErrorMm =
                    fabsf(
                        wheelSyncErrorMm);

                if (absSyncErrorMm >
                    maxAbsWheelSyncErrorMm)
                {
                    maxAbsWheelSyncErrorMm =
                        absSyncErrorMm;
                }
            }
        }


        /*
         * Keep both low-level wheel controllers active during the stopping
         * tail so the existing dynamic braking behaviour is retained.
         *
         * Steering is deliberately NOT centred here.
         */
        WheelSpeedController_Update(
            fixture->motionController.leftWheel,
            CAL_CONTROL_PERIOD_S);

        WheelSpeedController_Update(
            fixture->motionController.rightWheel,
            CAL_CONTROL_PERIOD_S);


        if (braking)
        {
            bool leftStationary =
                WheelSpeedController_IsStationary(
                    fixture->motionController.leftWheel);

            bool rightStationary =
                WheelSpeedController_IsStationary(
                    fixture->motionController.rightWheel);


            if (leftStationary &&
                rightStationary)
            {
                stationarySamples++;

                if (stationarySamples >=
                    CAL_STOP_STABLE_SAMPLES)
                {
                    break;
                }
            }
            else
            {
                stationarySamples = 0U;
            }
        }
    }


    /*
     * Final safe coast and encoder-reference resynchronisation.
     */
    WheelSpeedController_Stop(
        fixture->motionController.leftWheel);

    WheelSpeedController_Stop(
        fixture->motionController.rightWheel);


    result.durationMs =
        HAL_GetTick() -
        startTick;


    result.finalDistanceMm =
        measurement.centreTravelMm;

    result.finalYawDeg =
        measurement.yawDeg;


    /*
     * If an error occurred before MotionProfile naturally completed,
     * retain a diagnostic geometry estimate but do not mark it valid.
     */
    if (!measurementCaptured)
    {
        result.travelledDistanceMm =
            measurement.centreTravelMm;

        result.leftTravelMm =
            measurement.leftTravelMm;

        result.rightTravelMm =
            measurement.rightTravelMm;

        result.yawGyroDeg =
            measurement.yawDeg;

        result.desiredWheelTravelDifferenceMm =
            desiredWheelTravelDifferenceMm;

        result.wheelSyncErrorMm =
            (measurement.rightTravelMm -
             measurement.leftTravelMm) -
            desiredWheelTravelDifferenceMm;

        result.maxAbsWheelSyncErrorMm =
            maxAbsWheelSyncErrorMm;


        SteeringCal_ComputeGeometry(
            fixture,
            &result);
    }


    if (measurementCaptured &&
        !result.timedOut &&
        !result.aborted &&
        !result.imuReadFailed &&
        !result.profileFailed &&
        fabsf(
            result.travelledDistanceMm) >=
            CAL_MIN_VALID_TRAVEL_MM)
    {
        result.valid = 1U;
    }


    *destination =
        result;
}


/* -------------------------------------------------------------------------- */
/* Sweep execution                                                            */
/* -------------------------------------------------------------------------- */

static bool SteeringCal_RunSweep(
    RobotTestFixture *fixture,
    SteeringGeometryCalibrationPhase phase,
    int8_t sweepDirection)
{
    uint8_t sweepSlot =
        SteeringCal_GetSweepSlot(
            sweepDirection);


    SteeringCal_ShowMessage(
        SteeringCal_GetPhaseName(phase),
        sweepDirection ==
            STEERING_GEOMETRY_CAL_INCREASING
                ? "PRELOAD INC"
                : "PRELOAD DEC");


    SteeringCal_PreconditionSweep(
        fixture,
        sweepDirection);


    for (uint32_t ordinal = 0U;
         ordinal <
            STEERING_GEOMETRY_CAL_POINT_COUNT;
         ordinal++)
    {
        uint32_t commandIndex;


        if (sweepDirection ==
            STEERING_GEOMETRY_CAL_INCREASING)
        {
            commandIndex =
                ordinal;
        }
        else
        {
            commandIndex =
                (STEERING_GEOMETRY_CAL_POINT_COUNT -
                 1U) -
                ordinal;
        }


        uint32_t runOrdinal =
            g_steeringGeometryCalibrationResultCount +
            1U;


        SteeringCal_ShowReady(
            phase,
            sweepDirection,
            commandIndex,
            runOrdinal);


        SW1_WaitForPressAndRelease();


        /*
         * The operator should lift and reposition the robot before
         * pressing SW1.  No hand force is applied during measurement.
         */
        float propulsionCurvaturePerMm =
            0.0f;


        if (phase ==
            STEERING_GEOMETRY_CAL_REFINED)
        {
            propulsionCurvaturePerMm =
                g_steeringGeometryCalibrationScoutCurvaturePerMm[
                    sweepSlot]
                    [commandIndex];
        }


        uint32_t resultIndex =
            g_steeringGeometryCalibrationResultCount;


        if (resultIndex >=
            STEERING_GEOMETRY_CAL_RESULT_COUNT)
        {
            return false;
        }


        SteeringCal_RunOne(
            fixture,
            phase,
            sweepDirection,
            commandIndex,
            propulsionCurvaturePerMm,
            &g_steeringGeometryCalibrationResults[
                resultIndex]);


        SteeringGeometryCalibrationResult displayResult =
            g_steeringGeometryCalibrationResults[
                resultIndex];


        g_steeringGeometryCalibrationResultCount++;


        if (displayResult.valid)
        {
            if (phase ==
                STEERING_GEOMETRY_CAL_SCOUT)
            {
                g_steeringGeometryCalibrationScoutCurvaturePerMm[
                    sweepSlot]
                    [commandIndex] =
                        displayResult.curvatureGyroPerMm;
            }
            else
            {
                g_steeringGeometryCalibrationRefinedAngleRad[
                    sweepSlot]
                    [commandIndex] =
                        displayResult.effectiveAngleGyroRad;
            }
        }
        else
        {
            /*
             * A refined sweep must not continue through a point whose
             * scout curvature is invalid, because its propulsion geometry
             * would no longer be independently established.
             */
            SteeringCal_ShowResult(
                &displayResult);

            HAL_Delay(
                500U);

            return false;
        }


        SteeringCal_ShowResult(
            &displayResult);


        /*
         * Give the user time to read the result.
         * The next Ready screen then asks for lift/reposition + SW1.
         */
        HAL_Delay(
            500U);
    }


    return true;
}


/* -------------------------------------------------------------------------- */
/* Export reset                                                               */
/* -------------------------------------------------------------------------- */

static void SteeringCal_ResetExports(void)
{
    g_steeringGeometryCalibrationResultCount =
        0U;


    for (uint32_t i = 0U;
         i <
            STEERING_GEOMETRY_CAL_RESULT_COUNT;
         i++)
    {
        g_steeringGeometryCalibrationResults[i] =
            (SteeringGeometryCalibrationResult){0};
    }


    for (uint32_t sweep = 0U;
         sweep <
            STEERING_GEOMETRY_CAL_SWEEP_COUNT;
         sweep++)
    {
        for (uint32_t point = 0U;
             point <
                STEERING_GEOMETRY_CAL_POINT_COUNT;
             point++)
        {
            g_steeringGeometryCalibrationScoutCurvaturePerMm[
                sweep][point] = 0.0f;

            g_steeringGeometryCalibrationRefinedAngleRad[
                sweep][point] = 0.0f;
        }
    }
}


/* -------------------------------------------------------------------------- */
/* Initialisation                                                              */
/* -------------------------------------------------------------------------- */

static bool SteeringCal_Init(
    RobotTestFixture *fixture)
{
    OLED_Init();

    OLED_Clear();
    OLED_Refresh_Gram();


    /*
     * Use the already-proven fixture initializer.
     *
     * MotionController itself will NOT execute motion in this test.
     *
     * Heading gains are zero because the calibration is open-loop in
     * steering angle.
     */
    if (!RobotTestFixture_InitMotionController(
            fixture,

            0.0f,      /* heading Kp */
            0.0f,      /* heading Ki */
            0.0f,      /* heading Kd */
            0.010f,    /* valid non-zero heading output limit */

            CAL_SYNC_KP_CPS_PER_MM,
            CAL_SYNC_MAX_CORRECTION_CPS,

            CAL_ACCELERATION_MMPS2,
            CAL_DECELERATION_MMPS2))
    {
        return false;
    }


    WheelSpeedController_Stop(
        fixture->motionController.leftWheel);

    WheelSpeedController_Stop(
        fixture->motionController.rightWheel);


    return true;
}


/* -------------------------------------------------------------------------- */
/* Public test                                                                */
/* -------------------------------------------------------------------------- */

void SteeringGeometryCalibrationTestRun(void)
{
    RobotTestFixture fixture = {0};


    /*
     * Keep experiment-description string reachable for GDB.
     */
    volatile char infoKeepAlive =
        g_steeringGeometryCalibrationInfo[0];

    (void)infoKeepAlive;


    SteeringCal_ResetExports();


    SteeringCal_ShowMessage(
        "KEEP ROBOT STILL",
        "INITIALISING");


    HAL_Delay(
        1000U);


    if (!SteeringCal_Init(
            &fixture))
    {
        SteeringCal_ShowMessage(
            "INIT FAILED",
            NULL);

        SW1_WhileNotPressed();
        return;
    }


    SteeringCal_ShowMessage(
        "52 AUTONOMOUS RUNS",
        "SW1 TO BEGIN");


    SW1_WaitForPressAndRelease();


    /*
     * --------------------------------------------------------
     * PHASE 1: SCOUT
     * --------------------------------------------------------
     */

    if (!SteeringCal_RunSweep(
            &fixture,
            STEERING_GEOMETRY_CAL_SCOUT,
            STEERING_GEOMETRY_CAL_INCREASING))
    {
        goto calibration_failed;
    }


    if (!SteeringCal_RunSweep(
            &fixture,
            STEERING_GEOMETRY_CAL_SCOUT,
            STEERING_GEOMETRY_CAL_DECREASING))
    {
        goto calibration_failed;
    }


    SteeringCal_ShowMessage(
        "SCOUT COMPLETE",
        "SW1 -> REFINED");


    SW1_WaitForPressAndRelease();


    /*
     * --------------------------------------------------------
     * PHASE 2: REFINED
     * --------------------------------------------------------
     */

    if (!SteeringCal_RunSweep(
            &fixture,
            STEERING_GEOMETRY_CAL_REFINED,
            STEERING_GEOMETRY_CAL_INCREASING))
    {
        goto calibration_failed;
    }


    if (!SteeringCal_RunSweep(
            &fixture,
            STEERING_GEOMETRY_CAL_REFINED,
            STEERING_GEOMETRY_CAL_DECREASING))
    {
        goto calibration_failed;
    }


    /*
     * Safe final state.
     */
    WheelSpeedController_Stop(
        fixture.motionController.leftWheel);

    WheelSpeedController_Stop(
        fixture.motionController.rightWheel);


    SteeringController_Centre(
        &fixture.steeringController);


    OLED_Clear();

    OLED_Printf(
        0, 0,
        "STEER CAL DONE");

    OLED_Printf(
        0, 1,
        "RESULTS:%lu",
        (unsigned long)
            g_steeringGeometryCalibrationResultCount);

    OLED_Printf(
        0, 3,
        "EXPORT:");

    OLED_Printf(
        0, 4,
        "Results");

    OLED_Printf(
        0, 5,
        "RefinedAngleRad");

    OLED_Refresh_Gram();


    SW1_WhileNotPressed();

    return;


calibration_failed:

    WheelSpeedController_Stop(
        fixture.motionController.leftWheel);

    WheelSpeedController_Stop(
        fixture.motionController.rightWheel);


    SteeringController_Centre(
        &fixture.steeringController);


    OLED_Clear();

    OLED_Printf(
        0, 0,
        "CAL STOPPED");

    OLED_Printf(
        0, 1,
        "RESULTS:%lu",
        (unsigned long)
            g_steeringGeometryCalibrationResultCount);

    OLED_Printf(
        0, 3,
        "EXPORT PARTIAL");

    OLED_Refresh_Gram();


    SW1_WhileNotPressed();
}
