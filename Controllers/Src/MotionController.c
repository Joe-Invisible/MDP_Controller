/*
 * MotionController.c
 *
 * Created on: 2026年8月30日
 * Author: Joe
 */

#include "MotionController.h"

#include <math.h>
#include <stddef.h>

#define MOTION_PI 3.14159265358979323846f

 /*
  * For experiment purposes only
  */
#define TMP_2000CPS_BRAKING_DISTANCE_MM (22.0f)


static float MotionController_GetMmPerCount(
    const MotionController *controller)
{
    /*
     * Why are we recomputing a constant each time?
     * Does the wheel grow as the robot moves?
     * Relativistic physics?
     */
    return MOTION_PI *
        controller->kinematics->rearWheelDiameterMm /
        (float)controller->kinematics->rearEncoderCountsPerRev;
}

static float MotionController_GetEffectiveSteeringAngleRad(
    const MotionController* controller) {
    return 0.0f;
}


static void MotionController_ResetOdometry(
    MotionController *controller)
{
    controller->leftTravelMm = 0.0f;
    controller->rightTravelMm = 0.0f;
    controller->travelledDistanceMm = 0.0f;

    controller->desiredWheelTravelDifferenceMm = 0.0f;
    controller->wheelSyncErrorMm = 0.0f;

    controller->previousLeftEncoderCount =
        DCMotor_GetEncoderCount(controller->leftWheel->motor);

    controller->previousRightEncoderCount =
        DCMotor_GetEncoderCount(controller->rightWheel->motor);
}


static void MotionController_UpdateOdometry(
    MotionController *controller)
{
    int16_t currentLeft =
        DCMotor_GetEncoderCount(controller->leftWheel->motor);

    int16_t currentRight =
        DCMotor_GetEncoderCount(controller->rightWheel->motor);

    /*
     * Use the same wrap-safe subtraction used by
     * WheelSpeedController.
     */
    int16_t deltaLeft = (int16_t)(
        (uint16_t)currentLeft -
        (uint16_t)controller->previousLeftEncoderCount);

    int16_t deltaRight = (int16_t)(
        (uint16_t)currentRight -
        (uint16_t)controller->previousRightEncoderCount);

    controller->previousLeftEncoderCount = currentLeft;
    controller->previousRightEncoderCount = currentRight;

    float mmPerCount =
        MotionController_GetMmPerCount(controller);

    float deltaLeftMm =
        (float)deltaLeft * mmPerCount;

    float deltaRightMm =
        (float)deltaRight * mmPerCount;

    /*
     * Signed displacement of the rear-axle centre.
     */
    float deltaCentreMm =
        0.5f * (deltaLeftMm + deltaRightMm);

    /*
     * The encoder increments measured here correspond
     * approximately to motion performed under the steering
     * command from the previous controller update.
     */
    float steeringAngleRad =
        MotionController_GetEffectiveSteeringAngleRad(
            controller);

    float curvaturePerMm =
        tanf(steeringAngleRad) /
        controller->kinematics->wheelbaseMm;

    /*
     * Bicycle-model rear-wheel relationship:
     *
     *   dR - dL = W * kappa * ds
     *
     * Accumulate the relationship expected from the
     * commanded path curvature.
     */
    controller->desiredWheelTravelDifferenceMm +=
        controller->kinematics->rearTrackWidthMm *
        curvaturePerMm *
        deltaCentreMm;

    controller->leftTravelMm += deltaLeftMm;
    controller->rightTravelMm += deltaRightMm;

    controller->travelledDistanceMm =
        0.5f *
        (controller->leftTravelMm +
            controller->rightTravelMm);

    /*
     * Compare actual rear-wheel relationship with
     * the relationship required by the commanded path.
     *
     * Positive error:
     *     right traveled farther than required.
     *
     * Negative error:
     *     right traveled less far than required.
     */
    controller->wheelSyncErrorMm =
        (controller->rightTravelMm -
            controller->leftTravelMm)
        - controller->desiredWheelTravelDifferenceMm;
}


static void MotionController_BeginBraking(
    MotionController *controller)
{
    WheelSpeedController_SetTarget(controller->leftWheel, 0.0f);
    WheelSpeedController_SetTarget(controller->rightWheel, 0.0f);

    /*
     * Wheel synchronisation no longer commands speed
     * corrections once braking begins.
     */
    controller->wheelSyncCorrectionCps = 0.0f;
    // We preserve first the wheelSyncErrorMm for diagnostics purpose.

    Servo_Centre(controller->steeringServo);

    controller->steeringCommand = 0.0f;
    controller->stationarySamples = 0U;

    controller->mode = MOTIONCONTROLLER_BRAKING;
}


/**
 * Stops wheel speed controller and allow the robot
 * to coast once it is stationary.
 */
static void MotionController_FinishBraking(
    MotionController *controller)
{
    /*
     * Resynchronise the wheel speed controller encoder references
     * once more at the final stationary position.
     */
    WheelSpeedController_Stop(controller->leftWheel);
    WheelSpeedController_Stop(controller->rightWheel);

    Servo_Centre(controller->steeringServo);

    controller->stationarySamples = 0U;
    controller->mode = MOTIONCONTROLLER_IDLE;
}

bool MotionController_Init(
	MotionController *controller,
	WheelSpeedController *leftWheel,
	WheelSpeedController *rightWheel,
	Servo *steeringServo,
	ICM20948 *imu,
	const RobotKinematics *kinematics,
    float headingKp,
    float headingKi,
    float headingKd,
    float maxSteeringCorrection,
    float wheelSyncKpCpsPerMm,
    float maxWheelSyncCorrectionCps,
	int8_t steeringPolarity)
{
    if (controller == NULL ||
        leftWheel == NULL ||
        rightWheel == NULL ||
        leftWheel->motor == NULL ||
        rightWheel->motor == NULL ||
        steeringServo == NULL ||
        imu == NULL ||
        kinematics == NULL)
    {
        return false;
    }

    if (kinematics->rearEncoderCountsPerRev == 0U ||
        kinematics->rearWheelDiameterMm <= 0.0f ||
        kinematics->wheelbaseMm <= 0.0f ||
        kinematics->rearTrackWidthMm <= 0.0f) {
        return false;
    }

    if (maxSteeringCorrection <= 0.0f ||
        maxSteeringCorrection > 100.0f)
    {
        return false;
    }

    if (steeringPolarity != 1 &&
        steeringPolarity != -1)
    {
        return false;
    }

    if (wheelSyncKpCpsPerMm < 0.0f ||
        maxWheelSyncCorrectionCps < 0.0f)
    {
        return false;
    }

    *controller = (MotionController){0};

    controller->leftWheel = leftWheel;
    controller->rightWheel = rightWheel;
    controller->steeringServo = steeringServo;
    controller->imu = imu;
    controller->kinematics = kinematics;

    controller->steeringPolarity = steeringPolarity;

    controller->wheelSyncKpCpsPerMm =
        wheelSyncKpCpsPerMm;

    controller->maxWheelSyncCorrectionCps =
        maxWheelSyncCorrectionCps;

    controller->mode = MOTIONCONTROLLER_IDLE;

    if (!PIDController_Init(
        &controller->headingPID,
        headingKp,
        headingKi,
        headingKd,
        -maxSteeringCorrection,
            maxSteeringCorrection))
    {
        return false;
    }

    return true;
}

bool MotionController_MoveStraight(
    MotionController *controller,
    float distanceMm,
    float speedCps)
{
    if (controller == NULL)
        return false;

    if (controller->mode != MOTIONCONTROLLER_IDLE)
        return false;

    if (speedCps <= 0.0f)
        return false;

    if (distanceMm == 0.0f)
        return true;

    controller->motionDirection = distanceMm > 0.0f ? 1 : -1;

    controller->targetDistanceMm = fabsf(distanceMm);

    controller->targetSpeedCps =
        (float)controller->motionDirection *
        speedCps;

    /*
     * Straight-line heading is relative to the orientation
     * at the start of this command.
     */
    controller->yawDeg = 0.0f;
    controller->steeringCommand = 0.0f;

    /*
     * The desired wheel relationship starts at zero.
     * It may subsequently become non-zero while the
     * heading controller steers the robot back toward
     * the requested straight path.
     */
    controller->desiredWheelTravelDifferenceMm = 0.0f;
    controller->wheelSyncErrorMm = 0.0f;
    controller->wheelSyncCorrectionCps = 0.0f;

    PIDController_Reset(&controller->headingPID);

    MotionController_ResetOdometry(controller);

    Servo_Centre(controller->steeringServo);

    WheelSpeedController_SetTarget(
        controller->leftWheel,
        controller->targetSpeedCps);

    WheelSpeedController_SetTarget(
        controller->rightWheel,
        controller->targetSpeedCps);

    controller->mode = MOTIONCONTROLLER_STRAIGHT;

    return true;
}

bool MotionController_Brake(MotionController *controller) {
	if (controller == NULL)
	{
        return false;
    }

    if (controller->mode == MOTIONCONTROLLER_BRAKING) {
        return true;		// no-op
    }

    MotionController_BeginBraking(controller);

    return true;
}

static bool MotionController_UpdateYawEstimate(MotionController *controller, float dt) {
    ICM20948Measurement measurement;
    if (!ICM20948_ReadMeasurement(controller->imu, &measurement)) {
        return false;
    }
    /*
     * Relative yaw:
     *
     *     yaw[k+1] = yaw[k] + gyroZ * dt
     */
    controller->yawDeg += measurement.gyroDps.z * dt;

    return true;
}

static float MotionController_Clamp(
    float value,
    float min,
    float max)
{
    if (value > max)
        return max;

    if (value < min)
        return min;

    return value;
}


static void MotionController_UpdateWheelSynchronisation(
    MotionController *controller)
{
    /*
     * --------------------------------------------------------
     * KINEMATIC FEEDFORWARD
     * --------------------------------------------------------
     *
     * Bicycle model:
     *
     *     kappa = tan(delta) / L
     *
     * Rear-wheel path-speed relationship:
     *
     *     vL = v * (1 - W*kappa/2)
     *     vR = v * (1 + W*kappa/2)
     */
    float steeringAngleRad =
        MotionController_GetEffectiveSteeringAngleRad(
            controller);

    float curvaturePerMm =
        tanf(steeringAngleRad) /
        controller->kinematics->wheelbaseMm;

    float halfTrackCurvature =
        0.5f *
        controller->kinematics->rearTrackWidthMm *
        curvaturePerMm;

    float leftBaseTargetCps =
        controller->targetSpeedCps *
        (1.0f - halfTrackCurvature);

    float rightBaseTargetCps =
        controller->targetSpeedCps *
        (1.0f + halfTrackCurvature);

    /*
     * --------------------------------------------------------
     * RELATIONSHIP FEEDBACK
     * --------------------------------------------------------
     *
     * e_sync =
     *     actual(R-L) - desired(R-L)
     */
    controller->wheelSyncErrorMm =
        (controller->rightTravelMm -
            controller->leftTravelMm)
        - controller->desiredWheelTravelDifferenceMm;

    /*
     * P-only outer synchronization controller.
     *
     * Units:
     *     [CPS/mm] * [mm] = [CPS]
     */
    float correctionCps =
        controller->wheelSyncKpCpsPerMm *
        controller->wheelSyncErrorMm;

    /*
     * Prevent the synchronizer from overwhelming the
     * geometric base targets or reversing one wheel.
     *
     * Once steering makes the two base targets unequal,
     * the smaller base-target magnitude is the limiting one.
     */
    float smallestBaseTargetMagnitudeCps =
        fminf(
            fabsf(leftBaseTargetCps),
            fabsf(rightBaseTargetCps));

    float correctionLimitCps =
        fminf(
            controller->maxWheelSyncCorrectionCps,
            smallestBaseTargetMagnitudeCps);

    correctionCps =
        MotionController_Clamp(
            correctionCps,
            -correctionLimitCps,
            correctionLimitCps);

    controller->wheelSyncCorrectionCps =
        correctionCps;

    /*
     * Geometry determines the nominal left/right relationship.
     * The synchronizer only corrects deviations from it.
     */
    WheelSpeedController_SetTarget(
        controller->leftWheel,
        leftBaseTargetCps + correctionCps);

    WheelSpeedController_SetTarget(
        controller->rightWheel,
        rightBaseTargetCps - correctionCps);
}

bool MotionController_Update(
    MotionController *controller,
    float dt)
{
    if (controller == NULL || dt <= 0.0f)
        return false;

    if (controller->mode == MOTIONCONTROLLER_IDLE)
        return true;

    /*
     * Keep odometry running during both normal motion and
     * braking, so endpoint overshoot is captured.
     */
    MotionController_UpdateOdometry(controller);

    /*
     * --------------------------------------------------------
     * STOPPING
     * --------------------------------------------------------
     */
    if (controller->mode == MOTIONCONTROLLER_BRAKING)
    {
        /*
         * Target is already zero. Update() still measures
         * encoder velocity before handling the zero target,
         * which lets us determine when the chassis has
         * actually stopped.
         */
        WheelSpeedController_Update(
            controller->leftWheel,
            dt);

        WheelSpeedController_Update(
            controller->rightWheel,
            dt);

        /**
         * Continue to measure yaw,
         * Could be useful for inspecting yaw
         * caused by asymmetric braking.
         */
        MotionController_UpdateYawEstimate(controller, dt);

        bool leftStationary = WheelSpeedController_IsStationary(
            controller->leftWheel);

        bool rightStationary = WheelSpeedController_IsStationary(
            controller->rightWheel);

        if (leftStationary && rightStationary)
        {
            controller->stationarySamples++;

            if (controller->stationarySamples >=
                MOTIONCONTROLLER_STOP_STABLE_SAMPLES)
            {
                MotionController_FinishBraking(controller);
            }
        }
        else
        {
            controller->stationarySamples = 0U;
        }

        return true;
    }

    /*
     * --------------------------------------------------------
     * STRAIGHT
     * --------------------------------------------------------
     */

    float progressMm =
        (float)controller->motionDirection *
        controller->travelledDistanceMm;

    // future progress check: a mutex routing to
    // different completion conditions depending on state.
    if (progressMm >= controller->targetDistanceMm - TMP_2000CPS_BRAKING_DISTANCE_MM)
    {
        MotionController_BeginBraking(controller);
        return true;
    }

    if (!MotionController_UpdateYawEstimate(controller, dt)) {
        /*
         * Heading feedback has failed.
         * Stop rather than continuing open-loop.
         */
        MotionController_Stop(controller);
        return false;
    }

    /*
     * Desired relative heading for straight travel is 0 deg.
     */
    float headingError = -controller->yawDeg;

    float correction = PIDController_Update(
        &controller->headingPID,
        headingError,
        dt);

    /*
     * When reversing, a given steering angle produces the
     * opposite yaw direction. motionDirection compensates
     * for that sign reversal.
     *
     * steeringPolarity accounts for the physical servo and
     * IMU axis orientation on this chassis.
     */
    controller->steeringCommand =
        correction *
        (float)controller->steeringPolarity *
        (float)controller->motionDirection;

    Servo_SetSteering(
        controller->steeringServo,
        controller->steeringCommand);

    /*
     * --------------------------------------------------------
     * REAR-WHEEL SYNCHRONISATION
     * --------------------------------------------------------
     *
     * Convert accumulated wheel relationship error into
     * corrected left/right speed targets.
     */
    MotionController_UpdateWheelSynchronisation(controller);

    /*
     * Low-level speed loops remain responsible for motor PWM.
     */
    WheelSpeedController_Update(
        controller->leftWheel,
        dt);

    WheelSpeedController_Update(
        controller->rightWheel,
        dt);

    return true;
}

void MotionController_Stop(
    MotionController *controller)
{
    if (controller == NULL)
        return;

    WheelSpeedController_Stop(controller->leftWheel);
    WheelSpeedController_Stop(controller->rightWheel);

    Servo_Centre(controller->steeringServo);

    PIDController_Reset(&controller->headingPID);

    controller->mode = MOTIONCONTROLLER_IDLE;
}


bool MotionController_IsBusy(
    const MotionController *controller)
{
    if (controller == NULL)
        return false;

    return controller->mode != MOTIONCONTROLLER_IDLE;
}
