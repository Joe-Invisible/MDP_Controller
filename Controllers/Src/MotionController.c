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

#define MOTIONCONTROLLER_TIME_EPSILON_SEC                (1.0e-6f)

static float MotionController_GetMmPerCount(
    const MotionController *controller)
{
    /*
     * Why are we recomputing a constant each time?
     * Does the wheel grow as the robot moves?
     * Relativistic physics?
     */
    return MOTION_PI *
        controller->config->kinematics->rearWheelDiameterMm /
        (float)controller->config->kinematics->rearEncoderCountsPerRev;
}

static float MotionController_GetWheelReferenceCurvaturePerMm(
    const MotionController *controller)
{
    /*
     * ARC:
     * Rear-wheel coordination follows the requested path
     * curvature directly. It must not depend on the estimated
     * steering angle or wheelbase-based steering model.
     *
     * STRAIGHT:
     * Preserve the existing behaviour. Small heading
     * corrections have already been experimentally validated
     * with the existing steering calibration, so rear-wheel
     * coordination continues to accommodate those corrections.
     */
	if (controller->mode == MOTIONCONTROLLER_ARC)
	{
	    return controller->arcCommandedCurvaturePerMm;
	}

    float steeringAngleRad =
        SteeringController_GetEffectiveAngleRad(
            controller->steering);

    return RobotKinematics_GetCurvaturePerMm(
        controller->config->kinematics,
        steeringAngleRad);
}

static bool MotionController_ValidateArcFeedforwardBranch(
    const MotionControllerArcFeedforwardPoint *points,
    uint32_t pointCount,
    bool negativeCurvature)
{
    if (points == NULL || pointCount == 0U)
    {
        return false;
    }

    for (uint32_t i = 0U; i < pointCount; ++i)
    {
        if (!isfinite(points[i].curvaturePerMm) ||
            !isfinite(points[i].rawSteeringCommand) ||
            points[i].rawSteeringCommand < SERVO_STEER_MIN ||
            points[i].rawSteeringCommand > SERVO_STEER_MAX)
        {
            return false;
        }

        if ((negativeCurvature &&
             points[i].curvaturePerMm >= 0.0f) ||
            (!negativeCurvature &&
             points[i].curvaturePerMm <= 0.0f))
        {
            return false;
        }

        if (i > 0U &&
            (points[i].curvaturePerMm <=
                 points[i - 1U].curvaturePerMm ||
             points[i].rawSteeringCommand >=
                 points[i - 1U].rawSteeringCommand))
        {
            return false;
        }
    }

    return true;
}

static bool MotionController_ValidateArcConfig(
    const MotionControllerArcConfig *config)
{
    if (config == NULL ||
        !isfinite(config->steeringSettlingTimeSec) ||
        config->steeringSettlingTimeSec < 0.0f)
    {
        return false;
    }

    return MotionController_ValidateArcFeedforwardBranch(
            config->negativePoints,
            config->negativePointCount,
            true) &&
        MotionController_ValidateArcFeedforwardBranch(
            config->positivePoints,
            config->positivePointCount,
            false);
}

static bool MotionController_InterpolateArcFeedforwardBranch(
    const MotionControllerArcFeedforwardPoint *points,
    uint32_t pointCount,
    float curvaturePerMm,
    float *rawSteeringCommand)
{
    if (points == NULL ||
        pointCount == 0U ||
        rawSteeringCommand == NULL ||
        curvaturePerMm < points[0].curvaturePerMm ||
        curvaturePerMm > points[pointCount - 1U].curvaturePerMm)
    {
        return false;
    }

    if (pointCount == 1U)
    {
        if (curvaturePerMm != points[0].curvaturePerMm)
        {
            return false;
        }

        *rawSteeringCommand = points[0].rawSteeringCommand;
        return true;
    }

    for (uint32_t i = 0U; i + 1U < pointCount; ++i)
    {
        const MotionControllerArcFeedforwardPoint *lower =
            &points[i];
        const MotionControllerArcFeedforwardPoint *upper =
            &points[i + 1U];

        if (curvaturePerMm <= upper->curvaturePerMm)
        {
            float fraction =
                (curvaturePerMm - lower->curvaturePerMm) /
                (upper->curvaturePerMm - lower->curvaturePerMm);

            *rawSteeringCommand =
                lower->rawSteeringCommand +
                fraction *
                (upper->rawSteeringCommand -
                 lower->rawSteeringCommand);

            return true;
        }
    }

    return false;
}

static bool MotionController_GetArcSteeringFeedforwardCommand(
    const MotionController *controller,
    float curvaturePerMm,
    float *rawSteeringCommand)
{
    if (controller == NULL ||
        controller->config == NULL ||
        controller->config->arcConfig == NULL ||
        rawSteeringCommand == NULL ||
        curvaturePerMm == 0.0f)
    {
        return false;
    }

    if (curvaturePerMm < 0.0f)
    {
        return MotionController_InterpolateArcFeedforwardBranch(
            controller->config->arcConfig->negativePoints,
            controller->config->arcConfig->negativePointCount,
            curvaturePerMm,
            rawSteeringCommand);
    }

    return MotionController_InterpolateArcFeedforwardBranch(
        controller->config->arcConfig->positivePoints,
        controller->config->arcConfig->positivePointCount,
        curvaturePerMm,
        rawSteeringCommand);
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
     *
     * SteeringController retains the effective-angle estimate
     * associated with that command.
     */
    float curvaturePerMm =
        MotionController_GetWheelReferenceCurvaturePerMm(
            controller);

    /*
     * Bicycle-model rear-wheel relationship:
     *
     *   dR - dL = W * kappa * ds
     *
     * Accumulate the relationship expected from the
     * commanded path curvature.
     */
    controller->desiredWheelTravelDifferenceMm +=
        controller->config->kinematics->rearTrackWidthMm *
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
     * Abort active profile
     */
    MotionProfile_Stop(&controller->motionProfile);
    controller->targetSpeedCps = 0.0f;

    /*
     * Wheel synchronisation no longer commands speed
     * corrections once braking begins.
     */
    controller->wheelSyncCorrectionCps = 0.0f;
    // We preserve first the wheelSyncErrorMm for diagnostics purpose.

    SteeringController_Centre(controller->steering);

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

    SteeringController_Centre(controller->steering);

    controller->stationarySamples = 0U;
    controller->mode = MOTIONCONTROLLER_IDLE;
}

MotionControllerStatus MotionController_Init(
    MotionController *controller,
    WheelSpeedController *leftWheel,
    WheelSpeedController *rightWheel,
    SteeringController *steering,
    ICM20948 *imu,
    const MotionControllerConfig *config)
{
	if (controller == NULL ||
	    leftWheel == NULL ||
	    rightWheel == NULL ||
	    leftWheel->motor == NULL ||
	    rightWheel->motor == NULL ||
	    steering == NULL ||
	    imu == NULL ||
	    config == NULL ||
	    config->kinematics == NULL ||
	    config->arcConfig == NULL)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    if (config->kinematics->rearEncoderCountsPerRev == 0U ||
        !isfinite(config->kinematics->rearWheelDiameterMm) ||
        !isfinite(config->kinematics->wheelbaseMm) ||
        !isfinite(config->kinematics->rearTrackWidthMm) ||
        config->kinematics->rearWheelDiameterMm <= 0.0f ||
        config->kinematics->wheelbaseMm <= 0.0f ||
        config->kinematics->rearTrackWidthMm <= 0.0f)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
    }

    if (!isfinite(config->straightSteeringSettlingTimeSec) ||
        !isfinite(config->headingKp) ||
        !isfinite(config->headingKi) ||
        !isfinite(config->headingKd) ||
        !isfinite(config->maxHeadingSteeringAngleRad) ||
        !isfinite(config->arcYawRateKp) ||
        !isfinite(config->arcYawRateKi) ||
        !isfinite(config->arcYawRateKd) ||
        !isfinite(config->maxArcSteeringCommandCorrection) ||
        !isfinite(config->arcHeadingKpPerSec) ||
        !isfinite(config->wheelSyncKpCpsPerMm) ||
        !isfinite(config->maxWheelSyncCorrectionCps) ||
        !isfinite(config->motionAccelerationMmps2) ||
        !isfinite(config->motionDecelerationMmps2) ||
        !isfinite(config->motionCompletionToleranceMm) ||
        !isfinite(config->arcYawRateFilterTauSec))
    {
        return MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
    }

    float minEffectiveAngleRad =
        SteeringController_GetMinEffectiveAngleRad(
            steering);

    float maxEffectiveAngleRad =
        SteeringController_GetMaxEffectiveAngleRad(
            steering);

    /*
     * Straight-line heading control must be able to correct
     * in either direction, so use only the range available
     * symmetrically about zero.
     */
    if (!isfinite(minEffectiveAngleRad) ||
        !isfinite(maxEffectiveAngleRad) ||
        minEffectiveAngleRad >= 0.0f ||
        maxEffectiveAngleRad <= 0.0f)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
    }

    float maxSymmetricSteeringAngleRad =
        fminf(
            -minEffectiveAngleRad,
            maxEffectiveAngleRad);

    if (config->maxHeadingSteeringAngleRad <= 0.0f ||
        config->maxHeadingSteeringAngleRad >
            maxSymmetricSteeringAngleRad)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
    }

    if (config->maxArcSteeringCommandCorrection <= 0.0f)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
    }

    if (config->wheelSyncKpCpsPerMm < 0.0f ||
        config->maxWheelSyncCorrectionCps < 0.0f)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
    }

    if (config->straightSteeringSettlingTimeSec < 0.0f ||
        config->arcHeadingKpPerSec < 0.0f ||
        config->motionCompletionToleranceMm <= 0.0f ||
        config->arcYawRateFilterTauSec < 0.0f ||
        config->stopStableSampleCount == 0U)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
    }

    if (!MotionController_ValidateArcConfig(config->arcConfig))
    {
        return MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
    }

    *controller = (MotionController){0};

    controller->leftWheel = leftWheel;
    controller->rightWheel = rightWheel;
    controller->steering = steering;
    controller->imu = imu;
    controller->config = config;

    controller->mode = MOTIONCONTROLLER_IDLE;

    if (!MotionProfile_Init(
            &controller->motionProfile,
            config->motionAccelerationMmps2,
            config->motionDecelerationMmps2,
			config->motionCompletionToleranceMm))
    {
        return MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
    }


    if (!PIDController_Init(
        &controller->headingPID,
        config->headingKp,
        config->headingKi,
        config->headingKd,
        -config->maxHeadingSteeringAngleRad,
		config->maxHeadingSteeringAngleRad))
    {
        return MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
    }

    if (!PIDController_Init(
            &controller->arcYawRatePID,
            config->arcYawRateKp,
            config->arcYawRateKi,
            config->arcYawRateKd,
            -config->maxArcSteeringCommandCorrection,
            +config->maxArcSteeringCommandCorrection))
    {
        return MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
    }

    controller->initialized = true;

    return MOTIONCONTROLLER_STATUS_OK;
}

static MotionControllerStatus MotionController_ValidateMotionRequest(
    const MotionController *controller,
    float distanceMm,
    float speedCps,
    bool *motionRequired)
{
    if (controller == NULL || motionRequired == NULL)
        return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;

    *motionRequired = false;

    if (!controller->initialized)
        return MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;

    if (controller->mode != MOTIONCONTROLLER_IDLE)
        return MOTIONCONTROLLER_STATUS_BUSY;

    if (!isfinite(distanceMm))
        return MOTIONCONTROLLER_STATUS_INVALID_DISTANCE;

    if (!isfinite(speedCps) || speedCps <= 0.0f)
        return MOTIONCONTROLLER_STATUS_INVALID_SPEED;

    *motionRequired = distanceMm != 0.0f;

    return MOTIONCONTROLLER_STATUS_OK;
}

/**
 * Initialise state shared by straight and arc commands after the complete
 * request has been validated. This helper deliberately does not steer.
 */
static MotionControllerStatus MotionController_StartMotion(
	MotionController *controller,
	float distanceMm,
	float speedCps)
{
    float mmPerCount =
        MotionController_GetMmPerCount(controller);

    float targetDistanceMm =
        fabsf(distanceMm);

    float maxSpeedMmps =
        speedCps * mmPerCount;

    if (!MotionProfile_Start(
            &controller->motionProfile,
            targetDistanceMm,
            maxSpeedMmps))
    {
        return MOTIONCONTROLLER_STATUS_PROFILE_ERROR;
    }

    controller->motionDirection = distanceMm > 0.0f ? 1 : -1;
    controller->targetDistanceMm = targetDistanceMm;

    controller->maxSpeedCps = speedCps;
    controller->targetSpeedCps = 0.0f;

    controller->targetCurvaturePerMm = 0.0f;
    controller->targetSteeringAngleRad = 0.0f;

    controller->yawDeg = 0.0f;

    controller->desiredWheelTravelDifferenceMm = 0.0f;
    controller->wheelSyncErrorMm = 0.0f;
    controller->wheelSyncCorrectionCps = 0.0f;

    controller->wheelReferenceCurvaturePerMm = 0.0f;
    controller->leftBaseTargetCps = 0.0f;
    controller->rightBaseTargetCps = 0.0f;

    controller->yawRateDps = 0.0f;
    controller->filteredYawRateDps = 0.0f;

    controller->arcDesiredYawRad = 0.0f;
    controller->arcHeadingErrorRad = 0.0f;

    controller->arcFeedforwardYawRateRadPerSec = 0.0f;
    controller->arcHeadingYawRateCorrectionRadPerSec = 0.0f;
    controller->arcTargetYawRateRadPerSec = 0.0f;

    controller->arcCommandedCurvaturePerMm = 0.0f;
    controller->arcSteeringFeedforwardCommand = 0.0f;
    controller->steeringPreparationElapsedSec = 0.0f;

    PIDController_Reset(&controller->headingPID);
    PIDController_Reset(&controller->arcYawRatePID);

    MotionController_ResetOdometry(controller);

    WheelSpeedController_SetTarget(
        controller->leftWheel,
        0.0f);

    WheelSpeedController_SetTarget(
        controller->rightWheel,
        0.0f);

    return MOTIONCONTROLLER_STATUS_OK;
}

MotionControllerStatus MotionController_MoveStraight(
    MotionController *controller,
    float distanceMm,
    float speedCps)
{
    if (controller == NULL)
        return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;

    bool motionRequired = false;

    MotionControllerStatus status =
        MotionController_ValidateMotionRequest(
			controller,
			distanceMm,
			speedCps,
			&motionRequired);

    if (status != MOTIONCONTROLLER_STATUS_OK ||
        !motionRequired)
    {
			return status;
    }

    status = MotionController_StartMotion(
        controller,
        distanceMm,
        speedCps);

    if (status != MOTIONCONTROLLER_STATUS_OK)
    {
        return status;
    }

    /*
     * Centre is applied immediately, without software slew limiting.
     * Keep the rear wheels stationary while the physical servo/linkage
     * settles at centre.
     */
    SteeringController_Centre(controller->steering);

    controller->mode =
        controller->config->straightSteeringSettlingTimeSec > 0.0f
            ? MOTIONCONTROLLER_STRAIGHT_PREPARING
            : MOTIONCONTROLLER_STRAIGHT;

    return MOTIONCONTROLLER_STATUS_OK;
}

MotionControllerStatus MotionController_MoveArc(
    MotionController *controller,
    float distanceMm,
    float radiusMm,
    float speedCps)
{
    if (controller == NULL)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    if (!isfinite(radiusMm) || radiusMm == 0.0f)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_RADIUS;
    }

    float curvaturePerMm = 1.0f / radiusMm;

    if (!isfinite(curvaturePerMm))
    {
        return MOTIONCONTROLLER_STATUS_INVALID_RADIUS;
    }

    bool motionRequired = false;

    MotionControllerStatus status =
        MotionController_ValidateMotionRequest(
            controller,
            distanceMm,
            speedCps,
            &motionRequired);

    if (status != MOTIONCONTROLLER_STATUS_OK ||
        !motionRequired)
    {
        return status;
    }

    float feedforwardCommand;

    if (!MotionController_GetArcSteeringFeedforwardCommand(
            controller,
            curvaturePerMm,
            &feedforwardCommand))
    {
        return MOTIONCONTROLLER_STATUS_UNSUPPORTED_CURVATURE;
    }

    float maximumCorrectionCommand =
        fmaxf(
            fabsf(controller->arcYawRatePID.outputMin),
            fabsf(controller->arcYawRatePID.outputMax));

    if (feedforwardCommand - maximumCorrectionCommand <
            SERVO_STEER_MIN ||
        feedforwardCommand + maximumCorrectionCommand >
            SERVO_STEER_MAX)
    {
        return MOTIONCONTROLLER_STATUS_UNSUPPORTED_CURVATURE;
    }

    status = MotionController_StartMotion(
        controller,
        distanceMm,
        speedCps);

    if (status != MOTIONCONTROLLER_STATUS_OK)
    {
        return status;
    }

    controller->targetCurvaturePerMm =
        curvaturePerMm;

    controller->arcCommandedCurvaturePerMm =
        controller->targetCurvaturePerMm;

    controller->arcSteeringFeedforwardCommand =
        feedforwardCommand;

    /*
     * Preserve the experimentally accepted abrupt prepositioning behavior.
     * MotionController now owns both the physical command and the matching
     * SteeringController raw-command state.
     */
    SteeringController_SetRawCommand(
        controller->steering,
        feedforwardCommand);

    controller->mode =
        controller->config->arcConfig->steeringSettlingTimeSec > 0.0f
            ? MOTIONCONTROLLER_ARC_PREPARING
            : MOTIONCONTROLLER_ARC;

    return MOTIONCONTROLLER_STATUS_OK;
}

MotionControllerStatus MotionController_Brake(
    MotionController *controller)
{
	if (controller == NULL)
	{
        return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    if (!controller->initialized)
        return MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;

    if (controller->mode == MOTIONCONTROLLER_BRAKING) {
        return MOTIONCONTROLLER_STATUS_OK;
    }

    MotionController_BeginBraking(controller);

    return MOTIONCONTROLLER_STATUS_OK;
}

static bool MotionController_UpdateYawEstimate(
    MotionController *controller,
    float dt)
{
    ICM20948Measurement measurement;

    if (!ICM20948_ReadMeasurement(
            controller->imu,
            &measurement))
    {
        return false;
    }

    /*
     * Raw gyro yaw rate.
     *
     * Keep this unfiltered for yaw integration.
     */
    controller->yawRateDps =
        measurement.gyroDps.z;

    controller->yawDeg +=
        controller->yawRateDps * dt;

    /*
     * First-order low-pass filter used only by
     * ARC yaw-rate feedback.
     *
     * alpha = dt / (tau + dt)
     */
    const float tau =
        controller->config->arcYawRateFilterTauSec;

    float alpha =
        dt / (tau + dt);

    controller->filteredYawRateDps +=
        alpha *
        (controller->yawRateDps -
         controller->filteredYawRateDps);

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
	float curvaturePerMm =
	    MotionController_GetWheelReferenceCurvaturePerMm(
	        controller);

	controller->wheelReferenceCurvaturePerMm =
	    curvaturePerMm;

    float leftBaseTargetCps;
    float rightBaseTargetCps;

    RobotKinematics_GetRearWheelSpeedTargets(
        controller->config->kinematics,
        controller->targetSpeedCps,
        curvaturePerMm,
        &leftBaseTargetCps,
        &rightBaseTargetCps);

    controller->leftBaseTargetCps =
        leftBaseTargetCps;

    controller->rightBaseTargetCps =
        rightBaseTargetCps;

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
        controller->config->wheelSyncKpCpsPerMm *
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
            controller->config->maxWheelSyncCorrectionCps,
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

static MotionControllerStatus MotionController_UpdateSteeringPreparation(
    MotionController *controller,
    float dt,
    float settlingTimeSec,
    MotionControllerMode activeMode)
{
    float remainingSettlingTimeSec =
        settlingTimeSec -
        controller->steeringPreparationElapsedSec;

    if (dt + MOTIONCONTROLLER_TIME_EPSILON_SEC <
        remainingSettlingTimeSec)
    {
        controller->steeringPreparationElapsedSec += dt;
        return MOTIONCONTROLLER_STATUS_OK;
    }

    controller->steeringPreparationElapsedSec =
        settlingTimeSec;

    /*
     * Establish the motion origin after the mechanical hold so any
     * incidental encoder movement or yaw during preparation is excluded
     * from the commanded motion.
     */
    controller->yawDeg = 0.0f;
    controller->yawRateDps = 0.0f;
    controller->filteredYawRateDps = 0.0f;

    MotionController_ResetOdometry(controller);

    controller->mode = activeMode;

    return MOTIONCONTROLLER_STATUS_OK;
}


static MotionControllerStatus MotionController_UpdateBraking(
    MotionController *controller,
    float dt)
{
    /* Keep odometry running so endpoint overshoot is captured. */
    MotionController_UpdateOdometry(controller);

    /*
     * Target is already zero. Update() still measures encoder velocity
     * before handling the zero target, which lets us determine when the
     * chassis has actually stopped.
     */
    WheelSpeedController_Update(
        controller->leftWheel,
        dt);

    WheelSpeedController_Update(
        controller->rightWheel,
        dt);

    /* Retain yaw caused by asymmetric braking for diagnostics. */
    MotionController_UpdateYawEstimate(controller, dt);

    bool leftStationary = WheelSpeedController_IsStationary(
        controller->leftWheel);

    bool rightStationary = WheelSpeedController_IsStationary(
        controller->rightWheel);

    if (leftStationary && rightStationary)
    {
        controller->stationarySamples++;

        if (controller->stationarySamples >=
            controller->config->stopStableSampleCount)
        {
            MotionController_FinishBraking(controller);
        }
    }
    else
    {
        controller->stationarySamples = 0U;
    }

    return MOTIONCONTROLLER_STATUS_OK;
}


static void MotionController_UpdateStraightSteering(
    MotionController *controller,
    float dt)
{
    /* Straight-line heading feedback. Desired relative yaw = 0. */
    float headingErrorRad =
        -controller->yawDeg *
        (MOTION_PI / 180.0f);

    float steeringAngleCorrectionRad =
        PIDController_Update(
            &controller->headingPID,
            headingErrorRad,
            dt);

    /*
     * Reverse motion reverses the yaw response produced by a given
     * physical steering angle.
     */
    float targetSteeringAngleRad =
        steeringAngleCorrectionRad *
        (float)controller->motionDirection;

    SteeringController_SetEffectiveAngleRad(
        controller->steering,
        targetSteeringAngleRad,
        dt);
}


static void MotionController_UpdateArcSteering(
    MotionController *controller,
    float dt,
    float mmPerCount)
{
    /* Signed rear-axle-centre reference speed. */
    float signedSpeedMmps =
        controller->targetSpeedCps *
        mmPerCount;

    /*
     * --------------------------------------------------------
     * PHASE 2B: OUTER HEADING LOOP
     * --------------------------------------------------------
     *
     * Desired heading along the requested constant-curvature path:
     *
     *     psi_d = kappa_path * s
     *
     * travelledDistanceMm is signed, so reverse motion is naturally
     * handled here.
     */
    float desiredYawRad =
        controller->targetCurvaturePerMm *
        controller->travelledDistanceMm;

    float measuredYawRad =
        controller->yawDeg *
        (MOTION_PI / 180.0f);

    float headingErrorRad =
        desiredYawRad -
        measuredYawRad;

    /*
     * Nominal geometric yaw-rate feedforward:
     *
     *     omega_ff = kappa_path * v
     */
    float feedforwardYawRateRadPerSec =
        controller->targetCurvaturePerMm *
        signedSpeedMmps;

    /*
     * Heading error directly biases the requested yaw rate:
     *
     *     omega_heading = K_heading * e_heading
     */
    float headingYawRateCorrectionRadPerSec =
        controller->config->arcHeadingKpPerSec *
        headingErrorRad;

    /*
     * Keep the feedback-generated curvature bounded as the profile
     * approaches zero speed.
     *
     * Allow heading feedback to add up to 2x the nominal yaw-rate
     * magnitude, i.e. total demand can reach 3x nominal curvature.
     */
    float headingCorrectionLimitRadPerSec =
        2.0f *
        fabsf(feedforwardYawRateRadPerSec);

    headingYawRateCorrectionRadPerSec =
        MotionController_Clamp(
            headingYawRateCorrectionRadPerSec,
            -headingCorrectionLimitRadPerSec,
            +headingCorrectionLimitRadPerSec);

    /* Final yaw-rate request seen by the inner Phase-2A loop. */
    float targetYawRateRadPerSec =
        feedforwardYawRateRadPerSec +
        headingYawRateCorrectionRadPerSec;

    /*
     * Rear-wheel geometry follows the same corrected motion request as
     * the steering controller. Since heading correction is limited
     * relative to feedforward yaw rate, commanded curvature remains
     * bounded even near the ends of the speed profile.
     */
    float commandedCurvaturePerMm =
        controller->targetCurvaturePerMm;

    if (fabsf(signedSpeedMmps) > 1.0f)
    {
        commandedCurvaturePerMm =
            targetYawRateRadPerSec /
            signedSpeedMmps;
    }

    /*
     * --------------------------------------------------------
     * PHASE 2A: INNER YAW-RATE LOOP
     * --------------------------------------------------------
     */
    float measuredYawRateRadPerSec =
        controller->filteredYawRateDps *
        (MOTION_PI / 180.0f);

    float yawRateErrorRadPerSec =
        targetYawRateRadPerSec -
        measuredYawRateRadPerSec;

    float steeringCorrectionCommand =
        PIDController_Update(
            &controller->arcYawRatePID,
            yawRateErrorRadPerSec,
            dt);

    /*
     * Raw steering correction is relative to calibrated
     * command-to-curvature feedforward. Increasing raw command produces
     * negative steering.
     */
    float targetCommand =
        controller->arcSteeringFeedforwardCommand -
        (float)controller->motionDirection *
        steeringCorrectionCommand;

    /* Phase-2B diagnostics. */
    controller->arcDesiredYawRad =
        desiredYawRad;

    controller->arcHeadingErrorRad =
        headingErrorRad;

    controller->arcFeedforwardYawRateRadPerSec =
        feedforwardYawRateRadPerSec;

    controller->arcHeadingYawRateCorrectionRadPerSec =
        headingYawRateCorrectionRadPerSec;

    controller->arcTargetYawRateRadPerSec =
        targetYawRateRadPerSec;

    controller->arcCommandedCurvaturePerMm =
        commandedCurvaturePerMm;

    /* Phase-2A diagnostics. */
    controller->arcYawRateErrorRadPerSec =
        yawRateErrorRadPerSec;

    controller->arcSteeringCorrectionCommand =
        steeringCorrectionCommand;

    controller->arcSteeringTargetCommand =
        targetCommand;

    SteeringController_SetRawCommandRateLimited(
        controller->steering,
        targetCommand,
        dt);
}


static MotionControllerStatus MotionController_UpdateActiveMotion(
    MotionController *controller,
    float dt)
{
    /* Keep odometry running so endpoint overshoot is captured. */
    MotionController_UpdateOdometry(controller);

    /*
     * --------------------------------------------------------
     * ACTIVE MOTION PROFILE
     * --------------------------------------------------------
     */

    float progressMm =
        (float)controller->motionDirection *
        controller->travelledDistanceMm;

    float targetSpeedMmps =
        MotionProfile_Update(
            &controller->motionProfile,
            progressMm,
            dt);

    /*
     * MotionProfile becomes inactive once the requested
     * distance has been reached.
     *
     * Enter the existing braking state so that zero velocity
     * is actively enforced until both wheels are stationary.
     */
    if (!MotionProfile_IsActive(
            &controller->motionProfile))
    {
        MotionController_BeginBraking(controller);
        return MOTIONCONTROLLER_STATUS_OK;
    }

    /*
     * Convert vehicle-speed magnitude back to encoder CPS
     * and restore the commanded motion direction.
     */
    float mmPerCount =
        MotionController_GetMmPerCount(controller);

    controller->targetSpeedCps =
        (float)controller->motionDirection *
        targetSpeedMmps /
        mmPerCount;

    if (!MotionController_UpdateYawEstimate(controller, dt)) {
        /*
         * Heading feedback has failed.
         * Stop rather than continuing open-loop.
         */
        MotionController_Stop(controller);
        return MOTIONCONTROLLER_STATUS_IMU_ERROR;
    }

    switch (controller->mode)
    {
        case MOTIONCONTROLLER_STRAIGHT:
            MotionController_UpdateStraightSteering(
                controller,
                dt);
            break;

        case MOTIONCONTROLLER_ARC:
            MotionController_UpdateArcSteering(
                controller,
                dt,
                mmPerCount);
            break;

        default:
            MotionController_Stop(controller);
            return MOTIONCONTROLLER_STATUS_INVALID_STATE;
    }

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

    return MOTIONCONTROLLER_STATUS_OK;
}


MotionControllerStatus MotionController_Update(
    MotionController *controller,
    float dt)
{
    if (controller == NULL)
        return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;

    if (!controller->initialized)
        return MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;

    if (!isfinite(dt) || dt <= 0.0f)
        return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;

    switch (controller->mode)
    {
        case MOTIONCONTROLLER_IDLE:
            return MOTIONCONTROLLER_STATUS_OK;

        case MOTIONCONTROLLER_STRAIGHT_PREPARING:
        {
            MotionControllerStatus status =
                MotionController_UpdateSteeringPreparation(
                    controller,
                    dt,
                    controller->config->straightSteeringSettlingTimeSec,
                    MOTIONCONTROLLER_STRAIGHT);

            if (status != MOTIONCONTROLLER_STATUS_OK ||
                controller->mode == MOTIONCONTROLLER_STRAIGHT_PREPARING)
            {
                return status;
            }

            /*
             * Match ARC_PREPARING boundary behaviour: the update that
             * completes preparation also executes the first active
             * straight-motion control cycle.
             */
            return MotionController_UpdateActiveMotion(
                controller,
                dt);
        }

        case MOTIONCONTROLLER_ARC_PREPARING:
        {
            MotionControllerStatus status =
                MotionController_UpdateSteeringPreparation(
                    controller,
                    dt,
                    controller->config->arcConfig->steeringSettlingTimeSec,
                    MOTIONCONTROLLER_ARC);

            if (status != MOTIONCONTROLLER_STATUS_OK ||
                controller->mode == MOTIONCONTROLLER_ARC_PREPARING)
            {
                return status;
            }

            /*
             * Preserve the existing boundary behavior: the update that
             * completes preparation also executes the first active arc
             * control cycle.
             */
            return MotionController_UpdateActiveMotion(
                controller,
                dt);
        }

        case MOTIONCONTROLLER_STRAIGHT:
        case MOTIONCONTROLLER_ARC:
            return MotionController_UpdateActiveMotion(
                controller,
                dt);

        case MOTIONCONTROLLER_BRAKING:
            return MotionController_UpdateBraking(
                controller,
                dt);

        default:
            MotionController_Stop(controller);
            return MOTIONCONTROLLER_STATUS_INVALID_STATE;
    }
}


MotionControllerStatus MotionController_Stop(
    MotionController *controller)
{
    if (controller == NULL)
        return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;

    if (!controller->initialized)
        return MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;

    WheelSpeedController_Stop(controller->leftWheel);
    WheelSpeedController_Stop(controller->rightWheel);

    SteeringController_Centre(controller->steering);

    PIDController_Reset(&controller->headingPID);
    PIDController_Reset(&controller->arcYawRatePID);

    controller->mode = MOTIONCONTROLLER_IDLE;

    return MOTIONCONTROLLER_STATUS_OK;
}


bool MotionController_IsBusy(
    const MotionController *controller)
{
    if (controller == NULL)
        return false;

    return controller->initialized &&
        controller->mode != MOTIONCONTROLLER_IDLE;
}
