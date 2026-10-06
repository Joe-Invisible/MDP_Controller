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

/*
 * EXPERIMENTAL yaw-priority arc termination.
 *
 * Standalone straights and straight-ending profiles keep distance completion.
 * Near the nominal end of a standalone or batch-ending arc,
 * measured yaw is allowed to complete the command before distance does, or
 * to extend the arc slightly when distance finishes first.
 *
 * Before the final yaw-sensitive window, an additional deceleration envelope
 * guarantees that the profiled centre-speed request has fallen to the
 * terminal-approach speed by the window entry. This preserves fast bulk
 * motion while preventing the stopping predictor from firing immediately
 * after a high-speed cruise.
 *
 * The terminal braking decision predicts the yaw that will be accumulated
 * after braking begins from the freshest raw gyro rate. The 75 ms horizon is
 * the initial empirical estimate from the first two yaw-priority trials.
 */
#define MOTIONCONTROLLER_ARC_TERMINAL_ENTRY_DISTANCE_MM      MOTION_PATH_TERMINAL_ENTRY_DISTANCE_MM
#define MOTIONCONTROLLER_ARC_TERMINAL_MAX_OVERRUN_MM         (30.0f)
#define MOTIONCONTROLLER_ARC_TERMINAL_APPROACH_SPEED_CPS     (800.0f)
#define MOTIONCONTROLLER_ARC_TERMINAL_SPEED_CPS               (400.0f)
#define MOTIONCONTROLLER_ARC_TERMINAL_MIN_TARGET_YAW_DEG       MOTION_PATH_TERMINAL_MIN_YAW_DEG
#define MOTIONCONTROLLER_ARC_TERMINAL_BRAKE_PREDICTION_SEC     (0.075f)

/*
 * TEMPORARY hardware-regression exemption.
 *
 * The +/-275 mm regression deliberately retains the existing extreme
 * feedforward points together with the newly tuned +/-30 raw feedback
 * authority. Those endpoints do not leave a full symmetric 30 raw units of
 * steering headroom, so the normal MoveArc() request check would reject the
 * test before it can run. SteeringController still clamps the actual raw
 * command to the physical servo range.
 *
 * Restore this to 0 after the sharp-arc regression campaign.
 */
#define MOTIONCONTROLLER_TEST_ALLOW_LIMITED_ARC_HEADROOM (1)

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
    /* Braking centres steering and no longer requests path curvature. */
    if (controller->mode == MOTIONCONTROLLER_BRAKING ||
        controller->mode == MOTIONCONTROLLER_IDLE)
    {
        return 0.0f;
    }

    /* The angle model is used only by the temporary straight A/B baseline. */
    if (controller->mode == MOTIONCONTROLLER_STRAIGHT &&
        controller->config->useLegacyStraightSteering)
    {
        float steeringAngleRad =
            SteeringController_GetEffectiveAngleRad(controller->steering);

        return RobotKinematics_GetCurvaturePerMm(
            controller->config->kinematics,
            steeringAngleRad);
    }

    /* Both unified paths coordinate wheels from the same IMU motion request. */
    return controller->arcCommandedCurvaturePerMm;
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

static bool MotionController_GetPathSteeringFeedforwardCommand(
    const MotionController *controller,
    float curvaturePerMm,
    float *rawSteeringCommand)
{
    if (controller == NULL ||
        controller->config == NULL ||
        controller->config->arcConfig == NULL ||
        rawSteeringCommand == NULL)
    {
        return false;
    }

    if (curvaturePerMm == 0.0f)
    {
        /* Legacy zero-curvature path captures its calibrated centre. */
        *rawSteeringCommand =
            SteeringController_GetCommand(controller->steering);
        return true;
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

MotionControllerStatus MotionController_GetProfileFeedforward(
    const MotionController *controller, float curvaturePerMm, float *rawCommand)
{
    if (controller == NULL || rawCommand == NULL || !isfinite(curvaturePerMm))
        return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
    if (!controller->initialized) return MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;
    const MotionControllerConfig *c = controller->config;
    if (curvaturePerMm == 0.0f) {
        *rawCommand = c->straightSteeringFeedforwardCommand;
        return MOTIONCONTROLLER_STATUS_OK;
    }
    const MotionControllerArcConfig *a = c->arcConfig;
    const MotionControllerArcFeedforwardPoint *nearZero = curvaturePerMm < 0.0f
        ? &a->negativePoints[a->negativePointCount - 1U] : &a->positivePoints[0];
    if (fabsf(curvaturePerMm) < fabsf(nearZero->curvaturePerMm)) {
        float t = curvaturePerMm / nearZero->curvaturePerMm;
        *rawCommand = c->straightSteeringFeedforwardCommand +
            t * (nearZero->rawSteeringCommand - c->straightSteeringFeedforwardCommand);
        return MOTIONCONTROLLER_STATUS_OK;
    }
    return MotionController_GetPathSteeringFeedforwardCommand(controller,
        curvaturePerMm, rawCommand) ? MOTIONCONTROLLER_STATUS_OK :
        MOTIONCONTROLLER_STATUS_UNSUPPORTED_CURVATURE;
}

static bool MotionController_ValidPathSample(const MotionPathSample *s)
{
    return isfinite(s->curvaturePerMm) && isfinite(s->desiredYawRad) &&
        isfinite(s->speedLimitCps) && s->speedLimitCps > 0.0f &&
        isfinite(s->straightTuningWeight) && s->straightTuningWeight >= 0.0f &&
        s->straightTuningWeight <= 1.0f;
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

    float deltaCentreMm =
        0.5f * (deltaLeftMm + deltaRightMm);

    float curvaturePerMm =
        MotionController_GetWheelReferenceCurvaturePerMm(
            controller);

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

    MotionProfile_Stop(&controller->motionProfile);
    controller->targetSpeedCps = 0.0f;

    controller->wheelSyncCorrectionCps = 0.0f;

    SteeringController_Centre(controller->steering);

    controller->stationarySamples = 0U;

    controller->mode = MOTIONCONTROLLER_BRAKING;
}

static void MotionController_FinishBraking(
    MotionController *controller)
{
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
        !isfinite(config->straightSteeringFeedforwardCommand) ||
        !isfinite(config->straightYawRateKp) ||
        !isfinite(config->straightYawRateKi) ||
        !isfinite(config->straightYawRateKd) ||
        !isfinite(config->straightHeadingKpPerSec) ||
        !isfinite(config->arcYawRateKp) ||
        !isfinite(config->arcYawRateKi) ||
        !isfinite(config->arcYawRateKd) ||
        !isfinite(config->arcHeadingKpPerSec) ||
        !isfinite(config->maxArcSteeringCommandCorrection) ||
        !isfinite(config->maxPathCorrectionCurvaturePerMm) ||
        !isfinite(config->wheelSyncKpCpsPerMm) ||
        !isfinite(config->maxWheelSyncCorrectionCps) ||
        !isfinite(config->motionAccelerationMmps2) ||
        !isfinite(config->motionDecelerationMmps2) ||
        !isfinite(config->motionCompletionToleranceMm) ||
        !isfinite(config->arcYawRateFilterTauSec))
    {
        return MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
    }

    if (config->straightSteeringFeedforwardCommand < SERVO_STEER_MIN ||
        config->straightSteeringFeedforwardCommand > SERVO_STEER_MAX)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
    }

    if (config->useLegacyStraightSteering)
    {
        if (!isfinite(config->headingKp) ||
            !isfinite(config->headingKi) ||
            !isfinite(config->headingKd) ||
            !isfinite(config->maxHeadingSteeringAngleRad))
        {
            return MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
        }

        float minEffectiveAngleRad =
            SteeringController_GetMinEffectiveAngleRad(
                steering);

        float maxEffectiveAngleRad =
            SteeringController_GetMaxEffectiveAngleRad(
                steering);

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
        config->straightHeadingKpPerSec < 0.0f ||
        config->arcHeadingKpPerSec < 0.0f ||
        config->maxPathCorrectionCurvaturePerMm < 0.0f ||
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

    if (config->useLegacyStraightSteering &&
        !PIDController_Init(
            &controller->headingPID,
            config->headingKp,
            config->headingKi,
            config->headingKd,
            -config->maxHeadingSteeringAngleRad,
            +config->maxHeadingSteeringAngleRad))
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
    controller->terminalYawPredictedReached = false;
    controller->terminalDistanceLimitReached = false;

    controller->desiredWheelTravelDifferenceMm = 0.0f;
    controller->wheelSyncErrorMm = 0.0f;
    controller->wheelSyncCorrectionCps = 0.0f;

    controller->wheelReferenceCurvaturePerMm = 0.0f;
    controller->leftBaseTargetCps = 0.0f;
    controller->rightBaseTargetCps = 0.0f;

    controller->yawRateDps = 0.0f;
    controller->filteredYawRateDps = 0.0f;
    controller->measuredCentreSpeedMmps = 0.0f;
    controller->filteredMeasuredCentreSpeedMmps = 0.0f;

    controller->arcDesiredYawRad = 0.0f;
    controller->arcHeadingErrorRad = 0.0f;

    controller->arcFeedforwardYawRateRadPerSec = 0.0f;
    controller->arcHeadingYawRateCorrectionRadPerSec = 0.0f;
    controller->arcTargetYawRateRadPerSec = 0.0f;

    controller->arcCommandedCurvaturePerMm = 0.0f;
    controller->arcSteeringFeedforwardCommand = 0.0f;
    controller->arcYawRateErrorRadPerSec = 0.0f;
    controller->arcSteeringCorrectionCommand = 0.0f;
    controller->arcSteeringTargetCommand = 0.0f;
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

static void MotionController_SelectPathTuning(
    MotionController *controller,
    MotionControllerMode mode)
{
    if (mode == MOTIONCONTROLLER_STRAIGHT)
    {
        controller->arcYawRatePID.kp =
            controller->config->straightYawRateKp;
        controller->arcYawRatePID.ki =
            controller->config->straightYawRateKi;
        controller->arcYawRatePID.kd =
            controller->config->straightYawRateKd;
    }
    else
    {
        controller->arcYawRatePID.kp =
            controller->config->arcYawRateKp;
        controller->arcYawRatePID.ki =
            controller->config->arcYawRateKi;
        controller->arcYawRatePID.kd =
            controller->config->arcYawRateKd;
    }

    PIDController_Reset(&controller->arcYawRatePID);
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

    MotionController_SelectPathTuning(
        controller,
        MOTIONCONTROLLER_STRAIGHT);

    /*
     * First restore a deterministic calibrated centre. Unified straight then
     * replaces that legacy zero-angle crossing with its empirically selected
     * raw feedforward before the settling interval begins.
     */
    SteeringController_Centre(controller->steering);

    if (controller->config->useLegacyStraightSteering)
    {
        MotionController_GetPathSteeringFeedforwardCommand(
            controller,
            0.0f,
            &controller->arcSteeringFeedforwardCommand);
    }
    else
    {
        controller->arcSteeringFeedforwardCommand =
            controller->config->straightSteeringFeedforwardCommand;

        SteeringController_SetRawCommand(
            controller->steering,
            controller->arcSteeringFeedforwardCommand);
    }

    controller->arcSteeringTargetCommand =
        controller->arcSteeringFeedforwardCommand;

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

    if (!MotionController_GetPathSteeringFeedforwardCommand(
            controller,
            curvaturePerMm,
            &feedforwardCommand))
    {
        return MOTIONCONTROLLER_STATUS_UNSUPPORTED_CURVATURE;
    }

#if !MOTIONCONTROLLER_TEST_ALLOW_LIMITED_ARC_HEADROOM
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
#endif

    status = MotionController_StartMotion(
        controller,
        distanceMm,
        speedCps);

    if (status != MOTIONCONTROLLER_STATUS_OK)
    {
        return status;
    }

    MotionController_SelectPathTuning(
        controller,
        MOTIONCONTROLLER_ARC);

    controller->targetCurvaturePerMm =
        curvaturePerMm;

    controller->arcCommandedCurvaturePerMm =
        controller->targetCurvaturePerMm;

    controller->arcSteeringFeedforwardCommand =
        feedforwardCommand;
    controller->arcSteeringTargetCommand =
        feedforwardCommand;

    SteeringController_SetRawCommand(
        controller->steering,
        feedforwardCommand);

    controller->mode =
        controller->config->arcConfig->steeringSettlingTimeSec > 0.0f
            ? MOTIONCONTROLLER_ARC_PREPARING
            : MOTIONCONTROLLER_ARC;

    return MOTIONCONTROLLER_STATUS_OK;
}

MotionControllerStatus MotionController_FollowProfile(
    MotionController *controller, const MotionPathProfile *profile)
{
    if (controller == NULL || profile == NULL || profile->evaluate == NULL ||
        !isfinite(profile->steeringSettlingTimeSec) || profile->steeringSettlingTimeSec < 0.0f)
        return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
    bool required;
    MotionControllerStatus status = MotionController_ValidateMotionRequest(
        controller, profile->signedDistanceMm, profile->maxSpeedCps, &required);
    if (status != MOTIONCONTROLLER_STATUS_OK || !required) return status;
    if (controller->config->useLegacyStraightSteering)
        return MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
    MotionPathSample initial, final;
    float raw, finalRaw;
    if (!profile->evaluate(profile->context, 0.0f, &initial) ||
        !profile->evaluate(profile->context, fabsf(profile->signedDistanceMm), &final) ||
        !MotionController_ValidPathSample(&initial) ||
        !MotionController_ValidPathSample(&final))
        return MOTIONCONTROLLER_STATUS_PROFILE_ERROR;
    if (profile->terminalYawPriority &&
        (!isfinite(profile->terminalEntryProgressMm) ||
         profile->terminalEntryProgressMm < 0.0f ||
         profile->terminalEntryProgressMm >= fabsf(profile->signedDistanceMm) ||
         final.curvaturePerMm == 0.0f))
        return MOTIONCONTROLLER_STATUS_PROFILE_ERROR;
    status = MotionController_GetProfileFeedforward(controller, initial.curvaturePerMm, &raw);
    if (status != MOTIONCONTROLLER_STATUS_OK) return status;
    status = MotionController_GetProfileFeedforward(controller, final.curvaturePerMm, &finalRaw);
    if (status != MOTIONCONTROLLER_STATUS_OK) return status;
    status = MotionController_StartMotion(controller, profile->signedDistanceMm, profile->maxSpeedCps);
    if (status != MOTIONCONTROLLER_STATUS_OK) return status;
    controller->pathProfile = *profile;
    controller->pathSample = initial;
    controller->pathFinalSample = final;
    controller->targetCurvaturePerMm = initial.curvaturePerMm;
    controller->arcSteeringFeedforwardCommand = raw;
    controller->arcSteeringTargetCommand = raw;
    controller->arcCommandedCurvaturePerMm = initial.curvaturePerMm;
    SteeringController_SetRawCommand(controller->steering, raw);
    controller->mode = profile->steeringSettlingTimeSec > 0.0f
        ? MOTIONCONTROLLER_PROFILE_PREPARING : MOTIONCONTROLLER_PROFILE;
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

    if (controller->mode == MOTIONCONTROLLER_BRAKING)
        return MOTIONCONTROLLER_STATUS_OK;

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

    controller->yawRateDps =
        measurement.gyroDps.z;

    controller->yawDeg +=
        controller->yawRateDps * dt;

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

    controller->wheelSyncErrorMm =
        (controller->rightTravelMm -
            controller->leftTravelMm)
        - controller->desiredWheelTravelDifferenceMm;

    float correctionCps =
        controller->config->wheelSyncKpCpsPerMm *
        controller->wheelSyncErrorMm;

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

    controller->yawDeg = 0.0f;
    controller->yawRateDps = 0.0f;
    controller->filteredYawRateDps = 0.0f;
    controller->measuredCentreSpeedMmps = 0.0f;
    controller->filteredMeasuredCentreSpeedMmps = 0.0f;

    MotionController_ResetOdometry(controller);

    controller->mode = activeMode;

    return MOTIONCONTROLLER_STATUS_OK;
}

static MotionControllerStatus MotionController_UpdateBraking(
    MotionController *controller,
    float dt)
{
    MotionController_UpdateOdometry(controller);

    WheelSpeedController_Update(
        controller->leftWheel,
        dt);

    WheelSpeedController_Update(
        controller->rightWheel,
        dt);

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

static void MotionController_UpdateLegacyStraightSteering(
    MotionController *controller,
    float dt)
{
    float headingErrorRad =
        -controller->yawDeg *
        (MOTION_PI / 180.0f);

    float steeringAngleCorrectionRad =
        PIDController_Update(
            &controller->headingPID,
            headingErrorRad,
            dt);

    float targetSteeringAngleRad =
        steeringAngleCorrectionRad *
        (float)controller->motionDirection;

    SteeringController_SetEffectiveAngleRad(
        controller->steering,
        targetSteeringAngleRad,
        dt);
}

static void MotionController_UpdatePathSteering(
    MotionController *controller,
    float dt,
    float mmPerCount)
{
    float signedReferenceSpeedMmps =
        controller->targetSpeedCps *
        mmPerCount;

    controller->measuredCentreSpeedMmps =
        0.5f *
        (controller->leftWheel->measuredSpeedCps +
         controller->rightWheel->measuredSpeedCps) *
        mmPerCount;

    const float speedFilterTau =
        controller->config->arcYawRateFilterTauSec;

    float speedFilterAlpha =
        dt / (speedFilterTau + dt);

    controller->filteredMeasuredCentreSpeedMmps +=
        speedFilterAlpha *
        (controller->measuredCentreSpeedMmps -
         controller->filteredMeasuredCentreSpeedMmps);

    float desiredPathDistanceMm =
        controller->travelledDistanceMm;

    if (controller->mode == MOTIONCONTROLLER_ARC)
    {
        float directedProgressMm =
            (float)controller->motionDirection *
            desiredPathDistanceMm;

        if (directedProgressMm > controller->targetDistanceMm)
        {
            desiredPathDistanceMm =
                (float)controller->motionDirection *
                controller->targetDistanceMm;
        }
    }

    float desiredYawRad = controller->mode == MOTIONCONTROLLER_PROFILE
        ? controller->pathSample.desiredYawRad
        : controller->targetCurvaturePerMm * desiredPathDistanceMm;

    float measuredYawRad =
        controller->yawDeg *
        (MOTION_PI / 180.0f);

    float headingErrorRad =
        desiredYawRad -
        measuredYawRad;

    float feedforwardYawRateRadPerSec =
        controller->targetCurvaturePerMm *
        controller->filteredMeasuredCentreSpeedMmps;

    float headingKpPerSec =
        controller->mode == MOTIONCONTROLLER_STRAIGHT
            ? controller->config->straightHeadingKpPerSec
            : controller->config->arcHeadingKpPerSec;

    if (controller->mode == MOTIONCONTROLLER_PROFILE) {
        float w = controller->pathSample.straightTuningWeight;
        const MotionControllerConfig *c = controller->config;
        headingKpPerSec = c->arcHeadingKpPerSec + w *
            (c->straightHeadingKpPerSec - c->arcHeadingKpPerSec);
        float nextKi = c->arcYawRateKi + w * (c->straightYawRateKi - c->arcYawRateKi);
        /* Retain the integral OUTPUT across gain scheduling. */
        if (nextKi > 0.0f)
            controller->arcYawRatePID.integral *= controller->arcYawRatePID.ki / nextKi;
        else
            controller->arcYawRatePID.integral = 0.0f;
        controller->arcYawRatePID.kp = c->arcYawRateKp + w * (c->straightYawRateKp - c->arcYawRateKp);
        controller->arcYawRatePID.ki = nextKi;
        controller->arcYawRatePID.kd = c->arcYawRateKd + w * (c->straightYawRateKd - c->arcYawRateKd);
    }

    float headingYawRateCorrectionRadPerSec =
        headingKpPerSec *
        headingErrorRad;

    float headingCorrectionLimitRadPerSec =
        fabsf(signedReferenceSpeedMmps) *
        controller->config->maxPathCorrectionCurvaturePerMm;

    headingYawRateCorrectionRadPerSec =
        MotionController_Clamp(
            headingYawRateCorrectionRadPerSec,
            -headingCorrectionLimitRadPerSec,
            +headingCorrectionLimitRadPerSec);

    float targetYawRateRadPerSec =
        feedforwardYawRateRadPerSec +
        headingYawRateCorrectionRadPerSec;

    float commandedCurvaturePerMm =
        controller->targetCurvaturePerMm;

    if (signedReferenceSpeedMmps != 0.0f)
    {
        commandedCurvaturePerMm +=
            headingYawRateCorrectionRadPerSec /
            signedReferenceSpeedMmps;
    }

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

    float targetCommand =
        controller->arcSteeringFeedforwardCommand -
        (float)controller->motionDirection *
        steeringCorrectionCommand;

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

/* Shared terminal policy. targetYawRad is relative to the current motion/run
 * origin. For a mixed-curvature run its sign need not match the final turn. */
static float MotionController_ApplyArcTerminalPolicy(
    MotionController *controller, float progressMm, float terminalEntryProgressMm,
    float targetYawRad, float yawDirection, float terminalSpeedMmps,
    float targetSpeedMmps)
{
    float approachSpeed = MOTIONCONTROLLER_ARC_TERMINAL_APPROACH_SPEED_CPS *
        MotionController_GetMmPerCount(controller);
    float distanceToEntry = fmaxf(0.0f, terminalEntryProgressMm - progressMm);
    float approachLimit = sqrtf(approachSpeed * approachSpeed +
        2.0f * controller->config->motionDecelerationMmps2 * distanceToEntry);
    if (targetSpeedMmps > approachLimit) {
        targetSpeedMmps = approachLimit;
        controller->motionProfile.targetSpeedMmps = approachLimit;
    }
    if (progressMm >= terminalEntryProgressMm) {
        float measuredYawRad = controller->yawDeg * (MOTION_PI / 180.0f);
        float directedYawRate = fmaxf(0.0f, yawDirection * controller->yawRateDps *
            (MOTION_PI / 180.0f));
        float predictedStoppingYaw = yawDirection * measuredYawRad +
            directedYawRate * MOTIONCONTROLLER_ARC_TERMINAL_BRAKE_PREDICTION_SEC;
        controller->terminalYawPredictedReached =
            predictedStoppingYaw >= yawDirection * targetYawRad;
        controller->terminalDistanceLimitReached = progressMm >=
            controller->targetDistanceMm + MOTIONCONTROLLER_ARC_TERMINAL_MAX_OVERRUN_MM;
        if (controller->terminalYawPredictedReached || controller->terminalDistanceLimitReached) {
            MotionProfile_Stop(&controller->motionProfile);
            targetSpeedMmps = 0.0f;
        } else {
            controller->motionProfile.active = true;
            if (targetSpeedMmps < terminalSpeedMmps) {
                targetSpeedMmps = terminalSpeedMmps;
                controller->motionProfile.targetSpeedMmps = terminalSpeedMmps;
            }
        }
    }
    return targetSpeedMmps;
}

static MotionControllerStatus MotionController_UpdateProfile(
    MotionController *controller, float dt)
{
    MotionController_UpdateOdometry(controller);
    if (!MotionController_UpdateYawEstimate(controller, dt)) {
        MotionController_Stop(controller);
        return MOTIONCONTROLLER_STATUS_IMU_ERROR;
    }
    float progress = fmaxf(0.0f, controller->motionDirection * controller->travelledDistanceMm);
    MotionPathSample sample;
    float referenceProgress = controller->pathProfile.terminalYawPriority
        ? fminf(progress, controller->targetDistanceMm) : progress;
    if (!controller->pathProfile.evaluate(controller->pathProfile.context, referenceProgress, &sample) ||
        !MotionController_ValidPathSample(&sample)) {
        MotionController_Stop(controller);
        return MOTIONCONTROLLER_STATUS_PROFILE_ERROR;
    }
    float raw;
    MotionControllerStatus status = MotionController_GetProfileFeedforward(
        controller, sample.curvaturePerMm, &raw);
    if (status != MOTIONCONTROLLER_STATUS_OK) {
        MotionController_Stop(controller);
        return status;
    }
    float mmPerCount = MotionController_GetMmPerCount(controller);
    controller->pathSample = sample;
    controller->targetCurvaturePerMm = sample.curvaturePerMm;
    controller->arcSteeringFeedforwardCommand = raw;
    controller->motionProfile.maxSpeedMmps =
        fminf(controller->maxSpeedCps, sample.speedLimitCps) * mmPerCount;
    float speed = MotionProfile_Update(&controller->motionProfile, progress, dt);
    if (controller->pathProfile.terminalYawPriority) {
        float yawDirection = controller->motionDirection *
            controller->pathFinalSample.curvaturePerMm > 0.0f ? 1.0f : -1.0f;
        float terminalSpeed = fminf(MOTIONCONTROLLER_ARC_TERMINAL_SPEED_CPS,
            fminf(controller->maxSpeedCps, sample.speedLimitCps)) * mmPerCount;
        speed = MotionController_ApplyArcTerminalPolicy(controller, progress,
            controller->pathProfile.terminalEntryProgressMm,
            controller->pathFinalSample.desiredYawRad, yawDirection, terminalSpeed, speed);
    }
    if (!MotionProfile_IsActive(&controller->motionProfile)) {
        MotionController_BeginBraking(controller);
        return MOTIONCONTROLLER_STATUS_OK;
    }
    controller->targetSpeedCps = controller->motionDirection * speed / mmPerCount;
    MotionController_UpdatePathSteering(controller, dt, mmPerCount);
    MotionController_UpdateWheelSynchronisation(controller);
    WheelSpeedController_Update(controller->leftWheel, dt);
    WheelSpeedController_Update(controller->rightWheel, dt);
    return MOTIONCONTROLLER_STATUS_OK;
}

static MotionControllerStatus MotionController_UpdateActiveMotion(
    MotionController *controller,
    float dt)
{
    MotionController_UpdateOdometry(controller);

    float progressMm =
        (float)controller->motionDirection *
        controller->travelledDistanceMm;

    float targetSpeedMmps =
        MotionProfile_Update(
            &controller->motionProfile,
            progressMm,
            dt);

    if (controller->mode == MOTIONCONTROLLER_ARC)
    {
        if (!MotionController_UpdateYawEstimate(controller, dt))
        {
            MotionController_Stop(controller);
            return MOTIONCONTROLLER_STATUS_IMU_ERROR;
        }
    }

    if (controller->mode == MOTIONCONTROLLER_ARC)
    {
        float targetYawRad = controller->targetCurvaturePerMm *
            controller->motionDirection * controller->targetDistanceMm;
        if (fabsf(targetYawRad) > MOTIONCONTROLLER_ARC_TERMINAL_MIN_TARGET_YAW_DEG *
            (MOTION_PI / 180.0f))
        {
            targetSpeedMmps = MotionController_ApplyArcTerminalPolicy(controller, progressMm,
                fmaxf(0.0f, controller->targetDistanceMm -
                    MOTIONCONTROLLER_ARC_TERMINAL_ENTRY_DISTANCE_MM),
                targetYawRad, targetYawRad >= 0.0f ? 1.0f : -1.0f,
                MOTIONCONTROLLER_ARC_TERMINAL_SPEED_CPS * MotionController_GetMmPerCount(controller),
                targetSpeedMmps);
        }
    }

    if (!MotionProfile_IsActive(
            &controller->motionProfile))
    {
        MotionController_BeginBraking(controller);
        return MOTIONCONTROLLER_STATUS_OK;
    }

    float mmPerCount =
        MotionController_GetMmPerCount(controller);

    controller->targetSpeedCps =
        (float)controller->motionDirection *
        targetSpeedMmps /
        mmPerCount;

    if (controller->mode != MOTIONCONTROLLER_ARC &&
        !MotionController_UpdateYawEstimate(controller, dt))
    {
        MotionController_Stop(controller);
        return MOTIONCONTROLLER_STATUS_IMU_ERROR;
    }

    if (controller->mode == MOTIONCONTROLLER_STRAIGHT &&
        controller->config->useLegacyStraightSteering)
    {
        MotionController_UpdateLegacyStraightSteering(controller, dt);
    }
    else if (controller->mode == MOTIONCONTROLLER_STRAIGHT ||
             controller->mode == MOTIONCONTROLLER_ARC)
    {
        MotionController_UpdatePathSteering(controller, dt, mmPerCount);
    }
    else
    {
        MotionController_Stop(controller);
        return MOTIONCONTROLLER_STATUS_INVALID_STATE;
    }

    MotionController_UpdateWheelSynchronisation(controller);

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

            return MotionController_UpdateActiveMotion(
                controller,
                dt);
        }

        case MOTIONCONTROLLER_PROFILE_PREPARING:
        {
            MotionControllerStatus status = MotionController_UpdateSteeringPreparation(
                controller, dt, controller->pathProfile.steeringSettlingTimeSec,
                MOTIONCONTROLLER_PROFILE);
            if (status != MOTIONCONTROLLER_STATUS_OK ||
                controller->mode == MOTIONCONTROLLER_PROFILE_PREPARING)
                return status;
            return MotionController_UpdateProfile(controller, dt);
        }
        case MOTIONCONTROLLER_PROFILE:
            return MotionController_UpdateProfile(controller, dt);

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
