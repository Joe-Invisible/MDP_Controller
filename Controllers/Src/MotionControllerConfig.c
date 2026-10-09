/*
 * MotionControllerConfig.c
 *
 * Created on: 2026年8月30日
 * Author: Joe
 */

#include "MotionControllerConfig.h"

#define MG512P30_QUAD_RESOLUTION    1560U

#define WHEELBASE_MM                145.0f

#define WHEEL_WIDTH_MM              20.0f
#define ROBOT_REAR_WIDTH_MM         185.0f

/*
 * Rear track width is the centre-to-centre distance
 * between the two rear wheels.
 */
#define REAR_TRACK_WIDTH_MM \
    (ROBOT_REAR_WIDTH_MM - WHEEL_WIDTH_MM)

/*
 * Effective wheel diameter under load.
 */
#define WHEEL_DIAMETER_MM           65.5f

#define STRAIGHT_STEERING_SETTLING_TIME_SEC  (0.15f)
#define ARC_STEERING_SETTLING_TIME_SEC       (0.15f)

const RobotKinematics kinematics =
{
    .rearEncoderCountsPerRev = MG512P30_QUAD_RESOLUTION,
    .rearWheelDiameterMm = WHEEL_DIAMETER_MM,

    .wheelbaseMm = WHEELBASE_MM,
    .rearTrackWidthMm = REAR_TRACK_WIDTH_MM,
};

/*
 * Empirical curvature -> absolute raw steering feedforward calibration.
 *
 * Each branch is ordered by strictly increasing curvature. The branches
 * remain separate so interpolation can never cross the uncalibrated region
 * around zero or assume left/right symmetry.
 */
static const MotionControllerArcFeedforwardPoint
negativeArcFeedforwardPoints[] =
{
    { -1.0f /  275.0f, +95.0f },
    { -1.0f /  300.0f, +84.2f },
    { -1.0f /  400.0f, +59.7f },
    { -1.0f /  500.0f, +47.5f },
    { -1.0f / 1000.0f, +23.8f },
    { -1.0f / 1500.0f, +16.5f },
    { -1.0f / 2500.0f, +12.0f },
};

static const MotionControllerArcFeedforwardPoint
positiveArcFeedforwardPoints[] =
{
    { +1.0f / 1500.0f, -22.5f },
    { +1.0f / 1000.0f, -27.7f },
    { +1.0f /  500.0f, -47.5f },
    { +1.0f /  300.0f, -75.0f },
    { +1.0f /  275.0f, -81.7f },
};

#define NEGATIVE_ARC_FEEDFORWARD_POINT_COUNT \
    ((uint32_t)(sizeof(negativeArcFeedforwardPoints) / \
        sizeof(negativeArcFeedforwardPoints[0])))

#define POSITIVE_ARC_FEEDFORWARD_POINT_COUNT \
    ((uint32_t)(sizeof(positiveArcFeedforwardPoints) / \
        sizeof(positiveArcFeedforwardPoints[0])))

const MotionControllerArcConfig arcMotionConfig =
{
    .negativePoints = negativeArcFeedforwardPoints,
    .negativePointCount = NEGATIVE_ARC_FEEDFORWARD_POINT_COUNT,

    .positivePoints = positiveArcFeedforwardPoints,
    .positivePointCount = POSITIVE_ARC_FEEDFORWARD_POINT_COUNT,

    .steeringSettlingTimeSec = ARC_STEERING_SETTLING_TIME_SEC,
};

const MotionControllerConfig motionControllerConfig =
{
    .kinematics = &kinematics,
    .arcConfig = &arcMotionConfig,

    .straightSteeringSettlingTimeSec =
        STRAIGHT_STEERING_SETTLING_TIME_SEC,

    /*
     * Post-tuning straight feedforward refinement. The closed loop was
     * repeatedly cancelling the previous centre bias, so unified straight
     * motion now starts from raw zero and leaves only the residual error to
     * the already-tuned feedback loops.
     */
    .straightSteeringFeedforwardCommand = 0.0f,

    .useLegacyStraightSteering = false,

    .headingKp = 1.2f,
    .headingKi = 0.05f,
    .headingKd = 0.0f,
    .maxHeadingSteeringAngleRad = 0.015f,

    /*
     * Same unified path control law, with operating-point-specific gains.
     * Straight requires stronger centre-region bias rejection; finite-
     * curvature arcs are feedforward-dominant and were stable with the
     * lower PI gains below.
     */
    .straightYawRateKp = 270.0f,
    .straightYawRateKi = 150.0f,
    .straightYawRateKd = 0.0f,
    .straightHeadingKpPerSec = 0.0f,

    .arcYawRateKp = 270.0f,
    .arcYawRateKi = 150.0f,
    .arcYawRateKd = 0.0f,
    .arcHeadingKpPerSec = 0.0f,

    .maxArcSteeringCommandCorrection = 30.0f,

    /* Shared heading-generated curvature bound. */
    .maxPathCorrectionCurvaturePerMm = 1.0f / 1000.0f,

    .wheelSyncKpCpsPerMm = 10.0f,
    .maxWheelSyncCorrectionCps = 100.0f,

    /* Current production-candidate profile from the terminal-arc campaign. */
    .motionAccelerationMmps2 = 2500.0f,
    .motionDecelerationMmps2 = 1000.0f,
    .motionCompletionToleranceMm = 0.5f,

    .arcYawRateFilterTauSec = 0.10f,

    .stopStableSampleCount = 3U,
};
