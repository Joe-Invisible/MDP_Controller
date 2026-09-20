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

#define ARC_STEERING_SETTLING_TIME_SEC  (0.5f)

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

    .headingKp = 1.2f,
    .headingKi = 0.05f,
    .headingKd = 0.0f,
    .maxHeadingSteeringAngleRad = 0.015f,

    .arcYawRateKp = 10.0f,
    .arcYawRateKi = 0.0f,
    .arcYawRateKd = 0.0f,
    .maxArcSteeringCommandCorrection = 5.0f,

    .arcHeadingKpPerSec = 1.0f,

    .wheelSyncKpCpsPerMm = 10.0f,
    .maxWheelSyncCorrectionCps = 100.0f,

    .motionAccelerationMmps2 = 500.0f,
    .motionDecelerationMmps2 = 250.0f,
    .motionCompletionToleranceMm = 0.5f,

    .arcYawRateFilterTauSec = 0.10f,

    .stopStableSampleCount = 3U,
};
