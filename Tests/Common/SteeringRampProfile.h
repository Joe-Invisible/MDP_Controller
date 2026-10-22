#ifndef STEERING_RAMP_PROFILE_H
#define STEERING_RAMP_PROFILE_H

#include "MotionController.h"

#define STEERING_RAMP_KNOT_CAPACITY 16U

typedef enum
{
    STEERING_RAMP_PREPARING = 0,
    STEERING_RAMP_LEAD_IN,
    STEERING_RAMP_IN,
    STEERING_RAMP_ARC_HOLD,
    STEERING_RAMP_OUT,
    STEERING_RAMP_STRAIGHT_HOLD,
    STEERING_RAMP_STOPPING,
    STEERING_RAMP_FINISHED
} SteeringRampPhase;

typedef struct
{
    float distanceMm;
    float rawCommand;
    float curvaturePerMm;
    float yawRad;
} SteeringRampKnot;

typedef struct
{
    float speedCps;
    float nominalRawRatePerSec;
    float arcRadiusMm;
    float leadInMm;
    float arcHoldMm;
    float straightHoldMm;
    float stoppingAllowanceMm;
} SteeringRampTrial;

typedef struct
{
    SteeringRampTrial trial;
    SteeringRampKnot knots[STEERING_RAMP_KNOT_CAPACITY];
    uint32_t knotCount;
    float nominalSpeedMmps;
    float rampLengthMm;
    float totalDistanceMm;
    float nominalYawRad;
} SteeringRampProfile;

/* Pure distance-domain profile: constant raw-command slope during each ramp.
 * The raw/s rate is nominal at trial.speedCps; actual rate is logged by the test.
 * Nominal yaw is integrated analytically through every calibration-table knot. */
bool SteeringRampProfile_Build(SteeringRampProfile *profile,
                               const MotionController *controller,
                               const SteeringRampTrial *trial);
bool SteeringRampProfile_Evaluate(const void *context, float progressMm, MotionPathSample *sample);
SteeringRampPhase SteeringRampProfile_Phase(const SteeringRampProfile *profile, float progressMm);
float SteeringRampProfile_RawCommand(const SteeringRampProfile *profile, float progressMm);

#endif
