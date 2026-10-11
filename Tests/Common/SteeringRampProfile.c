#include "SteeringRampProfile.h"
#include <math.h>
#include <stddef.h>

#define RAMP_PI 3.14159265358979323846f

static bool AppendKnot(SteeringRampProfile *profile, float raw, float curvature)
{
    if (profile->knotCount >= STEERING_RAMP_KNOT_CAPACITY)
        return false;
    SteeringRampKnot knot = {.rawCommand = raw, .curvaturePerMm = curvature};
    if (profile->knotCount > 0U)
    {
        const SteeringRampKnot *previous = &profile->knots[profile->knotCount - 1U];
        if (raw >= previous->rawCommand || curvature <= previous->curvaturePerMm)
            return false;
        float distance = (previous->rawCommand - raw) * profile->nominalSpeedMmps /
                         profile->trial.nominalRawRatePerSec;
        knot.distanceMm = previous->distanceMm + distance;
        knot.yawRad = previous->yawRad +
                      0.5f * (previous->curvaturePerMm + curvature) * distance;
    }
    profile->knots[profile->knotCount++] = knot;
    return true;
}

bool SteeringRampProfile_Build(SteeringRampProfile *profile,
                              const MotionController *controller,
                              const SteeringRampTrial *trial)
{
    if (profile == NULL || controller == NULL || controller->config == NULL ||
        controller->config->kinematics == NULL || controller->config->arcConfig == NULL ||
        trial == NULL || !isfinite(trial->speedCps) || trial->speedCps <= 0.0f ||
        !isfinite(trial->nominalRawRatePerSec) || trial->nominalRawRatePerSec <= 0.0f ||
        !isfinite(trial->arcRadiusMm) || trial->arcRadiusMm <= 0.0f ||
        !isfinite(trial->leadInMm) || trial->leadInMm <= 0.0f ||
        !isfinite(trial->arcHoldMm) || trial->arcHoldMm <= 0.0f ||
        !isfinite(trial->straightHoldMm) || trial->straightHoldMm <= 0.0f ||
        !isfinite(trial->stoppingAllowanceMm) || trial->stoppingAllowanceMm <= 0.0f)
        return false;

    *profile = (SteeringRampProfile){.trial = *trial};
    const RobotKinematics *geometry = controller->config->kinematics;
    if (geometry->rearEncoderCountsPerRev == 0U)
        return false;
    profile->nominalSpeedMmps = trial->speedCps * RAMP_PI * geometry->rearWheelDiameterMm /
                               geometry->rearEncoderCountsPerRev;
    if (!isfinite(profile->nominalSpeedMmps) || profile->nominalSpeedMmps <= 0.0f)
        return false;
    float endCurvature = 1.0f / trial->arcRadiusMm;
    float endRaw;
    if (MotionController_GetProfileFeedforward(controller, endCurvature, &endRaw) !=
        MOTIONCONTROLLER_STATUS_OK ||
        !AppendKnot(profile, controller->config->straightSteeringFeedforwardCommand, 0.0f))
        return false;

    const MotionControllerArcConfig *arc = controller->config->arcConfig;
    for (uint32_t i = 0U; i < arc->positivePointCount; ++i)
    {
        const MotionControllerArcFeedforwardPoint *point = &arc->positivePoints[i];
        if (point->curvaturePerMm >= endCurvature)
            break;
        if (!AppendKnot(profile, point->rawSteeringCommand, point->curvaturePerMm))
            return false;
    }
    if (!AppendKnot(profile, endRaw, endCurvature))
        return false;
    const SteeringRampKnot *end = &profile->knots[profile->knotCount - 1U];
    profile->rampLengthMm = end->distanceMm;
    profile->totalDistanceMm = trial->leadInMm + 2.0f * profile->rampLengthMm +
                              trial->arcHoldMm + trial->straightHoldMm +
                              trial->stoppingAllowanceMm;
    profile->nominalYawRad = 2.0f * end->yawRad + endCurvature * trial->arcHoldMm;
    return isfinite(profile->totalDistanceMm) && isfinite(profile->nominalYawRad);
}

static SteeringRampKnot RampAt(const SteeringRampProfile *profile, float distanceMm)
{
    float distance = fminf(fmaxf(distanceMm, 0.0f), profile->rampLengthMm);
    for (uint32_t i = 1U; i < profile->knotCount; ++i)
    {
        const SteeringRampKnot *a = &profile->knots[i - 1U];
        const SteeringRampKnot *b = &profile->knots[i];
        if (distance <= b->distanceMm)
        {
            float offset = distance - a->distanceMm;
            float fraction = offset / (b->distanceMm - a->distanceMm);
            float curvature = a->curvaturePerMm + fraction *
                              (b->curvaturePerMm - a->curvaturePerMm);
            return (SteeringRampKnot){
                .distanceMm = distance,
                .rawCommand = a->rawCommand + fraction * (b->rawCommand - a->rawCommand),
                .curvaturePerMm = curvature,
                .yawRad = a->yawRad + 0.5f * (a->curvaturePerMm + curvature) * offset};
        }
    }
    return profile->knots[profile->knotCount - 1U];
}

SteeringRampPhase SteeringRampProfile_Phase(const SteeringRampProfile *profile,
                                          float progressMm)
{
    float distance = fmaxf(progressMm, 0.0f);
    if (distance < profile->trial.leadInMm)
        return STEERING_RAMP_LEAD_IN;
    distance -= profile->trial.leadInMm;
    if (distance < profile->rampLengthMm)
        return STEERING_RAMP_IN;
    distance -= profile->rampLengthMm;
    if (distance < profile->trial.arcHoldMm)
        return STEERING_RAMP_ARC_HOLD;
    distance -= profile->trial.arcHoldMm;
    if (distance < profile->rampLengthMm)
        return STEERING_RAMP_OUT;
    distance -= profile->rampLengthMm;
    return distance < profile->trial.straightHoldMm
               ? STEERING_RAMP_STRAIGHT_HOLD : STEERING_RAMP_STOPPING;
}

bool SteeringRampProfile_Evaluate(const void *context, float progressMm,
                                  MotionPathSample *sample)
{
    const SteeringRampProfile *profile = context;
    if (profile == NULL || sample == NULL || profile->knotCount < 2U ||
        !isfinite(progressMm))
        return false;
    const SteeringRampKnot *end = &profile->knots[profile->knotCount - 1U];
    float distance = fmaxf(progressMm, 0.0f) - profile->trial.leadInMm;
    *sample = (MotionPathSample){.speedLimitCps = profile->trial.speedCps,
                               .straightTuningWeight = 1.0f};
    if (distance < 0.0f)
        return true;
    if (distance < profile->rampLengthMm)
    {
        SteeringRampKnot knot = RampAt(profile, distance);
        sample->curvaturePerMm = knot.curvaturePerMm;
        sample->desiredYawRad = knot.yawRad;
        sample->straightTuningWeight = 1.0f - knot.curvaturePerMm / end->curvaturePerMm;
        return true;
    }
    distance -= profile->rampLengthMm;
    if (distance < profile->trial.arcHoldMm)
    {
        sample->curvaturePerMm = end->curvaturePerMm;
        sample->desiredYawRad = end->yawRad + distance * end->curvaturePerMm;
        sample->straightTuningWeight = 0.0f;
        return true;
    }
    distance -= profile->trial.arcHoldMm;
    if (distance < profile->rampLengthMm)
    {
        SteeringRampKnot knot = RampAt(profile, profile->rampLengthMm - distance);
        sample->curvaturePerMm = knot.curvaturePerMm;
        sample->desiredYawRad = profile->nominalYawRad - knot.yawRad;
    }
    else
        sample->desiredYawRad = profile->nominalYawRad;
    sample->straightTuningWeight = 1.0f - sample->curvaturePerMm / end->curvaturePerMm;
    return true;
}

float SteeringRampProfile_RawCommand(const SteeringRampProfile *profile, float progressMm)
{
    float distance = fmaxf(progressMm, 0.0f) - profile->trial.leadInMm;
    if (distance < 0.0f)
        return profile->knots[0].rawCommand;
    if (distance < profile->rampLengthMm)
        return RampAt(profile, distance).rawCommand;
    distance -= profile->rampLengthMm;
    if (distance < profile->trial.arcHoldMm)
        return profile->knots[profile->knotCount - 1U].rawCommand;
    distance -= profile->trial.arcHoldMm;
    return RampAt(profile, profile->rampLengthMm - distance).rawCommand;
}
