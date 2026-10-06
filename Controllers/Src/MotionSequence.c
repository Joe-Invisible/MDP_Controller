#include "MotionSequence.h"
#include <math.h>
#include <stddef.h>

#define SEQUENCE_PI 3.14159265358979323846f

bool MotionSequence_IsBusy(const MotionSequence *s)
{
    return s != NULL && s->initialized &&
        (s->state == MOTION_SEQUENCE_RUNNING || s->state == MOTION_SEQUENCE_ABORTING);
}

MotionControllerStatus MotionSequence_Begin(MotionSequence *s,
    MotionController *m, const MotionSequenceConfig *config)
{
    if (s == NULL || m == NULL || config == NULL)
        return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
    if (!m->initialized) return MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;
    if (MotionController_IsBusy(m)) return MOTIONCONTROLLER_STATUS_BUSY;
    if (m->config->useLegacyStraightSteering ||
        !isfinite(config->blendLengthMm) || config->blendLengthMm <= 0.0f ||
        !isfinite(config->junctionSpeedCps) || config->junctionSpeedCps <= 0.0f)
        return MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
    *s = (MotionSequence){0};
    s->controller = m;
    s->config = *config;
    s->initialized = true;
    return MOTIONCONTROLLER_STATUS_OK;
}

static MotionControllerStatus add(MotionSequence *s, float distance,
                                  float curvature, float speed, bool stopAfter)
{
    if (s == NULL) return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
    if (!s->initialized) return MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;
    if (s->state != MOTION_SEQUENCE_BUILDING) return MOTIONCONTROLLER_STATUS_INVALID_STATE;
    if (!isfinite(distance)) return MOTIONCONTROLLER_STATUS_INVALID_DISTANCE;
    if (!isfinite(speed) || speed <= 0.0f) return MOTIONCONTROLLER_STATUS_INVALID_SPEED;
    if (!isfinite(curvature)) return MOTIONCONTROLLER_STATUS_INVALID_RADIUS;
    if (distance == 0.0f) {
        /* A zero-distance stop waypoint still marks the preceding junction. */
        if (stopAfter && s->plan.count > 0U)
            s->plan.segments[s->plan.count - 1U].stopAfter = true;
        return MOTIONCONTROLLER_STATUS_OK;
    }
    if (s->plan.count == MOTION_SEQUENCE_CAPACITY)
        return MOTIONCONTROLLER_STATUS_PROFILE_ERROR;
    float raw;
    MotionControllerStatus status = MotionController_GetProfileFeedforward(
        s->controller, curvature, &raw);
    if (status != MOTIONCONTROLLER_STATUS_OK) return status;
    s->plan.segments[s->plan.count++] = (MotionSequenceSegment){distance, curvature, speed, stopAfter};
    return MOTIONCONTROLLER_STATUS_OK;
}

MotionControllerStatus MotionSequence_AddStraight(MotionSequence *s,
    float signedDistanceMm, float speedCps, bool stopAfter)
{
    return add(s, signedDistanceMm, 0.0f, speedCps, stopAfter);
}

MotionControllerStatus MotionSequence_AddArc(MotionSequence *s,
    float signedDistanceMm, float radiusMm, float speedCps, bool stopAfter)
{
    if (!isfinite(radiusMm) || radiusMm == 0.0f)
        return MOTIONCONTROLLER_STATUS_INVALID_RADIUS;
    return add(s, signedDistanceMm, 1.0f / radiusMm, speedCps, stopAfter);
}

static float rawSlopeBound(const MotionControllerConfig *c)
{
    const MotionControllerArcConfig *a = c->arcConfig;
    float bound = 0.0f;
    for (unsigned branch = 0U; branch < 2U; ++branch) {
        const MotionControllerArcFeedforwardPoint *points = branch == 0U
            ? a->negativePoints : a->positivePoints;
        uint32_t count = branch == 0U ? a->negativePointCount : a->positivePointCount;
        uint32_t near = branch == 0U ? count - 1U : 0U;
        bound = fmaxf(bound, fabsf((points[near].rawSteeringCommand -
            c->straightSteeringFeedforwardCommand) / points[near].curvaturePerMm));
        for (uint32_t i = 1U; i < count; ++i)
            bound = fmaxf(bound, fabsf((points[i].rawSteeringCommand - points[i - 1U].rawSteeringCommand) /
                (points[i].curvaturePerMm - points[i - 1U].curvaturePerMm)));
    }
    return bound;
}

static MotionControllerStatus launchRun(MotionSequence *s, uint32_t first)
{
    s->expectedBraking = false;
    s->run = (MotionSequenceRun){&s->plan, first, MotionSequencePlan_RunEnd(&s->plan, first)};
    float length = 0.0f, maxSpeed = 0.0f;
    for (uint32_t i = first; i <= s->run.last; ++i) {
        length += fabsf(s->plan.segments[i].signedDistanceMm);
        maxSpeed = fmaxf(maxSpeed, s->plan.segments[i].speedCps);
    }
    const MotionSequenceSegment *initial = &s->plan.segments[first];
    const MotionControllerConfig *c = s->controller->config;
    const MotionSequenceSegment *final = &s->plan.segments[s->run.last];
    bool finalArc = s->run.last + 1U == s->plan.count &&
        final->curvaturePerMm != 0.0f &&
        fabsf(final->signedDistanceMm * final->curvaturePerMm) >
            MOTION_PATH_TERMINAL_MIN_YAW_DEG * (SEQUENCE_PI / 180.0f);
    float finalConstantStartMm = length - fabsf(final->signedDistanceMm);
    if (s->run.last > first)
        finalConstantStartMm += s->plan.junctionHalfLengthMm[s->run.last - 1U];
    MotionPathProfile profile = {
        .signedDistanceMm = initial->signedDistanceMm > 0.0f ? length : -length,
        .maxSpeedCps = maxSpeed,
        .steeringSettlingTimeSec = initial->curvaturePerMm == 0.0f
            ? c->straightSteeringSettlingTimeSec : c->arcConfig->steeringSettlingTimeSec,
        .evaluate = MotionSequencePlan_Evaluate,
        .context = &s->run,
        .terminalYawPriority = finalArc,
        .terminalEntryProgressMm = fmaxf(finalConstantStartMm,
            length - MOTION_PATH_TERMINAL_ENTRY_DISTANCE_MM)
    };
    return MotionController_FollowProfile(s->controller, &profile);
}

MotionControllerStatus MotionSequence_Execute(MotionSequence *s)
{
    if (s == NULL) return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
    if (!s->initialized) return MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;
    if (s->state != MOTION_SEQUENCE_BUILDING) return MOTIONCONTROLLER_STATUS_INVALID_STATE;
    if (MotionController_IsBusy(s->controller)) return MOTIONCONTROLLER_STATUS_BUSY;
    const MotionControllerConfig *c = s->controller->config;
    float mmPerCount = SEQUENCE_PI * c->kinematics->rearWheelDiameterMm /
        c->kinematics->rearEncoderCountsPerRev;
    if (!MotionSequencePlan_Prepare(&s->plan, &s->config, mmPerCount,
        c->motionDecelerationMmps2, rawSlopeBound(c),
        s->controller->steering->calibration->maxCommandRatePerSec))
        return MOTIONCONTROLLER_STATUS_PROFILE_ERROR;
    if (s->plan.count == 0U) {
        s->state = MOTION_SEQUENCE_COMPLETE;
        return MOTIONCONTROLLER_STATUS_OK;
    }
    s->lastStatus = launchRun(s, 0U);
    s->state = s->lastStatus == MOTIONCONTROLLER_STATUS_OK
        ? MOTION_SEQUENCE_RUNNING : MOTION_SEQUENCE_FAILED;
    return s->lastStatus;
}

MotionControllerStatus MotionSequence_Update(MotionSequence *s, float dt)
{
    if (s == NULL) return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
    if (!s->initialized) return MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;
    if (!isfinite(dt) || dt <= 0.0f) return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
    if (!MotionSequence_IsBusy(s)) return s->lastStatus;
    /* A direct Stop/Brake by another caller invalidates ownership. It must
     * never be mistaken for successful completion and launch the next run. */
    MotionControllerMode before = s->controller->mode;
    if (before != MOTIONCONTROLLER_PROFILE_PREPARING &&
        before != MOTIONCONTROLLER_PROFILE &&
        !(before == MOTIONCONTROLLER_BRAKING && s->expectedBraking)) {
        s->state = MOTION_SEQUENCE_FAILED;
        s->lastStatus = MOTIONCONTROLLER_STATUS_INVALID_STATE;
        MotionController_Stop(s->controller);
        return s->lastStatus;
    }
    s->lastStatus = MotionController_Update(s->controller, dt);
    if (s->lastStatus != MOTIONCONTROLLER_STATUS_OK) {
        s->state = MOTION_SEQUENCE_FAILED;
        return s->lastStatus;
    }
    s->expectedBraking = s->controller->mode == MOTIONCONTROLLER_BRAKING;
    if (MotionController_IsBusy(s->controller)) return s->lastStatus;
    s->completedMeasuredTravelMm += fabsf(s->controller->travelledDistanceMm);
    s->completedMeasuredYawRad += s->controller->yawDeg * (SEQUENCE_PI / 180.0f);
    if (s->state == MOTION_SEQUENCE_ABORTING) {
        s->state = MOTION_SEQUENCE_ABORTED;
        return s->lastStatus;
    }
    ++s->completedRuns;
    uint32_t next = s->run.last + 1U;
    if (next == s->plan.count) {
        s->state = MOTION_SEQUENCE_COMPLETE;
        return s->lastStatus;
    }
    s->lastStatus = launchRun(s, next);
    if (s->lastStatus != MOTIONCONTROLLER_STATUS_OK) s->state = MOTION_SEQUENCE_FAILED;
    return s->lastStatus;
}

MotionControllerStatus MotionSequence_Brake(MotionSequence *s)
{
    if (s == NULL) return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
    if (!s->initialized) return MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;
    if (!MotionSequence_IsBusy(s)) return MOTIONCONTROLLER_STATUS_OK;
    s->lastStatus = MotionController_Brake(s->controller);
    s->expectedBraking = s->lastStatus == MOTIONCONTROLLER_STATUS_OK;
    s->state = s->lastStatus == MOTIONCONTROLLER_STATUS_OK
        ? MOTION_SEQUENCE_ABORTING : MOTION_SEQUENCE_FAILED;
    return s->lastStatus;
}
