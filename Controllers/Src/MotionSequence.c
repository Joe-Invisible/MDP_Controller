#include "MotionSequence.h"
#include <math.h>
#include <stddef.h>

#define SEQUENCE_PI 3.14159265358979323846f

bool MotionSequence_IsBusy(const MotionSequence *sequence)
{
    return sequence != NULL && sequence->initialized &&
           (sequence->state == MOTION_SEQUENCE_RUNNING ||
            sequence->state == MOTION_SEQUENCE_ABORTING);
}

MotionControllerStatus MotionSequence_Begin(
    MotionSequence *sequence,
    MotionController *controller,
    const MotionSequenceConfig *config)
{
    if (sequence == NULL || controller == NULL || config == NULL)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
    }
    if (!controller->initialized)
    {
        return MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;
    }
    if (MotionController_IsBusy(controller))
    {
        return MOTIONCONTROLLER_STATUS_BUSY;
    }
    if (controller->config->useLegacyStraightSteering ||
        !isfinite(config->blendLengthMm) || config->blendLengthMm <= 0.0f ||
        !isfinite(config->junctionSpeedCps) || config->junctionSpeedCps <= 0.0f ||
        !isfinite(config->steeringCommandRatePerSec) ||
        config->steeringCommandRatePerSec < 0.0f)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
    }

    *sequence = (MotionSequence){0};
    sequence->controller = controller;
    sequence->config = *config;
    sequence->initialized = true;
    return MOTIONCONTROLLER_STATUS_OK;
}

static MotionControllerStatus MotionSequence_AddSegment(
    MotionSequence *sequence,
    float signedDistanceMm,
    float curvaturePerMm,
    float speedCps,
    bool stopAfter)
{
    if (sequence == NULL)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
    }
    if (!sequence->initialized)
    {
        return MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;
    }
    if (sequence->state != MOTION_SEQUENCE_BUILDING)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_STATE;
    }
    if (!isfinite(signedDistanceMm))
    {
        return MOTIONCONTROLLER_STATUS_INVALID_DISTANCE;
    }
    if (!isfinite(speedCps) || speedCps <= 0.0f)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_SPEED;
    }
    if (!isfinite(curvaturePerMm))
    {
        return MOTIONCONTROLLER_STATUS_INVALID_RADIUS;
    }

    if (signedDistanceMm == 0.0f)
    {
        /* A zero-distance stop waypoint still marks the preceding junction. */
        if (stopAfter && sequence->plan.count > 0U)
        {
            sequence->plan.segments[sequence->plan.count - 1U].stopAfter = true;
        }
        return MOTIONCONTROLLER_STATUS_OK;
    }
    if (sequence->plan.count == MOTION_SEQUENCE_CAPACITY)
    {
        return MOTIONCONTROLLER_STATUS_PROFILE_ERROR;
    }

    float rawSteeringCommand;
    MotionControllerStatus status = MotionController_GetProfileFeedforward(
        sequence->controller, curvaturePerMm, &rawSteeringCommand);
    if (status != MOTIONCONTROLLER_STATUS_OK)
    {
        return status;
    }

    sequence->plan.segments[sequence->plan.count++] =
        (MotionSequenceSegment){signedDistanceMm, curvaturePerMm, speedCps, stopAfter};
    return MOTIONCONTROLLER_STATUS_OK;
}

MotionControllerStatus MotionSequence_AddStraight(
    MotionSequence *sequence, float signedDistanceMm, float speedCps, bool stopAfter)
{
    return MotionSequence_AddSegment(
        sequence, signedDistanceMm, 0.0f, speedCps, stopAfter);
}

MotionControllerStatus MotionSequence_AddArc(
    MotionSequence *sequence,
    float signedDistanceMm,
    float radiusMm,
    float speedCps,
    bool stopAfter)
{
    if (!isfinite(radiusMm) || radiusMm == 0.0f)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_RADIUS;
    }
    return MotionSequence_AddSegment(
        sequence, signedDistanceMm, 1.0f / radiusMm, speedCps, stopAfter);
}

static float MotionSequence_GetRawSlopeBound(const MotionControllerConfig *config)
{
    const MotionControllerArcConfig *arcConfig = config->arcConfig;
    float bound = 0.0f;
    for (unsigned branch = 0U; branch < 2U; ++branch)
    {
        const MotionControllerArcFeedforwardPoint *points =
            branch == 0U ? arcConfig->negativePoints : arcConfig->positivePoints;
        uint32_t count = branch == 0U ? arcConfig->negativePointCount
                                      : arcConfig->positivePointCount;
        uint32_t nearestZeroPointIndex = branch == 0U ? count - 1U : 0U;
        bound = fmaxf(
            bound,
            fabsf(
                (points[nearestZeroPointIndex].rawSteeringCommand -
                 config->straightSteeringFeedforwardCommand) /
                points[nearestZeroPointIndex].curvaturePerMm));
        for (uint32_t i = 1U; i < count; ++i)
        {
            bound = fmaxf(
                bound,
                fabsf(
                    (points[i].rawSteeringCommand - points[i - 1U].rawSteeringCommand) /
                    (points[i].curvaturePerMm - points[i - 1U].curvaturePerMm)));
        }
    }
    return bound;
}

static MotionControllerStatus MotionSequence_LaunchRun(
    MotionSequence *sequence, uint32_t first)
{
    sequence->expectedBraking = false;
    sequence->run = (MotionSequenceRun){
        &sequence->plan, first, MotionSequencePlan_RunEnd(&sequence->plan, first)};

    float runLengthMm = 0.0f;
    float maxSpeedCps = 0.0f;
    for (uint32_t i = first; i <= sequence->run.last; ++i)
    {
        runLengthMm += fabsf(sequence->plan.segments[i].signedDistanceMm);
        maxSpeedCps = fmaxf(maxSpeedCps, sequence->plan.segments[i].speedCps);
    }

    /* Choose preparation and terminal policy for this continuous run. */
    const MotionSequenceSegment *initialSegment = &sequence->plan.segments[first];
    const MotionControllerConfig *config = sequence->controller->config;
    const MotionSequenceSegment *finalSegment =
        &sequence->plan.segments[sequence->run.last];
    bool terminalYawPriority =
        sequence->run.last + 1U == sequence->plan.count &&
        finalSegment->curvaturePerMm != 0.0f &&
        fabsf(finalSegment->signedDistanceMm * finalSegment->curvaturePerMm) >
            MOTION_PATH_TERMINAL_MIN_YAW_DEG * (SEQUENCE_PI / 180.0f);
    float finalConstantStartMm = runLengthMm - fabsf(finalSegment->signedDistanceMm);
    if (sequence->run.last > first)
    {
        finalConstantStartMm +=
            sequence->plan.junctionHalfLengthMm[sequence->run.last - 1U];
    }

    MotionPathProfile profile = {
        .signedDistanceMm =
            initialSegment->signedDistanceMm > 0.0f ? runLengthMm : -runLengthMm,
        .maxSpeedCps = maxSpeedCps,
        .steeringSettlingTimeSec = initialSegment->curvaturePerMm == 0.0f
                                       ? config->straightSteeringSettlingTimeSec
                                       : config->arcConfig->steeringSettlingTimeSec,
        .evaluate = MotionSequencePlan_Evaluate,
        .context = &sequence->run,
        .terminalYawPriority = terminalYawPriority,
        .terminalEntryProgressMm = fmaxf(
            finalConstantStartMm,
            runLengthMm - MOTION_PATH_TERMINAL_ENTRY_DISTANCE_MM),
        .steeringCommandRatePerSec = sequence->config.steeringCommandRatePerSec,
    };
    return MotionController_FollowProfile(sequence->controller, &profile);
}

MotionControllerStatus MotionSequence_Execute(MotionSequence *sequence)
{
    if (sequence == NULL)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
    }
    if (!sequence->initialized)
    {
        return MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;
    }
    if (sequence->state != MOTION_SEQUENCE_BUILDING)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_STATE;
    }
    if (MotionController_IsBusy(sequence->controller))
    {
        return MOTIONCONTROLLER_STATUS_BUSY;
    }
    const MotionControllerConfig *config = sequence->controller->config;
    float mmPerCount = SEQUENCE_PI * config->kinematics->rearWheelDiameterMm /
                       config->kinematics->rearEncoderCountsPerRev;
    if (!MotionSequencePlan_Prepare(
            &sequence->plan,
            &sequence->config,
            mmPerCount,
            config->straightAccelerationMmps2,
            config->straightDecelerationMmps2,
            config->arcAccelerationMmps2,
            config->arcDecelerationMmps2,
            MotionSequence_GetRawSlopeBound(config),
            sequence->controller->steering->calibration->maxCommandRatePerSec))
    {
        return MOTIONCONTROLLER_STATUS_PROFILE_ERROR;
    }

    if (sequence->plan.count == 0U)
    {
        sequence->state = MOTION_SEQUENCE_COMPLETE;
        return MOTIONCONTROLLER_STATUS_OK;
    }

    sequence->lastStatus = MotionSequence_LaunchRun(sequence, 0U);
    sequence->state = sequence->lastStatus == MOTIONCONTROLLER_STATUS_OK
                          ? MOTION_SEQUENCE_RUNNING
                          : MOTION_SEQUENCE_FAILED;
    return sequence->lastStatus;
}

MotionControllerStatus MotionSequence_Update(MotionSequence *sequence, float dt)
{
    if (sequence == NULL)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
    }
    if (!sequence->initialized)
    {
        return MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(dt) || dt <= 0.0f)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
    }
    if (!MotionSequence_IsBusy(sequence))
    {
        return sequence->lastStatus;
    }

    /* A direct Stop/Brake by another caller invalidates ownership. It must
     * never be mistaken for successful completion and launch the next run. */
    MotionControllerMode modeBeforeUpdate = sequence->controller->mode;
    if (modeBeforeUpdate != MOTIONCONTROLLER_PROFILE_PREPARING &&
        modeBeforeUpdate != MOTIONCONTROLLER_PROFILE &&
        !(modeBeforeUpdate == MOTIONCONTROLLER_BRAKING && sequence->expectedBraking))
    {
        sequence->state = MOTION_SEQUENCE_FAILED;
        sequence->lastStatus = MOTIONCONTROLLER_STATUS_INVALID_STATE;
        MotionController_Stop(sequence->controller);
        return sequence->lastStatus;
    }

    sequence->lastStatus = MotionController_Update(sequence->controller, dt);
    if (sequence->lastStatus != MOTIONCONTROLLER_STATUS_OK)
    {
        sequence->state = MOTION_SEQUENCE_FAILED;
        return sequence->lastStatus;
    }

    sequence->expectedBraking = sequence->controller->mode == MOTIONCONTROLLER_BRAKING;
    if (MotionController_IsBusy(sequence->controller))
    {
        return sequence->lastStatus;
    }

    /* Accumulate the finished run, including its braking drift. */
    sequence->completedMeasuredTravelMm +=
        fabsf(sequence->controller->travelledDistanceMm);
    sequence->completedMeasuredYawRad +=
        sequence->controller->yawDeg * (SEQUENCE_PI / 180.0f);
    if (sequence->state == MOTION_SEQUENCE_ABORTING)
    {
        sequence->state = MOTION_SEQUENCE_ABORTED;
        return sequence->lastStatus;
    }

    ++sequence->completedRuns;
    uint32_t nextSegmentIndex = sequence->run.last + 1U;
    if (nextSegmentIndex == sequence->plan.count)
    {
        sequence->state = MOTION_SEQUENCE_COMPLETE;
        return sequence->lastStatus;
    }

    sequence->lastStatus = MotionSequence_LaunchRun(sequence, nextSegmentIndex);
    if (sequence->lastStatus != MOTIONCONTROLLER_STATUS_OK)
    {
        sequence->state = MOTION_SEQUENCE_FAILED;
    }
    return sequence->lastStatus;
}

MotionControllerStatus MotionSequence_Brake(MotionSequence *sequence)
{
    if (sequence == NULL)
    {
        return MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT;
    }
    if (!sequence->initialized)
    {
        return MOTIONCONTROLLER_STATUS_NOT_INITIALIZED;
    }
    if (!MotionSequence_IsBusy(sequence))
    {
        return MOTIONCONTROLLER_STATUS_OK;
    }
    sequence->lastStatus = MotionController_Brake(sequence->controller);
    sequence->expectedBraking = sequence->lastStatus == MOTIONCONTROLLER_STATUS_OK;
    sequence->state = sequence->lastStatus == MOTIONCONTROLLER_STATUS_OK
                          ? MOTION_SEQUENCE_ABORTING
                          : MOTION_SEQUENCE_FAILED;
    return sequence->lastStatus;
}
