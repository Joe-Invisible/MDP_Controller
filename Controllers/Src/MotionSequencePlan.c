#include "MotionSequencePlan.h"
#include <math.h>
#include <stddef.h>

/* Deliberately faster fusion experiment. These are configurable software
 * limits, not measured servo physics. Standalone steering keeps its own rate. */
const MotionSequenceConfig motionSequenceConfig = {
    .blendLengthMm = 200.0f,
    .junctionSpeedCps = 8000.0f,
    .steeringCommandRatePerSec = 480.0f,
    .allowBlendOverlap = false,
    .referenceGuardSec = 0.02f,
};

void MotionSequencePlan_SetFeedforward(MotionSequencePlan *plan,
    const MotionControllerArcConfig *config, float straightCommand)
{
    if (plan == NULL)
        return;
    plan->prepared = false;
    plan->feedforwardConfig = config;
    plan->straightFeedforwardCommand = straightCommand;
}

static bool MotionSequencePlan_ValidFeedforward(const MotionSequencePlan *plan)
{
    if (plan->feedforwardConfig == NULL)
        return true;
    if (!isfinite(plan->straightFeedforwardCommand))
        return false;
    const MotionControllerArcConfig *config = plan->feedforwardConfig;
    for (unsigned branch = 0U; branch < 2U; ++branch)
    {
        const MotionControllerArcFeedforwardPoint *points = branch == 0U
            ? config->negativePoints : config->positivePoints;
        uint32_t count = branch == 0U ? config->negativePointCount : config->positivePointCount;
        if (points == NULL || count == 0U)
            return false;
        for (uint32_t i = 0U; i < count; ++i)
            if (!isfinite(points[i].curvaturePerMm) || !isfinite(points[i].rawSteeringCommand) ||
                (branch == 0U ? points[i].curvaturePerMm >= 0.0f : points[i].curvaturePerMm <= 0.0f) ||
                (i > 0U && points[i].curvaturePerMm <= points[i - 1U].curvaturePerMm))
                return false;
    }
    return true;
}

/* Identical centre bridge and interpolation slopes to GetProfileFeedforward.
 * Called strictly inside a calibration interval, rather than at its knots. */
static float MotionSequencePlan_LocalSlope(const MotionSequencePlan *plan, float curvature)
{
    const MotionControllerArcConfig *config = plan->feedforwardConfig;
    bool negative = curvature < 0.0f;
    const MotionControllerArcFeedforwardPoint *points = negative
        ? config->negativePoints : config->positivePoints;
    uint32_t count = negative ? config->negativePointCount : config->positivePointCount;
    const MotionControllerArcFeedforwardPoint *nearZero = &points[negative ? count - 1U : 0U];
    if (fabsf(curvature) <= fabsf(nearZero->curvaturePerMm))
        return fabsf((nearZero->rawSteeringCommand - plan->straightFeedforwardCommand) /
                     nearZero->curvaturePerMm);
    for (uint32_t i = 1U; i < count; ++i)
        if (curvature <= points[i].curvaturePerMm)
            return fabsf((points[i].rawSteeringCommand - points[i - 1U].rawSteeringCommand) /
                         (points[i].curvaturePerMm - points[i - 1U].curvaturePerMm));
    return INFINITY;
}

static bool MotionSequencePlan_IsRunBoundary(const MotionSequencePlan *plan, uint32_t i)
{
    return plan->segments[i].stopAfter ||
           ((plan->segments[i].signedDistanceMm > 0.0f) !=
            (plan->segments[i + 1U].signedDistanceMm > 0.0f));
}

/* Partition a continuous run at all ramp endpoints. Overlap is additive:
 * k(s) = k0 + sum(delta_k * ramp(s)); ramps stay ordered, so the equivalent
 * segment weights are nonnegative. Each interval has constant dk/ds. */
static uint32_t MotionSequencePlan_OverlapEdges(
    const MotionSequenceRun *run, float *edges)
{
    const MotionSequencePlan *plan = run->plan;
    uint32_t count = 1U;
    edges[0] = 0.0f;
    float position = 0.0f;
    for (uint32_t i = run->first; i <= run->last; ++i)
    {
        position += fabsf(plan->segments[i].signedDistanceMm);
        if (i < run->last)
        {
            float half = plan->junctionHalfLengthMm[i];
            edges[count++] = position - half;
            edges[count++] = position + half;
        }
    }
    edges[count++] = position;
    for (uint32_t i = 1U; i < count; ++i)
    {
        float value = edges[i];
        uint32_t j = i;
        while (j > 0U && edges[j - 1U] > value)
        {
            edges[j] = edges[j - 1U];
            --j;
        }
        edges[j] = value;
    }
    return count;
}

static float MotionSequencePlan_OverlapIntervalLimit(
    const MotionSequenceRun *run, float position)
{
    const MotionSequencePlan *plan = run->plan;
    float boundary = 0.0f, slope = 0.0f, cap = INFINITY;
    for (uint32_t i = run->first; i < run->last; ++i)
    {
        boundary += fabsf(plan->segments[i].signedDistanceMm);
        float half = plan->junctionHalfLengthMm[i];
        if (position > boundary - half && position < boundary + half)
        {
            cap = fminf(cap, fminf(plan->segments[i].speedCps,
                                   plan->segments[i + 1U].speedCps));
            float change = plan->segments[i + 1U].curvaturePerMm -
                           plan->segments[i].curvaturePerMm;
            if (change != 0.0f)
                cap = fminf(cap, plan->junctionSpeedCeilingCps);
            slope += change / (2.0f * half);
        }
    }
    /* Cap the SUM of active derivatives. Limiting each ramp separately can
     * exceed the feedforward slew budget when overlapping changes add up. */
    if (slope != 0.0f && plan->feedforwardSlopeBound > 0.0f)
        cap = fminf(cap, 0.5f * plan->steeringRatePerSec /
            (plan->feedforwardSlopeBound * fabsf(slope) * plan->mmPerCount));
    return cap;
}

bool MotionSequencePlan_Prepare(
    MotionSequencePlan *plan,
    const MotionSequenceConfig *config,
    float mmPerCount,
    float straightAccelerationMmps2,
    float straightDecelerationMmps2,
    float arcAccelerationMmps2,
    float arcDecelerationMmps2,
    float rawSlopeBound,
    float rawRatePerSec)
{
    if (plan == NULL)
    {
        return false;
    }
    plan->prepared = false;
    if (config == NULL || !MotionSequencePlan_ValidFeedforward(plan) ||
        !isfinite(config->referenceGuardSec) || config->referenceGuardSec < 0.0f || plan->count > MOTION_SEQUENCE_CAPACITY ||
        !isfinite(config->blendLengthMm) || config->blendLengthMm <= 0.0f ||
        !isfinite(config->junctionSpeedCps) || config->junctionSpeedCps <= 0.0f ||
        !isfinite(config->steeringCommandRatePerSec) ||
        config->steeringCommandRatePerSec < 0.0f ||
        !isfinite(mmPerCount) || mmPerCount <= 0.0f ||
        !isfinite(straightAccelerationMmps2) || straightAccelerationMmps2 <= 0.0f ||
        !isfinite(straightDecelerationMmps2) || straightDecelerationMmps2 <= 0.0f ||
        !isfinite(arcAccelerationMmps2) || arcAccelerationMmps2 <= 0.0f ||
        !isfinite(arcDecelerationMmps2) || arcDecelerationMmps2 <= 0.0f ||
        !isfinite(rawSlopeBound) || rawSlopeBound < 0.0f ||
        !isfinite(rawRatePerSec) || rawRatePerSec <= 0.0f)
    {
        return false;
    }

    /* Validate segments and initialise nominal totals before blending. */
    plan->totalTravelMm = 0.0f;
    plan->nominalFinalYawRad = 0.0f;
    plan->mmPerCount = mmPerCount;
    plan->straightAccelerationMmps2 = straightAccelerationMmps2;
    plan->straightDecelerationMmps2 = straightDecelerationMmps2;
    plan->arcAccelerationMmps2 = arcAccelerationMmps2;
    plan->arcDecelerationMmps2 = arcDecelerationMmps2;
    plan->requestedBlendLengthMm = config->blendLengthMm;
    plan->allowBlendOverlap = config->allowBlendOverlap;
    plan->junctionSpeedCeilingCps = config->junctionSpeedCps;
    plan->feedforwardSlopeBound = rawSlopeBound;
    for (uint32_t i = 0U; i < plan->count; ++i)
    {
        const MotionSequenceSegment *segment = &plan->segments[i];
        if (!isfinite(segment->signedDistanceMm) || segment->signedDistanceMm == 0.0f ||
            !isfinite(segment->curvaturePerMm) || !isfinite(segment->speedCps) ||
            segment->speedCps <= 0.0f ||
            !isfinite(plan->junctionBlendLimitMm[i]) ||
            plan->junctionBlendLimitMm[i] < 0.0f)
        {
            return false;
        }
        if (plan->feedforwardConfig != NULL &&
            (segment->curvaturePerMm < plan->feedforwardConfig->negativePoints[0].curvaturePerMm ||
             segment->curvaturePerMm > plan->feedforwardConfig->positivePoints[
                 plan->feedforwardConfig->positivePointCount - 1U].curvaturePerMm))
            return false;
        plan->totalTravelMm += fabsf(segment->signedDistanceMm);
        plan->nominalFinalYawRad += segment->signedDistanceMm * segment->curvaturePerMm;
        plan->junctionHalfLengthMm[i] = 0.0f;
        plan->junctionSpeedCps[i] = 0.0f;
        float speedMmps = segment->speedCps * mmPerCount;
        if (!isfinite(
                speedMmps * speedMmps +
                2.0f * fmaxf(straightDecelerationMmps2, arcDecelerationMmps2) *
                    plan->totalTravelMm))
        {
            return false;
        }
    }
    if (!isfinite(plan->totalTravelMm) || !isfinite(plan->nominalFinalYawRad))
    {
        return false;
    }

    float effectiveRawRatePerSec = config->steeringCommandRatePerSec > 0.0f
                                      ? config->steeringCommandRatePerSec
                                      : rawRatePerSec;
    plan->steeringRatePerSec = effectiveRawRatePerSec;
    plan->referenceGuardSec = config->referenceGuardSec;

    /* Size each blend and cap its speed using the same slew budget as execution. */
    for (uint32_t i = 0U; i + 1U < plan->count; ++i)
    {
        if (MotionSequencePlan_IsRunBoundary(plan, i))
        {
            continue;
        }
        const MotionSequenceSegment *previousSegment = &plan->segments[i];
        const MotionSequenceSegment *nextSegment = &plan->segments[i + 1U];
        float blendLengthMm = config->blendLengthMm;
        if (plan->junctionBlendLimitMm[i] > 0.0f)
            blendLengthMm = fminf(blendLengthMm, plan->junctionBlendLimitMm[i]);
        float halfLengthMm = fminf(
            0.5f * blendLengthMm,
            0.25f * fminf(
                        fabsf(previousSegment->signedDistanceMm),
                        fabsf(nextSegment->signedDistanceMm)));
        if (config->allowBlendOverlap)
        {
            float before = 0.0f, after = 0.0f;
            uint32_t first = i;
            while (first > 0U && !MotionSequencePlan_IsRunBoundary(plan, first - 1U))
                --first;
            uint32_t last = MotionSequencePlan_RunEnd(plan, i);
            for (uint32_t j = first; j <= i; ++j)
                before += fabsf(plan->segments[j].signedDistanceMm);
            for (uint32_t j = i + 1U; j <= last; ++j)
                after += fabsf(plan->segments[j].signedDistanceMm);
            halfLengthMm = fminf(0.5f * blendLengthMm, fminf(before, after));
        }
        float curvatureChangePerMm =
            fabsf(nextSegment->curvaturePerMm - previousSegment->curvaturePerMm);
        float speedLimitCps = fminf(previousSegment->speedCps, nextSegment->speedCps);
        /* Equal curvature needs no steering transition or junction slowdown. */
        if (curvatureChangePerMm > 0.0f)
        {
            speedLimitCps = fminf(speedLimitCps, config->junctionSpeedCps);
        }
        /* Reserve half the configured software slew rate for feedback.
         * The global piecewise-linear FF slope bound also covers the centre
         * bridge and opposite-sign transitions. No physical servo claim. */
        if (plan->feedforwardConfig == NULL && curvatureChangePerMm > 0.0f && rawSlopeBound > 0.0f)
        {
            speedLimitCps = fminf(
                speedLimitCps,
                0.5f * effectiveRawRatePerSec * (2.0f * halfLengthMm) /
                    (rawSlopeBound * curvatureChangePerMm * mmPerCount));
        }
        if (!isfinite(speedLimitCps) || speedLimitCps <= 0.0f || halfLengthMm <= 0.0f)
        {
            return false;
        }
        plan->junctionHalfLengthMm[i] = halfLengthMm;
        plan->junctionSpeedCps[i] = speedLimitCps;
    }

    if (config->allowBlendOverlap && plan->feedforwardConfig == NULL)
    {
        for (uint32_t first = 0U; first < plan->count;)
        {
            MotionSequenceRun run = {plan, first, MotionSequencePlan_RunEnd(plan, first)};
            float edges[2U * MOTION_SEQUENCE_CAPACITY];
            uint32_t count = MotionSequencePlan_OverlapEdges(&run, edges);
            float boundary = 0.0f;
            for (uint32_t i = first; i < run.last; ++i)
            {
                boundary += fabsf(plan->segments[i].signedDistanceMm);
                float half = plan->junctionHalfLengthMm[i];
                float cap = INFINITY;
                for (uint32_t j = 0U; j + 1U < count; ++j)
                {
                    if (edges[j + 1U] > edges[j] &&
                        edges[j] < boundary + half && edges[j + 1U] > boundary - half)
                        cap = fminf(cap, MotionSequencePlan_OverlapIntervalLimit(
                            &run, 0.5f * (edges[j] + edges[j + 1U])));
                }
                if (!isfinite(cap) || cap <= 0.0f)
                    return false;
                plan->junctionSpeedCps[i] = cap; /* Worst cap in this ramp, for diagnostics. */
            }
            first = run.last + 1U;
        }
    }

    plan->prepared = true;
    return true;
}

uint32_t MotionSequencePlan_RunEnd(const MotionSequencePlan *plan, uint32_t first)
{
    if (plan == NULL || first >= plan->count)
    {
        return first;
    }
    uint32_t last = first;
    while (last + 1U < plan->count && !MotionSequencePlan_IsRunBoundary(plan, last))
    {
        ++last;
    }
    return last;
}

/* Integrate deceleration over the actual constant sections and blends. Run
 * coordinates exclude earlier stopped/reversed runs; their blends are zero. */
static float MotionSequencePlan_DecelerationIntegral(
    const MotionSequenceRun *run, float fromMm, float toMm)
{
    const MotionSequencePlan *plan = run->plan;
    if (plan->allowBlendOverlap)
        return fminf(plan->straightDecelerationMmps2, plan->arcDecelerationMmps2) *
               fmaxf(0.0f, toMm - fromMm);
    float positionMm = 0.0f;
    float integral = 0.0f;
    for (uint32_t i = run->first; i <= run->last; ++i)
    {
        float incomingMm = i > run->first ? plan->junctionHalfLengthMm[i - 1U] : 0.0f;
        float outgoingMm = i < run->last ? plan->junctionHalfLengthMm[i] : 0.0f;
        float constantEndMm = positionMm +
            fabsf(plan->segments[i].signedDistanceMm) - incomingMm - outgoingMm;
        float constantDeceleration = plan->segments[i].curvaturePerMm == 0.0f
                                         ? plan->straightDecelerationMmps2
                                         : plan->arcDecelerationMmps2;
        integral += constantDeceleration * fmaxf(
            0.0f, fminf(toMm, constantEndMm) - fmaxf(fromMm, positionMm));
        positionMm = constantEndMm;
        float blendEndMm = positionMm + 2.0f * outgoingMm;
        /* Equal straight curvature needs no arc dynamics in the blend. */
        float blendDeceleration = i < run->last &&
            plan->segments[i].curvaturePerMm == 0.0f &&
            plan->segments[i + 1U].curvaturePerMm == 0.0f
                ? plan->straightDecelerationMmps2 : plan->arcDecelerationMmps2;
        integral += blendDeceleration * fmaxf(
            0.0f, fminf(toMm, blendEndMm) - fmaxf(fromMm, positionMm));
        positionMm = blendEndMm;
    }
    return integral;
}

static float MotionSequencePlan_GetApproachSpeedCps(
    const MotionSequenceRun *run, float capCps, float fromMm, float toMm)
{
    const MotionSequencePlan *plan = run->plan;
    float speedMmps = capCps * plan->mmPerCount;
    return sqrtf(speedMmps * speedMmps +
                 2.0f * MotionSequencePlan_DecelerationIntegral(run, fromMm, toMm)) /
           plan->mmPerCount;
}

/* Curvature is affine between ramp endpoints, even with overlap. Summing
 * active derivatives permits cancellation and catches reinforcing ramps. */
static void MotionSequencePlan_IntervalGeometry(const MotionSequenceRun *run,
    float progress, float *curvature, float *derivative, float *baseCap)
{
    const MotionSequencePlan *plan = run->plan;
    float boundary = 0.0f;
    *curvature = plan->segments[run->first].curvaturePerMm;
    *derivative = 0.0f;
    *baseCap = INFINITY;
    for (uint32_t i = run->first; i < run->last; ++i)
    {
        boundary += fabsf(plan->segments[i].signedDistanceMm);
        float half = plan->junctionHalfLengthMm[i];
        float change = plan->segments[i + 1U].curvaturePerMm - plan->segments[i].curvaturePerMm;
        float fraction = fmaxf(0.0f, fminf(1.0f, (progress - boundary + half) / (2.0f * half)));
        *curvature += change * fraction;
        if (progress > boundary - half && progress < boundary + half)
        {
            *derivative += change / (2.0f * half);
            *baseCap = fminf(*baseCap, fminf(plan->segments[i].speedCps,
                                           plan->segments[i + 1U].speedCps));
            if (change != 0.0f)
                *baseCap = fminf(*baseCap, plan->junctionSpeedCeilingCps);
        }
    }
}

/* Split at every FF knot, including zero. Each resulting interval has an
 * exact constant d(raw)/ds. No coarse sampling can miss a narrow centre bridge. */
static float MotionSequencePlan_NextFeedforwardEdge(const MotionSequencePlan *plan,
    float from, float to, float curvatureAtFrom, float derivative, float cursor)
{
    float next = to;
    const MotionControllerArcConfig *config = plan->feedforwardConfig;
    for (unsigned branch = 0U; branch < 3U; ++branch)
    {
        const MotionControllerArcFeedforwardPoint *points = branch == 0U
            ? config->negativePoints : config->positivePoints;
        uint32_t count = branch == 0U ? config->negativePointCount : config->positivePointCount;
        if (branch == 2U)
            count = 1U;
        for (uint32_t i = 0U; i < count; ++i)
        {
            float knot = branch == 2U ? 0.0f : points[i].curvaturePerMm;
            float position = from + (knot - curvatureAtFrom) / derivative;
            if (position > cursor && position < next)
                next = position;
        }
    }
    return next;
}

static float MotionSequencePlan_LocalSteeringLimit(const MotionSequenceRun *run, float progress)
{
    const MotionSequencePlan *plan = run->plan;
    float edges[2U * MOTION_SEQUENCE_CAPACITY];
    uint32_t count = MotionSequencePlan_OverlapEdges(run, edges);
    float limit = INFINITY;
    for (uint32_t i = 0U; i + 1U < count; ++i)
    {
        float from = edges[i], to = edges[i + 1U];
        if (to <= from)
            continue;
        float curvature, derivative, baseCap;
        MotionSequencePlan_IntervalGeometry(run, 0.5f * (from + to),
                                           &curvature, &derivative, &baseCap);
        if (!isfinite(baseCap))
            continue;
        if (derivative == 0.0f)
        {
            if (progress > to)
                continue;
            limit = fminf(limit, MotionSequencePlan_GetApproachSpeedCps(
                run, baseCap, progress, fmaxf(progress, from)));
            continue;
        }
        float curvatureAtFrom = curvature - 0.5f * (to - from) * derivative;
        for (float cursor = from; cursor < to;)
        {
            float next = MotionSequencePlan_NextFeedforwardEdge(
                plan, from, to, curvatureAtFrom, derivative, cursor);
            {
                float middleCurvature = curvatureAtFrom +
                    (0.5f * (cursor + next) - from) * derivative;
                float rawDerivative = MotionSequencePlan_LocalSlope(plan, middleCurvature) * fabsf(derivative);
                float cap = rawDerivative > 0.0f ? fminf(baseCap,
                    0.5f * plan->steeringRatePerSec / (rawDerivative * plan->mmPerCount)) : baseCap;
                /* Protect calibration knots against one held reference step.
                 * The guard uses this local cap, not maximum cruise speed, so
                 * a narrow steep bridge does not slow an entire long blend. */
                float guard = cap * plan->mmPerCount * plan->referenceGuardSec +
                    0.5f * fmaxf(plan->straightDecelerationMmps2, plan->arcDecelerationMmps2) *
                    plan->referenceGuardSec * plan->referenceGuardSec;
                if (progress <= next + guard)
                    limit = fminf(limit, MotionSequencePlan_GetApproachSpeedCps(
                        run, cap, progress, fmaxf(progress, cursor - guard)));
            }
            cursor = next;
        }
    }
    return limit;
}

static bool MotionSequencePlan_EvaluateOverlap(
    const MotionSequenceRun *run, float progress, float length, MotionPathSample *sample)
{
    const MotionSequencePlan *plan = run->plan;
    *sample = (MotionPathSample){0};
    sample->profileLimitsProvided = true;
    float boundary = 0.0f, previousFraction = 1.0f;
    float heading = plan->segments[run->first].curvaturePerMm * progress;
    for (uint32_t i = run->first; i <= run->last; ++i)
    {
        float fraction = 0.0f;
        if (i < run->last)
        {
            boundary += fabsf(plan->segments[i].signedDistanceMm);
            float half = plan->junctionHalfLengthMm[i];
            float offset = progress - (boundary - half);
            fraction = fmaxf(0.0f, fminf(1.0f, offset / (2.0f * half)));
            float area = offset <= 0.0f ? 0.0f :
                offset < 2.0f * half ? offset * offset / (4.0f * half) :
                progress - boundary;
            heading += (plan->segments[i + 1U].curvaturePerMm -
                        plan->segments[i].curvaturePerMm) * area;
        }
        float weight = fmaxf(0.0f, previousFraction - fraction);
        sample->curvaturePerMm += weight * plan->segments[i].curvaturePerMm;
        if (plan->segments[i].curvaturePerMm == 0.0f)
            sample->straightTuningWeight += weight;
        previousFraction = fraction;
    }
    sample->desiredYawRad = heading *
        (plan->segments[run->first].signedDistanceMm > 0.0f ? 1.0f : -1.0f);
    sample->straightTuningWeight = fminf(1.0f, sample->straightTuningWeight);
    bool pureStraight = sample->straightTuningWeight >= 1.0f;
    sample->accelerationMmps2 = pureStraight ? plan->straightAccelerationMmps2 :
        fminf(plan->straightAccelerationMmps2, plan->arcAccelerationMmps2);
    /* Conservative braking across overlapping straight/arc contributions. */
    sample->decelerationMmps2 =
        fminf(plan->straightDecelerationMmps2, plan->arcDecelerationMmps2);

    float cap = INFINITY, position = 0.0f;
    for (uint32_t i = run->first; i <= run->last; ++i)
    {
        float end = position + fabsf(plan->segments[i].signedDistanceMm);
        if (progress <= end || i == run->last)
            cap = fminf(cap, MotionSequencePlan_GetApproachSpeedCps(
                run, plan->segments[i].speedCps, progress, fmaxf(progress, position)));
        position = end;
    }
    if (plan->feedforwardConfig != NULL)
        cap = fminf(cap, MotionSequencePlan_LocalSteeringLimit(run, progress));
    else
    {
        float edges[2U * MOTION_SEQUENCE_CAPACITY];
        uint32_t count = MotionSequencePlan_OverlapEdges(run, edges);
        for (uint32_t i = 0U; i + 1U < count; ++i)
        {
            if (edges[i + 1U] <= edges[i] || progress > edges[i + 1U])
                continue;
            float intervalCap = MotionSequencePlan_OverlapIntervalLimit(
                run, 0.5f * (edges[i] + edges[i + 1U]));
            if (isfinite(intervalCap))
                cap = fminf(cap, MotionSequencePlan_GetApproachSpeedCps(
                    run, intervalCap, progress, fmaxf(progress, edges[i])));
        }
    }
    sample->speedLimitCps = cap;
    sample->brakingSpeedLimitCps =
        MotionSequencePlan_GetApproachSpeedCps(run, 0.0f, progress, length);
    return isfinite(cap) && cap > 0.0f &&
           isfinite(sample->curvaturePerMm) && isfinite(sample->desiredYawRad);
}

bool MotionSequencePlan_Evaluate(
    const void *context, float progressMm, MotionPathSample *sample)
{
    const MotionSequenceRun *run = context;
    if (run == NULL || sample == NULL || run->plan == NULL || !run->plan->prepared ||
        !isfinite(progressMm) || run->first > run->last ||
        run->last >= run->plan->count ||
        MotionSequencePlan_RunEnd(run->plan, run->first) != run->last)
    {
        return false;
    }

    const MotionSequencePlan *plan = run->plan;
    float runLengthMm = 0.0f;
    for (uint32_t i = run->first; i <= run->last; ++i)
    {
        runLengthMm += fabsf(plan->segments[i].signedDistanceMm);
    }

    float clampedProgressMm = fmaxf(0.0f, fminf(progressMm, runLengthMm));
    if (plan->allowBlendOverlap)
        return MotionSequencePlan_EvaluateOverlap(run, clampedProgressMm, runLengthMm, sample);
    float sectionStartMm = 0.0f;
    float integratedCurvatureRad = 0.0f;
    float speedLimitCps = plan->segments[run->first].speedCps;
    bool found = false;
    *sample = (MotionPathSample){0};
    sample->profileLimitsProvided = true;

    /* Emit constant sections and symmetric linear ramps. Analytic integrals
     * keep heading independent of update rate and encoder sample skipping. */
    for (uint32_t i = run->first; i <= run->last; ++i)
    {
        const MotionSequenceSegment *segment = &plan->segments[i];
        float incomingHalfLengthMm =
            i > run->first ? plan->junctionHalfLengthMm[i - 1U] : 0.0f;
        float outgoingHalfLengthMm =
            i < run->last ? plan->junctionHalfLengthMm[i] : 0.0f;
        float constantLengthMm = fabsf(segment->signedDistanceMm) -
                                 incomingHalfLengthMm - outgoingHalfLengthMm;
        if (!found &&
            (clampedProgressMm <= sectionStartMm + constantLengthMm || i == run->last))
        {
            sample->curvaturePerMm = segment->curvaturePerMm;
            sample->desiredYawRad =
                integratedCurvatureRad +
                segment->curvaturePerMm *
                    fmaxf(
                        0.0f,
                        fminf(clampedProgressMm - sectionStartMm, constantLengthMm));
            sample->straightTuningWeight =
                segment->curvaturePerMm == 0.0f ? 1.0f : 0.0f;
            sample->accelerationMmps2 = segment->curvaturePerMm == 0.0f
                                           ? plan->straightAccelerationMmps2
                                           : plan->arcAccelerationMmps2;
            sample->decelerationMmps2 = segment->curvaturePerMm == 0.0f
                                           ? plan->straightDecelerationMmps2
                                           : plan->arcDecelerationMmps2;
            found = true;
        }

        integratedCurvatureRad += segment->curvaturePerMm * constantLengthMm;
        sectionStartMm += constantLengthMm;

        if (i < run->last)
        {
            float blendLengthMm = 2.0f * outgoingHalfLengthMm;
            float startCurvaturePerMm = segment->curvaturePerMm;
            float endCurvaturePerMm = plan->segments[i + 1U].curvaturePerMm;
            if (!found && clampedProgressMm <= sectionStartMm + blendLengthMm)
            {
                float blendFraction = fmaxf(
                    0.0f,
                    fminf(1.0f, (clampedProgressMm - sectionStartMm) / blendLengthMm));
                sample->curvaturePerMm = fmaxf(
                    fminf(startCurvaturePerMm, endCurvaturePerMm),
                    fminf(
                        fmaxf(startCurvaturePerMm, endCurvaturePerMm),
                        startCurvaturePerMm +
                            (endCurvaturePerMm - startCurvaturePerMm) * blendFraction));
                sample->desiredYawRad =
                    integratedCurvatureRad +
                    blendLengthMm * (startCurvaturePerMm * blendFraction +
                                     0.5f * (endCurvaturePerMm - startCurvaturePerMm) *
                                         blendFraction * blendFraction);
                float startStraightWeight = startCurvaturePerMm == 0.0f ? 1.0f : 0.0f;
                float endStraightWeight = endCurvaturePerMm == 0.0f ? 1.0f : 0.0f;
                sample->straightTuningWeight =
                    startStraightWeight +
                    (endStraightWeight - startStraightWeight) * blendFraction;
                bool straightBlend = startCurvaturePerMm == 0.0f &&
                                     endCurvaturePerMm == 0.0f;
                sample->accelerationMmps2 = straightBlend
                                               ? plan->straightAccelerationMmps2
                                               : plan->arcAccelerationMmps2;
                sample->decelerationMmps2 = straightBlend
                                               ? plan->straightDecelerationMmps2
                                               : plan->arcDecelerationMmps2;
                found = true;
            }

            integratedCurvatureRad +=
                0.5f * (startCurvaturePerMm + endCurvaturePerMm) * blendLengthMm;
            sectionStartMm += blendLengthMm;
        }
    }

    if (!found)
    {
        return false;
    }
    sample->desiredYawRad *=
        plan->segments[run->first].signedDistanceMm > 0.0f ? 1.0f : -1.0f;

    /* Look ahead to every lower segment cap and every blend interval.
     * Current/earlier constraints are released only after their interval. */
    float nominalPositionMm = 0.0f;
    speedLimitCps = INFINITY;
    for (uint32_t i = run->first; i <= run->last; ++i)
    {
        float segmentEndMm =
            nominalPositionMm + fabsf(plan->segments[i].signedDistanceMm);
        if (clampedProgressMm <= segmentEndMm || i == run->last)
        {
            speedLimitCps = fminf(
                speedLimitCps,
                MotionSequencePlan_GetApproachSpeedCps(
                    run,
                    plan->segments[i].speedCps,
                    clampedProgressMm,
                    fmaxf(clampedProgressMm, nominalPositionMm)));
        }

        if (i < run->last && plan->feedforwardConfig == NULL)
        {
            float halfLengthMm = plan->junctionHalfLengthMm[i];
            if (clampedProgressMm <= segmentEndMm + halfLengthMm)
            {
                speedLimitCps = fminf(
                    speedLimitCps,
                    MotionSequencePlan_GetApproachSpeedCps(
                        run,
                        plan->junctionSpeedCps[i],
                        clampedProgressMm,
                        fmaxf(clampedProgressMm, segmentEndMm - halfLengthMm)));
            }
        }
        nominalPositionMm = segmentEndMm;
    }

    if (plan->feedforwardConfig != NULL)
        speedLimitCps = fminf(speedLimitCps, MotionSequencePlan_LocalSteeringLimit(run, clampedProgressMm));
    sample->speedLimitCps = speedLimitCps;
    sample->brakingSpeedLimitCps = MotionSequencePlan_GetApproachSpeedCps(
        run, 0.0f, clampedProgressMm, runLengthMm);
    return isfinite(speedLimitCps) && speedLimitCps > 0.0f;
}
