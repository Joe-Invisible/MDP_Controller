#include "MotionSequencePlan.h"
#include <math.h>
#include <stddef.h>

/* Deliberately faster fusion experiment. These are configurable software
 * limits, not measured servo physics. Standalone steering keeps its own rate. */
const MotionSequenceConfig motionSequenceConfig = {
    .blendLengthMm = 100.0f,
    .junctionSpeedCps = 2000.0f,
    .steeringCommandRatePerSec = 480.0f,
};

static bool MotionSequencePlan_IsRunBoundary(const MotionSequencePlan *plan, uint32_t i)
{
    return plan->segments[i].stopAfter ||
           ((plan->segments[i].signedDistanceMm > 0.0f) !=
            (plan->segments[i + 1U].signedDistanceMm > 0.0f));
}

bool MotionSequencePlan_Prepare(
    MotionSequencePlan *plan,
    const MotionSequenceConfig *config,
    float mmPerCount,
    float decelerationMmps2,
    float rawSlopeBound,
    float rawRatePerSec)
{
    if (plan == NULL)
    {
        return false;
    }
    plan->prepared = false;
    if (config == NULL || plan->count > MOTION_SEQUENCE_CAPACITY ||
        !isfinite(config->blendLengthMm) || config->blendLengthMm <= 0.0f ||
        !isfinite(config->junctionSpeedCps) || config->junctionSpeedCps <= 0.0f ||
        !isfinite(config->steeringCommandRatePerSec) ||
        config->steeringCommandRatePerSec < 0.0f ||
        !isfinite(mmPerCount) || mmPerCount <= 0.0f || !isfinite(decelerationMmps2) ||
        decelerationMmps2 <= 0.0f || !isfinite(rawSlopeBound) || rawSlopeBound < 0.0f ||
        !isfinite(rawRatePerSec) || rawRatePerSec <= 0.0f)
    {
        return false;
    }

    /* Validate segments and initialise nominal totals before blending. */
    plan->totalTravelMm = 0.0f;
    plan->nominalFinalYawRad = 0.0f;
    plan->mmPerCount = mmPerCount;
    plan->decelerationMmps2 = decelerationMmps2;
    for (uint32_t i = 0U; i < plan->count; ++i)
    {
        const MotionSequenceSegment *segment = &plan->segments[i];
        if (!isfinite(segment->signedDistanceMm) || segment->signedDistanceMm == 0.0f ||
            !isfinite(segment->curvaturePerMm) || !isfinite(segment->speedCps) ||
            segment->speedCps <= 0.0f)
        {
            return false;
        }
        plan->totalTravelMm += fabsf(segment->signedDistanceMm);
        plan->nominalFinalYawRad += segment->signedDistanceMm * segment->curvaturePerMm;
        plan->junctionHalfLengthMm[i] = 0.0f;
        plan->junctionSpeedCps[i] = 0.0f;
        float speedMmps = segment->speedCps * mmPerCount;
        if (!isfinite(
                speedMmps * speedMmps + 2.0f * decelerationMmps2 * plan->totalTravelMm))
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

    /* Size each blend and cap its speed using the same slew budget as execution. */
    for (uint32_t i = 0U; i + 1U < plan->count; ++i)
    {
        if (MotionSequencePlan_IsRunBoundary(plan, i))
        {
            continue;
        }
        const MotionSequenceSegment *previousSegment = &plan->segments[i];
        const MotionSequenceSegment *nextSegment = &plan->segments[i + 1U];
        float halfLengthMm = fminf(
            0.5f * config->blendLengthMm,
            0.25f * fminf(
                        fabsf(previousSegment->signedDistanceMm),
                        fabsf(nextSegment->signedDistanceMm)));
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
        if (curvatureChangePerMm > 0.0f && rawSlopeBound > 0.0f)
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

static float MotionSequencePlan_GetApproachSpeedCps(
    const MotionSequencePlan *plan, float capCps, float distanceMm)
{
    float speedMmps = capCps * plan->mmPerCount;
    return sqrtf(speedMmps * speedMmps + 2.0f * plan->decelerationMmps2 * distanceMm) /
           plan->mmPerCount;
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
    float sectionStartMm = 0.0f;
    float integratedCurvatureRad = 0.0f;
    float speedLimitCps = plan->segments[run->first].speedCps;
    bool found = false;
    *sample = (MotionPathSample){0};

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
                    plan,
                    plan->segments[i].speedCps,
                    fmaxf(0.0f, nominalPositionMm - clampedProgressMm)));
        }

        if (i < run->last)
        {
            float halfLengthMm = plan->junctionHalfLengthMm[i];
            if (clampedProgressMm <= segmentEndMm + halfLengthMm)
            {
                speedLimitCps = fminf(
                    speedLimitCps,
                    MotionSequencePlan_GetApproachSpeedCps(
                        plan,
                        plan->junctionSpeedCps[i],
                        fmaxf(0.0f, segmentEndMm - halfLengthMm - clampedProgressMm)));
            }
        }
        nominalPositionMm = segmentEndMm;
    }

    sample->speedLimitCps = speedLimitCps;
    return isfinite(speedLimitCps) && speedLimitCps > 0.0f;
}
