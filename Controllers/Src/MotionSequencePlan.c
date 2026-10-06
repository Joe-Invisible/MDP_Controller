#include "MotionSequencePlan.h"
#include <math.h>
#include <stddef.h>

/* Deliberately conservative starting experiment: 200 mm transition at no
 * more than 500 CPS. These are configurable protections, not servo physics. */
const MotionSequenceConfig motionSequenceConfig = {200.0f, 500.0f};

static bool boundary(const MotionSequencePlan *p, uint32_t i)
{
    return p->segments[i].stopAfter ||
        ((p->segments[i].signedDistanceMm > 0.0f) !=
         (p->segments[i + 1U].signedDistanceMm > 0.0f));
}

bool MotionSequencePlan_Prepare(MotionSequencePlan *p,
    const MotionSequenceConfig *c, float mmPerCount,
    float decelerationMmps2, float rawSlopeBound, float rawRatePerSec)
{
    if (p == NULL) return false;
    p->prepared = false;
    if (c == NULL || p->count > MOTION_SEQUENCE_CAPACITY ||
        !isfinite(c->blendLengthMm) || c->blendLengthMm <= 0.0f ||
        !isfinite(c->junctionSpeedCps) || c->junctionSpeedCps <= 0.0f ||
        !isfinite(mmPerCount) || mmPerCount <= 0.0f ||
        !isfinite(decelerationMmps2) || decelerationMmps2 <= 0.0f ||
        !isfinite(rawSlopeBound) || rawSlopeBound < 0.0f ||
        !isfinite(rawRatePerSec) || rawRatePerSec <= 0.0f)
        return false;
    p->totalTravelMm = 0.0f;
    p->nominalFinalYawRad = 0.0f;
    p->mmPerCount = mmPerCount;
    p->decelerationMmps2 = decelerationMmps2;
    for (uint32_t i = 0U; i < p->count; ++i) {
        const MotionSequenceSegment *s = &p->segments[i];
        if (!isfinite(s->signedDistanceMm) || s->signedDistanceMm == 0.0f ||
            !isfinite(s->curvaturePerMm) ||
            !isfinite(s->speedCps) || s->speedCps <= 0.0f)
            return false;
        p->totalTravelMm += fabsf(s->signedDistanceMm);
        p->nominalFinalYawRad += s->signedDistanceMm * s->curvaturePerMm;
        p->junctionHalfLengthMm[i] = 0.0f;
        p->junctionSpeedCps[i] = 0.0f;
        float speedMmps = s->speedCps * mmPerCount;
        if (!isfinite(speedMmps * speedMmps +
            2.0f * decelerationMmps2 * p->totalTravelMm)) return false;
    }
    if (!isfinite(p->totalTravelMm) || !isfinite(p->nominalFinalYawRad))
        return false;
    for (uint32_t i = 0U; i + 1U < p->count; ++i) {
        if (boundary(p, i)) continue;
        const MotionSequenceSegment *a = &p->segments[i];
        const MotionSequenceSegment *b = &p->segments[i + 1U];
        float half = fminf(0.5f * c->blendLengthMm,
            0.25f * fminf(fabsf(a->signedDistanceMm), fabsf(b->signedDistanceMm)));
        float delta = fabsf(b->curvaturePerMm - a->curvaturePerMm);
        float cap = fminf(a->speedCps, b->speedCps);
        /* Equal curvature needs no steering transition or junction slowdown. */
        if (delta > 0.0f) cap = fminf(cap, c->junctionSpeedCps);
        /* Reserve half the configured software slew rate for feedback.
         * The global piecewise-linear FF slope bound also covers the centre
         * bridge and opposite-sign transitions. No physical servo claim. */
        if (delta > 0.0f && rawSlopeBound > 0.0f)
            cap = fminf(cap, 0.5f * rawRatePerSec * (2.0f * half) /
                (rawSlopeBound * delta * mmPerCount));
        if (!isfinite(cap) || cap <= 0.0f || half <= 0.0f) return false;
        p->junctionHalfLengthMm[i] = half;
        p->junctionSpeedCps[i] = cap;
    }
    p->prepared = true;
    return true;
}

uint32_t MotionSequencePlan_RunEnd(const MotionSequencePlan *p, uint32_t first)
{
    if (p == NULL || first >= p->count) return first;
    uint32_t last = first;
    while (last + 1U < p->count && !boundary(p, last)) ++last;
    return last;
}

static float envelope(const MotionSequencePlan *p, float capCps, float distanceMm)
{
    float speed = capCps * p->mmPerCount;
    return sqrtf(speed * speed + 2.0f * p->decelerationMmps2 * distanceMm) /
        p->mmPerCount;
}

bool MotionSequencePlan_Evaluate(const void *context, float progressMm,
                                 MotionPathSample *out)
{
    const MotionSequenceRun *run = context;
    if (run == NULL || out == NULL || run->plan == NULL ||
        !run->plan->prepared || !isfinite(progressMm) ||
        run->first > run->last || run->last >= run->plan->count ||
        MotionSequencePlan_RunEnd(run->plan, run->first) != run->last)
        return false;
    const MotionSequencePlan *p = run->plan;
    float total = 0.0f;
    for (uint32_t i = run->first; i <= run->last; ++i)
        total += fabsf(p->segments[i].signedDistanceMm);
    float x = fmaxf(0.0f, fminf(progressMm, total));
    float position = 0.0f, area = 0.0f;
    float speed = p->segments[run->first].speedCps;
    bool found = false;
    *out = (MotionPathSample){0};
    /* Emit constant sections and symmetric linear ramps. Analytic integrals
     * keep heading independent of update rate and encoder sample skipping. */
    for (uint32_t i = run->first; i <= run->last; ++i) {
        const MotionSequenceSegment *s = &p->segments[i];
        float left = i > run->first ? p->junctionHalfLengthMm[i - 1U] : 0.0f;
        float right = i < run->last ? p->junctionHalfLengthMm[i] : 0.0f;
        float constantLength = fabsf(s->signedDistanceMm) - left - right;
        if (!found && (x <= position + constantLength || i == run->last)) {
            out->curvaturePerMm = s->curvaturePerMm;
            out->desiredYawRad = area + s->curvaturePerMm *
                fmaxf(0.0f, fminf(x - position, constantLength));
            out->straightTuningWeight = s->curvaturePerMm == 0.0f ? 1.0f : 0.0f;
            found = true;
        }
        area += s->curvaturePerMm * constantLength;
        position += constantLength;
        if (i < run->last) {
            float length = 2.0f * right;
            float k0 = s->curvaturePerMm;
            float k1 = p->segments[i + 1U].curvaturePerMm;
            if (!found && x <= position + length) {
                float t = fmaxf(0.0f, fminf(1.0f, (x - position) / length));
                out->curvaturePerMm = fmaxf(fminf(k0, k1),
                    fminf(fmaxf(k0, k1), k0 + (k1 - k0) * t));
                out->desiredYawRad = area + length * (k0 * t + 0.5f * (k1 - k0) * t * t);
                float w0 = k0 == 0.0f ? 1.0f : 0.0f;
                float w1 = k1 == 0.0f ? 1.0f : 0.0f;
                out->straightTuningWeight = w0 + (w1 - w0) * t;
                found = true;
            }
            area += 0.5f * (k0 + k1) * length;
            position += length;
        }
    }
    if (!found) return false;
    out->desiredYawRad *= p->segments[run->first].signedDistanceMm > 0.0f ? 1.0f : -1.0f;
    /* Look ahead to every lower segment cap and every blend interval.
     * Current/earlier constraints are released only after their interval. */
    float nominalPosition = 0.0f;
    speed = INFINITY;
    for (uint32_t i = run->first; i <= run->last; ++i) {
        float end = nominalPosition + fabsf(p->segments[i].signedDistanceMm);
        if (x <= end || i == run->last)
            speed = fminf(speed, envelope(p, p->segments[i].speedCps,
                                         fmaxf(0.0f, nominalPosition - x)));
        if (i < run->last) {
            float half = p->junctionHalfLengthMm[i];
            if (x <= end + half)
                speed = fminf(speed, envelope(p, p->junctionSpeedCps[i],
                                             fmaxf(0.0f, end - half - x)));
        }
        nominalPosition = end;
    }
    out->speedLimitCps = speed;
    return isfinite(speed) && speed > 0.0f;
}
