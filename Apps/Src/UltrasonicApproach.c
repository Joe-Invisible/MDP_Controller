#include "UltrasonicApproach.h"

#include <math.h>
#include <stddef.h>

static const char *CheckReading(const UltrasonicReading *reading, uint32_t now)
{
    if (reading == NULL || reading->status == SENSOR_NOT_READY) return "US_NOT_READY";
    if ((uint32_t)(now - reading->capturedMs) > ULTRASONIC_APPROACH_MAX_AGE_MS)
        return "US_STALE";
    if (reading->status == SENSOR_TIMEOUT) return "US_ECHO";
    if (reading->status != SENSOR_OK) return "US_SENSOR";
    if (!isfinite(reading->distanceMm) || reading->distanceMm <= 0.0f ||
        reading->distanceMm > SENSOR_ULTRASONIC_MAX_MM) return "US_SENSOR";
    return NULL;
}

static UltrasonicApproachAction Fail(UltrasonicApproach *state, const char *reason)
{
    state->fault = reason;
    state->action = ULTRASONIC_APPROACH_FAULT;
    return state->action;
}

const char *UltrasonicApproach_Start(UltrasonicApproach *state, float targetMm,
                                    const UltrasonicReading *reading, uint32_t now)
{
    *state = (UltrasonicApproach){ .targetMm = targetMm, .startedMs = now,
        .action = ULTRASONIC_APPROACH_DRIVE };
    const char *fault = CheckReading(reading, now);
    if (!isfinite(targetMm) || targetMm < ULTRASONIC_APPROACH_MIN_TARGET_MM ||
        targetMm > ULTRASONIC_APPROACH_MAX_TARGET_MM) fault = "US_TARGET";
    if (fault != NULL) { Fail(state, fault); return fault; }
    float gap = reading->distanceMm - targetMm;
    if (gap < -ULTRASONIC_APPROACH_TOLERANCE_MM) fault = "US_TOO_CLOSE";
    if (gap > ULTRASONIC_APPROACH_MAX_TRAVEL_MM - ULTRASONIC_APPROACH_TRAVEL_MARGIN_MM)
        fault = "US_TRAVEL";
    if (fault != NULL) { Fail(state, fault); return fault; }
    /* A bound based on the initial range: never chase a receding target. */
    state->travelLimitMm = fmaxf(ULTRASONIC_APPROACH_TRAVEL_MARGIN_MM,
        gap + ULTRASONIC_APPROACH_TRAVEL_MARGIN_MM);
    /* The command starts stationary; the initial snapshot needs no travel
     * compensation. New snapshots replace this endpoint while driving.
     */
    state->profileTargetMm = fmaxf(0.0f, gap);
    state->sampleSequence = reading->sequence;
    return NULL;
}

UltrasonicApproachAction UltrasonicApproach_Update(UltrasonicApproach *state,
    const UltrasonicReading *reading, uint32_t now, float progressMm,
    float speedMmps, bool controllerBusy, bool controllerBraking)
{
    if (state->action == ULTRASONIC_APPROACH_FAULT ||
        state->action == ULTRASONIC_APPROACH_DONE) return state->action;
    const char *fault = CheckReading(reading, now);
    if (fault != NULL) return Fail(state, fault);
    if ((uint32_t)(now - state->startedMs) >= ULTRASONIC_APPROACH_TIMEOUT_MS)
        return Fail(state, "US_TIMEOUT");
    if (!isfinite(progressMm) || !isfinite(speedMmps) || speedMmps < 0.0f)
        return Fail(state, "US_ODOMETRY");

    if (progressMm >= state->travelLimitMm) return Fail(state, "US_TRAVEL");

    if (state->action == ULTRASONIC_APPROACH_BRAKE) {
        if (controllerBusy) return state->action;
        state->action = ULTRASONIC_APPROACH_VERIFY;
        state->stoppedMs = now;
        state->stoppedSequence = reading->sequence;
        return state->action;
    }
    if (state->action == ULTRASONIC_APPROACH_VERIFY) {
        if (controllerBusy) return Fail(state, "US_STATE");
        if ((uint32_t)(now - state->stoppedMs) >= ULTRASONIC_APPROACH_VERIFY_MS)
            return Fail(state, "US_VERIFY");
        /* A cached pre-brake reading cannot certify the final gap. */
        if (reading->sequence == state->stoppedSequence ||
            (int32_t)(reading->capturedMs - state->stoppedMs) < 0)
            return state->action;
        float error = reading->distanceMm - state->targetMm;
        if (error < -ULTRASONIC_APPROACH_TOLERANCE_MM) return Fail(state, "US_TOO_CLOSE");
        if (error > ULTRASONIC_APPROACH_TOLERANCE_MM) return Fail(state, "US_RANGE");
        state->stoppedSequence = reading->sequence;
        if (++state->verifiedSamples < ULTRASONIC_APPROACH_VERIFY_SAMPLES)
            return state->action;
        state->action = ULTRASONIC_APPROACH_DONE;
        return state->action;
    }

    if (progressMm >= state->travelLimitMm - 1.0f)
        return Fail(state, "US_TRAVEL");
    /* The unchanged MotionProfile may have reached its endpoint in the last
     * control tick. Latch braking: later echoes must never restart this move.
     */
    if (controllerBraking) {
        state->action = ULTRASONIC_APPROACH_BRAKE;
        return state->action;
    }
    if (!controllerBusy) return Fail(state, "US_STATE");
    float gap = reading->distanceMm - state->targetMm;
    if (gap < -ULTRASONIC_APPROACH_TOLERANCE_MM) return Fail(state, "US_TOO_CLOSE");
    if (reading->sequence != state->sampleSequence) {
        /* Range belongs to capture time, while progress belongs to the latest
         * control tick. Approximate intervening travel with encoder speed.
         * Between echoes the endpoint stays fixed and encoder odometry consumes
         * the remaining distance, exactly as for F. Never reuse a cached range
         * as a new measurement or extend the original absolute travel bound.
         */
        float ageSec = (float)(uint32_t)(now - reading->capturedMs) * 0.001f;
        float endpoint = progressMm + gap - speedMmps * ageSec;
        state->profileTargetMm = fminf(state->travelLimitMm, fmaxf(0.0f, endpoint));
        state->sampleSequence = reading->sequence;
    }
    /* The existing profile handles gradual deceleration over the whole gap.
     * Full braking is reserved for the near endpoint, not v^2/(2*a) away.
     */
    if (state->profileTargetMm - progressMm <= ULTRASONIC_APPROACH_STOP_MARGIN_MM)
        state->action = ULTRASONIC_APPROACH_BRAKE;
    return state->action;
}
