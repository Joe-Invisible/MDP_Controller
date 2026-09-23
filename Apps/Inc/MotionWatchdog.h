#ifndef MOTIONWATCHDOG_H
#define MOTIONWATCHDOG_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/* Runtime guard, independent of the calibrated controller. Times are ms. */
#define MOTIONWATCH_PROGRESS_MM 1.0f
#define MOTIONWATCH_STALL_MS 2000U
#define MOTIONWATCH_PHASE_MS 2000U
typedef enum { WATCH_MOVE, WATCH_PREPARE, WATCH_BRAKE } MotionWatchPhase;
typedef struct {
    uint32_t started, phaseStarted, progressed, limitMs;
    float highWaterMm;
    MotionWatchPhase phase;
} MotionWatchdog;

static inline bool MotionWatchdog_Start(MotionWatchdog *w, uint32_t now,
                                        float distanceMm, float speedMmps,
                                        MotionWatchPhase phase) {
    if (!isfinite(distanceMm) || distanceMm == 0.0f ||
        !isfinite(speedMmps) || speedMmps <= 0.0f)
        return false;
    /* Five seconds for startup/braking, plus four nominal cruise durations.
     * Stay below half the uint32 timer range for wrap-safe elapsed checks.
     */
    float duration = 5000.0f + 4000.0f * fabsf(distanceMm) / speedMmps;
    if (!isfinite(duration) || duration >= 2147483648.0f)
        return false;
    *w = (MotionWatchdog){ .started = now, .phaseStarted = now,
        .progressed = now, .limitMs = (uint32_t)duration, .phase = phase };
    return true;
}

/* progressMm is signed in the commanded direction. Backward drift and
 * oscillation must not keep a stalled command alive. Called only while busy.
 * Arc wheel speeds differ, so use centre distance, not equal wheel progress.
 */
static inline const char *MotionWatchdog_Check(MotionWatchdog *w, uint32_t now,
                                              MotionWatchPhase phase,
                                              float progressMm) {
    if (!isfinite(progressMm)) return "ODOMETRY";
    if (phase != w->phase) {
        w->phase = phase;
        w->phaseStarted = now;
        w->progressed = now;
        w->highWaterMm = progressMm;
    }
    if (phase == WATCH_BRAKE && now - w->phaseStarted >= MOTIONWATCH_PHASE_MS)
        return "BRAKE_TIMEOUT";
    if (phase == WATCH_PREPARE && now - w->phaseStarted >= MOTIONWATCH_PHASE_MS)
        return "PREP_TIMEOUT";
    if (phase == WATCH_MOVE) {
        if (progressMm >= w->highWaterMm + MOTIONWATCH_PROGRESS_MM) {
            w->highWaterMm = progressMm;
            w->progressed = now;
        }
        if (now - w->progressed >= MOTIONWATCH_STALL_MS) return "NO_PROGRESS";
    }
    if (now - w->started >= w->limitMs) return "MOVE_TIMEOUT";
    return NULL;
}
#endif
