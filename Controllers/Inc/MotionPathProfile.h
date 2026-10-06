#ifndef MOTION_PATH_PROFILE_H
#define MOTION_PATH_PROFILE_H

#include <stdbool.h>

/* References at unsigned, measured progress from the start of one run.
 * desiredYawRad includes travel direction and is relative to that run's origin.
 * speedLimitCps is an unsigned local ceiling, not an instantaneous speed.
 * straightTuningWeight: 1 = straight gains, 0 = arc gains. */
typedef struct {
    float curvaturePerMm;
    float desiredYawRad;
    float speedLimitCps;
    float straightTuningWeight;
} MotionPathSample;

typedef bool (*MotionPathEvaluate)(const void *context, float progressMm,
                                  MotionPathSample *sample);

typedef struct {
    float signedDistanceMm;
    float maxSpeedCps;
    float steeringSettlingTimeSec;
    MotionPathEvaluate evaluate;
    const void *context;
} MotionPathProfile;

#endif
