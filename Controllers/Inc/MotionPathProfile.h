#ifndef MOTION_PATH_PROFILE_H
#define MOTION_PATH_PROFILE_H

#include <stdbool.h>

#define MOTION_PATH_TERMINAL_ENTRY_DISTANCE_MM 30.0f
#define MOTION_PATH_TERMINAL_MIN_YAW_DEG 0.5f

/* References at unsigned, measured progress from the start of one run.
 * desiredYawRad includes travel direction and is relative to that run's origin.
 * speedLimitCps is an unsigned local ceiling, not an instantaneous speed.
 * straightTuningWeight: 1 = straight gains, 0 = arc gains. */
typedef struct
{
    float curvaturePerMm;
    float desiredYawRad;
    float speedLimitCps;
    float straightTuningWeight;
    /* Optional distance-aware speed profile. When supplied, both local limits
     * must be positive and brakingSpeedLimitCps is sqrt(2*integral(d ds)) from
     * this point to the run end. Blends use arc limits, including zero curvature
     * inside an opposite-turn blend. Zero-initialised custom paths inherit the
     * controller's straight/arc limits and its ordinary stopping envelope. */
    bool profileLimitsProvided;
    float accelerationMmps2;
    float decelerationMmps2;
    float brakingSpeedLimitCps;
} MotionPathSample;

typedef bool (*MotionPathEvaluate)(
    const void *context, float progressMm, MotionPathSample *sample);

typedef struct
{
    float signedDistanceMm;
    float maxSpeedCps;
    float steeringSettlingTimeSec;
    MotionPathEvaluate evaluate;
    const void *context;
    /* Opt-in for a final constant-curvature arc. Entry must follow its last
     * blend. Final heading is evaluate(abs(signedDistanceMm)).desiredYawRad;
     * turn direction comes from final curvature, not the sign of net yaw. */
    bool terminalYawPriority;
    float terminalEntryProgressMm;
    /* Per-run raw slew limit; zero inherits the normal steering limit. */
    float steeringCommandRatePerSec;
} MotionPathProfile;

#endif
