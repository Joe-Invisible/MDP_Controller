#ifndef STEERING_RAMP_RESPONSE_TEST_H
#define STEERING_RAMP_RESPONSE_TEST_H

#include "SteeringRampProfile.h"

#define STEERING_RAMP_TRIAL_COUNT 3U
#define STEERING_RAMP_SAMPLES_PER_TRIAL 500U
#define STEERING_RAMP_LOG_CAPACITY \
    (STEERING_RAMP_TRIAL_COUNT * STEERING_RAMP_SAMPLES_PER_TRIAL)

typedef enum
{
    STEERING_RAMP_NOT_RUN = 0,
    STEERING_RAMP_COMPLETED,
    STEERING_RAMP_CANCELLED,
    STEERING_RAMP_TIMED_OUT,
    STEERING_RAMP_LOG_FULL,
    STEERING_RAMP_CONTROLLER_ERROR,
    STEERING_RAMP_BRAKE_TIMED_OUT,
    STEERING_RAMP_PREPARATION_ERROR
} SteeringRampOutcome;

/* 64 bytes/sample: the full three-trial batch fits in the target's main RAM. */
typedef struct
{
    uint32_t timeMs; /* Relative to FollowProfile(), including initial settling. */
    uint16_t trialIndex;
    uint8_t mode;
    uint8_t phase;
    float progressMm;
    float rawReference;
    float rawCommand;
    float rawCommandRatePerSec;
    float curvaturePerMm;
    float measuredCentreSpeedMmps;
    float yawRateDps;
    float filteredYawRateDps;
    float yawDeg;
    float leftTargetCps;
    float rightTargetCps;
    float leftMeasuredCps;
    float rightMeasuredCps;
    float wheelSyncCorrectionCps;
} SteeringRampResponseSample;

typedef struct
{
    SteeringRampOutcome outcome;
    MotionControllerStatus status;
    uint32_t firstSample;
    uint32_t sampleCount;
    uint32_t elapsedMs;
    uint32_t maxUpdateIntervalMs;
    float measuredDistanceMm;
    float measuredYawDeg;
    /* Planned centre-path extents relative to the initial rear-axle centre.
     * Display adds a body envelope and clearance; these are estimates. */
    float minXmm, maxXmm, minYmm, maxYmm;
} SteeringRampResponseResult;

extern const SteeringRampTrial steeringRampTrials[STEERING_RAMP_TRIAL_COUNT];
extern SteeringRampProfile steeringRampProfiles[STEERING_RAMP_TRIAL_COUNT];
extern MotionControllerConfig steeringRampMotionConfig;
extern MotionController *const steeringRampController;
extern SteeringRampResponseSample steeringRampLog[STEERING_RAMP_LOG_CAPACITY];
extern SteeringRampResponseResult steeringRampResults[STEERING_RAMP_TRIAL_COUNT];
extern uint32_t steeringRampLogCount;
extern uint32_t steeringRampResultCount;
extern uint32_t steeringRampLogPeriodMs;
extern bool steeringRampFinished;
extern bool steeringRampBatchComplete;

void SteeringRampResponseTestRun(void);
/* Set a debugger breakpoint here to export all trials once. */
void SteeringRampResponseTestFinished(void);

#endif
