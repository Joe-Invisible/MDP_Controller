#ifndef MOTION_SEQUENCE_H
#define MOTION_SEQUENCE_H

#include "MotionController.h"
#include "MotionSequencePlan.h"

typedef enum
{
    MOTION_SEQUENCE_BUILDING = 0,
    MOTION_SEQUENCE_RUNNING,
    MOTION_SEQUENCE_COMPLETE,
    MOTION_SEQUENCE_ABORTING,
    MOTION_SEQUENCE_ABORTED,
    MOTION_SEQUENCE_FAILED
} MotionSequenceState;

/* Single owner of MotionController while running. Call ONLY
 * MotionSequence_Update (not also MotionController_Update) at the control
 * period. Keep this object alive and unchanged until execution finishes. */
typedef struct
{
    MotionController *controller;
    MotionSequenceConfig config;
    MotionSequencePlan plan;
    MotionSequenceRun run;
    MotionSequenceState state;
    MotionControllerStatus lastStatus;
    float completedMeasuredTravelMm; /* includes braking drift */
    float completedMeasuredYawRad;
    uint32_t completedRuns;
    bool expectedBraking;
    bool initialized;
} MotionSequence;

MotionControllerStatus MotionSequence_Begin(
    MotionSequence *sequence,
    MotionController *controller,
    const MotionSequenceConfig *config);
MotionControllerStatus MotionSequence_AddStraight(
    MotionSequence *sequence, float signedDistanceMm, float speedCps, bool stopAfter);
MotionControllerStatus MotionSequence_AddArc(
    MotionSequence *sequence,
    float signedDistanceMm,
    float radiusMm,
    float speedCps,
    bool stopAfter);
/* Preflight and launch asynchronously. Empty sequence completes immediately.
 * Capacity or infeasible plan is PROFILE_ERROR. No heap allocation. */
MotionControllerStatus MotionSequence_Execute(MotionSequence *sequence);
MotionControllerStatus MotionSequence_Update(MotionSequence *sequence, float dt);
/* Cancel remaining segments and brake; keep calling Update until not busy. */
MotionControllerStatus MotionSequence_Brake(MotionSequence *sequence);
bool MotionSequence_IsBusy(const MotionSequence *sequence);

#endif
