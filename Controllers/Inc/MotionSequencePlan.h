#ifndef MOTION_SEQUENCE_PLAN_H
#define MOTION_SEQUENCE_PLAN_H

#include <stdint.h>
#include "MotionPathProfile.h"
#include "MotionControllerConfig.h"

#define MOTION_SEQUENCE_CAPACITY 20U

typedef struct
{
    float signedDistanceMm;
    float curvaturePerMm;
    float speedCps;
    bool stopAfter;
} MotionSequenceSegment;

typedef struct
{
    /* Full blend length; each neighbour supplies half. Short motions shrink
     * the blend so at most half of any segment is consumed by both blends. */
    float blendLengthMm;
    float junctionSpeedCps;
    /* Raw units/s for this sequence only; zero inherits steering calibration. */
    float steeringCommandRatePerSec;
    /* Experimental: preserve the requested length across short neighbours.
     * Independent curvature ramps may overlap; stop/reversal boundaries still
     * contain every ramp. False retains the established quarter-segment bound. */
    bool allowBlendOverlap;
    /* Local FF intervals include a guard for discrete reference updates. */
    float referenceGuardSec;
} MotionSequenceConfig;

typedef struct
{
    MotionSequenceSegment segments[MOTION_SEQUENCE_CAPACITY];
    /* Optional full-length ceiling after each segment; zero inherits config.
     * Set while building. Stops still disable their junction entirely. */
    float junctionBlendLimitMm[MOTION_SEQUENCE_CAPACITY];
    float junctionHalfLengthMm[MOTION_SEQUENCE_CAPACITY];
    float junctionSpeedCps[MOTION_SEQUENCE_CAPACITY];
    uint32_t count;
    float totalTravelMm;
    float nominalFinalYawRad;
    float mmPerCount;
    float straightAccelerationMmps2;
    float straightDecelerationMmps2;
    float arcAccelerationMmps2;
    float arcDecelerationMmps2;
    float requestedBlendLengthMm;
    float junctionSpeedCeilingCps;
    float feedforwardSlopeBound; /* Fallback for pure plans without a table. */
    /* Borrowed calibration; keep it valid and unchanged through execution.
     * With a table, junctionSpeedCps is the base ceiling; Evaluate applies
     * piecewise local steering limits and braking lookahead. */
    const MotionControllerArcConfig *feedforwardConfig;
    float straightFeedforwardCommand;
    float steeringRatePerSec;
    float referenceGuardSec;
    bool allowBlendOverlap;
    bool prepared;
} MotionSequencePlan;

typedef struct
{
    const MotionSequencePlan *plan;
    uint32_t first;
    uint32_t last; /* inclusive; all segments have the same direction */
} MotionSequenceRun;

extern const MotionSequenceConfig motionSequenceConfig;

/* Optional local-slope model. NULL selects the supplied scalar fallback. */
void MotionSequencePlan_SetFeedforward(MotionSequencePlan *plan,
                                       const MotionControllerArcConfig *config,
                                       float straightCommand);

bool MotionSequencePlan_Prepare(MotionSequencePlan *plan,
                                const MotionSequenceConfig *config,
                                float mmPerCount,
                                float straightAccelerationMmps2,
                                float straightDecelerationMmps2,
                                float arcAccelerationMmps2,
                                float arcDecelerationMmps2,
                                float rawSlopeBound,
                                float rawRatePerSec);
bool MotionSequencePlan_Evaluate(const void *context, float progressMm, MotionPathSample *sample);
uint32_t MotionSequencePlan_RunEnd(const MotionSequencePlan *plan, uint32_t first);

#endif
