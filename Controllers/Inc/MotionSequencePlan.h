#ifndef MOTION_SEQUENCE_PLAN_H
#define MOTION_SEQUENCE_PLAN_H

#include <stdint.h>
#include "MotionPathProfile.h"

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
} MotionSequenceConfig;

typedef struct
{
    MotionSequenceSegment segments[MOTION_SEQUENCE_CAPACITY];
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
    bool prepared;
} MotionSequencePlan;

typedef struct
{
    const MotionSequencePlan *plan;
    uint32_t first;
    uint32_t last; /* inclusive; all segments have the same direction */
} MotionSequenceRun;

extern const MotionSequenceConfig motionSequenceConfig;

bool MotionSequencePlan_Prepare(
    MotionSequencePlan *plan,
    const MotionSequenceConfig *config,
    float mmPerCount,
    float straightAccelerationMmps2,
    float straightDecelerationMmps2,
    float arcAccelerationMmps2,
    float arcDecelerationMmps2,
    float rawSlopeBound,
    float rawRatePerSec);
bool MotionSequencePlan_Evaluate(
    const void *context, float progressMm, MotionPathSample *sample);
uint32_t MotionSequencePlan_RunEnd(const MotionSequencePlan *plan, uint32_t first);

#endif
