#ifndef MOTION_SEQUENCE_FUSION_TEST_H
#define MOTION_SEQUENCE_FUSION_TEST_H

#include "MotionSequence.h"

typedef struct
{
    uint32_t timeMs;
    uint32_t mode;
    uint32_t sequenceState;
    uint32_t firstSegment;
    uint32_t lastSegment;
    float runDistanceMm;
    float sequenceTravelMm;
    float sequenceYawRad;
    float desiredYawRad;
    float curvaturePerMm;
    float commandedCurvaturePerMm;
    float localSpeedLimitCps;
    float targetSpeedCps;
    float measuredCentreSpeedMmps;
    float filteredMeasuredCentreSpeedMmps;
    float yawRateDps;
    float filteredYawRateDps;
    float targetYawRateRadPerSec;
    float headingErrorRad;
    float steeringFeedforward;
    float steeringCorrection;
    float steeringTarget;
    float steeringCommand;
    float leftTargetCps;
    float rightTargetCps;
    float leftMeasuredCps;
    float rightMeasuredCps;
    float wheelSyncErrorMm;
    float wheelSyncCorrectionCps;
    float straightTuningWeight;
} MotionSequenceFusionTestSample;

void MotionSequenceFusionTestRun(void);
/* Debugger breakpoint: result globals and final log sample are ready. */
void MotionSequenceFusionTestFinished(void);

#endif
