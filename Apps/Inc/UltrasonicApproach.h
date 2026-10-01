#ifndef ULTRASONIC_APPROACH_H
#define ULTRASONIC_APPROACH_H

#include <stdbool.h>
#include "SensorReading.h"
#include "UltrasonicApproachConfig.h"

/* Pure decision logic. MotionTask owns this state and applies its actions.
 * It consumes SensorTask snapshots; it never triggers or waits for an echo.
 */
typedef enum {
    ULTRASONIC_APPROACH_DRIVE,
    ULTRASONIC_APPROACH_BRAKE,
    ULTRASONIC_APPROACH_VERIFY,
    ULTRASONIC_APPROACH_DONE,
    ULTRASONIC_APPROACH_FAULT
} UltrasonicApproachAction;

typedef struct {
    UltrasonicApproachAction action;
    float targetMm;
    float travelLimitMm;
    float profileTargetMm; /* Absolute forward encoder endpoint, bounded by travelLimitMm. */
    uint32_t sampleSequence;
    uint32_t startedMs;
    uint32_t stoppedMs;
    uint32_t stoppedSequence;
    unsigned verifiedSamples;
    const char *fault; /* Static protocol reason token. */
} UltrasonicApproach;

/* Returns NULL on success, otherwise a fault token; no motor side effects. */
const char *UltrasonicApproach_Start(UltrasonicApproach *state, float targetMm,
                                    const UltrasonicReading *reading, uint32_t now);

/* DRIVE: update the existing MotionProfile targetDistanceMm, without restarting
 * the profile. Keep its acceleration, deceleration and cruise speed unchanged.
 * BRAKE: request existing controller braking (idempotent).
 * VERIFY: stay stationary awaiting distinct post-stop samples; DONE: may advance
 * the batch, but only after controllerBusy became false and range is verified.
 * FAULT: brake and cancel the rest of the batch. No automatic retry/reverse.
 */
UltrasonicApproachAction UltrasonicApproach_Update(UltrasonicApproach *state,
    const UltrasonicReading *reading, uint32_t now, float progressMm,
    float speedMmps, bool controllerBusy, bool controllerBraking);

#endif
