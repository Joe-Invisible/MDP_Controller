#ifndef MOTIONDIAGNOSTICS_H
#define MOTIONDIAGNOSTICS_H

/* Set -DMOTION_DIAGNOSTICS=0 to remove capture/storage and the D query.
 * This never disables the movement watchdog. Default on for floor diagnosis.
 */
#ifndef MOTION_DIAGNOSTICS
#define MOTION_DIAGNOSTICS 1
#endif

#if MOTION_DIAGNOSTICS
#include <stdbool.h>
#include <stddef.h>

#define MOTION_DIAGNOSTICS_REPLY_SIZE 256U
typedef struct {
    bool valid, numbered;
    unsigned seq, step;
    char command;
    const char *event, *mode; /* Static strings only. */
    float parameter, targetMm, travelledMm, leftMm, rightMm;
    float leftCps, rightCps, steeringCommand, yawDeg;
} MotionDiagnostics;

/* On-demand single-line reply. No HAL, I/O, allocation or state mutation. */
void MotionDiagnostics_Format(const MotionDiagnostics *d, char *out, size_t size);
#endif
#endif
