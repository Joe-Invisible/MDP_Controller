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
#include <stdint.h>

#define MOTION_DIAGNOSTICS_REPLY_SIZE 384U
typedef struct {
    bool valid, numbered;
    unsigned seq, step;
    char command;
    const char *event, *mode; /* Static strings only. */
    float parameter, targetMm, travelledMm, leftMm, rightMm;
    float leftCps, rightCps, steeringCommand, yawDeg;
    /* Captured before fault braking; D W reads this same snapshot. */
    float profileSpeedMmps, leftTargetCps, rightTargetCps;
    float leftPWM, rightPWM, leftBrakePWM, rightBrakePWM;
    float lastDtMs, maxDtMs;
    unsigned leftActuator, rightActuator, leftMotorMode, rightMotorMode;
    float leftMotorDuty, rightMotorDuty;
    /* Register configuration only; not a voltage/current measurement. */
    uint32_t leftArr, leftCcr1, leftCcr2, leftCr1, leftCcer;
    uint32_t rightArr, rightCcr1, rightCcr2, rightCr1, rightCcer, rightBdtr;
} MotionDiagnostics;

/* On-demand single-line reply. No HAL, I/O, allocation or state mutation. */
void MotionDiagnostics_Format(const MotionDiagnostics *d, char *out, size_t size);
void MotionDiagnostics_FormatWheels(const MotionDiagnostics *d, char *out, size_t size);
void MotionDiagnostics_FormatHardware(const MotionDiagnostics *d, char *out, size_t size);
#endif
#endif
