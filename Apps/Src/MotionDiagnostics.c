#include "MotionDiagnostics.h"

#if MOTION_DIAGNOSTICS
#include <stdio.h>

void MotionDiagnostics_Format(const MotionDiagnostics *d, char *out, size_t size) {
    if (!d->valid) {
        snprintf(out, size, "D NONE\n");
        return;
    }
    char seq[12];
    if (d->numbered) snprintf(seq, sizeof(seq), "%u", d->seq);
    else snprintf(seq, sizeof(seq), "-");
    /* Bounded significant digits also keep unreasonable/nonfinite values
     * readable without overflowing a line. Distances mm, speeds counts/s.
     */
    int n = snprintf(out, size,
        "D %s step=%u %s%.6g %s mode=%s target=%.6g travel=%.6g "
        "left=%.6g right=%.6g vl=%.6g vr=%.6g steer=%.6g yaw=%.6g\n",
        seq, d->step, d->command ? d->command : "-", (double)d->parameter, d->event, d->mode,
        (double)d->targetMm, (double)d->travelledMm,
        (double)d->leftMm, (double)d->rightMm,
        (double)d->leftCps, (double)d->rightCps,
        (double)d->steeringCommand, (double)d->yawDeg);
    if (n < 0 || (size_t)n >= size) snprintf(out, size, "D OVERFLOW\n");
}
/* Explicit diagnostic query only: no streaming from the control loop. */
void MotionDiagnostics_FormatWheels(const MotionDiagnostics *d, char *out, size_t size) {
    if (!d->valid) {
        snprintf(out, size, "D W NONE\n");
        return;
    }
    char seq[12];
    if (d->numbered) snprintf(seq, sizeof(seq), "%u", d->seq);
    else snprintf(seq, sizeof(seq), "-");
    int n = snprintf(out, size,
        "D W %s step=%u %s vp=%.6g tl=%.6g tr=%.6g "
        "pl=%.6g pr=%.6g bl=%.6g br=%.6g al=%u ar=%u "
        "ml=%u mr=%u dl=%.6g dr=%.6g dt=%.6g maxdt=%.6g\n",
        seq, d->step, d->event,
        (double)d->profileSpeedMmps, (double)d->leftTargetCps,
        (double)d->rightTargetCps, (double)d->leftPWM, (double)d->rightPWM,
        (double)d->leftBrakePWM, (double)d->rightBrakePWM,
        d->leftActuator, d->rightActuator, d->leftMotorMode, d->rightMotorMode,
        (double)d->leftMotorDuty, (double)d->rightMotorDuty,
        (double)d->lastDtMs, (double)d->maxDtMs);
    if (n < 0 || (size_t)n >= size) snprintf(out, size, "D W OVERFLOW\n");
}
void MotionDiagnostics_FormatHardware(const MotionDiagnostics *d, char *out, size_t size) {
    if (!d->valid) {
        snprintf(out, size, "D H NONE\n");
        return;
    }
    char seq[12];
    if (d->numbered) snprintf(seq, sizeof(seq), "%u", d->seq);
    else snprintf(seq, sizeof(seq), "-");
    int n = snprintf(out, size,
        "D H %s step=%u %s la=%lu l1=%lu l2=%lu lc=%lu le=%lu "
        "ra=%lu r1=%lu r2=%lu rc=%lu re=%lu rb=%lu\n",
        seq, d->step, d->event,
        (unsigned long)d->leftArr, (unsigned long)d->leftCcr1,
        (unsigned long)d->leftCcr2, (unsigned long)d->leftCr1,
        (unsigned long)d->leftCcer, (unsigned long)d->rightArr,
        (unsigned long)d->rightCcr1, (unsigned long)d->rightCcr2,
        (unsigned long)d->rightCr1, (unsigned long)d->rightCcer,
        (unsigned long)d->rightBdtr);
    if (n < 0 || (size_t)n >= size) snprintf(out, size, "D H OVERFLOW\n");
}
#endif
