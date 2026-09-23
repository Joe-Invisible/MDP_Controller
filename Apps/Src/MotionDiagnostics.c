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
        "D %s step=%u %c%.6g %s mode=%s target=%.6g travel=%.6g "
        "left=%.6g right=%.6g vl=%.6g vr=%.6g steer=%.6g yaw=%.6g\n",
        seq, d->step, d->command, (double)d->parameter, d->event, d->mode,
        (double)d->targetMm, (double)d->travelledMm,
        (double)d->leftMm, (double)d->rightMm,
        (double)d->leftCps, (double)d->rightCps,
        (double)d->steeringCommand, (double)d->yawDeg);
    if (n < 0 || (size_t)n >= size) snprintf(out, size, "D OVERFLOW\n");
}
#endif
