/* Run: cc -Wall -Wextra -Werror -IApps/Inc hosttests/diagnostics_test.c
 * Apps/Src/MotionDiagnostics.c -o /tmp/mdp_diag_test && /tmp/mdp_diag_test
 * Verify on-demand telemetry is bounded and preserves the fault snapshot.
 */
#include "MotionDiagnostics.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void) {
    char out[MOTION_DIAGNOSTICS_REPLY_SIZE];
    MotionDiagnostics d = {0};
    MotionDiagnostics_Format(&d, out, sizeof(out));
    assert(!strcmp(out, "D NONE\n"));
    d = (MotionDiagnostics){ .valid = true, .numbered = true, .seq = 31,
        .step = 1, .command = 'F', .parameter = 18.8f,
        .event = "NO_PROGRESS", .mode = "STRAIGHT", .targetMm = 18.8f,
        .travelledMm = 0, .steeringCommand = 0.123f, .yawDeg = -1.25f };
    MotionDiagnostics copy = d;
    MotionDiagnostics_Format(&d, out, sizeof(out));
    assert(strstr(out, "D 31 step=1 F18.8 NO_PROGRESS mode=STRAIGHT"));
    assert(strstr(out, "target=18.8 travel=0"));
    assert(strstr(out, "steer=0.123 yaw=-1.25\n"));
    assert(!memcmp(&d, &copy, sizeof(d)));
    d.numbered = false;
    d.targetMm = FLT_MAX;
    d.travelledMm = -FLT_MAX;
    d.yawDeg = NAN;
    d.leftCps = INFINITY;
    MotionDiagnostics_Format(&d, out, sizeof(out));
    assert(!strncmp(out, "D - step=1", 10));
    assert(out[strlen(out)-1] == '\n');
    char tiny[16];
    MotionDiagnostics_Format(&d, tiny, sizeof(tiny));
    assert(!strcmp(tiny, "D OVERFLOW\n"));
    puts("diagnostics tests passed");
}
