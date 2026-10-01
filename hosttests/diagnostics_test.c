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
    d = (MotionDiagnostics){0};
    MotionDiagnostics_FormatWheels(&d, out, sizeof(out));
    assert(!strcmp(out, "D W NONE\n"));
    d = (MotionDiagnostics){ .valid = true, .numbered = true, .seq = 15,
        .step = 1, .event = "NO_PROGRESS", .profileSpeedMmps = 18.5f,
        .leftTargetCps = 130, .rightTargetCps = 150,
        .leftPWM = 57, .rightPWM = 58, .leftActuator = 1, .rightActuator = 1,
        .leftMotorMode = 1, .rightMotorMode = 1,
        .leftMotorDuty = 57, .rightMotorDuty = 58, .lastDtMs = 10, .maxDtMs = 14 };
    copy = d;
    MotionDiagnostics_FormatWheels(&d, out, sizeof(out));
    assert(strstr(out, "D W 15 step=1 NO_PROGRESS vp=18.5 tl=130 tr=150"));
    assert(strstr(out, "pl=57 pr=58 bl=0 br=0 al=1 ar=1"));
    assert(strstr(out, "ml=1 mr=1 dl=57 dr=58 dt=10 maxdt=14\n"));
    assert(!memcmp(&d, &copy, sizeof(d))); /* retained pre-brake data */
    char small[20];
    MotionDiagnostics_FormatWheels(&d, small, sizeof(small));
    assert(!strcmp(small, "D W OVERFLOW\n"));
    d.numbered = false;
    d.profileSpeedMmps = FLT_MAX;
    d.leftTargetCps = -FLT_MAX;
    d.rightTargetCps = NAN;
    d.leftPWM = INFINITY;
    MotionDiagnostics_FormatWheels(&d, out, sizeof(out));
    assert(!strncmp(out, "D W - step=1", 12));
    assert(out[strlen(out)-1] == '\n');
    d = (MotionDiagnostics){0};
    MotionDiagnostics_FormatHardware(&d, out, sizeof(out));
    assert(!strcmp(out, "D H NONE\n"));
    d = (MotionDiagnostics){ .valid = true, .numbered = true, .seq = 1,
        .step = 1, .event = "NO_PROGRESS", .leftArr = 7999,
        .leftCcr1 = 4500, .leftCr1 = 1, .leftCcer = 17,
        .rightArr = 7999, .rightCcr2 = 4560, .rightCr1 = 1,
        .rightCcer = 17, .rightBdtr = 32768 };
    copy = d;
    MotionDiagnostics_FormatHardware(&d, out, sizeof(out));
    assert(strstr(out, "D H 1 step=1 NO_PROGRESS la=7999 l1=4500 l2=0 lc=1 le=17"));
    assert(strstr(out, "ra=7999 r1=0 r2=4560 rc=1 re=17 rb=32768\n"));
    assert(!memcmp(&d, &copy, sizeof(d)));
    MotionDiagnostics_FormatHardware(&d, small, sizeof(small));
    assert(!strcmp(small, "D H OVERFLOW\n"));
    d.leftArr = d.rightArr = UINT32_MAX;
    MotionDiagnostics_FormatHardware(&d, out, sizeof(out));
    assert(strstr(out, "la=4294967295"));
    assert(strstr(out, "ra=4294967295"));
    puts("diagnostics tests passed");
}
