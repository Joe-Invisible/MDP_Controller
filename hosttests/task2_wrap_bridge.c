/*
 * Host bridge for driving the real Task2Wrap from a Python physics model (task2_sim/option_a/wrapsim.py).
 * Python supplies the motion controller and sensors through the callbacks below; this file adapts them to
 * Task2AppIo and exposes the course state. Build as a shared library; see wrapsim.py.
 */
#include <stdio.h>
#include <string.h>

#include "Task2Wrap.h"

typedef int (*MoveFn)(float distanceMm, float radiusMm, float speedCps);   /* 0 accepted */
typedef int (*FlagFn)(void);
typedef void (*VoidFn)(void);
typedef int (*SpeedFn)(float speedCps);
typedef void (*MotionFn)(float *distanceMm, float *yawRad, int *moving, int *braking);
typedef void (*UssFn)(uint32_t *sequence, uint32_t *capturedMs, float *distanceMm, int *status);
typedef void (*IrFn)(uint32_t *sequence, uint32_t *capturedMs, uint16_t *left, uint16_t *right, int *status);

static struct { MoveFn move; FlagFn busy; VoidFn brake; SpeedFn speed; MotionFn motion; UssFn uss; IrFn ir; } cb;
static Task2Wrap wrap;
static Task2Commands commands;
static Task2AppIo io;
static uint32_t nowMs;

static const char *Straight(void *c, float d, float v) { (void)c; return cb.move(d, 0.0f, v) ? "REJECTED" : NULL; }
static const char *Arc(void *c, float d, float r, float v) { (void)c; return cb.move(d, r, v) ? "REJECTED" : NULL; }
static bool Busy(void *c) { (void)c; return cb.busy() != 0; }
static void Brake(void *c) { (void)c; cb.brake(); }
static const char *Speed(void *c, float v) { (void)c; return cb.speed(v) ? "T2_SPEED" : NULL; }
static const char *Continuous(void *c, float v) { (void)c; (void)v; return "REJECTED"; }
static Task2MotionReading Motion(void *c) {
    (void)c; float d = 0.0f, y = 0.0f; int m = 0, b = 0;
    cb.motion(&d, &y, &m, &b);
    return (Task2MotionReading){d, y, m != 0, b != 0};
}
static UltrasonicReading Ultrasonic(void *c) {
    (void)c; UltrasonicReading r = {0}; int status = 0;
    cb.uss(&r.sequence, &r.capturedMs, &r.distanceMm, &status);
    r.status = (SensorStatus)status;
    return r;
}
static IRPairReading Infrared(void *c) {
    (void)c; IRPairReading r = {0}; uint32_t captured = 0; int status = 0;
    cb.ir(&r.sequence, &captured, &r.left.adc, &r.right.adc, &status);
    r.left.capturedMs = r.right.capturedMs = captured;
    r.left.status = r.right.status = (SensorStatus)status;
    return r;
}

void bridge_callbacks(MoveFn move, FlagFn busy, VoidFn brake, SpeedFn speed, MotionFn motion, UssFn uss, IrFn ir) {
    cb.move = move; cb.busy = busy; cb.brake = brake; cb.speed = speed; cb.motion = motion; cb.uss = uss; cb.ir = ir;
}
const char *bridge_start(uint32_t now, int arrow1, float mmPerCount) {
    nowMs = now;
    io = (Task2AppIo){.moveStraight = Straight, .moveArc = Arc, .busy = Busy, .brake = Brake, .requestSpeed = Speed,
        .moveContinuous = Continuous, .motion = Motion, .ultrasonic = Ultrasonic, .infrared = Infrared,
        .mmPerCount = mmPerCount, .decelerationMmps2 = 1700.0f};
    Task2Commands_Init(&commands);
    Task2Commands_Begin(&commands, 1U, (Task2CommandArrow)arrow1);
    return Task2Wrap_Start(&wrap, &io, now);
}
int bridge_update(uint32_t now) { nowMs = now; return (int)Task2Wrap_Update(&wrap, &commands, now); }
void bridge_arrow2(int arrow) {
    char line[24], reply[TASK2COMMANDS_REPLY_SIZE];
    snprintf(line, sizeof(line), "A 1 2 %c", arrow > 0 ? 'L' : 'R');
    (void)Task2Commands_Receive(&commands, line, reply);
}
int bridge_stage(void) { return (int)commands.stage; }
int bridge_phase(void) { return (int)wrap.phase; }
const char *bridge_fault(void) { return wrap.fault ? wrap.fault : ""; }
void bridge_pose(float *x, float *y, float *yaw) { *x = wrap.pose.xMm; *y = wrap.pose.yMm; *yaw = wrap.pose.yawRad; }
/* Decision records in order, as "what:reason" lines. */
int bridge_records(char *out, int size) {
    int n = 0;
#if TASK2_DIAGNOSTICS
    const Task2Diagnostics *d = &wrap.diag;
    for (unsigned i = 0U; i < d->count && n < size; ++i) {
        const Task2DiagRecord *r = &d->records[(d->head + i) % TASK2_DIAG_RECORDS];
        n += snprintf(out + n, (size_t)(size - n), "%s:%s;", r->what ? r->what : "?", r->reason ? r->reason : "-");
    }
#endif
    return n;
}
void bridge_abort(void) { Task2Wrap_Abort(&wrap); }
