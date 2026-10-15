#include "Task2Course.h"

#include <math.h>
#include <string.h>

#if TASK2_COURSE_DRY_RUN
/*
 * No motion: each stage lasts a fixed time, roughly a real run's, and the
 * decision points wait for the RPi's arrows exactly as the route must.
 */
#define DRY_APPROACH1_MS 2000U
#define DRY_SWERVE1_MS 1000U
#define DRY_APPROACH2_MS 1800U
#define DRY_ROUND2_MS 3500U
#define DRY_HOME_MS 3000U

static void Enter(Task2Course *c, Task2Commands *t, Task2Stage stage, uint32_t nowMs) {
    Task2Commands_SetStage(t, stage);
    c->stageStartedMs = nowMs;
}

const char *Task2Course_Start(Task2Course *c, const Task2CourseIo *io,
                              Task2Commands *t, uint32_t nowMs) {
    (void)t;
    *c = (Task2Course){ .io = io, .stageStartedMs = nowMs };
    return NULL;
}

Task2CourseStatus Task2Course_Update(Task2Course *c, Task2Commands *t, uint32_t nowMs) {
    uint32_t elapsed = nowMs - c->stageStartedMs;
    switch (t->stage) {
    case TASK2_STAGE_APPROACH1:
        if (elapsed >= DRY_APPROACH1_MS)
            Enter(c, t, Task2Commands_Arrow(t, 1U) ? TASK2_STAGE_SWERVE1 : TASK2_STAGE_WAIT1, nowMs);
        break;
    case TASK2_STAGE_WAIT1:
        if (Task2Commands_Arrow(t, 1U))
            Enter(c, t, TASK2_STAGE_SWERVE1, nowMs);
        break;
    case TASK2_STAGE_SWERVE1:
        if (elapsed >= DRY_SWERVE1_MS)
            Enter(c, t, TASK2_STAGE_APPROACH2, nowMs);
        break;
    case TASK2_STAGE_APPROACH2:
        if (elapsed >= DRY_APPROACH2_MS)
            Enter(c, t, Task2Commands_Arrow(t, 2U) ? TASK2_STAGE_ROUND2 : TASK2_STAGE_WAIT2, nowMs);
        break;
    case TASK2_STAGE_WAIT2:
        if (Task2Commands_Arrow(t, 2U))
            Enter(c, t, TASK2_STAGE_ROUND2, nowMs);
        break;
    case TASK2_STAGE_ROUND2:
        if (elapsed >= DRY_ROUND2_MS)
            Enter(c, t, TASK2_STAGE_HOME, nowMs);
        break;
    case TASK2_STAGE_HOME:
        if (elapsed >= DRY_HOME_MS) {
            /* Nothing measured in a dry run; the span is not a width. */
            Task2Commands_Summary(t, NAN, NAN, NAN, NAN);
            return TASK2COURSE_DONE;
        }
        break;
    default:
        c->fault = "T2_STATE";
        return TASK2COURSE_FAULT;
    }
    return TASK2COURSE_RUNNING;
}

void Task2Course_Abort(Task2Course *c) {
    c->io = NULL;
}

void Task2Course_NoteFault(Task2Course *c, const char *reason) {
    (void)c; (void)reason;
}
void Task2Course_UsePrestart(Task2Course *c, const Task2WrapPrestartRange *range) {
    (void)c; (void)range;
}
void Task2Course_UseCameraO1(Task2Course *c, unsigned depthMm) {
    (void)c; (void)depthMm;
}

#elif TASK2_COURSE_WRAP
const char *Task2Course_Start(Task2Course *c, const Task2CourseIo *io,
                              Task2Commands *t, uint32_t nowMs) {
    (void)t;
    memset(c, 0, sizeof(*c));
    c->io = io; c->stageStartedMs = nowMs;
    c->fault = Task2Wrap_Start(&c->wrap, io, nowMs);
    return c->fault;
}
Task2CourseStatus Task2Course_Update(Task2Course *c, Task2Commands *t, uint32_t nowMs) {
    Task2AppStatus status = Task2Wrap_Update(&c->wrap, t, nowMs);
    c->fault = c->wrap.fault;
    return status == TASK2_APP_DONE ? TASK2COURSE_DONE
        : status == TASK2_APP_FAULT ? TASK2COURSE_FAULT : TASK2COURSE_RUNNING;
}
void Task2Course_Abort(Task2Course *c) {
    Task2Wrap_Abort(&c->wrap);
    c->io = NULL;
}
void Task2Course_NoteFault(Task2Course *c, const char *reason) {
    Task2Wrap_NoteFault(&c->wrap, reason);
}
void Task2Course_UsePrestart(Task2Course *c, const Task2WrapPrestartRange *range) {
    Task2Wrap_UsePrestart(&c->wrap, range);
}
void Task2Course_UseCameraO1(Task2Course *c, unsigned depthMm) {
    Task2Wrap_UseCameraO1(&c->wrap, depthMm);
}

#else
const char *Task2Course_Start(Task2Course *c, const Task2CourseIo *io,
                              Task2Commands *t, uint32_t nowMs) {
    (void)t;
    memset(c, 0, sizeof(*c));
    c->io = io; c->stageStartedMs = nowMs;
    c->fault = Task2App_Start(&c->app, io, nowMs);
    return c->fault;
}
Task2CourseStatus Task2Course_Update(Task2Course *c, Task2Commands *t, uint32_t nowMs) {
    Task2AppStatus status = Task2App_Update(&c->app, t, nowMs);
    c->fault = c->app.fault;
    return status == TASK2_APP_DONE ? TASK2COURSE_DONE
        : status == TASK2_APP_FAULT ? TASK2COURSE_FAULT : TASK2COURSE_RUNNING;
}
void Task2Course_Abort(Task2Course *c) {
    Task2App_Abort(&c->app);
    c->io = NULL;
}
void Task2Course_NoteFault(Task2Course *c, const char *reason) {
    Task2App_NoteFault(&c->app, reason);
}
void Task2Course_UsePrestart(Task2Course *c, const Task2WrapPrestartRange *range) {
    (void)c; (void)range;
}
void Task2Course_UseCameraO1(Task2Course *c, unsigned depthMm) {
    (void)c; (void)depthMm;
}
#endif

#if TASK2_DIAGNOSTICS
#if TASK2_COURSE_WRAP
const Task2Diagnostics *Task2Course_Diagnostics(const Task2Course *c) { return &c->wrap.diag; }
const Task2Trace *Task2Course_Trace(const Task2Course *c) { return &c->wrap.trace; }
#else
const Task2Diagnostics *Task2Course_Diagnostics(const Task2Course *c) { return &c->app.diag; }
const Task2Trace *Task2Course_Trace(const Task2Course *c) { return &c->app.trace; }
#endif
#endif
