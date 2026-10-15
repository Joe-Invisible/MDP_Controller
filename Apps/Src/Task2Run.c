#include "Task2Run.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

void Task2Run_Init(Task2Run *r) {
    memset(r, 0, sizeof(*r));
    Task2Commands_Init(&r->commands);
}

const char *Task2Run_Start(Task2Run *r, uint8_t run, Task2CommandArrow arrow1,
                           const Task2CourseIo *io, uint32_t nowMs) {
    r->active = r->stopping = r->finishing = false;
    unsigned cameraO1 = r->commands.previewO1DepthMm;    /* Begin clears it */
    Task2Commands_Begin(&r->commands, run, arrow1);
    const char *fault = Task2Course_Start(&r->course, io, &r->commands, nowMs);
    if (fault != NULL) {
        Task2Commands_ClearEvents(&r->commands);
        Task2Commands_SetStage(&r->commands, TASK2_STAGE_FAULT);
        return fault;
    }
#if TASK2_COURSE_WRAP && !TASK2_COURSE_DRY_RUN
    Task2WrapPrestartRange prestart = Task2WrapPrestart_Range(&r->prestart, nowMs);
    Task2Course_UsePrestart(&r->course, &prestart);
    Task2Course_UseCameraO1(&r->course, cameraO1);
    r->prestart.count = 0U;
#else
    (void)cameraO1;
#endif
    r->active = true;
    return NULL;
}

void Task2Run_Idle(Task2Run *r, const Task2CourseIo *io, bool moving, uint32_t nowMs) {
#if TASK2_COURSE_WRAP && !TASK2_COURSE_DRY_RUN
    if (!r->active) Task2WrapPrestart_Update(&r->prestart, io, moving, nowMs);
#else
    (void)r; (void)io; (void)moving; (void)nowMs;
#endif
}

Task2RunStatus Task2Run_Update(Task2Run *r, bool busy, uint32_t nowMs, const char **fault) {
    if (!r->active)
        return TASK2RUN_RUNNING;
    if (!r->finishing) {
        Task2CourseStatus status = Task2Course_Update(&r->course, &r->commands, nowMs);
        if (status == TASK2COURSE_FAULT) {
            *fault = r->course.fault ? r->course.fault : "T2_COURSE";
            return TASK2RUN_FAULT;
        }
        r->finishing = status == TASK2COURSE_DONE;
    }
    if (r->finishing && !busy) {
        r->finishing = false;
        r->active = false;
        Task2Commands_SetStage(&r->commands, TASK2_STAGE_DONE);
        return TASK2RUN_DONE;
    }
    return TASK2RUN_RUNNING;
}

static void Abort(Task2Run *r) {
    Task2Course_Abort(&r->course);
    Task2Commands_ClearEvents(&r->commands);
    r->active = false;
    r->finishing = false;
}

void Task2Run_Stop(Task2Run *r) {
    bool faulted = r->commands.started && r->commands.stage == TASK2_STAGE_FAULT;
    if (r->active || faulted)
        r->stopping = true;
    if (r->active)
        Abort(r);
}

void Task2Run_Fault(Task2Run *r) {
    if (!r->active && !r->stopping)
        return;                          /* another batch: leave this run's record */
    Abort(r);
    r->stopping = false;
    Task2Commands_SetStage(&r->commands, TASK2_STAGE_FAULT);
}

void Task2Run_FaultReason(Task2Run *r, const char *reason) {
    if (r->active)
        Task2Course_NoteFault(&r->course, reason);
    Task2Run_Fault(r);
}

void Task2Run_Stationary(Task2Run *r) {
    if (r->stopping) {
        r->stopping = false;
        Task2Commands_SetStage(&r->commands, TASK2_STAGE_STOPPED);
    }
}

bool Task2Run_NextEvent(Task2Run *r, char line[TASK2COMMANDS_REPLY_SIZE]) {
    return !r->finishing && Task2Commands_NextEvent(&r->commands, line);
}

bool Task2Run_Receive(Task2Run *r, const char *line, char reply[TASK2COMMANDS_REPLY_SIZE]) {
    bool handled = Task2Commands_Receive(&r->commands, line, reply);
    if (handled && strncmp(reply, "ACK - Z ", 8U) == 0) {
        /* With the camera's O1 depth, report the ultrasonic's own steady O1 range so the Pi can compare before START. */
        size_t n = strlen(reply);
        if (n && reply[n - 1U] == '\n') reply[--n] = '\0';
#if TASK2_COURSE_WRAP && !TASK2_COURSE_DRY_RUN
        float u = Task2WrapPrestart_Steady(&r->prestart);
#else
        float u = NAN;
#endif
        if (isfinite(u)) snprintf(reply + n, TASK2COMMANDS_REPLY_SIZE - n, " %ld\n", lroundf(u));
        else snprintf(reply + n, TASK2COMMANDS_REPLY_SIZE - n, " -\n");
    }
    return handled;
}

unsigned Task2Run_Diagnostics(const Task2Run *r, const char *line,
                              char lines[][TASK2_DIAG_LINE_SIZE], unsigned maxLines) {
#if TASK2_DIAGNOSTICS
    return Task2Diag_Reply(Task2Course_Diagnostics(&r->course), r->commands.run, line, lines, maxLines);
#else
    (void)r; (void)line; (void)lines; (void)maxLines;
    return 0U;
#endif
}

unsigned Task2Run_Trace(const Task2Run *r, const char *line,
                        char lines[][TASK2_DIAG_LINE_SIZE], unsigned maxLines) {
#if TASK2_DIAGNOSTICS
    return Task2Trace_Reply(Task2Course_Trace(&r->course), r->commands.run, line, lines, maxLines);
#else
    (void)r; (void)line; (void)lines; (void)maxLines;
    return 0U;
#endif
}
