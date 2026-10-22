#ifndef INC_TASK2COURSE_H_
#define INC_TASK2COURSE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "Task2Commands.h"
#include "Task2App.h"
#include "Task2Wrap.h"

/*
 * The Task 2 course adapter, run by MotionTask for batch "r:T2".
 * It selects the R275 sequence (Task2Wrap), the planner (Task2App) or the explicit dry run. It sees the RPi only through Task2Commands (arrows,
 * bullseye span, stage events) and moves the robot only through Task2CourseIo, so it
 * can be driven with fake Io (hosttests/test_task2_app.py; dry-run tests are separate).
 *
 * Contract:
 *   - Report every stage with Task2Commands_SetStage; APPROACH2 only once the
 *     camera is past obstacle 1's face.
 *   - Read arrows with Task2Commands_Arrow at each decision point. With none,
 *     stop, set WAITn and keep returning TASK2COURSE_RUNNING: never guess.
 *   - Start primitives only through io; each one is guarded by MotionTask's
 *     movement watchdog. Start the next only when io->busy() is false.
 *   - io->brake() ends the current primitive (e.g. at a sensed obstacle edge);
 *     the run continues. Only MotionTask aborts the whole run (S or fault).
 *   - Update runs on the 10 ms control tick. Planning occurs only while stationary
 *     and uses a bounded clearance-check budget; sensor callbacks only copy snapshots.
 *   - Queue Task2Commands_Summary, then return TASK2COURSE_DONE once parked.
 *     MotionTask sets the DONE stage and sends the summary and "DONE r" only
 *     when the wheels are stationary.
 *   - S and faults are handled by MotionTask, which brakes and calls Abort.
 */

/* 1: no motion; stages advance on timers (Pi link bench test). 0: the real route. */
#ifndef TASK2_COURSE_DRY_RUN
#define TASK2_COURSE_DRY_RUN 0
#endif
/* 1: the fixed R275 sequence (Task2Wrap). 0: the route planner (Task2App). */
#ifndef TASK2_COURSE_WRAP
#define TASK2_COURSE_WRAP 1
#endif

typedef enum
{
    TASK2COURSE_RUNNING,
    TASK2COURSE_DONE,
    TASK2COURSE_FAULT,
} Task2CourseStatus;

typedef Task2AppIo Task2CourseIo;

typedef struct
{
    const Task2CourseIo *io;
    uint32_t stageStartedMs;
#if TASK2_COURSE_WRAP
    Task2Wrap wrap;
#else
    Task2App app;
#endif
    const char *fault; /* static token when Update returns TASK2COURSE_FAULT */
} Task2Course;

/* Called once Task2Commands_Begin has run. NULL on success, else a fault token. */
const char *Task2Course_Start(Task2Course *course,
                              const Task2CourseIo *io,
                              Task2Commands *commands,
                              uint32_t nowMs);
Task2CourseStatus Task2Course_Update(Task2Course *course, Task2Commands *commands, uint32_t nowMs);
/* O1's range from before START, right after a successful Start. Only the R275 sequence uses it. */
void Task2Course_UsePrestart(Task2Course *course, const Task2WrapPrestartRange *range);
/* The camera's depth to O1 before START (mm, 0: none); right after a successful Start. Only the R275 sequence uses it. */
void Task2Course_UseCameraO1(Task2Course *course, unsigned depthMm);
/* MotionTask has braked (S or fault); forget the run. */
void Task2Course_Abort(Task2Course *course);
/* A fault from outside the course; call before Task2Course_Abort so its state is still there. */
void Task2Course_NoteFault(Task2Course *course, const char *reason);
#if TASK2_DIAGNOSTICS
/* The selected route's decision records and run trace, for the Pi's "D T" / "D R" requests. */
const Task2Diagnostics *Task2Course_Diagnostics(const Task2Course *course);
const Task2Trace *Task2Course_Trace(const Task2Course *course);
#endif

#endif
