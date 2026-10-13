#ifndef INC_TASK2RUN_H_
#define INC_TASK2RUN_H_

#include <stdbool.h>
#include <stdint.h>

#include "Task2Commands.h"
#include "Task2Course.h"

/*
 * MotionTask's side of a T2 batch: owns the course and its RPi state, and
 * decides when the run's terminal state may be reported. No HAL/OS, so the
 * exact logic MotionTask runs is host-tested (hosttests/task2_run_test.c).
 *
 * Rules:
 *   - DONE (stage and "DONE r") only once the wheels are stationary; the
 *     course's SUMMARY is held until then.
 *   - S aborts at once; STOPPED is set at the first stationary tick.
 *   - A fault aborts at once and sets FAULT, also while braking for S.
 *   - S after a fault: STOPPED at the first stationary tick.
 *   - Nothing here changes an earlier run's state when a later batch faults.
 */

typedef enum {
    TASK2RUN_RUNNING,
    TASK2RUN_DONE,    /* send events, then complete the batch ("DONE r") */
    TASK2RUN_FAULT,   /* send events, then fault the batch with the token */
} Task2RunStatus;

typedef struct {
    Task2Commands commands;
    Task2Course course;
    bool active;      /* the T2 command is running */
    bool stopping;    /* S accepted; STOPPED once stationary */
    bool finishing;   /* course done; waiting for stationary wheels */
} Task2Run;

void Task2Run_Init(Task2Run *r);

/* The batch's T2 command is starting, with arrow 1 from its start line (NONE for plain T2).
 * NULL on success, else a fault token. */
const char *Task2Run_Start(Task2Run *r, uint8_t run, Task2CommandArrow arrow1,
                           const Task2CourseIo *io, uint32_t nowMs);

/* Every control tick while active. On TASK2RUN_FAULT, *fault is a static token. */
Task2RunStatus Task2Run_Update(Task2Run *r, bool busy, uint32_t nowMs, const char **fault);

/* S was accepted while this run was active or faulted (braking requested). */
void Task2Run_Stop(Task2Run *r);

/* The batch is faulting (braking requested). */
void Task2Run_Fault(Task2Run *r);

/* Call on every tick the wheels are stationary, before CommandSession_Stopped. */
void Task2Run_Stationary(Task2Run *r);

/* Next event to send; none while a finished run waits for stationary wheels. */
bool Task2Run_NextEvent(Task2Run *r, char line[TASK2COMMANDS_REPLY_SIZE]);

/* A/W/Q T lines; false for anything else. */
bool Task2Run_Receive(Task2Run *r, const char *line, char reply[TASK2COMMANDS_REPLY_SIZE]);

#endif
