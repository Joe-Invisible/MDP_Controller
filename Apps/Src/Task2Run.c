#include "Task2Run.h"

#include <string.h>

void Task2Run_Init(Task2Run *r)
{
    memset(r, 0, sizeof(*r));
    Task2Commands_Init(&r->commands);
}

const char *Task2Run_Start(
    Task2Run *r, uint8_t run, Task2CommandArrow arrow1, const Task2CourseIo *io, uint32_t nowMs)
{
    r->active = r->stopping = r->finishing = false;
    Task2Commands_Begin(&r->commands, run, arrow1);
    const char *fault = Task2Course_Start(&r->course, io, &r->commands, nowMs);
    if (fault != NULL)
    {
        Task2Commands_ClearEvents(&r->commands);
        Task2Commands_SetStage(&r->commands, TASK2_STAGE_FAULT);
        return fault;
    }
    r->active = true;
    return NULL;
}

Task2RunStatus Task2Run_Update(Task2Run *r, bool busy, uint32_t nowMs, const char **fault)
{
    if (!r->active) return TASK2RUN_RUNNING;
    if (!r->finishing)
    {
        Task2CourseStatus status = Task2Course_Update(&r->course, &r->commands, nowMs);
        if (status == TASK2COURSE_FAULT)
        {
            *fault = r->course.fault ? r->course.fault : "T2_COURSE";
            return TASK2RUN_FAULT;
        }
        r->finishing = status == TASK2COURSE_DONE;
    }
    if (r->finishing && !busy)
    {
        r->finishing = false;
        r->active = false;
        Task2Commands_SetStage(&r->commands, TASK2_STAGE_DONE);
        return TASK2RUN_DONE;
    }
    return TASK2RUN_RUNNING;
}

static void Abort(Task2Run *r)
{
    Task2Course_Abort(&r->course);
    Task2Commands_ClearEvents(&r->commands);
    r->active = false;
    r->finishing = false;
}

void Task2Run_Stop(Task2Run *r)
{
    bool faulted = r->commands.started && r->commands.stage == TASK2_STAGE_FAULT;
    if (r->active || faulted) r->stopping = true;
    if (r->active) Abort(r);
}

void Task2Run_Fault(Task2Run *r)
{
    if (!r->active && !r->stopping) return; /* another batch: leave this run's record */
    Abort(r);
    r->stopping = false;
    Task2Commands_SetStage(&r->commands, TASK2_STAGE_FAULT);
}

void Task2Run_Stationary(Task2Run *r)
{
    if (r->stopping)
    {
        r->stopping = false;
        Task2Commands_SetStage(&r->commands, TASK2_STAGE_STOPPED);
    }
}

bool Task2Run_NextEvent(Task2Run *r, char line[TASK2COMMANDS_REPLY_SIZE])
{
    return !r->finishing && Task2Commands_NextEvent(&r->commands, line);
}

bool Task2Run_Receive(Task2Run *r, const char *line, char reply[TASK2COMMANDS_REPLY_SIZE])
{
    return Task2Commands_Receive(&r->commands, line, reply);
}
