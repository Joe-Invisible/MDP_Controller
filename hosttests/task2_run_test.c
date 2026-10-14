/* cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -DTASK2_COURSE_DRY_RUN=1 -IApps/Inc hosttests/task2_run_test.c
 *    Apps/Src/Task2Run.c Apps/Src/Task2Course.c Apps/Src/Task2Commands.c -lm -o /tmp/task2_run_test
 * The exact terminal-state logic MotionTask uses, with the dry-run course and a
 * controller that is busy or stationary as each case needs.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "Task2Run.h"

static Task2Run r;
static char line[TASK2COMMANDS_REPLY_SIZE];
static const Task2CourseIo io = {0};
static uint32_t now;

static void QT(const char *expected) {
    assert(Task2Run_Receive(&r, "Q T", line));
    if (strcmp(line, expected) != 0) {
        fprintf(stderr, "Q T: expected [%s], got [%s]\n", expected, line);
        assert(0);
    }
}

static void Event(const char *expected) {
    assert(Task2Run_NextEvent(&r, line));
    if (strcmp(line, expected) != 0) {
        fprintf(stderr, "event: expected [%s], got [%s]\n", expected, line);
        assert(0);
    }
}

static void NoEvent(void) { assert(!Task2Run_NextEvent(&r, line)); }

static void Arrow(const char *a, const char *expected) {
    assert(Task2Run_Receive(&r, a, line));
    assert(strcmp(line, expected) == 0);
}

/* Ticks the course (stationary) until it reaches stage or limit ms pass. */
static void RunTo(Task2Stage stage, uint32_t limitMs) {
    const char *fault = NULL;
    for (uint32_t t = 0; r.commands.stage != stage && t < limitMs; t += 10U, now += 10U)
        assert(Task2Run_Update(&r, false, now, &fault) == TASK2RUN_RUNNING);
    assert(r.commands.stage == stage);
}

static void StartWithArrows(uint8_t run) {
    assert(Task2Run_Start(&r, run, TASK2_COMMAND_ARROW_NONE, &io, now) == NULL && r.active);
    char expected[TASK2COMMANDS_REPLY_SIZE];
    snprintf(expected, sizeof(expected), "EV %u APPROACH 1\n", (unsigned)run);
    Event(expected);
    snprintf(expected, sizeof(expected), "A %u 1 L", (unsigned)run);
    assert(Task2Run_Receive(&r, expected, line));
}

int main(void) {
    const char *fault = NULL;
    Task2Run_Init(&r);
    QT("T - IDLE - -\n");
    Task2Run_Stop(&r);                       /* S with no run: nothing to record */
    Task2Run_Stationary(&r);
    Task2Run_Fault(&r);                      /* fault of another batch: nothing either */
    QT("T - IDLE - -\n");

    /* 1. DONE only once stationary; summary held until then. */
    StartWithArrows(12);
    RunTo(TASK2_STAGE_APPROACH2, 10000U);
    Event("EV 12 APPROACH 2\n");
    Arrow("A 12 2 R", "ACK 12 2 R\n");
    RunTo(TASK2_STAGE_HOME, 10000U);
    Event("EV 12 ROUND 2\n");
    Task2RunStatus status;
    while ((status = Task2Run_Update(&r, true, now, &fault)) == TASK2RUN_RUNNING && !r.finishing)
        now += 10U;
    assert(status == TASK2RUN_RUNNING && r.finishing);
    for (int i = 0; i < 50; ++i, now += 10U)      /* still moving: keep waiting */
        assert(Task2Run_Update(&r, true, now, &fault) == TASK2RUN_RUNNING);
    QT("T 12 HOME L R\n");
    NoEvent();                                     /* SUMMARY held */
    assert(Task2Run_Update(&r, false, now, &fault) == TASK2RUN_DONE);
    assert(!r.active);
    QT("T 12 DONE L R\n");
    Event("EV 12 SUMMARY - - - -\n");
    NoEvent();

    /* 2. A later batch faulting leaves this run's record alone. */
    Task2Run_Fault(&r);
    Task2Run_Stop(&r);
    Task2Run_Stationary(&r);
    QT("T 12 DONE L R\n");

    /* 3. S mid-run: aborted at once, STOPPED only once stationary. */
    StartWithArrows(13);
    RunTo(TASK2_STAGE_SWERVE1, 10000U);
    Task2Run_Stop(&r);
    assert(!r.active);
    QT("T 13 SWERVE1 L -\n");
    NoEvent();
    assert(Task2Run_Update(&r, true, now, &fault) == TASK2RUN_RUNNING);   /* no course activity */
    Task2Run_Stationary(&r);
    QT("T 13 STOPPED L -\n");
    Arrow("A 13 2 R", "NAK 13 2 STATE\n");

    /* 4. Fault while braking for S wins over the pending STOPPED. */
    StartWithArrows(14);
    Task2Run_Stop(&r);
    Task2Run_Fault(&r);
    Task2Run_Stationary(&r);
    QT("T 14 FAULT L -\n");
    /* ...and the S that clears the fault then gives STOPPED once stationary. */
    Task2Run_Stop(&r);
    QT("T 14 FAULT L -\n");
    Task2Run_Stationary(&r);
    QT("T 14 STOPPED L -\n");

    /* 5. Fault mid-run, then S. */
    StartWithArrows(15);
    RunTo(TASK2_STAGE_SWERVE1, 10000U);
    Task2Run_Fault(&r);
    QT("T 15 FAULT L -\n");
    NoEvent();
    Task2Run_Stop(&r);
    Task2Run_Stationary(&r);
    QT("T 15 STOPPED L -\n");

    /* 6. Fault while a finished course waits for stationary wheels: no DONE, no summary. */
    StartWithArrows(16);
    RunTo(TASK2_STAGE_APPROACH2, 10000U);
    Arrow("A 16 2 L", "ACK 16 2 L\n");
    while (!r.finishing) {
        assert(Task2Run_Update(&r, true, now, &fault) == TASK2RUN_RUNNING);
        now += 10U;
    }
    Task2Run_Fault(&r);
    assert(!r.active && !r.finishing);
    QT("T 16 FAULT L L\n");
    NoEvent();

    /* 7. A course that finds itself in an impossible stage faults with its token. */
    StartWithArrows(17);
    r.commands.stage = TASK2_STAGE_DONE;
    assert(Task2Run_Update(&r, false, now, &fault) == TASK2RUN_FAULT);
    assert(fault && strcmp(fault, "T2_STATE") == 0);
    Task2Run_Fault(&r);
    QT("T 17 FAULT L -\n");

    /* 8. A new run starts clean after any of the above. */
    StartWithArrows(18);
    QT("T 18 APPROACH1 L -\n");

    puts("task2_run_test: all passed");
    return 0;
}
