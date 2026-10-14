/* cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -DTASK2_COURSE_DRY_RUN=1 -IApps/Inc hosttests/task2_course_test.c Apps/Src/Task2Course.c Apps/Src/Task2Commands.c -lm -o /tmp/task2_course_test */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "Task2Course.h"

/* A robot that records every request; the dry run must never use it. */
static unsigned moves, brakes;
static const char *FakeStraight(void *c, float d, float s) { (void)c; (void)d; (void)s; ++moves; return NULL; }
static const char *FakeArc(void *c, float d, float r, float s) { (void)c; (void)d; (void)r; (void)s; ++moves; return NULL; }
static bool FakeBusy(void *c) { (void)c; return false; }
static void FakeBrake(void *c) { (void)c; ++brakes; }
static const Task2CourseIo io = { .moveStraight = FakeStraight, .moveArc = FakeArc,
                                  .busy = FakeBusy, .brake = FakeBrake };

static Task2Commands t;
static Task2Course course;
static char reply[TASK2COMMANDS_REPLY_SIZE];

/* Runs the course on 10 ms ticks until it leaves `stage` or limitMs passes. */
static uint32_t RunWhile(Task2Stage stage, uint32_t *now, uint32_t limitMs) {
    uint32_t start = *now;
    while (t.stage == stage && *now - start < limitMs) {
        assert(Task2Course_Update(&course, &t, *now) == TASK2COURSE_RUNNING);
        *now += 10U;
    }
    return *now - start;
}

static void Drain(const char *expected) {
    char line[TASK2COMMANDS_REPLY_SIZE];
    assert(Task2Commands_NextEvent(&t, line));
    if (strcmp(line, expected) != 0) {
        fprintf(stderr, "expected [%s], got [%s]\n", expected, line);
        assert(0);
    }
}

static void Send(const char *line, const char *expected) {
    assert(Task2Commands_Receive(&t, line, reply));
    assert(strcmp(reply, expected) == 0);
}

int main(void) {
    /* Late arrows: the course waits at both decision points and never guesses. */
    uint32_t now = 1000U;
    Task2Commands_Begin(&t, 7, TASK2_COMMAND_ARROW_NONE);
    assert(Task2Course_Start(&course, &io, &t, now) == NULL);
    Drain("EV 7 APPROACH 1\n");
    RunWhile(TASK2_STAGE_APPROACH1, &now, 10000U);
    assert(t.stage == TASK2_STAGE_WAIT1);
    Drain("EV 7 WAIT 1\n");
    assert(RunWhile(TASK2_STAGE_WAIT1, &now, 60000U) >= 60000U);   /* indefinitely */
    Send("A 7 1 R", "ACK 7 1 R\n");
    RunWhile(TASK2_STAGE_WAIT1, &now, 100U);
    assert(t.stage == TASK2_STAGE_SWERVE1);
    Send("A 7 2 L", "NAK 7 2 STATE\n");                            /* not past obstacle 1 */
    RunWhile(TASK2_STAGE_SWERVE1, &now, 10000U);
    assert(t.stage == TASK2_STAGE_APPROACH2);
    Drain("EV 7 APPROACH 2\n");
    RunWhile(TASK2_STAGE_APPROACH2, &now, 10000U);
    assert(t.stage == TASK2_STAGE_WAIT2);
    Drain("EV 7 WAIT 2\n");
    RunWhile(TASK2_STAGE_WAIT2, &now, 5000U);
    assert(t.stage == TASK2_STAGE_WAIT2);
    Send("A 7 2 L", "ACK 7 2 L\n");
    RunWhile(TASK2_STAGE_WAIT2, &now, 100U);
    assert(t.stage == TASK2_STAGE_ROUND2);
    Drain("EV 7 ROUND 2\n");
    RunWhile(TASK2_STAGE_ROUND2, &now, 10000U);
    assert(t.stage == TASK2_STAGE_HOME);
    Task2CourseStatus status;
    while ((status = Task2Course_Update(&course, &t, now)) == TASK2COURSE_RUNNING)
        now += 10U;
    /* DONE stage belongs to MotionTask, set once stationary. */
    assert(status == TASK2COURSE_DONE && t.stage == TASK2_STAGE_HOME);
    Send("Q T", "T 7 HOME R L\n");
    Drain("EV 7 SUMMARY - - - -\n");
    assert(!Task2Commands_NextEvent(&t, reply));
    Task2Commands_SetStage(&t, TASK2_STAGE_DONE);
    Send("Q T", "T 7 DONE R L\n");

    /* Arrows in time: no waits; the stages take their dry-run durations. */
    now = 0xFFFFF000U;                  /* across the 32-bit tick wrap */
    Task2Commands_Begin(&t, 8, TASK2_COMMAND_ARROW_NONE);
    assert(Task2Course_Start(&course, &io, &t, now) == NULL);
    Drain("EV 8 APPROACH 1\n");
    Send("A 8 1 L", "ACK 8 1 L\n");
    uint32_t start = now;
    RunWhile(TASK2_STAGE_APPROACH1, &now, 10000U);
    assert(t.stage == TASK2_STAGE_SWERVE1);
    RunWhile(TASK2_STAGE_SWERVE1, &now, 10000U);
    Drain("EV 8 APPROACH 2\n");
    Send("A 8 2 R", "ACK 8 2 R\n");
    RunWhile(TASK2_STAGE_APPROACH2, &now, 10000U);
    assert(t.stage == TASK2_STAGE_ROUND2);
    Drain("EV 8 ROUND 2\n");
    while ((status = Task2Course_Update(&course, &t, now)) == TASK2COURSE_RUNNING)
        now += 10U;
    assert(status == TASK2COURSE_DONE);
    uint32_t elapsed = now - start;
    assert(elapsed >= 11300U && elapsed <= 11400U);

    /* An unexpected stage is a fault, not a guess. */
    Task2Commands_SetStage(&t, TASK2_STAGE_STOPPED);
    assert(Task2Course_Update(&course, &t, now) == TASK2COURSE_FAULT);
    assert(course.fault && strcmp(course.fault, "T2_STATE") == 0);

    assert(moves == 0U && brakes == 0U);
    printf("task2_course_test: all passed (dry run %lu ms with arrows in time)\n", (unsigned long)elapsed);
    return 0;
}
