/* cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -IApps/Inc hosttests/task2_commands_test.c Apps/Src/Task2Commands.c -lm -o /tmp/task2_commands_test */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "Task2Commands.h"

static Task2Commands t;
static char reply[TASK2COMMANDS_REPLY_SIZE];

static void Expect(const char *line, const char *expected) {
    assert(Task2Commands_Receive(&t, line, reply));
    if (strcmp(reply, expected) != 0) {
        fprintf(stderr, "%s: expected [%s], got [%s]\n", line, expected, reply);
        assert(0);
    }
}

static void Event(const char *expected) {
    char line[TASK2COMMANDS_REPLY_SIZE];
    assert(Task2Commands_NextEvent(&t, line));
    if (strcmp(line, expected) != 0) {
        fprintf(stderr, "event: expected [%s], got [%s]\n", expected, line);
        assert(0);
    }
}

static void NoEvent(void) {
    char line[TASK2COMMANDS_REPLY_SIZE];
    assert(!Task2Commands_NextEvent(&t, line));
}

int main(void) {
    Task2Commands_Init(&t);

    /* Lines that are not Task 2 stay with CommandSession. */
    const char *others[] = { "Q", "Q P", "S", "12:F100", "STATUS", "G", "A", "W", "AB 1 1 L", "Q T " };
    for (size_t i = 0; i < sizeof(others) / sizeof(others[0]); ++i)
        assert(!Task2Commands_Receive(&t, others[i], reply) && reply[0] == '\0');

    /* Before any run. */
    Expect("Q T", "T - IDLE - -\n");
    Expect("A 0 1 L", "NAK 0 1 STATE\n");
    Expect("W 0 450", "NAK 0 W STATE\n");
    assert(Task2Commands_Arrow(&t, 1) == TASK2_COMMAND_ARROW_NONE);
    Task2Commands_SetStage(&t, TASK2_STAGE_APPROACH2);   /* no run: ignored */
    NoEvent();

    /* Malformed. */
    const char *bad[] = { "A 12 1 X", "A 12 3 L", "A 256 1 L", "A 12 1 LL", "A 12  1 L",
                          "A -1 1 L", "A 12 1", "A 123456 1 L", "W 12 0", "W 12 10000",
                          "W 12 4.5", "W 12", "W 12 450 ", "W x 450" };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i)
        Expect(bad[i], "NAK - PARSE\n");

    /* Run 12 starts in APPROACH1. */
    Task2Commands_Begin(&t, 12, TASK2_COMMAND_ARROW_NONE);
    Event("EV 12 APPROACH 1\n");
    NoEvent();
    Expect("Q T", "T 12 APPROACH1 - -\n");

    /* Arrow 2 is not armed yet; wrong run is refused. */
    Expect("A 12 2 R", "NAK 12 2 STATE\n");
    Expect("A 11 1 L", "NAK 11 1 STATE\n");
    assert(Task2Commands_Arrow(&t, 2) == TASK2_COMMAND_ARROW_NONE);

    /* Arrow 1: stored once; a resend is ACKed again; the other side is refused. */
    Expect("A 12 1 L", "ACK 12 1 L\n");
    Expect("A 12 1 L", "ACK 12 1 L\n");
    Expect("A 12 1 R", "NAK 12 1 ARROW\n");
    assert(Task2Commands_Arrow(&t, 1) == TASK2_COMMAND_ARROW_LEFT);
    Expect("Q T", "T 12 APPROACH1 L -\n");

    /* Span is reserved: stored and echoed until ROUND2. */
    uint16_t span = 0;
    assert(!Task2Commands_Span(&t, &span));
    Expect("W 12 450", "ACK 12 W 450\n");
    Expect("W 12 488", "ACK 12 W 488\n");
    assert(Task2Commands_Span(&t, &span) && span == 488);
    Expect("W 13 450", "NAK 13 W STATE\n");

    /* Stage events, each once. */
    Task2Commands_SetStage(&t, TASK2_STAGE_SWERVE1);
    NoEvent();
    Expect("A 12 1 L", "ACK 12 1 L\n");          /* resend after moving on */
    Expect("A 12 2 R", "NAK 12 2 STATE\n");
    Task2Commands_SetStage(&t, TASK2_STAGE_APPROACH2);
    Task2Commands_SetStage(&t, TASK2_STAGE_APPROACH2);
    Event("EV 12 APPROACH 2\n");
    NoEvent();
    Task2Commands_SetStage(&t, TASK2_STAGE_WAIT2);
    Event("EV 12 WAIT 2\n");
    Expect("Q T", "T 12 WAIT2 L -\n");
    Expect("A 12 2 R", "ACK 12 2 R\n");
    assert(Task2Commands_Arrow(&t, 2) == TASK2_COMMAND_ARROW_RIGHT);
    Task2Commands_SetStage(&t, TASK2_STAGE_ROUND2);
    Event("EV 12 ROUND 2\n");
    Expect("W 12 500", "NAK 12 W STATE\n");
    assert(Task2Commands_Span(&t, &span) && span == 488);
    Task2Commands_SetStage(&t, TASK2_STAGE_HOME);
    NoEvent();

    Task2Commands_Summary(&t, 1050.4f, 2200.0f, NAN, 7123.6f);
    Event("EV 12 SUMMARY 1050 2200 - 7124\n");
    Task2Commands_SetStage(&t, TASK2_STAGE_DONE);
    NoEvent();
    Expect("Q T", "T 12 DONE L R\n");
    Expect("A 12 1 L", "ACK 12 1 L\n");          /* the first ACK may have been lost */
    Expect("A 12 2 L", "NAK 12 2 ARROW\n");

    /* A new run forgets the old arrows and span. */
    Task2Commands_Begin(&t, 13, TASK2_COMMAND_ARROW_NONE);
    Event("EV 13 APPROACH 1\n");
    Expect("Q T", "T 13 APPROACH1 - -\n");
    assert(!Task2Commands_Span(&t, &span));
    Expect("A 12 1 L", "NAK 12 1 STATE\n");
    Task2Commands_SetStage(&t, TASK2_STAGE_WAIT1);
    Event("EV 13 WAIT 1\n");
    Expect("A 13 1 R", "ACK 13 1 R\n");

    /* Run 255 and the widest SUMMARY fit the reply size. */
    Task2Commands_Begin(&t, 255, TASK2_COMMAND_ARROW_NONE);
    Event("EV 255 APPROACH 1\n");
    Task2Commands_Summary(&t, -99999.0f, 99999.0f, 99999.0f, 99999.0f);
    Event("EV 255 SUMMARY -99999 99999 99999 99999\n");
    Task2Commands_Summary(&t, INFINITY, NAN, 1e9f, -1e9f);
    Event("EV 255 SUMMARY - - - -\n");

    /* A full outbox drops and counts rather than blocking; clearing empties it. */
    for (unsigned i = 0; i < TASK2COMMANDS_OUTBOX + 2U; ++i)
        Task2Commands_Summary(&t, 1.0f, 2.0f, 3.0f, (float)i);
    assert(t.outboxDropped == 2U);
    Event("EV 255 SUMMARY 1 2 3 0\n");
    Task2Commands_ClearEvents(&t);
    NoEvent();

    /* Stop/fault stages are recorded without an event. */
    Task2Commands_SetStage(&t, TASK2_STAGE_STOPPED);
    NoEvent();
    Expect("Q T", "T 255 STOPPED - -\n");
    Expect("A 255 1 L", "NAK 255 1 STATE\n");

    /* Arrow 1 previewed before START: replaceable and clearable, for planning only. */
    Task2Commands_Init(&t);
    Expect("A - 1 L", "ACK - 1 L\n");
    Expect("A - 1 R", "ACK - 1 R\n");
    assert(Task2Commands_Preview(&t) == TASK2_COMMAND_ARROW_RIGHT);
    Expect("A - 1 -", "ACK - 1 -\n");
    assert(Task2Commands_Preview(&t) == TASK2_COMMAND_ARROW_NONE);
    Expect("A - 2 L", "NAK - 2 STATE\n");
    const char *badPreview[] = { "A - 1 X", "A - 3 L", "A - 1", "A -  1 L", "A - 1 L ", "A -1 L" };
    for (size_t i = 0; i < sizeof(badPreview) / sizeof(badPreview[0]); ++i)
        Expect(badPreview[i], "NAK - PARSE\n");

    /* The run's arrow 1 comes only from its start line; a preview never carries over. */
    Expect("A - 1 L", "ACK - 1 L\n");
    Task2Commands_Begin(&t, 20, TASK2_COMMAND_ARROW_NONE);          /* plain T2 after a preview */
    Event("EV 20 APPROACH 1\n");
    assert(Task2Commands_Arrow(&t, 1) == TASK2_COMMAND_ARROW_NONE);
    assert(Task2Commands_Preview(&t) == TASK2_COMMAND_ARROW_NONE);
    Expect("A 20 1 R", "ACK 20 1 R\n");                            /* the camera decides it */
    Expect("A - 1 L", "NAK - 1 STATE\n");                          /* no preview during a run */
    Task2Commands_SetStage(&t, TASK2_STAGE_DONE);
    Expect("A - 1 L", "ACK - 1 L\n");                              /* between runs again */
    Task2Commands_Begin(&t, 21, TASK2_COMMAND_ARROW_RIGHT);         /* T2R: the start line wins */
    Event("EV 21 APPROACH 1\n");
    assert(Task2Commands_Arrow(&t, 1) == TASK2_COMMAND_ARROW_RIGHT);
    Expect("Q T", "T 21 APPROACH1 R -\n");
    Expect("A 21 1 R", "ACK 21 1 R\n");                            /* the Pi's resend after START */
    Expect("A 21 1 L", "NAK 21 1 ARROW\n");
    Task2Commands_SetStage(&t, TASK2_STAGE_DONE);
    Task2Commands_Begin(&t, 22, TASK2_COMMAND_ARROW_LEFT);          /* T2L with no preview at all */
    Event("EV 22 APPROACH 1\n");
    assert(Task2Commands_Arrow(&t, 1) == TASK2_COMMAND_ARROW_LEFT);

    puts("task2_commands_test: all passed");
    return 0;
}
