/* cc hosttests/session_test.c Apps/Src/Command{Parser,Session}.c -IApps/Inc -o /tmp/session_test */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "CommandSession.h"

static CommandSession session;
static char reply[COMMANDSESSION_REPLY_SIZE];

static void Receive(const char *line, const char *expected, bool brake) {
    assert(CommandSession_Receive(&session, line, reply) == brake);
    if (strcmp(reply, expected) != 0) {
        fprintf(stderr, "%s: expected [%s], got [%s]\n", line, expected, reply);
        assert(0);
    }
}

static void FinishCommand(const char *expected) {
    CommandSession_CommandDone(&session, reply);
    assert(strcmp(reply, expected) == 0);
}

int main(void) {
    CommandSession_Init(&session);
    Receive("STATUS", "READY\n", false);
    Receive("S", "STOPPED\n", false);

    /* Entire unsupported/mixed-stop frames are rejected before any move. */
    Receive("12:F100;L90;F100", "NAK 12 UNSUPPORTED\n", false);
    Receive("12:F100;S;F100", "NAK 12 STOP_STANDALONE\n", false);
    Receive("12:S", "NAK 12 STOP_STANDALONE\n", false);
    Receive("12:F1e999", "NAK 12 PARSE\n", false);
    Receive("12:F1;F1;F1;F1;F1;F1;F1;F1;F1", "NAK 12 TOO_MANY\n", false);
    assert(CommandSession_Current(&session) == NULL);

    /* Silent acceptance, bounded batch, completion only after the last move. */
    Receive("12:F100;B50", "", false);
    Receive("STATUS", "BUSY 12\n", false);
    Receive("13:F200", "NAK 13 BUSY\n", false);
    Receive("12:F100;B50", "BUSY 12\n", false);
    Receive("12:F200", "NAK 12 ID_REUSE\n", false);
    assert(CommandSession_Current(&session)->type == COMMAND_FORWARD);
    FinishCommand("");
    assert(CommandSession_Current(&session)->type == COMMAND_BACKWARD);
    Receive("12:F100;B50", "BUSY 12\n", false);
    assert(session.next == 1); /* A retry must not restart the batch. */
    FinishCommand("DONE 12\n");
    assert(CommandSession_Current(&session) == NULL);
    Receive("12:F100;B50", "DONE 12\n", false);
    Receive("STATUS", "DONE 12\n", false);
    Receive("S", "STOPPED\n", false);
    Receive("STATUS", "DONE 12\n", false);
    Receive("14:F100", "NAK 13 SEQ\n", false);
    Receive("F100", "NAK - SEQ_REQUIRED\n", false);

    /* Stop is independent of batch IDs and cancels all remaining moves. */
    Receive("13:F100;B50", "", false);
    Receive("S", "", true);
    assert(CommandSession_Current(&session) == NULL);
    Receive("STATUS", "STOPPING 13\n", false);
    Receive("S", "STOPPING 13\n", false); /* Do not restart the brake. */
    Receive("14:F100", "NAK 14 BUSY\n", false);
    FinishCommand(""); /* The interrupted move must not produce DONE. */
    CommandSession_Stopped(&session, reply);
    assert(strcmp(reply, "STOPPED 13\n") == 0);
    Receive("13:F100;B50", "STOPPED 13\n", false);
    Receive("12:F100;B50", "NAK 14 SEQ\n", false);

    /* Execution failure stays latched until S and physical stop. */
    Receive("14:F100;B50", "", false);
    CommandSession_Fault(&session, "UPDATE", reply);
    assert(strcmp(reply, "FAULT 14 UPDATE\n") == 0);
    assert(CommandSession_Current(&session) == NULL);
    FinishCommand("");
    Receive("15:F100", "NAK 15 FAULT\n", false);
    Receive("14:F100;B50", "FAULT 14 UPDATE\n", false);
    Receive("STATUS", "FAULT 14 UPDATE\n", false);
    Receive("S", "", true);
    Receive("15:F100", "NAK 15 BUSY\n", false);
    CommandSession_Stopped(&session, reply);
    assert(strcmp(reply, "STOPPED 14\n") == 0);
    Receive("15:F100", "", false);
    FinishCommand("DONE 15\n");

    /* Many successive batches need no history slots; exercise byte wrap. */
    for (unsigned n = 16; n < 520; ++n) {
        char frame[32], done[32];
        snprintf(frame, sizeof(frame), "%u:F1", n % 256);
        snprintf(done, sizeof(done), "DONE %u\n", n % 256);
        Receive(frame, "", false);
        FinishCommand(done);
        Receive(frame, done, false);
    }

    /* Unnumbered frames are bench-only; STATUS still works. */
    CommandSession_Init(&session);
    Receive("F1;B1", "", false);
    Receive("STATUS", "BUSY -\n", false);
    FinishCommand("");
    FinishCommand("DONE -\n");
    Receive("F1", "", false); /* Intentionally no dedup without an ID. */
    FinishCommand("DONE -\n");

    /* Maximum batch size, then recovery from a rejected next batch. */
    Receive("200:F1;F2;F3;F4;F5;F6;F7;F8", "", false);
    for (unsigned n = 0; n < 7; ++n)
        FinishCommand("");
    FinishCommand("DONE 200\n");
    Receive("201:R90", "NAK 201 UNSUPPORTED\n", false);
    Receive("201:F10", "", false);
    FinishCommand("DONE 201\n");

    puts("session_test: all checks passed");
    return 0;
}
