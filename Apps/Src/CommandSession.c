#include "CommandSession.h"
#include "CommandMotion.h"

#include <stdio.h>
#include <string.h>

static void Reply(char *reply, const char *kind, bool hasSeq, uint8_t seq,
                  const char *reason) {
    char id[4];
    if (hasSeq)
        snprintf(id, sizeof(id), "%u", (unsigned)seq);
    else
        strcpy(id, "-");
    snprintf(reply, COMMANDSESSION_REPLY_SIZE, "%s %s%s%s\n", kind, id,
             reason ? " " : "", reason ? reason : "");
}

static void Status(const CommandSession *session, char *reply) {
    static const char *const names[] = {
        "READY", "BUSY", "DONE", "FAULT", "STOPPING", "STOPPED"
    };
    if (session->state == COMMANDSESSION_READY) {
        strcpy(reply, "READY\n");
        return;
    }
    Reply(reply, names[session->state], session->hasSeq, session->seq,
          session->state == COMMANDSESSION_FAULT ? session->faultReason : NULL);
}

void CommandSession_Init(CommandSession *session) {
    memset(session, 0, sizeof(*session));
}

bool CommandSession_Receive(CommandSession *session, const char *line,
                            char reply[COMMANDSESSION_REPLY_SIZE]) {
    reply[0] = '\0';
    if (strcmp(line, "STATUS") == 0) {
        Status(session, reply);
        return false;
    }
    if (strcmp(line, "S") == 0) {
        if (session->state == COMMANDSESSION_STOPPING) {
            Status(session, reply);
        } else if (session->state == COMMANDSESSION_BUSY ||
                   session->state == COMMANDSESSION_FAULT) {
            session->state = COMMANDSESSION_STOPPING;
            return true;
        } else {
            /* Already stationary. Preserve the latest batch's outcome. */
            strcpy(reply, "STOPPED\n");
        }
        return false;
    }

    Command commands[COMMANDSESSION_MAX_COMMANDS];
    CommandFrame frame = {
        .commands = commands, .capacity = COMMANDSESSION_MAX_COMMANDS,
    };
    CommandParserStatus parsed = CommandParser_ParseFrame(line, &frame);
    if (parsed != COMMANDPARSER_OK) {
        Reply(reply, "NAK", frame.hasSeq, frame.seq,
              CommandParser_StatusName(parsed));
        return false;
    }

    /* Reject the entire batch before moving, including embedded stops. */
    for (size_t i = 0; i < frame.count; ++i) {
        CommandMotion motion;
        if (!CommandMotion_Resolve(&commands[i], &motion)) {
            Reply(reply, "NAK", frame.hasSeq, frame.seq,
                  commands[i].type == COMMAND_STOP ? "STOP_STANDALONE" : "RANGE");
            return false;
        }
    }

    if (frame.hasSeq && session->hasSeq && frame.seq == session->seq) {
        bool same = frame.count == session->count;
        for (size_t i = 0; same && i < frame.count; ++i)
            same = commands[i].type == session->commands[i].type &&
                   commands[i].param == session->commands[i].param;
        if (same)
            Status(session, reply);
        else
            Reply(reply, "NAK", true, frame.seq, "ID_REUSE");
        return false;
    }

    if (session->state == COMMANDSESSION_BUSY ||
        session->state == COMMANDSESSION_STOPPING ||
        session->state == COMMANDSESSION_FAULT) {
        Reply(reply, "NAK", frame.hasSeq, frame.seq,
              session->state == COMMANDSESSION_FAULT ? "FAULT" : "BUSY");
        return false;
    }
    if (session->seqEstablished && !frame.hasSeq) {
        Reply(reply, "NAK", false, 0, "SEQ_REQUIRED");
        return false;
    }
    if (frame.hasSeq && session->seqEstablished &&
        frame.seq != session->expectedSeq) {
        Reply(reply, "NAK", true, session->expectedSeq, "SEQ");
        return false;
    }

    memcpy(session->commands, commands, frame.count * sizeof(Command));
    session->count = frame.count;
    session->next = 0;
    session->seq = frame.seq;
    session->hasSeq = frame.hasSeq;
    session->faultReason = NULL;
    session->state = COMMANDSESSION_BUSY;
    if (frame.hasSeq) {
        session->seqEstablished = true;
        session->expectedSeq = (uint8_t)(frame.seq + 1U);
    }
    return false;
}

const Command *CommandSession_Current(const CommandSession *session) {
    return session->state == COMMANDSESSION_BUSY && session->next < session->count
        ? &session->commands[session->next] : NULL;
}

void CommandSession_CommandDone(CommandSession *session,
                                char reply[COMMANDSESSION_REPLY_SIZE]) {
    reply[0] = '\0';
    if (session->state != COMMANDSESSION_BUSY)
        return;
    if (++session->next == session->count) {
        session->state = COMMANDSESSION_DONE;
        Status(session, reply);
    }
}

void CommandSession_Fault(CommandSession *session, const char *reason,
                          char reply[COMMANDSESSION_REPLY_SIZE]) {
    session->state = COMMANDSESSION_FAULT;
    session->faultReason = reason;
    Status(session, reply);
}

void CommandSession_Stopped(CommandSession *session,
                            char reply[COMMANDSESSION_REPLY_SIZE]) {
    reply[0] = '\0';
    if (session->state == COMMANDSESSION_STOPPING) {
        session->state = COMMANDSESSION_STOPPED;
        Status(session, reply);
    }
}
