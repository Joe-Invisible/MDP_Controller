#ifndef INC_COMMANDSESSION_H_
#define INC_COMMANDSESSION_H_

#include "CommandParser.h"

#define COMMANDSESSION_MAX_COMMANDS 8U
#define COMMANDSESSION_REPLY_SIZE 48U

typedef enum {
    COMMANDSESSION_READY,
    COMMANDSESSION_BUSY,
    COMMANDSESSION_DONE,
    COMMANDSESSION_FAULT,
    COMMANDSESSION_STOPPING,
    COMMANDSESSION_STOPPED,
} CommandSessionState;

/* One current/latest batch. Owned exclusively by MotionTask; no RTOS state. */
typedef struct {
    Command commands[COMMANDSESSION_MAX_COMMANDS];
    size_t count;
    size_t next;
    uint8_t seq;
    uint8_t expectedSeq;
    bool hasSeq;
    bool seqEstablished;
    CommandSessionState state;
    const char *faultReason; /* Static reason tokens only. */
} CommandSession;

void CommandSession_Init(CommandSession *session);

/*
 * Handles a line without its newline. Empty reply means accepted silently.
 * Returns true only when the caller must initiate braking (standalone S).
 * STATUS and duplicate frames only report state; they never start motion.
 */
bool CommandSession_Receive(CommandSession *session, const char *line,
                            char reply[COMMANDSESSION_REPLY_SIZE]);
const Command *CommandSession_Current(const CommandSession *session);
void CommandSession_CommandDone(CommandSession *session,
                                char reply[COMMANDSESSION_REPLY_SIZE]);
void CommandSession_Fault(CommandSession *session, const char *reason,
                          char reply[COMMANDSESSION_REPLY_SIZE]);
/* Call after braking has physically completed. */
void CommandSession_Stopped(CommandSession *session,
                            char reply[COMMANDSESSION_REPLY_SIZE]);

#endif
