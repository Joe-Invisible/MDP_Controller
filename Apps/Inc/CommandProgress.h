#ifndef COMMAND_PROGRESS_H
#define COMMAND_PROGRESS_H

#include "CommandSession.h"

#define COMMANDPROGRESS_REPLY_SIZE 192U

typedef enum {
    COMMANDPROGRESS_NOT_STARTED,
    COMMANDPROGRESS_ACTIVE,
    COMMANDPROGRESS_BRAKING,
    COMMANDPROGRESS_IDLE
} CommandProgressPhase;

/* Only the current/latest command, not a history. Owned by MotionTask.
 * Distances/heading are relative to this command's start, never a global pose.
 */
typedef struct {
    unsigned step; /* 1-based attempted command, 0 before any attempt. */
    Command command;
    CommandProgressPhase phase;
    bool measured;
    float travelMm;
    float yawDeg;
} CommandProgress;

void CommandProgress_Attempt(CommandProgress *progress, unsigned step,
                              const Command *command);
void CommandProgress_Sample(CommandProgress *progress, float travelMm,
                            float yawDeg, CommandProgressPhase phase);
/* Format only when explicitly queried; invalid measurements are '-'. */
void CommandProgress_Format(const CommandSession *session,
    const CommandProgress *progress, char *reply, size_t size);

#endif
