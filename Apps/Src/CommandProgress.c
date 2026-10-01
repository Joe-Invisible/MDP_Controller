#include "CommandProgress.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

void CommandProgress_Attempt(CommandProgress *progress, unsigned step,
                              const Command *command)
{
    *progress = (CommandProgress){.step=step, .command=*command,
        .phase=COMMANDPROGRESS_NOT_STARTED};
}

void CommandProgress_Sample(CommandProgress *progress, float travelMm,
                            float yawDeg, CommandProgressPhase phase)
{
    progress->phase = phase;
    progress->measured = isfinite(travelMm) && isfinite(yawDeg);
    progress->travelMm = travelMm;
    progress->yawDeg = yawDeg;
}

void CommandProgress_Format(const CommandSession *session,
    const CommandProgress *progress, char *reply, size_t size)
{
    static const char *const states[] = {"READY","BUSY","DONE","FAULT","STOPPING","STOPPED"};
    static const char *const phases[] = {"NOT_STARTED","ACTIVE","BRAKING","IDLE"};
    static const char letters[] = "FBLRSU";
    char id[4]="-", parameter[24]="-", travel[24]="-", yaw[24]="-";
    char command='-';
    if (session->hasSeq) (void)snprintf(id,sizeof(id),"%u",(unsigned)session->seq);
    if (progress->step && (unsigned)progress->command.type < sizeof(letters)-1U) {
        command=letters[progress->command.type];
        (void)snprintf(parameter,sizeof(parameter),"%.7g",(double)progress->command.param);
    }
    if (progress->measured) {
        (void)snprintf(travel,sizeof(travel),"%.7g",(double)progress->travelMm);
        (void)snprintf(yaw,sizeof(yaw),"%.6g",(double)progress->yawDeg);
    }
    int written=snprintf(reply,size,"P %s %s %u %u %u %c %s %s %s %s\n",
        id, (unsigned)session->state < sizeof(states)/sizeof(states[0])
            ? states[session->state] : "UNKNOWN",
        (unsigned)session->next,(unsigned)session->count,progress->step,command,
        parameter,travel,yaw,
        (unsigned)progress->phase < sizeof(phases)/sizeof(phases[0])
            ? phases[progress->phase] : "UNKNOWN");
    if (written < 0 || (size_t)written >= size)
        (void)snprintf(reply,size,"P OVERFLOW\n");
}
