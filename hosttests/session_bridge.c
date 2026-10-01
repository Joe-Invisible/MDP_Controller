/* Host-only bridge for the actual session and on-demand progress formatter.
 * @finish/@complete simulate completion; @sample travel yaw supplies simulated
 * odometry; @fault latches a failure. No hardware, timers, motors or accuracy model.
 */
#include "CommandSession.h"
#include "CommandProgress.h"
#include <stdio.h>
#include <string.h>

static void BeginCurrent(CommandSession *session, CommandProgress *progress)
{
    const Command *cmd=CommandSession_Current(session);
    if(cmd)CommandProgress_Attempt(progress,(unsigned)session->next+1U,cmd);
}
int main(void) {
    CommandSession session;CommandProgress progress={0};
    char line[128], reply[COMMANDPROGRESS_REPLY_SIZE];
    unsigned completed=0;
    CommandSession_Init(&session);
    while(fgets(line,sizeof(line),stdin)) {
        line[strcspn(line,"\r\n")]='\0';reply[0]='\0';
        if(strcmp(line,"Q P")==0) {
            CommandProgress_Format(&session,&progress,reply,sizeof(reply));
        } else if(strcmp(line,"@finish")==0 || strcmp(line,"@complete")==0) {
            while(CommandSession_Current(&session)) {
                ++completed;
                CommandProgress_Sample(&progress,100,0,COMMANDPROGRESS_IDLE);
                CommandSession_CommandDone(&session,reply);
                BeginCurrent(&session,&progress);
                if(strcmp(line,"@complete")==0)break;
            }
        } else if(strncmp(line,"@sample ",8)==0) {
            float travel,yaw;
            if(sscanf(line+8,"%f %f",&travel,&yaw)==2)
                CommandProgress_Sample(&progress,travel,yaw,COMMANDPROGRESS_ACTIVE);
        } else if(strcmp(line,"@fault")==0) {
            CommandSession_Fault(&session,"NO_PROGRESS",reply);
        } else if(strcmp(line,"@count")==0) {
            snprintf(reply,sizeof(reply),"%u\n",completed);
        } else {
            CommandSessionState before=session.state;
            if(CommandSession_Receive(&session,line,reply)) {
                if(progress.measured)
                    CommandProgress_Sample(&progress,progress.travelMm,progress.yawDeg,COMMANDPROGRESS_IDLE);
                CommandSession_Stopped(&session,reply);
            } else if(session.state==COMMANDSESSION_BUSY && before!=COMMANDSESSION_BUSY) {
                BeginCurrent(&session,&progress);
            }
        }
        fputs(reply[0]?reply:"-\n",stdout);fflush(stdout);
    }
    return 0;
}
