/* From MDP_Controller, no hardware:
 * cc -std=c11 -Wall -Wextra -Werror -IApps/Inc hosttests/progress_test.c
 *    Apps/Src/CommandProgress.c Apps/Src/Command{Parser,Session,Motion}.c
 *    -lm -o /tmp/progress_test && /tmp/progress_test
 */
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "CommandProgress.h"

int main(void)
{
    CommandSession session;CommandProgress progress={0};
    char reply[COMMANDSESSION_REPLY_SIZE], out[COMMANDPROGRESS_REPLY_SIZE];
    CommandSession_Init(&session);
    CommandProgress_Format(&session,&progress,out,sizeof(out));
    assert(strcmp(out,"P - READY 0 0 0 - - - - NOT_STARTED\n")==0);
    CommandSession_Receive(&session,"255:F100;B100;U200",reply);
    CommandProgress_Attempt(&progress,1,CommandSession_Current(&session));
    CommandProgress_Sample(&progress,100,0,COMMANDPROGRESS_IDLE);
    CommandSession_CommandDone(&session,reply);
    assert(session.next==1);
    CommandProgress_Attempt(&progress,2,CommandSession_Current(&session));
    CommandProgress_Format(&session,&progress,out,sizeof(out));
    assert(strcmp(out,"P 255 BUSY 1 3 2 B 100 - - NOT_STARTED\n")==0);
    CommandProgress_Sample(&progress,-42.25f,-1.5f,COMMANDPROGRESS_ACTIVE);
    CommandProgress_Format(&session,&progress,out,sizeof(out));
    assert(strcmp(out,"P 255 BUSY 1 3 2 B 100 -42.25 -1.5 ACTIVE\n")==0);
    CommandSession before=session;
    for(unsigned i=0;i<10000;++i)CommandProgress_Format(&session,&progress,out,sizeof(out));
    assert(memcmp(&before,&session,sizeof(session))==0); /* Query never advances a batch. */
    assert(CommandSession_Receive(&session,"S",reply));
    CommandProgress_Sample(&progress,-45,-2,COMMANDPROGRESS_BRAKING);
    CommandProgress_Format(&session,&progress,out,sizeof(out));
    assert(strstr(out,"STOPPING 1 3 2 B 100 -45 -2 BRAKING")!=NULL);
    CommandProgress_Sample(&progress,-48,-2.2f,COMMANDPROGRESS_IDLE);
    CommandSession_Stopped(&session,reply);
    CommandProgress_Format(&session,&progress,out,sizeof(out));
    assert(strcmp(out,"P 255 STOPPED 1 3 2 B 100 -48 -2.2 IDLE\n")==0);
    CommandSession_Receive(&session,"255:F100;B100;U200",reply);
    assert(strcmp(reply,"STOPPED 255\n")==0 && session.next==1);
    CommandSession_Receive(&session,"0:U200",reply);
    CommandProgress_Attempt(&progress,1,CommandSession_Current(&session));
    CommandSession_Fault(&session,"US_NOT_READY",reply);
    CommandProgress_Format(&session,&progress,out,sizeof(out));
    assert(strcmp(out,"P 0 FAULT 0 1 1 U 200 - - NOT_STARTED\n")==0);
    CommandProgress_Sample(&progress,NAN,0,COMMANDPROGRESS_IDLE);
    CommandProgress_Format(&session,&progress,out,sizeof(out));
    assert(strstr(out," - - IDLE")!=NULL); /* Invalid odometry is never fabricated as zero. */
    progress.command.param=FLT_MAX;
    CommandProgress_Sample(&progress,FLT_MAX,-FLT_MAX,COMMANDPROGRESS_IDLE);
    CommandProgress_Format(&session,&progress,out,sizeof(out));
    assert(strlen(out)<sizeof(out) && strchr(out,'\n')!=NULL);
    struct { char tiny[12];char guard[8]; } bounded;
    memset(&bounded,'X',sizeof(bounded));
    CommandProgress_Format(&session,&progress,bounded.tiny,sizeof(bounded.tiny));
    assert(memcmp(bounded.guard,"XXXXXXXX",8)==0);
    assert(bounded.tiny[sizeof(bounded.tiny)-1]=='\0');
    puts("progress_test: all checks passed");
}
