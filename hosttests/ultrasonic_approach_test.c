/* From MDP_Controller: compile with Apps/Src/UltrasonicApproach.c,
 * CommandParser.c, CommandSession.c, CommandMotion.c and the unchanged
 * Controllers/Src/MotionProfile.c. Includes: Apps/Inc, Controllers/Inc.
 * Run both defaults and -DULTRASONIC_APPROACH_CRUISE_MMPS=300.0f.
 * Pure host tests: no hardware or firmware flashing.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "UltrasonicApproach.h"
#include "MotionProfile.h"
#include "CommandSession.h"

static UltrasonicReading Reading(float distance, uint32_t now, uint32_t seq)
{
    return (UltrasonicReading){.sequence=seq, .capturedMs=now,
        .pulseUs=2000, .distanceMm=distance, .status=SENSOR_OK};
}
static void Start(UltrasonicApproach *s, UltrasonicReading *r)
{
    *r=Reading(500,1000,1);
    assert(UltrasonicApproach_Start(s,200,r,1000)==NULL);
    assert(s->travelLimitMm==350 && s->profileTargetMm==300);
}
static void PreconditionTests(void)
{
    UltrasonicApproach s; UltrasonicReading r;
    const float bad[]={0,99.9f,1000.1f,NAN,INFINITY};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i){
        r=Reading(500,0,1);
        assert(strcmp(UltrasonicApproach_Start(&s,bad[i],&r,0),"US_TARGET")==0);
    }
    Start(&s,&r);r.status=SENSOR_TIMEOUT;
    assert(strcmp(UltrasonicApproach_Start(&s,200,&r,1000),"US_ECHO")==0);
    r.status=SENSOR_OK;r.distanceMm=NAN;
    assert(strcmp(UltrasonicApproach_Start(&s,200,&r,1000),"US_SENSOR")==0);
    r=Reading(500,0,1);
    assert(strcmp(UltrasonicApproach_Start(&s,200,&r,151),"US_STALE")==0);
    r=Reading(179,0,1);
    assert(strcmp(UltrasonicApproach_Start(&s,200,&r,0),"US_TOO_CLOSE")==0);
    r=Reading(1151,0,1);
    assert(strcmp(UltrasonicApproach_Start(&s,200,&r,0),"US_TRAVEL")==0);
    r=Reading(1150,0,1);
    assert(UltrasonicApproach_Start(&s,200,&r,0)==NULL);
    assert(s.travelLimitMm==1000);
}
static void ProfileEndpointTests(void)
{
    UltrasonicApproach s;UltrasonicReading r;
    Start(&s,&r);
    /* Regression: 300 mm/s and 180 mm remaining means gradual deceleration,
     * not a full brake. This input triggered BRAKE in the old implementation.
     */
    r=Reading(380,1100,2);
    assert(UltrasonicApproach_Update(&s,&r,1100,120,300,true,false)==ULTRASONIC_APPROACH_DRIVE);
    assert(s.profileTargetMm==300);
    r=Reading(400,1170,3);
    assert(UltrasonicApproach_Update(&s,&r,1200,100,100,true,false)==ULTRASONIC_APPROACH_DRIVE);
    assert(s.profileTargetMm==297); /* Capture age 30 ms -> 3 mm compensation. */
    assert(UltrasonicApproach_Update(&s,&r,1220,102,100,true,false)==ULTRASONIC_APPROACH_DRIVE);
    assert(s.profileTargetMm==297); /* Cached echo must not move the endpoint. */
    r=Reading(500,1240,4);
    assert(UltrasonicApproach_Update(&s,&r,1240,110,100,true,false)==ULTRASONIC_APPROACH_DRIVE);
    assert(s.profileTargetMm==350 && s.travelLimitMm==350); /* Bounded target recession. */
}
static void CompletionTests(void)
{
    UltrasonicApproach s;UltrasonicReading r;
    Start(&s,&r);
    r=Reading(209,1100,2);
    assert(UltrasonicApproach_Update(&s,&r,1100,291,60,true,false)==ULTRASONIC_APPROACH_BRAKE);
    assert(UltrasonicApproach_Update(&s,&r,1110,294,20,true,true)==ULTRASONIC_APPROACH_BRAKE);
    assert(UltrasonicApproach_Update(&s,&r,1120,296,0,false,false)==ULTRASONIC_APPROACH_VERIFY);
    assert(UltrasonicApproach_Update(&s,&r,1130,296,0,false,false)==ULTRASONIC_APPROACH_VERIFY);
    r=Reading(204,1115,3); /* New sequence but pre-stop capture cannot certify the gap. */
    assert(UltrasonicApproach_Update(&s,&r,1130,296,0,false,false)==ULTRASONIC_APPROACH_VERIFY);
    r=Reading(204,1140,4);
    assert(UltrasonicApproach_Update(&s,&r,1140,296,0,false,false)==ULTRASONIC_APPROACH_VERIFY);
    assert(UltrasonicApproach_Update(&s,&r,1150,296,0,false,false)==ULTRASONIC_APPROACH_VERIFY);
    r=Reading(204,1210,5);
    assert(UltrasonicApproach_Update(&s,&r,1210,296,0,false,false)==ULTRASONIC_APPROACH_DONE);
    /* Already at target: brake and verify before any forward control tick. */
    r=Reading(200,0,1);assert(UltrasonicApproach_Start(&s,200,&r,0)==NULL);
    assert(UltrasonicApproach_Update(&s,&r,0,0,0,true,false)==ULTRASONIC_APPROACH_BRAKE);
    /* A profile endpoint crossed between control ticks must latch braking,
     * even if a new echo would move the endpoint farther away. */
    Start(&s,&r);r=Reading(240,1100,2);
    assert(UltrasonicApproach_Update(&s,&r,1100,292,40,true,true)==ULTRASONIC_APPROACH_BRAKE);
    assert(UltrasonicApproach_Update(&s,&r,1110,294,0,false,false)==ULTRASONIC_APPROACH_VERIFY);
    r=Reading(240,1180,3);
    assert(UltrasonicApproach_Update(&s,&r,1180,294,0,false,false)==ULTRASONIC_APPROACH_FAULT);
    assert(strcmp(s.fault,"US_RANGE")==0); /* Never auto-restart to correct the gap. */
}
static void FaultTests(void)
{
    UltrasonicApproach s;UltrasonicReading r;
    Start(&s,&r);
    assert(UltrasonicApproach_Update(&s,&r,1151,0,0,true,false)==ULTRASONIC_APPROACH_FAULT);
    assert(strcmp(s.fault,"US_STALE")==0);
    Start(&s,&r);r.status=SENSOR_TIMEOUT;
    assert(UltrasonicApproach_Update(&s,&r,1001,2,50,true,false)==ULTRASONIC_APPROACH_FAULT);
    assert(strcmp(s.fault,"US_ECHO")==0);
    r.status=SENSOR_OK;
    assert(UltrasonicApproach_Update(&s,&r,1002,2,50,true,false)==ULTRASONIC_APPROACH_FAULT);
    Start(&s,&r);r=Reading(400,16000,2);
    assert(UltrasonicApproach_Update(&s,&r,16000,50,50,true,false)==ULTRASONIC_APPROACH_FAULT);
    assert(strcmp(s.fault,"US_TIMEOUT")==0);
    Start(&s,&r);
    assert(UltrasonicApproach_Update(&s,&r,1001,349,60,true,false)==ULTRASONIC_APPROACH_FAULT);
    assert(strcmp(s.fault,"US_TRAVEL")==0);
    Start(&s,&r);
    assert(UltrasonicApproach_Update(&s,&r,1001,100,0,false,false)==ULTRASONIC_APPROACH_FAULT);
    assert(strcmp(s.fault,"US_STATE")==0);
    Start(&s,&r);
    assert(UltrasonicApproach_Update(&s,&r,1001,NAN,0,true,false)==ULTRASONIC_APPROACH_FAULT);
    assert(strcmp(s.fault,"US_ODOMETRY")==0);
    for(int tooClose=0;tooClose<2;++tooClose){
        Start(&s,&r);r=Reading(205,1100,2);
        assert(UltrasonicApproach_Update(&s,&r,1100,290,60,true,false)==ULTRASONIC_APPROACH_BRAKE);
        assert(UltrasonicApproach_Update(&s,&r,1110,295,0,false,false)==ULTRASONIC_APPROACH_VERIFY);
        r=Reading(tooClose?179:230,1170,3);
        assert(UltrasonicApproach_Update(&s,&r,1170,295,0,false,false)==ULTRASONIC_APPROACH_FAULT);
        assert(strcmp(s.fault,tooClose?"US_TOO_CLOSE":"US_RANGE")==0);
    }
}
static void WrapTests(void)
{
    UltrasonicApproach s;UltrasonicReading r=Reading(500,UINT32_MAX-10,UINT32_MAX);
    assert(UltrasonicApproach_Start(&s,200,&r,UINT32_MAX-10)==NULL);
    r=Reading(400,UINT32_MAX-5,0);
    assert(UltrasonicApproach_Update(&s,&r,4,100,100,true,false)==ULTRASONIC_APPROACH_DRIVE);
    assert(fabsf(s.profileTargetMm-299)<.001f);
    r=Reading(205,15,1);
    assert(UltrasonicApproach_Update(&s,&r,20,295,10,true,false)==ULTRASONIC_APPROACH_BRAKE);
    assert(UltrasonicApproach_Update(&s,&r,30,296,0,false,false)==ULTRASONIC_APPROACH_VERIFY);
    r=Reading(204,40,2);
    assert(UltrasonicApproach_Update(&s,&r,40,296,0,false,false)==ULTRASONIC_APPROACH_VERIFY);
    r=Reading(204,110,3);
    assert(UltrasonicApproach_Update(&s,&r,110,296,0,false,false)==ULTRASONIC_APPROACH_DONE);
}
static void SessionTests(void)
{
    CommandSession session;char reply[COMMANDSESSION_REPLY_SIZE];
    CommandSession_Init(&session);
    CommandSession_Receive(&session,"60:F100;U99;B100",reply);
    assert(strcmp(reply,"NAK 60 RANGE\n")==0 && CommandSession_Current(&session)==NULL);
    CommandSession_Receive(&session,"60:U200;B100",reply);
    assert(reply[0]==0 && CommandSession_Current(&session)->type==COMMAND_ULTRASONIC);
    CommandSession_Receive(&session,"60:U200;B100",reply);
    assert(strcmp(reply,"BUSY 60\n")==0 && session.next==0);
    CommandSession_Receive(&session,"Q",reply);
    assert(strcmp(reply,"BUSY 60\n")==0);
    assert(CommandSession_Receive(&session,"S",reply));
    assert(CommandSession_Current(&session)==NULL);
    CommandSession_CommandDone(&session,reply);assert(reply[0]==0);
    CommandSession_Stopped(&session,reply);assert(strcmp(reply,"STOPPED 60\n")==0);
    CommandSession_Receive(&session,"61:U200;F100",reply);
    CommandSession_Fault(&session,"US_ECHO",reply);
    assert(strcmp(reply,"FAULT 61 US_ECHO\n")==0);
    assert(CommandSession_Current(&session)==NULL);
    CommandSession_CommandDone(&session,reply);assert(reply[0]==0);
}
/* Exercise the REAL shared profile, not a second copy of its equations.
 * A bounded synthetic plant models wheel acceleration/deceleration, delayed
 * 60-100 ms echoes, and +/-3 mm deterministic measurement noise.
 */
static float minGap=1e9f,maxGap=0;
static unsigned simulations;
static void SimulatedApproach(float initial, uint32_t period, uint32_t delay,
                              float brakeDecel, bool noisy)
{
    UltrasonicApproach s;UltrasonicReading r=Reading(initial,0,1);
    assert(UltrasonicApproach_Start(&s,200,&r,0)==NULL);
    MotionProfile profile;
    assert(MotionProfile_Init(&profile,500,250,ULTRASONIC_APPROACH_STOP_MARGIN_MM));
    assert(MotionProfile_Start(&profile,s.travelLimitMm,ULTRASONIC_APPROACH_CRUISE_MMPS));
    float history[1501]={0};
    float progress=0,speed=0;bool braking=false,busy=true;
    uint32_t nextSample=period;
    for(uint32_t t=0;t<15000;t+=10){
        history[t/10]=progress;
        if(t>=nextSample){
            uint32_t captured=t-delay;
            float noise=noisy ? ((r.sequence%3)-1.0f)*3.0f : 0.0f;
            r=Reading(initial-history[captured/10]+noise,captured,r.sequence+1);
            nextSample=t+period;
        }
        UltrasonicApproachAction a=UltrasonicApproach_Update(&s,&r,t,progress,speed,busy,braking&&busy);
        if(a==ULTRASONIC_APPROACH_DONE){
            float final=initial-progress;
            assert(!busy && fabsf(final-200)<=20);
            minGap=fminf(minGap,final);maxGap=fmaxf(maxGap,final);++simulations;
            return;
        }
        if(a==ULTRASONIC_APPROACH_FAULT){
            fprintf(stderr,"start=%.0f period=%u delay=%u brake=%.0f noise=%d t=%u gap=%.2f fault=%s\n",initial,period,delay,brakeDecel,noisy,t,initial-progress,s.fault);
            assert(a!=ULTRASONIC_APPROACH_FAULT);
        }
        if(a==ULTRASONIC_APPROACH_BRAKE)braking=true;
        if(!braking){
            profile.targetDistanceMm=s.profileTargetMm;
            float request=MotionProfile_Update(&profile,progress,.01f);
            if(!MotionProfile_IsActive(&profile))braking=true;
            else speed=fmaxf(speed-brakeDecel*.01f,fminf(request,speed+500*.01f));
        }
        if(braking){speed=fmaxf(0,speed-brakeDecel*.01f);if(speed==0)busy=false;}
        progress+=speed*.01f;
    }
    assert(!"approach did not finish");
}
int main(void)
{
    PreconditionTests();ProfileEndpointTests();CompletionTests();FaultTests();WrapTests();SessionTests();
    const unsigned starts[]={400,500,560,800,1000};
    for(unsigned period=60;period<=100;period+=10)
        for(unsigned i=0;i<sizeof(starts)/sizeof(starts[0]);++i)
            for(unsigned delay=0;delay<=40;delay+=20)
                for(unsigned brake=250;brake<=1000;brake*=2)
                    for(unsigned noise=0;noise<=1;++noise)
                        SimulatedApproach(starts[i],period,delay,brake,noise);
    printf("ultrasonic_approach_test: policy checks + %u simulated approaches at %.0f mm/s, final %.2f..%.2f mm (not floor accuracy)\n",simulations,(double)ULTRASONIC_APPROACH_CRUISE_MMPS,(double)minGap,(double)maxGap);
    return 0;
}
