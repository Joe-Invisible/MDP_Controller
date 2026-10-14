#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "Task2Run.h"
#include "CommandSession.h"

#define PI 3.14159265358979323846f
static Task2Run run;
static CommandSession session;
static bool summarySeen, expectFault;
static uint32_t now;
static Task2MotionReading motion;
static UltrasonicReading uss;
static IRPairReading infrared;
static Task2RoutePose physical;
static float remaining, radius, cruise, physicalGap, physicalWidth;
static bool continuous, stopSensors, initialAbsent, irFault, stuckUSS, forceAcquisition;
static unsigned moves, brakes, brakeTicks, samples;
static char reply[COMMANDSESSION_REPLY_SIZE];

static const char *Move(void *ctx, float d, float r, float v) {
    (void)ctx; assert(!motion.moving); assert(isfinite(d) && isfinite(r) && v > 0);
    ++moves; motion = (Task2MotionReading){.moving = true};
    remaining = d; radius = r; cruise = v; continuous = false; return NULL;
}
static const char *Straight(void *c, float d, float v) { return Move(c,d,0,v); }
static const char *Arc(void *c, float d, float r, float v) { return Move(c,d,r,v); }
static const char *Continuous(void *c, float v) { const char *f=Move(c,1,0,v); continuous=true; return f; }
static bool Busy(void *c) { (void)c; return motion.moving; }
static void Brake(void *c) { (void)c; ++brakes; if(motion.moving && !motion.braking) {motion.braking=true; brakeTicks=2;} }
static const char *Speed(void *c, float v) { (void)c; assert(motion.moving && !motion.braking); cruise=v; return NULL; }
static Task2MotionReading Motion(void *c) { (void)c; return motion; }
static UltrasonicReading USS(void *c) { (void)c; return uss; }
static IRPairReading IR(void *c) { (void)c; return infrared; }
static const Task2CourseIo io = {.moveStraight=Straight,.moveArc=Arc,.moveContinuous=Continuous,
    .busy=Busy,.brake=Brake,.requestSpeed=Speed,.motion=Motion,.ultrasonic=USS,.infrared=IR,
    .mmPerCount=PI*65.5f/1560,.decelerationMmps2=2000};

static void Advance(void) {
    if (motion.moving) {
        if (motion.braking) { if (!--brakeTicks) motion.moving=motion.braking=false; }
        else {
            float ds=fminf(fabsf(remaining),cruise*io.mmPerCount*0.01f);
            if(continuous) ds=cruise*io.mmPerCount*0.01f;
            if(!continuous && remaining<0)ds=-ds;
            float yaw=radius ? ds/radius : 0;
            float old=physical.yawRad;
            if(radius) { physical.xMm+=radius*(sinf(old+yaw)-sinf(old)); physical.yMm+=radius*(cosf(old)-cosf(old+yaw)); }
            else { physical.xMm+=ds*cosf(old); physical.yMm+=ds*sinf(old); }
            physical.yawRad+=yaw; motion.distanceMm+=ds; motion.yawRad+=yaw; remaining-=ds;
            if(!continuous && fabsf(remaining)<0.001f) {motion.braking=true;brakeTicks=2;}
        }
    }
    now+=10U;
    if(stopSensors)return;
    Task2App *a=&run.course.app;
    if(now%70U<10U) {
        ++samples;
        float x=physical.xMm+210*cosf(physical.yawRad), y=physical.yMm+210*sinf(physical.yawRad);
        bool second=run.commands.stage>=TASK2_STAGE_APPROACH2;
        float face=1050+(second ? 100+physicalGap : 0);
        float range=INFINITY;
        for(unsigned i=0;i<=120;++i) {
            float beam=physical.yawRad+(7.5f*PI/180)*(2.0f*i/120-1);
            if(cosf(beam)<0.05f || face<=x)continue;
            float d=(face-x)/cosf(beam);
            if(fabsf(y+d*sinf(beam))<=(second ? physicalWidth : 200)/2)range=fminf(range,d);
        }
        if(stuckUSS && second)range=(face-x)/cosf(fmaxf(0,fabsf(physical.yawRad)-7.5f*PI/180));
        /* Isolated uneven-surface readings must not bias either ranging median. */
        if ((a->phase==TASK2_APP_RANGE1 || a->phase==TASK2_APP_RANGE2) && samples%5==0) range=2400;
        if(forceAcquisition && a->phase==TASK2_APP_ROUTE && a->step==3U) range=INFINITY;
        uss=(UltrasonicReading){.sequence=uss.sequence+1,.capturedMs=now,
            .status=isfinite(range) ? SENSOR_OK : SENSOR_TIMEOUT,.distanceMm=isfinite(range)?range:0};
    }
    if(now%50U<10U) {
        ++infrared.sequence;
        for(unsigned side=0;side<2;++side) {
            Task2SensorMount m=side ? a->sensors.rightIR : a->sensors.leftIR;
            float x=physical.xMm+m.aheadMm*cosf(physical.yawRad)-m.leftMm*sinf(physical.yawRad);
            float y=physical.yMm+m.aheadMm*sinf(physical.yawRad)+m.leftMm*cosf(physical.yawRad);
            float beam=physical.yawRad+m.yawDeg*PI/180;
            float face=1150+physicalGap;
            bool onFace=cosf(beam)>0.05f && face>x && fabsf(y+(face-x)*tanf(beam))<=physicalWidth/2;
            IRReading value={.capturedMs=now,.status=irFault ? SENSOR_ERROR : SENSOR_OK,
                .adc=initialAbsent ? 300 : onFace ? 900 : 300};
            if(side)infrared.right=value;else infrared.left=value;
        }
    }
}
static void Drain(void) {
    while(Task2Run_NextEvent(&run,reply)) {
        if(strstr(reply,"SUMMARY")) { assert(!motion.moving);summarySeen=true; }
    }
}
static Task2RunStatus Tick(void) {
    Advance(); const char *fault=NULL;
    Task2RunStatus result=Task2Run_Update(&run,motion.moving,now,&fault);
    if(result==TASK2RUN_FAULT && !expectFault) {fprintf(stderr,"width %.1f gap %.1f min %.1f budget %u\n",run.course.app.detection.observedWidthMm,run.course.app.geometry.obstacleFaceGapMm,run.course.app.route.report.minimumObstacleGapMm,run.course.app.route.report.plannerPoseChecks);fprintf(stderr,"fault %s phase %u step %u pose %.1f %.1f %.3f\n",fault,run.course.app.phase,run.course.app.step,physical.xMm,physical.yMm,physical.yawRad);}
    Drain(); return result;
}
static void Reset(uint32_t tick) {
    Task2Run_Init(&run); CommandSession_Init(&session); summarySeen=expectFault=false; now=tick; motion=(Task2MotionReading){0};uss=(UltrasonicReading){0};infrared=(IRPairReading){0};
    physical=(Task2RoutePose){-250,0,0}; moves=brakes=brakeTicks=samples=0;
    stopSensors=initialAbsent=irFault=stuckUSS=forceAcquisition=false; physicalGap=1050;physicalWidth=500;
}
static void Send(const char *line, const char *expected) {
    assert(Task2Run_Receive(&run,line,reply)); assert(strcmp(reply,expected)==0);
}
static void RunTo(Task2AppPhase phase) {
    for(unsigned n=0;run.course.app.phase!=phase && n<12000;++n)assert(Tick()==TASK2RUN_RUNNING);
    assert(run.course.app.phase==phase);
}
static void Complete(char first, char second, float gap, float width, bool absence) {
    Reset(0); physicalGap=gap; physicalWidth=width; initialAbsent=absence;
    char batch[32];snprintf(batch,sizeof(batch),"12:T2%c",first);
    assert(!CommandSession_Receive(&session,batch,reply));assert(reply[0] == '\0');
    const Command *cmd=CommandSession_Current(&session);assert(cmd && cmd->type==COMMAND_TASK2);
    assert(Task2Run_Start(&run,session.seq,cmd->param>0 ? TASK2_COMMAND_ARROW_LEFT:TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
    RunTo(TASK2_APP_RANGE2);
    char message[32],expected[32];snprintf(message,sizeof(message),"A 12 2 %c",second);snprintf(expected,sizeof(expected),"ACK 12 2 %c\n",second);Send(message,expected);
    Task2RunStatus status=TASK2RUN_RUNNING;
    for(unsigned n=0;n<12000 && status==TASK2RUN_RUNNING;++n)status=Tick();
    assert(status==TASK2RUN_DONE && !motion.moving && run.commands.stage==TASK2_STAGE_DONE);
    assert(run.course.app.detection.detected && summarySeen);
    CommandSession_CommandDone(&session,reply);assert(strcmp(reply,"DONE 12\n")==0);
    assert(fabsf(run.course.app.estimates.faceGap.mm-gap)<15);
    assert(Task2Route_IsParked(&run.course.app.geometry,&physical));
    printf("PASS %c%c gap %.0f width %.0f%s: %u primitives\n",first,second,gap,width,absence?" initial absence":"",moves);
}
int main(void) {
    setbuf(stdout,NULL);
    for(unsigned gap=600;gap<=1500;gap+=450) {
        Complete('L','R',gap,500,false);Complete('R','L',gap,500,false);
        Complete('L','L',gap,500,false);Complete('R','R',gap,500,false);
    }
    Complete('L','R',1050,300,true);Complete('R','L',1050,300,true);
    /* Window samples must be fresh; a copied snapshot cannot satisfy the median. */
    Reset(0xfffffff0U);assert(Task2Run_Start(&run,7,TASK2_COMMAND_ARROW_NONE,&io,now)==NULL);
    stopSensors=true;uss=(UltrasonicReading){.sequence=1,.capturedMs=now,.status=SENSOR_OK,.distanceMm=1090};
    for(unsigned n=0;n<20;++n)assert(Tick()==TASK2RUN_RUNNING);
    assert(run.course.app.estimates.obstacle1Face.valid==false && run.course.app.faceCount<=1 && !moves);
    stopSensors=false;RunTo(TASK2_APP_WAIT1);unsigned before=moves;
    for(unsigned n=0;n<13000;++n)assert(Tick()==TASK2RUN_RUNNING);
    assert(moves==before && !motion.moving); /* Waiting excludes run motion timeout. */
    Send("A 8 1 R","NAK 8 1 STATE\n");Send("A 7 2 R","NAK 7 2 STATE\n");
    Send("A 7 1 L","ACK 7 1 L\n");RunTo(TASK2_APP_WAIT2);
    before=moves;for(unsigned n=0;n<50;++n)assert(Tick()==TASK2RUN_RUNNING);assert(before==moves);
    Send("A 7 2 R","ACK 7 2 R\n");RunTo(TASK2_APP_ROUTE);
    Brake(NULL);Task2Run_Stop(&run);assert(!run.active);
    while(motion.moving)Advance();
    Task2Run_Stationary(&run);assert(run.commands.stage==TASK2_STAGE_STOPPED);
    assert(Task2Run_Start(&run,8,TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
    assert(!run.course.app.estimates.faceGap.valid && !run.course.app.detection.detected && !run.commands.arrows[1]);
    puts("PASS late arrows, wrap, cancellation and per-run reset");
    Reset(0);assert(Task2Run_Start(&run,9,TASK2_COMMAND_ARROW_LEFT,&io,now)==NULL);
    stopSensors=expectFault=true;Task2RunStatus status=TASK2RUN_RUNNING;for(unsigned n=0;n<300 && status==TASK2RUN_RUNNING;++n)status=Tick();
    assert(status==TASK2RUN_FAULT && moves==0 && brakes>0);Task2Run_Fault(&run);assert(run.commands.stage==TASK2_STAGE_FAULT);
    puts("PASS missing startup range fails stationary");
    Reset(0);assert(Task2Run_Start(&run,10,TASK2_COMMAND_ARROW_LEFT,&io,now)==NULL);
    RunTo(TASK2_APP_RANGE2);Send("A 10 2 R","ACK 10 2 R\n");
    while(!(run.course.app.phase==TASK2_APP_ROUTE && run.course.app.step==5U))assert(Tick()==TASK2RUN_RUNNING);
    irFault=expectFault=true;status=TASK2RUN_RUNNING;for(unsigned n=0;n<30 && status==TASK2RUN_RUNNING;++n)status=Tick();
    assert(status==TASK2RUN_FAULT && run.course.app.fault && !strcmp(run.course.app.fault,"T2_IR_SENSOR"));
    Reset(0);assert(Task2Run_Start(&run,11,TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
    RunTo(TASK2_APP_RANGE2);Send("A 11 2 R","ACK 11 2 R\n");stuckUSS=expectFault=true;
    status=TASK2RUN_RUNNING;for(unsigned n=0;n<12000 && status==TASK2RUN_RUNNING;++n)status=Tick();
    assert(status==TASK2RUN_FAULT && !run.course.app.detection.detected && !strcmp(run.course.app.fault,"T2_NO_EDGE"));
    Reset(0);assert(Task2Run_Start(&run,13,TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
    RunTo(TASK2_APP_RANGE2);Send("A 13 2 L","ACK 13 2 L\n");
    while(!(run.course.app.phase==TASK2_APP_ROUTE && run.course.app.step==5U))assert(Tick()==TASK2RUN_RUNNING);
    stopSensors=expectFault=true;status=TASK2RUN_RUNNING;for(unsigned n=0;n<30 && status==TASK2RUN_RUNNING;++n)status=Tick();
    assert(status==TASK2RUN_FAULT && !strcmp(run.course.app.fault,"T2_SENSOR_STALE"));
    puts("PASS IR failure, finite USS no-edge cap and stale-sensor braking");
    Reset(0);physicalGap=600;physicalWidth=600;
    assert(Task2Run_Start(&run,14,TASK2_COMMAND_ARROW_LEFT,&io,now)==NULL);
    RunTo(TASK2_APP_RANGE2);Send("A 14 2 R","ACK 14 2 R\n");forceAcquisition=true;RunTo(TASK2_APP_ACQUIRE2);
    Send("W 14 600","ACK 14 W 600\n");RunTo(TASK2_APP_ROUTE);
    assert(run.commands.stage==TASK2_STAGE_ROUND2 && run.course.app.route.config.obstacle2WidthMm==600);
    assert(run.course.app.estimates.cameraSpan.bound && run.course.app.estimates.cameraSpan.mm==600);
    Send("W 14 601","NAK 14 W STATE\n");
    status=TASK2RUN_RUNNING;for(unsigned n=0;n<12000 && status==TASK2RUN_RUNNING;++n)status=Tick();
    assert(status==TASK2RUN_DONE && Task2Route_IsParked(&run.course.app.geometry,&physical));
    puts("PASS late camera span consumed at measured-stop departure commitment");
    Task2Estimate estimates;Task2Estimate_Init(&estimates,&task2RouteConfig);
    assert(Task2Estimate_SetFace(&estimates,1,1050,20));assert(Task2Estimate_SetGap(&estimates,600,20,TASK2_ESTIMATE_CAMERA));
    Task2Estimate_SetSpan(&estimates,350);Task2RouteConfig geometry=task2RouteConfig;
    assert(Task2Estimate_Apply(&estimates,&geometry));assert(geometry.obstacle2WidthMm==500 && estimates.cameraSpan.bound);
    Task2Estimate_SetSpan(&estimates,600);assert(Task2Estimate_Apply(&estimates,&geometry));assert(geometry.obstacle2WidthMm==600);
    assert(fabsf(geometry.obstacle2EndSpaceMm+geometry.obstacle2WidthMm/2-750)<0.01f);
    assert(!Task2Estimate_SetGap(&estimates,NAN,20,TASK2_ESTIMATE_ULTRASONIC));
    puts("PASS replaceable estimates and camera lower-bound semantics");
}
