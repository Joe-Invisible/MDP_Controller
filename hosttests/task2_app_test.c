#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "SensorConfig.h"
#include "Task2Run.h"
#include "Task2Wire.h"
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
static float remaining, radius, cruise, physicalGap, physicalWidth, physicalO1 = 1050; /* O1 face x */
static bool continuous, stopSensors, initialAbsent, irFault, stuckUSS, frozenUSS, forceAcquisition, missedEcho;
static bool strayEcho, strayEchoDone; /* one false short echo at the end of the O2 approach */
static float ussShortMm, irOvershootMm; /* ultrasonic reading short in round 2; IR "present" past O2's end */
static unsigned irFaceAdc = 900, irPastEndAdc = 900; /* face reading, and the reading in the overshoot */
static float missedEchoExtraMm = 700; /* how far beyond O2 a lost RANGE2 echo reads */
static bool noEchoReversing; /* the ultrasonic hears nothing while a recovery reverse runs */
static float irLowFromMm; static unsigned irLowAdc; /* the face reads irLowAdc beyond irLowFromMm from its centre */
/* Measured IR model: readings from the true range on the face, from our sensors' 30 Sep tissue-face medians
 * (calibration/sensors/ir_comparison_20260930_both450.json, the lower of left and right at each distance),
 * with that capture's spread of about -30..+110 counts; past the end, an object irBehindCm away, or the
 * open-room background measured on 14 Oct (332..614). */
static bool irSharp; static float irBehindCm; static unsigned irNoise = 1U;
static float SharpAdc(float cm) {
    static const float mm[] = {100, 150, 200, 300, 400, 500, 600}, adc[] = {2848, 2041, 1549, 1097.5f, 822.5f, 686.5f, 576};
    float d = fmaxf(cm * 10.0f, 100.0f);
    unsigned i = 0;
    while (i < 5 && d > mm[i + 1]) ++i;
    return fmaxf(300.0f, adc[i] + (adc[i + 1] - adc[i]) * (d - mm[i]) / (mm[i + 1] - mm[i]));
}
static float heldRange;
static float launched[64]; /* Distance of each move the app launched, in order. */
static unsigned moves, brakes, brakeTicks, samples;
static char reply[COMMANDSESSION_REPLY_SIZE];

static const char *Move(void *ctx, float d, float r, float v) {
    (void)ctx; assert(!motion.moving); assert(isfinite(d) && isfinite(r) && v > 0);
    ++moves; motion = (Task2MotionReading){.moving = true};
    if(moves<=64U)launched[moves-1U]=d;
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
static Task2CourseIo io = {.moveStraight=Straight,.moveArc=Arc,.moveContinuous=Continuous,
    .busy=Busy,.brake=Brake,.requestSpeed=Speed,.motion=Motion,.ultrasonic=USS,.infrared=IR,
    .mmPerCount=PI*65.5f/1560,.decelerationMmps2=2000};

/* A planning host in this process: the real codec and planner, with faults to inject. */
enum { HOST_OK, HOST_SILENT, HOST_STOP, HOST_TAMPER };
static int hostMode;
static unsigned hostTamper, hostCalls;
static size_t Host(void *c, uint16_t generation, const uint8_t *request, size_t length, uint8_t *reply, size_t capacity) {
    (void)c; (void)generation;
    ++hostCalls;
    if (hostMode == HOST_SILENT) return 0;
    if (hostMode == HOST_STOP) return TASK2_PLAN_CANCELLED;
    size_t n = Task2Wire_Execute(request, length, reply, capacity);
    if (hostMode != HOST_TAMPER || !n) return n;
    static Task2WireReply r;
    assert(Task2Wire_DecodeReply(reply, n, &r));
    Task2RouteStep *last = &r.route.steps[r.route.stepCount ? r.route.stepCount - 1U : 0U];
    switch (hostTamper) {
    case 0: r.route.config.obstacle2EndSpaceMm += 200; break;            /* more side room than proven */
    case 1: r.route.config.bodyWidthMm -= 50; break;                     /* a thinner robot */
    case 2: r.route.steps[0].distanceMm += 10; break;                    /* a step already driven */
    case 3: {                                                            /* unbounded travel */
        unsigned k = r.route.stepCount;
        while (k > 0U && r.route.steps[k - 1U].sensorTerminated) --k;
        r.route.steps[k ? k - 1U : 0U].sensorTerminated = true;
        break;
    }
    case 4: last->distanceMm = NAN; break;
    case 5: last->radiusMm = 100; break;                                 /* tighter than the steering */
    case 6: r.route.returnFallbackActive = true; r.route.returnObstacleBufferMm = 60; break;
    case 7: r.generation += 1; break;                                    /* someone else's reply */
    case 8: r.ok = false; break;
    case 9: last->distanceMm += 6000; break;                             /* far beyond the arena */
    case 11: r.route.obstacle2LateralDeparture = !r.route.obstacle2LateralDeparture; break;
    case 12: {                                                           /* nothing new to drive */
        static Task2WireRequest q;
        assert(Task2Wire_DecodeRequest(request, length, &q));
        r.route.stepCount = q.call == TASK2_WIRE_REPLAN_DEPARTURE || q.call == TASK2_WIRE_REPLAN_IR_TRAVERSE
            ? TASK2_ROUTE_O2_DEPARTURE_STEP : q.step + 1U;
        break;
    }
    case 13: last->speedCps = 15000; break;                              /* faster than any phase allows */
    case 14:                                                             /* the later replanning skipped */
        if (r.route.obstacle2ReturnPending || r.route.obstacle2ParkReturnPending || r.route.obstacle2ParallelReturnPending)
            r.route.obstacle2ReturnPending = r.route.obstacle2ParkReturnPending = r.route.obstacle2ParallelReturnPending = false;
        else
            r.route.obstacle2ParallelReturnPending = true;
        break;
    case 15: {                                                           /* a sensor-ended step that never moves */
        bool sensor = false;
        for (unsigned k = 0U; k < r.route.stepCount; ++k)
            if (r.route.steps[k].sensorTerminated && k == TASK2_ROUTE_O2_IR_STRAIGHT_STEP) { r.route.steps[k].distanceMm = 0; sensor = true; }
        if (!sensor) last->speedCps = 0;
        break;
    }
    case 16: r.request ^= 1U; break;                                     /* same generation, another request */
    case 19: case 20: case 21:                                          /* a fallback with impossible buffers */
        r.route.returnFallbackActive = true;
        r.route.returnObstacleBufferMm = r.route.returnBoundaryBufferMm = hostTamper == 19 ? NAN : hostTamper == 20 ? INFINITY : 1e6f;
        break;
    case 17: case 18: {                                                  /* an impossible traverse preview */
        static Task2WireRequest q;
        assert(Task2Wire_DecodeRequest(request, length, &q));
        float w = hostTamper == 17 ? INFINITY : 1e6f;
        if (q.call == TASK2_WIRE_REPLAN_IR_TRAVERSE) { r.route.config.obstacle2WidthMm = w; r.route.config.obstacle2EndSpaceMm = w; }
        else r.route.config.bodyWidthMm = hostTamper == 17 ? NAN : 1e6f;
        break;
    }
    default: break;
    }
    n = Task2Wire_EncodeReply(&r, reply, capacity);
    if (hostTamper == 10 && n) reply[n / 2] ^= 0x40;                    /* corrupted on the wire */
    return n;
}

/* As SensorTask publishes it: the range limits apply to the raw pulse distance, which is the corrected
 * distance minus HCSR04_DISTANCE_OFFSET_MM (-5 mm); no echo is a timeout. */
static SensorStatus UssStatus(float range) {
    if (!isfinite(range)) return SENSOR_TIMEOUT;
    float raw = range + 5.0f;
    return raw >= SENSOR_ULTRASONIC_MIN_MM && raw <= SENSOR_ULTRASONIC_MAX_MM ? SENSOR_OK : SENSOR_OUT_OF_RANGE;
}
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
        float ahead=a->sensors.ultrasonic.aheadMm;
        float x=physical.xMm+ahead*cosf(physical.yawRad), y=physical.yMm+ahead*sinf(physical.yawRad);
        bool second=run.commands.stage>=TASK2_STAGE_APPROACH2;
        float face=physicalO1+(second ? 100+physicalGap : 0);
        float range=INFINITY;
        for(unsigned i=0;i<=120;++i) {
            float beam=physical.yawRad+(7.5f*PI/180)*(2.0f*i/120-1);
            if(cosf(beam)<0.05f || face<=x)continue;
            float d=(face-x)/cosf(beam);
            if(fabsf(y+d*sinf(beam))<=(second ? physicalWidth : 200)/2)range=fminf(range,d);
        }
        if(frozenUSS && second){ if(heldRange<0 && isfinite(range))heldRange=range; if(heldRange>=0)range=heldRange; }
        if(stuckUSS && second)range=(face-x)/cosf(fmaxf(0,fabsf(physical.yawRad)-7.5f*PI/180));
        /* Isolated uneven-surface readings must not bias either ranging median. */
        if ((a->phase==TASK2_APP_RANGE1 || a->phase==TASK2_APP_RANGE2) && samples%5==0) range=2400;
        if(forceAcquisition && a->phase==TASK2_APP_ROUTE && a->step==3U) range=INFINITY;
        /* O2's echo lost at an angle: something 700 mm beyond its face is timed instead. */
        if(missedEcho && a->phase==TASK2_APP_RANGE2) range=(face+missedEchoExtraMm-x)/cosf(physical.yawRad);
        if(second && ussShortMm>0 && isfinite(range)) range-=ussShortMm;
        if(noEchoReversing && (a->backingUp || a->departureBackingUp || a->rewinding)) range=INFINITY;
        if(strayEcho && !strayEchoDone && (a->phase==TASK2_APP_ACQUIRE2 ||
           (a->phase==TASK2_APP_ROUTE && a->step==TASK2_ROUTE_O2_APPROACH_STEP && fabsf(remaining)<150))) {
            /* Also when the beam misses O2 at this heading: the stray reading does not need the face. */
            range=0.4f*(isfinite(range) ? range : (face-x)/fmaxf(0.05f,cosf(physical.yawRad))); strayEchoDone=true;
        }
        uss=(UltrasonicReading){.sequence=uss.sequence+1,.capturedMs=now,
            .status=UssStatus(range),.distanceMm=isfinite(range)?fmaxf(0.0f,range):0};
    }
    if(now%50U<10U) {
        ++infrared.sequence;
        for(unsigned side=0;side<2;++side) {
            Task2SensorMount m=side ? a->sensors.rightIR : a->sensors.leftIR;
            float x=physical.xMm+m.aheadMm*cosf(physical.yawRad)-m.leftMm*sinf(physical.yawRad);
            float y=physical.yMm+m.aheadMm*sinf(physical.yawRad)+m.leftMm*cosf(physical.yawRad);
            float beam=physical.yawRad+m.yawDeg*PI/180;
            float face=physicalO1+100+physicalGap;
            float along=fabsf(y+(face-x)*tanf(beam));
            bool onFace=cosf(beam)>0.05f && face>x && along<=physicalWidth/2+irOvershootMm;
            unsigned faceAdc = along<=physicalWidth/2 ? (irLowFromMm>0 && along>irLowFromMm ? irLowAdc : irFaceAdc) : irPastEndAdc;
            if(irSharp) {
                irNoise=irNoise*1103515245U+12345U; unsigned r=(irNoise>>16)%1000U;
                if(cosf(beam)>0.05f && face>x && along<=physicalWidth/2)
                    faceAdc=(unsigned)fmaxf(0.0f,SharpAdc((face-x)/cosf(beam)/10.0f)+(float)(r%141U)-30.0f);
                else faceAdc=irBehindCm>0 ? (unsigned)(SharpAdc(irBehindCm)+(float)(r%141U)-30.0f) : 332U+r%283U;
            }
            IRReading value={.capturedMs=now,.status=irFault ? SENSOR_ERROR : SENSOR_OK,
                .adc=initialAbsent ? 300 : irSharp ? faceAdc : onFace ? faceAdc : 300};
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
    physical=(Task2RoutePose){-250,0,0}; moves=brakes=brakeTicks=samples=0; memset(launched,0,sizeof(launched));
    stopSensors=initialAbsent=irFault=stuckUSS=frozenUSS=forceAcquisition=missedEcho=strayEcho=strayEchoDone=false;
    ussShortMm=irOvershootMm=0; irFaceAdc=irPastEndAdc=900; irSharp=false; irBehindCm=0; physicalO1=1050; irLowFromMm=0; irLowAdc=0; missedEchoExtraMm=700; noEchoReversing=false; heldRange=-1; physicalGap=1050;physicalWidth=500;
}
static void Send(const char *line, const char *expected) {
    assert(Task2Run_Receive(&run,line,reply)); assert(strcmp(reply,expected)==0);
}
static void RunTo(Task2AppPhase phase) {
    for(unsigned n=0;run.course.app.phase!=phase && n<12000;++n)assert(Tick()==TASK2RUN_RUNNING);
    assert(run.course.app.phase==phase);
}
/* Every record the index lists comes back whole: six numbered parts, none cut. One must contain each of
 * the given texts (NULL ends the list). Returns the number of records. */
static unsigned CheckDiagnostics(unsigned runId, const char *const *mustHave) {
    static char lines[TASK2_DIAG_PARTS][TASK2_DIAG_LINE_SIZE];
    char index[TASK2_DIAG_LINE_SIZE], prefix[48];
    assert(Task2Run_Diagnostics(&run,"D T",lines,TASK2_DIAG_PARTS)==1U);
    strcpy(index,lines[0]);
    snprintf(prefix,sizeof(prefix),"DT %u INDEX v=%u fw=",runId,TASK2_DIAG_FORMAT);
    assert(strncmp(index,prefix,strlen(prefix))==0 && !strstr(index," ~") && index[strlen(index)-1]=='\n');
    unsigned found=0, records=0, wanted=0;
    while(mustHave && mustHave[wanted])++wanted;
    unsigned generations[TASK2_DIAG_RECORDS+1U], count=0;
    const char *g=strstr(index," g=");assert(g);g+=3;
    while(*g>='0' && *g<='9') { generations[count++]=(unsigned)strtoul(g,(char **)&g,10); if(*g==',')++g; }
    const char *f=strstr(index," fault=");assert(f);
    if(f[7]>='0' && f[7]<='9') generations[count++]=(unsigned)strtoul(f+7,NULL,10);
    for(unsigned i=0;i<count;++i) {
        char request[16];snprintf(request,sizeof(request),"D T %u",generations[i]);
        assert(Task2Run_Diagnostics(&run,request,lines,TASK2_DIAG_PARTS)==TASK2_DIAG_PARTS);
        for(unsigned k=0;k<TASK2_DIAG_PARTS;++k) {
            snprintf(prefix,sizeof(prefix),"DT %u %u %u/%u ",runId,generations[i],k+1U,TASK2_DIAG_PARTS);
            size_t n=strlen(lines[k]);
            assert(strncmp(lines[k],prefix,strlen(prefix))==0 && n<TASK2_DIAG_LINE_SIZE && lines[k][n-1]=='\n' && !strstr(lines[k]," ~"));
        }
        for(unsigned w=0;w<wanted;++w) if(strstr(lines[0],mustHave[w]) && !(found&(1U<<w))) found|=1U<<w;
        ++records;
    }
    assert(found==(1U<<wanted)-1U);
    assert(Task2Run_Diagnostics(&run,"D T 65535",lines,TASK2_DIAG_PARTS)==0U);
    assert(Task2Run_Diagnostics(&run,"D T x",lines,TASK2_DIAG_PARTS)==0U);
    return records;
}
static unsigned reversesChecked;
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
    /* The ring keeps the newest records: the early plans are only certain to be there if none was overwritten. */
    static const char *const planned[]={"w=PLAN_O1 ","w=PLAN_O2 ","w=DEPARTURE ",NULL};
    static const char *const newest[]={NULL};
    assert(CheckDiagnostics(12U,run.course.app.diag.overwritten ? newest : planned)>=3U && !run.course.app.diag.hasFault);
    /* A recovery reverse in a record is a move the app really launched. */
    for(unsigned i=0;i<run.course.app.diag.count;++i) {
        const Task2DiagRecord *r=&run.course.app.diag.records[(run.course.app.diag.head+i)%TASK2_DIAG_RECORDS];
        if(!r->hasNext)continue;
        bool found=false;
        for(unsigned k=0;k<moves && k<64U;++k) found|=fabsf(launched[k]-r->next.distanceMm)<0.01f;
        assert(found && r->next.distanceMm<0);
        ++reversesChecked;
    }
    printf("PASS %c%c gap %.0f width %.0f%s: %u primitives\n",first,second,gap,width,absence?" initial absence":"",moves);
}
int main(void) {
    setbuf(stdout,NULL);
    for(unsigned gap=600;gap<=1500;gap+=450) {
        Complete('L','R',gap,500,false);Complete('R','L',gap,500,false);
        Complete('L','L',gap,500,false);Complete('R','R',gap,500,false);
    }
    Complete('L','R',1050,300,true);Complete('R','L',1050,300,true);
    {
        /* Planning on a host: the same routes as planning here, and a reply that does not check out is
         * never used. Each layout runs locally first; the host runs must launch exactly the same moves. */
        static const struct { char first, second; float gap, width; } layouts[] = {
            {'L','R',600,500},{'R','L',1050,500},{'L','L',600,500},{'R','R',1500,500},
            {'L','L',600,1000},{'R','R',1500,2000},{'L','R',1050,1200}};
        static float localMoves[64]; unsigned localCount, accepted = 0U;
        for (unsigned i = 0U; i < sizeof layouts / sizeof layouts[0]; ++i) {
            io.plan = NULL;
            Complete(layouts[i].first, layouts[i].second, layouts[i].gap, layouts[i].width, false);
            localCount = moves; memcpy(localMoves, launched, sizeof(localMoves));
            io.plan = Host; hostMode = HOST_OK; hostCalls = 0U;
            Complete(layouts[i].first, layouts[i].second, layouts[i].gap, layouts[i].width, false);
            assert(moves == localCount && memcmp(localMoves, launched, sizeof(localMoves)) == 0);
            if(!(hostCalls > 0U && run.course.app.planHostAccepted + run.course.app.planHostFailed == hostCalls && run.course.app.planHostRejected == 0U)) {
                fprintf(stderr,"PARITY layout %u: calls %u accepted %u rejected %u last refusal %c\n",i,hostCalls,run.course.app.planHostAccepted,run.course.app.planHostRejected,run.course.app.planHostRefusal);
                for(unsigned k=0;k<run.course.app.route.stepCount;++k) fprintf(stderr,"  step %u d %.0f r %.0f s %d\n",k,run.course.app.route.steps[k].distanceMm,run.course.app.route.steps[k].radiusMm,run.course.app.route.steps[k].sensorTerminated);
            }
            assert(hostCalls > 0U && run.course.app.planHostAccepted + run.course.app.planHostFailed == hostCalls &&
                   run.course.app.planHostRejected == 0U && run.course.app.planHostAccepted > 0U);
            accepted += run.course.app.planHostAccepted;
            /* Every way a reply can be wrong: refused, planned here instead, same run. */
            /* The two layouts that traverse also get the bad traverse previews. */
            for (hostTamper = i < 4U ? 0U : 17U; hostTamper <= 21U && i < 6U; ++hostTamper) {
                hostMode = HOST_TAMPER; hostCalls = 0U;
                Complete(layouts[i].first, layouts[i].second, layouts[i].gap, layouts[i].width, false);
                if(!(moves == localCount && memcmp(localMoves, launched, sizeof(localMoves)) == 0 && hostCalls > 0U &&
                     run.course.app.planHostAccepted == 0U &&
                     run.course.app.planHostRejected + run.course.app.planHostFailed == hostCalls))
                    fprintf(stderr,"TAMPER %u layout %u: moves %u vs %u, calls %u accepted %u rejected %u\n",hostTamper,i,
                            moves,localCount,hostCalls,run.course.app.planHostAccepted,run.course.app.planHostRejected);
                assert(moves == localCount && memcmp(localMoves, launched, sizeof(localMoves)) == 0);
                /* A reply marked failed (tamper 8) is planned here like any other failure. */
                assert(hostCalls > 0U && run.course.app.planHostAccepted == 0U &&
                       run.course.app.planHostRejected + run.course.app.planHostFailed == hostCalls);
            }
            /* A silent host costs nothing but the wait. */
            hostMode = HOST_SILENT; hostCalls = 0U;
            Complete(layouts[i].first, layouts[i].second, layouts[i].gap, layouts[i].width, false);
            assert(moves == localCount && memcmp(localMoves, launched, sizeof(localMoves)) == 0 && hostCalls > 0U);
            assert(run.course.app.planHostAccepted == 0U && run.course.app.planHostRejected == 0U);
        }
        printf("PASS planning host: %u host routes identical to local, 22 kinds of bad reply refused, silent host falls back\n", accepted);
        /* A stop while waiting for the host: nothing is committed, nothing moves, and the stop ends the run. */
        Reset(0); io.plan = Host; hostMode = HOST_STOP; hostCalls = 0U; physicalGap = 1050; physicalWidth = 500;
        assert(Task2Run_Start(&run,15,TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
        RunTo(TASK2_APP_RANGE2); Send("A 15 2 R","ACK 15 2 R\n");
        for(unsigned n=0;n<12000 && !run.course.app.planCancelled;++n) assert(Tick()==TASK2RUN_RUNNING);
        assert(run.course.app.planCancelled && hostCalls == 1U);
        static Task2Route before; before = run.course.app.route; unsigned movesBefore = moves;
        while(motion.moving) Advance();
        for(unsigned n=0;n<200;++n) assert(Tick()==TASK2RUN_RUNNING);
        assert(moves == movesBefore && hostCalls == 1U && memcmp(&before, &run.course.app.route, sizeof(before)) == 0);
        Brake(NULL); Task2Run_Stop(&run); while(motion.moving) Advance(); Task2Run_Stationary(&run);
        assert(run.commands.stage == TASK2_STAGE_STOPPED);
        io.plan = NULL; hostMode = HOST_OK;
        puts("PASS a stop while waiting for the host commits nothing and stops the run");
    }
    /* Long same-side obstacle 2: the face run, its reverses and their records. */
    Complete('L','L',600,1000,false);Complete('R','R',1500,2000,false);
    assert(reversesChecked>=2U);
    printf("PASS %u recovery reverses match their records\n",reversesChecked);
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
    /* A long fallback route stopped at step 15: the record keeps a window with that step and says it is cut. */
    run.course.app.route.stepCount=20U;run.course.app.step=15U;
    for(unsigned i=0;i<20U;++i)run.course.app.route.steps[i]=(Task2RouteStep){.distanceMm=1000.0f+i};
    Brake(NULL);Task2Run_Stop(&run);assert(!run.active);
    while(motion.moving)Advance();
    Task2Run_Stationary(&run);assert(run.commands.stage==TASK2_STAGE_STOPPED);
    static const char *const stopped[]={"k=S ok=0 w=ABORT ms=",NULL};
    CheckDiagnostics(7U,stopped);
    {
        static char part[TASK2_DIAG_PARTS][TASK2_DIAG_LINE_SIZE];char request[16];
        snprintf(request,sizeof(request),"D T %u",(unsigned)run.course.app.diag.records[(run.course.app.diag.head+run.course.app.diag.count-1U)%TASK2_DIAG_RECORDS].generation);
        assert(Task2Run_Diagnostics(&run,request,part,TASK2_DIAG_PARTS)==TASK2_DIAG_PARTS);
        assert(strstr(part[0]," st=15 ") && strstr(part[4]," n=20 from=8 got=12 cut=1 8:1008/0/0 ") && strstr(part[5]," 15:1015/0/0 ") && strstr(part[5]," 19:1019/0/0\n"));
    }
    assert(Task2Run_Start(&run,8,TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
    assert(!run.course.app.estimates.faceGap.valid && !run.course.app.detection.detected && !run.commands.arrows[1]);
    puts("PASS late arrows, wrap, cancellation and per-run reset");
    {
        /* A fault from outside the app (watchdog, controller) keeps the run's state, then S stops it. */
        Reset(0);assert(Task2Run_Start(&run,9,TASK2_COMMAND_ARROW_LEFT,&io,now)==NULL);
        RunTo(TASK2_APP_SWERVE1);unsigned phase=(unsigned)run.course.app.phase,step=run.course.app.step;
        expectFault=true;Brake(NULL);Task2Run_FaultReason(&run,"NO_PROGRESS");
        assert(run.course.app.phase==TASK2_APP_ABORTED && run.commands.stage==TASK2_STAGE_FAULT);
        assert(run.course.app.diag.hasFault && run.course.app.diag.fault.phase==phase && run.course.app.diag.fault.step==step);
        static const char *const external[]={"k=F ok=0 w=EXTERNAL ","why=NO_PROGRESS",NULL};
        CheckDiagnostics(9U,external);
        Task2Run_Stop(&run);while(motion.moving)Advance();Task2Run_Stationary(&run);
        assert(run.commands.stage==TASK2_STAGE_STOPPED && run.course.app.diag.hasFault);
        CheckDiagnostics(9U,external);
        /* A fault the app already recorded is not recorded twice. */
        unsigned before=run.course.app.diag.count;Task2Run_FaultReason(&run,"AGAIN");
        assert(run.course.app.diag.count==before && !strcmp(run.course.app.diag.fault.reason,"NO_PROGRESS"));
        puts("PASS external fault recorded before the abort, kept through S");
    }
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
    RunTo(TASK2_APP_RANGE2);Send("A 11 2 R","ACK 11 2 R\n");frozenUSS=expectFault=true;
    status=TASK2RUN_RUNNING;for(unsigned n=0;n<12000 && status==TASK2RUN_RUNNING;++n)status=Tick();
    assert(status==TASK2RUN_FAULT && !run.course.app.detection.detected && !strcmp(run.course.app.fault,"T2_USS_SENSOR"));
    {
        /* The fault's record survives the abort and keeps the phase it failed in. */
        unsigned failedPhase=(unsigned)run.course.app.phase;
        Task2Run_Fault(&run);assert(run.course.app.phase==TASK2_APP_ABORTED && run.course.app.diag.hasFault);
        static const char *const frozen[]={"why=T2_USS_SENSOR",NULL};
        CheckDiagnostics(11U,frozen);
        assert(run.course.app.diag.fault.phase==failedPhase && failedPhase!=TASK2_APP_ABORTED);
        assert(run.course.app.diag.fault.ussFrozen);
    }
    /* A USS reporting the face plane at every pose cannot be told from a real face: only a bounded end is checked. */
    Reset(0);assert(Task2Run_Start(&run,12,TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
    RunTo(TASK2_APP_RANGE2);Send("A 12 2 R","ACK 12 2 R\n");stuckUSS=expectFault=true;
    status=TASK2RUN_RUNNING;for(unsigned n=0;n<12000 && status==TASK2RUN_RUNNING;++n)status=Tick();
    assert(status!=TASK2RUN_RUNNING && run.course.app.departureExtensions<=7U && run.course.app.traverseRewinds<=3U);
    Reset(0);assert(Task2Run_Start(&run,13,TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
    RunTo(TASK2_APP_RANGE2);Send("A 13 2 L","ACK 13 2 L\n");
    while(!(run.course.app.phase==TASK2_APP_ROUTE && run.course.app.step==5U))assert(Tick()==TASK2RUN_RUNNING);
    stopSensors=expectFault=true;status=TASK2RUN_RUNNING;for(unsigned n=0;n<30 && status==TASK2RUN_RUNNING;++n)status=Tick();
    assert(status==TASK2RUN_FAULT && !strcmp(run.course.app.fault,"T2_SENSOR_STALE"));
    puts("PASS IR failure, frozen USS fault, bounded phantom-face USS and stale-sensor braking");
    {
        /* Frozen-USS tracker against O2's face at x 496. */
        static Task2App f; memset(&f,0,sizeof f); f.geometry=task2RouteConfig; f.sensors=task2AppSensorDefaults;
        f.geometry.parkToObstacle1Mm=396; f.geometry.obstacleFaceGapMm=0;
        /* A healthy range along a 275 mm arc can hold within the tolerance while the car moves 60 mm. */
        f.frozenRangeMm=f.frozenExpectedMm=NAN;
        Task2RoutePose first={0}, last={0}; float r0=NAN, worst=0;
        for(unsigned i=0; i<=51U; ++i) {
            float deg=i<51U ? 31.737f+0.25f*i : 44.30f;
            float yaw=deg*PI/180; Task2RoutePose p={275*sinf(yaw),275-275*cosf(yaw),yaw};
            if(i==0U)first=p; last=p;
            float r=Task2App_FaceRange(&f,&p); assert(isfinite(r)); if(i==0U)r0=r; worst=fmaxf(worst,fabsf(r-r0));
            assert(!Task2App_FrozenUSS(&f,true,r,r));
        }
        /* Every reading within 3 mm of the first over at least 60 mm: a travel-based rule would call this frozen. */
        assert(hypotf(last.xMm-first.xMm,last.yMm-first.yMm)>=60 && worst<=3);
        /* A range held while closing straight on the face is frozen within about 30 mm. */
        f.frozenRangeMm=f.frozenExpectedMm=NAN; f.ussFrozen=false; float held=NAN; float frozenAt=NAN;
        for(float x=0; x<=100 && isnan(frozenAt); x+=5) {
            Task2RoutePose p={x,0,0}; float e=Task2App_FaceRange(&f,&p); if(isnan(held))held=e;
            if(Task2App_FrozenUSS(&f,true,held,e))frozenAt=x;
        }
        assert(frozenAt>=30 && frozenAt<=35);
        /* An invalid reading restarts the comparison from the next valid one. */
        f.frozenRangeMm=f.frozenExpectedMm=NAN; f.ussFrozen=false; held=NAN; frozenAt=NAN;
        for(float x=0; x<=120 && isnan(frozenAt); x+=5) {
            Task2RoutePose p={x,0,0}; float e=Task2App_FaceRange(&f,&p); if(isnan(held))held=e;
            if(Task2App_FrozenUSS(&f,x!=20,held,e))frozenAt=x;
        }
        assert(frozenAt>=55 && frozenAt<=60);
        /* An unknown expected range also restarts the comparison, from the next usable pair. */
        f.frozenRangeMm=f.frozenExpectedMm=NAN; f.ussFrozen=false;
        assert(!Task2App_FrozenUSS(&f,true,500,NAN));
        assert(!Task2App_FrozenUSS(&f,true,500,500));
        assert(Task2App_FrozenUSS(&f,true,500,400));
        assert(Task2App_FrozenUSS(&f,false,0,NAN)); /* latched */
        puts("PASS frozen-USS tracker: healthy arc, held range, invalid reading and unknown expectation reset");
    }
    {
        /* Floor, 60/60 LR, O2 626 mm: on the IR straight the reading fell from its first sample while still on the
         * face. The falling-trend end must not be taken when the face has already been seen beyond it. */
        Task2RouteConfig r = task2RouteConfig;
        r.parkToObstacle1Mm = 600; r.obstacleFaceGapMm = 600; r.firstArrow = TASK2_ARROW_LEFT; r.secondArrow = TASK2_ARROW_RIGHT;
        const float half = 313.0f, robotX = 1022.0f;
        Task2ObstacleDetection d; memset(&d, 0, sizeof d);
        Task2RoutePose start = {robotX, 22.0f, -0.5f * PI};
        Task2ObstacleDetection_BeginIRTraverse(&d, &start);
        unsigned k = 0U;
        for (float y = 22.0f; !d.detected && y > -600.0f; y -= 13.0f, ++k) {
            Task2RoutePose p = {robotX, y, -0.5f * PI};
            float sensorY = y - task2AppSensorDefaults.leftIR.aheadMm;
            float adc = sensorY >= -half ? fmaxf(720.0f, 950.0f - 60.0f * k) : 300.0f;
            Task2ObstacleObservation o = {.valid = true, .value = adc, .pose = p};
            Task2ObstacleDetection_Observe(&d, &task2AppSensorDefaults, &r, TASK2_SENSOR_LEFT_IR, &o, true);
        }
        printf("IR dip: method %d observed %.0f proven half %.0f\n", (int)d.edgeMethod, d.observedWidthMm, d.faceExtentLowerMm);
        assert(d.detected && d.faceExtentLowerMm > 200.0f);
        assert(d.edgeMethod == TASK2_EDGE_THRESHOLD && 0.5f * d.observedWidthMm >= d.faceExtentLowerMm);
        assert(0.5f * d.observedWidthMm >= half);
        puts("PASS IR end from a falling reading is not taken inside the face already seen");
    }
    {
        /* An estimated O2 gap under the buffer is recorded and the run goes on (a touch costs time, a stop fails the
         * run); a side-wall shortfall still stops, and its record holds the margins the check measured. */
        Reset(0); io.plan = NULL; physicalGap = 1050; physicalWidth = 500;
        assert(Task2Run_Start(&run,21,TASK2_COMMAND_ARROW_LEFT,&io,now)==NULL);
        RunTo(TASK2_APP_RANGE2); Send("A 21 2 L","ACK 21 2 L\n");
        Task2App *a = &run.course.app;
        for(unsigned n=0;n<12000 && !(a->phase==TASK2_APP_ROUTE && a->step==TASK2_ROUTE_O2_IR_STRAIGHT_STEP);++n)
            assert(Tick()==TASK2RUN_RUNNING);
        assert(a->phase==TASK2_APP_ROUTE && a->step==TASK2_ROUTE_O2_IR_STRAIGHT_STEP);
        float face = a->geometry.parkToObstacle1Mm + 100.0f + a->geometry.obstacleFaceGapMm;
        Task2RoutePose was = a->pose;
        /* Bring the estimated front to ~40 mm from O2's face on the lateral departure (100 mm buffer). */
        float front = a->geometry.bodyCenterAheadOfRearAxleMm + 0.5f * a->geometry.bodyLengthMm + a->geometry.frontProtrusionMm;
        a->pose.xMm = face - front - 40.0f;
        a->pose.yawRad = 0.0f;
        for(unsigned n=0;n<5;++n) assert(Tick()==TASK2RUN_RUNNING);
        bool noted = false;
        for(unsigned i=0;i<TASK2_DIAG_RECORDS;++i) {
            const Task2DiagRecord *r = &a->diag.records[i];
            if(r->what && strcmp(r->what,"OBSTACLE_SHORTFALL")==0) noted = r->obstacleGapMm < 100.0f;
        }
        printf("O2 shortfall: shortfalls %u noted %d\n", a->obstacleShortfalls, noted);
        assert(a->obstacleShortfalls > 0U && noted && !a->fault);
        a->pose = was;
        /* Past the side limit: still a stop, with the measured side margin in its record. */
        a->pose.yMm = -(0.5f * a->geometry.obstacle2WidthMm + a->geometry.obstacle2EndSpaceMm + 50.0f);
        Task2RunStatus st = TASK2RUN_RUNNING;
        for(unsigned n=0;n<5 && st==TASK2RUN_RUNNING;++n) st = Tick();
        printf("side: status %d fault %s record side %.1f\n", (int)st, a->fault ? a->fault : "-", a->diag.fault.sideMarginMm);
        assert(a->fault && strcmp(a->fault,"T2_CLEARANCE")==0 && a->diag.hasFault && a->diag.fault.sideMarginMm < 1.0f);
        puts("PASS O1/O2 clearance shortfall goes on and is recorded; side shortfall stops with measured margins");
    }
    {
        /* An end measured shorter than the face already proven (here the proof is raised by hand, as another sensor
         * might) is widened to the proof and the run finishes; it is recorded, not a stop. */
        Reset(0); io.plan = NULL; physicalGap = 1050; physicalWidth = 500;
        assert(Task2Run_Start(&run,22,TASK2_COMMAND_ARROW_LEFT,&io,now)==NULL);
        RunTo(TASK2_APP_RANGE2); Send("A 22 2 R","ACK 22 2 R\n");
        Task2App *a = &run.course.app;
        for(unsigned n=0;n<12000 && !(a->phase==TASK2_APP_ROUTE && a->step==TASK2_ROUTE_O2_IR_STRAIGHT_STEP);++n)
            assert(Tick()==TASK2RUN_RUNNING);
        a->provenHalfMm = 400.0f;
        Task2RunStatus st = TASK2RUN_RUNNING;
        for(unsigned n=0;n<12000 && st==TASK2RUN_RUNNING;++n) st = Tick();
        bool noted = false;
        for(unsigned i=0;i<TASK2_DIAG_RECORDS;++i)
            noted |= a->diag.records[i].what && strcmp(a->diag.records[i].what,"SENSOR_CONFLICT")==0;
        printf("conflict: status %d fault %s conflicts %u width %.0f noted %d\n", (int)st, a->fault ? a->fault : "-",
               a->sensorConflicts, a->detection.observedWidthMm, noted);
        assert(st==TASK2RUN_DONE && !a->fault && a->sensorConflicts==1U && noted && a->detection.observedWidthMm >= 840.0f);
        assert(Task2Route_IsParked(&a->geometry,&physical));
        puts("PASS an end shorter than the proven face is widened to it and the run finishes");
    }
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
    {
        /* The camera's depth sets O2's face at RANGE2; the ultrasonic is used only without one. */
        const float gaps[] = {600, 1050, 1500};
        for (unsigned g = 0; g < 3; ++g) for (unsigned withDepth = 0; withDepth < 3; ++withDepth) {
            Reset(0); physicalGap = gaps[g]; physicalWidth = 500; missedEcho = withDepth == 0;
            assert(Task2Run_Start(&run,15,TASK2_COMMAND_ARROW_LEFT,&io,now)==NULL);
            Send("Z 15 500","NAK 15 Z STATE\n");
            RunTo(TASK2_APP_RANGE2);
            Task2App *a = &run.course.app;
            const Task2RouteConfig *c = &a->geometry;
            float lens = c->bodyCenterAheadOfRearAxleMm + 0.5f*c->bodyLengthMm - c->cameraBehindFrontMm;
            float cx = physical.xMm + lens*cosf(physical.yawRad), cy = physical.yMm + lens*sinf(physical.yawRad);
            float depth = (physicalO1 + 100 + physicalGap - cx)*cosf(physical.yawRad) - cy*sinf(physical.yawRad);
            char line[32], ack[32];
            unsigned sent = withDepth == 0 ? (unsigned)lroundf(depth) : 9000U;   /* 9000: outside the course */
            if (withDepth != 2) {
                snprintf(line,sizeof line,"Z 15 %u",sent); snprintf(ack,sizeof ack,"ACK 15 Z %u\n",sent);
                Send(line,ack);
            }
            for (unsigned n = 0; n < 30; ++n) assert(Tick()==TASK2RUN_RUNNING);
            Send("A 15 2 R","ACK 15 2 R\n");
            for(unsigned n=0;n<12000 && a->phase==TASK2_APP_RANGE2;++n) assert(Tick()==TASK2RUN_RUNNING);
            float gap = a->estimates.faceGap.mm;
            printf("camera depth: gap %.0f depth %s -> estimate %.1f (camera face %.1f, ultrasonic face %.1f)\n",
                   physicalGap, withDepth == 0 ? "sent" : withDepth == 1 ? "outside" : "none", gap, a->cameraFaceMm, a->ussFaceMm);
            if (withDepth == 0) assert(fabsf(gap - physicalGap) < 2.0f);
            else if (withDepth == 1) assert(fabsf(gap - physicalGap) < 25.0f && a->cameraFaceMm > 9000);  /* refused */
            else assert(fabsf(gap - physicalGap) < 25.0f && !isfinite(a->cameraFaceMm));
            Task2RunStatus st=TASK2RUN_RUNNING;for(unsigned n=0;n<12000 && st==TASK2RUN_RUNNING;++n)st=Tick();
            assert(st==TASK2RUN_DONE && Task2Route_IsParked(&a->geometry,&physical));
        }
        puts("PASS camera depth sets O2's face over a lost ultrasonic echo; the ultrasonic without one");
    }
    {
        /* No camera depth and a lost RANGE2 echo: O2 is planned 700 mm too far. The approach must brake on
         * the near echo, move the estimate in and still finish parked. */
        const float gaps[] = {600, 1050};
        const char pairs[][2] = {{'L','R'},{'R','L'},{'L','L'},{'R','R'}};
        for (unsigned g = 0; g < 2; ++g) for (unsigned k = 0; k < 4; ++k) {
            Reset(0); physicalGap = gaps[g]; physicalWidth = 500; missedEcho = true;
            assert(Task2Run_Start(&run,16,pairs[k][0]=='L' ? TASK2_COMMAND_ARROW_LEFT : TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
            RunTo(TASK2_APP_RANGE2);
            char line[16], ack[16];
            snprintf(line,sizeof line,"A 16 2 %c",pairs[k][1]); snprintf(ack,sizeof ack,"ACK 16 2 %c\n",pairs[k][1]);
            Send(line,ack);
            Task2App *a = &run.course.app;
            Task2RunStatus st=TASK2RUN_RUNNING;
            float planned = NAN;
            for(unsigned n=0;n<12000 && st==TASK2RUN_RUNNING;++n) {
                st=Tick();
                if (isnan(planned) && a->phase==TASK2_APP_ROUTE) planned = a->estimates.faceGap.mm;
            }
            printf("lost RANGE2 echo: gap %.0f %c%c planned %.0f -> %s, bumper %d corrected %d estimate %.0f near %.0f valid %d uss %.0f\n",
                   physicalGap, pairs[k][0], pairs[k][1], planned, st==TASK2RUN_DONE ? "DONE" : a->fault ? a->fault : "?",
                   a->approachBraking, a->faceCorrected, a->estimates.faceGap.mm, a->nearFaceMm, a->detection.ultrasonicValid, a->detection.lastUltrasonicRangeMm);
            assert(planned >= fminf(1500.0f, physicalGap + 600.0f));
            assert(st==TASK2RUN_DONE && Task2Route_IsParked(&a->geometry,&physical));
            assert(a->faceCorrected && fabsf(a->estimates.faceGap.mm - physicalGap) < 40.0f);
        }
        puts("PASS a lost RANGE2 echo is caught on the approach: braked, estimate moved in, run finished");
    }
    {
        /* A wrong RANGE2 echo (700 or 1200 mm beyond O2) and arrow 2 three seconds late: the robot creeps
         * toward O2 for the camera with no arrow yet. It must stop short of the face, plan from there and
         * finish. Correct ranging with the same delay is the control. */
        const float extras[] = {700, 1200, 0}, gaps[] = {600, 1050};
        const char pairs[][2] = {{'L','R'},{'R','L'},{'L','L'},{'R','R'}};
        unsigned failed = 0;
        for (unsigned e = 0; e < 3; ++e) for (unsigned g = 0; g < 2; ++g) for (unsigned k = 0; k < 4; ++k) {
            Reset(0); physicalGap = gaps[g]; physicalWidth = 500; missedEcho = extras[e] > 0; missedEchoExtraMm = extras[e];
            assert(Task2Run_Start(&run,22,pairs[k][0]=='L' ? TASK2_COMMAND_ARROW_LEFT : TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
            RunTo(TASK2_APP_RANGE2);
            Task2App *a = &run.course.app;
            const Task2RouteConfig *c = &a->geometry;
            float face = physicalO1 + 100 + physicalGap, minFront = INFINITY;
            float f = c->bodyCenterAheadOfRearAxleMm + 0.5f * c->bodyLengthMm + c->frontProtrusionMm;
            Task2RunStatus st=TASK2RUN_RUNNING;
            for (unsigned n = 0; n < 300 && st==TASK2RUN_RUNNING; ++n) {
                st=Tick();
                minFront = fminf(minFront, face - (physical.xMm + f*cosf(physical.yawRad) + 0.5f*c->bodyWidthMm*fabsf(sinf(physical.yawRad))));
            }
            char line[16], ack[16];
            snprintf(line,sizeof line,"A 22 2 %c",pairs[k][1]); snprintf(ack,sizeof ack,"ACK 22 2 %c\n",pairs[k][1]);
            Send(line,ack);
            bool departed = false;
            expectFault = true;
            for(unsigned n=0;n<15000 && st==TASK2RUN_RUNNING;++n) {
                st=Tick();
                departed |= a->phase==TASK2_APP_ROUTE && a->step >= TASK2_ROUTE_O2_DEPARTURE_STEP;
                if (!departed) minFront = fminf(minFront, face - (physical.xMm + f*cosf(physical.yawRad) + 0.5f*c->bodyWidthMm*fabsf(sinf(physical.yawRad))));
            }
            expectFault = false;
            bool ok = st==TASK2RUN_DONE && Task2Route_IsParked(&a->geometry,&physical) && minFront > 0.0f;
            failed += !ok;
            printf("wrong RANGE2 +%.0f, A2 3 s late: gap %.0f %c%c -> %s, front to O2 before departure %.0f mm, corrected %d\n",
                   extras[e], physicalGap, pairs[k][0], pairs[k][1], st==TASK2RUN_DONE ? "DONE" : a->fault ? a->fault : "?",
                   minFront, a->faceCorrected);
        }
        assert(failed == 0);
        puts("PASS a wrong RANGE2 range with a late arrow 2 stops short of O2 and finishes");
    }
    {
        /* The camera's face is right; one stray short echo near the decision must not overwrite it. */
        const float gaps[] = {600, 1050, 1500};
        const char pairs[][2] = {{'L','R'},{'R','L'},{'L','L'},{'R','R'}};
        for (unsigned g = 0; g < 3; ++g) for (unsigned k = 0; k < 4; ++k) {
            Reset(0); physicalGap = gaps[g]; physicalWidth = 500; strayEcho = true;
            assert(Task2Run_Start(&run,17,pairs[k][0]=='L' ? TASK2_COMMAND_ARROW_LEFT : TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
            RunTo(TASK2_APP_RANGE2);
            Task2App *a = &run.course.app;
            const Task2RouteConfig *c = &a->geometry;
            float lens = c->bodyCenterAheadOfRearAxleMm + 0.5f*c->bodyLengthMm - c->cameraBehindFrontMm;
            float cx = physical.xMm + lens*cosf(physical.yawRad), cy = physical.yMm + lens*sinf(physical.yawRad);
            float depth = (physicalO1 + 100 + physicalGap - cx)*cosf(physical.yawRad) - cy*sinf(physical.yawRad);
            char line[32], ack[32];
            snprintf(line,sizeof line,"Z 17 %u",(unsigned)lroundf(depth)); snprintf(ack,sizeof ack,"ACK 17 Z %u\n",(unsigned)lroundf(depth));
            Send(line,ack);
            snprintf(line,sizeof line,"A 17 2 %c",pairs[k][1]); snprintf(ack,sizeof ack,"ACK 17 2 %c\n",pairs[k][1]);
            Send(line,ack);
            Task2RunStatus st=TASK2RUN_RUNNING;
            for(unsigned n=0;n<12000 && st==TASK2RUN_RUNNING;++n) st=Tick();
            printf("stray echo: gap %.0f %c%c injected %d -> %s, corrected %d estimate %.0f\n", physicalGap, pairs[k][0], pairs[k][1],
                   strayEchoDone, st==TASK2RUN_DONE ? "DONE" : a->fault ? a->fault : "?", a->faceCorrected, a->estimates.faceGap.mm);
            assert(strayEchoDone);
            assert(st==TASK2RUN_DONE && Task2Route_IsParked(&a->geometry,&physical));
            assert(!a->faceCorrected && fabsf(a->estimates.faceGap.mm - physicalGap) < 2.0f);
        }
        puts("PASS one stray short echo at the decision leaves the camera's face and the run finishes");
    }
    {
        /* Floor, 14 Oct 19:35: the ultrasonic read about 55 mm short with the gap already at the rules' 600 mm
         * minimum. Nothing can be nearer, so the robot departs at once: no correction, no wait to re-lock. */
        const char pairs[][2] = {{'L','R'},{'R','L'},{'L','L'},{'R','R'}};
        for (unsigned k = 0; k < 4; ++k) {
            Reset(0); physicalGap = 600; physicalWidth = 600; ussShortMm = 55;
            assert(Task2Run_Start(&run,18,pairs[k][0]=='L' ? TASK2_COMMAND_ARROW_LEFT : TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
            RunTo(TASK2_APP_RANGE2);
            char line[16], ack[16];
            snprintf(line,sizeof line,"A 18 2 %c",pairs[k][1]); snprintf(ack,sizeof ack,"ACK 18 2 %c\n",pairs[k][1]);
            Send(line,ack);
            Task2App *a = &run.course.app;
            Task2RunStatus st=TASK2RUN_RUNNING; unsigned acquireTicks = 0;
            for(unsigned n=0;n<12000 && st==TASK2RUN_RUNNING;++n) { st=Tick(); acquireTicks += a->phase==TASK2_APP_ACQUIRE2; }
            printf("short ultrasonic at 600: %c%c -> %s, corrected %d, acquire %u ms\n", pairs[k][0], pairs[k][1],
                   st==TASK2RUN_DONE ? "DONE" : a->fault ? a->fault : "?", a->faceCorrected, acquireTicks*10U);
            assert(st==TASK2RUN_DONE && Task2Route_IsParked(&a->geometry,&physical));
            /* Same-side arrows lock the ultrasonic in ACQUIRE2 on every run; only opposite arrows must not wait. */
            assert(!a->faceCorrected && (pairs[k][0]==pairs[k][1] || acquireTicks*10U < 500U));
        }
        puts("PASS a short ultrasonic at the 600 mm minimum departs without a correction or an extra wait");
    }
    {
        /* Floor, 14 Oct 19:35: the IR kept reading "present" about 30 cm past O2's real end. When no return fits
         * the measured end, plan around the proven face instead of stopping the run. */
        const char pairs[][2] = {{'L','R'},{'R','L'},{'L','R'},{'R','L'},{'L','R'},{'R','L'}};
        const float overshoots[] = {300, 300, 520, 520, 520, 520}; /* 520: the floor run's 1644 mm width */
        const float o1s[] = {1050, 1050, 1050, 1050, 600, 600};   /* 600 as on the floor */
        for (unsigned k = 0; k < 6; ++k) {
            Reset(0); physicalGap = 600; physicalWidth = 600; irOvershootMm = overshoots[k]; physicalO1 = o1s[k];
            assert(Task2Run_Start(&run,19,pairs[k][0]=='L' ? TASK2_COMMAND_ARROW_LEFT : TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
            RunTo(TASK2_APP_RANGE2);
            char line[16], ack[16];
            snprintf(line,sizeof line,"A 19 2 %c",pairs[k][1]); snprintf(ack,sizeof ack,"ACK 19 2 %c\n",pairs[k][1]);
            Send(line,ack);
            Task2App *a = &run.course.app;
            Task2RunStatus st=TASK2RUN_RUNNING;
            for(unsigned n=0;n<12000 && st==TASK2RUN_RUNNING;++n) st=Tick();
            bool narrowed = false;
            for(unsigned i=0;i<TASK2_DIAG_RECORDS;++i)
                narrowed |= a->diag.records[i].what && strcmp(a->diag.records[i].what,"EDGE_RETURN_PROVEN")==0;
            printf("IR present %.0f mm past the end, O1 %.0f: %c%c -> %s, observed width %.0f, narrowed %d\n", overshoots[k], o1s[k], pairs[k][0], pairs[k][1],
                   st==TASK2RUN_DONE ? "DONE" : a->fault ? a->fault : "?", a->detection.observedWidthMm, narrowed);
            assert(st==TASK2RUN_DONE && Task2Route_IsParked(&a->geometry,&physical));
        }
        puts("PASS an IR reading past O2's end does not stop the run");
    }
    {
        /* The face reads 1100, then a lower 760 still on the face (a darker panel, a joint between modules)
         * from 250 or 350 mm off its centre to the real end at 500 mm. A reading at the present level is never
         * the end, so the robot does not turn into the rest of the face. */
        const char pairs[][2] = {{'L','R'},{'R','L'}};
        const float lowFrom[] = {250, 350}, o1s[] = {600, 1050};
        for (unsigned k = 0; k < 2; ++k) for (unsigned f = 0; f < 2; ++f) for (unsigned o = 0; o < 2; ++o) {
            Reset(0); physicalGap = 600; physicalWidth = 1000; physicalO1 = o1s[o];
            irFaceAdc = 1100; irLowFromMm = lowFrom[f]; irLowAdc = 760; irPastEndAdc = 300;
            assert(Task2Run_Start(&run,20,pairs[k][0]=='L' ? TASK2_COMMAND_ARROW_LEFT : TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
            RunTo(TASK2_APP_RANGE2);
            char line[16], ack[16];
            snprintf(line,sizeof line,"A 20 2 %c",pairs[k][1]); snprintf(ack,sizeof ack,"ACK 20 2 %c\n",pairs[k][1]);
            Send(line,ack);
            Task2App *a = &run.course.app;
            Task2RunStatus st=TASK2RUN_RUNNING;
            for(unsigned n=0;n<12000 && st==TASK2RUN_RUNNING;++n) st=Tick();
            printf("face 1100 then 760 from %.0f mm, O1 %.0f: %c%c -> %s, observed width %.0f, face level %u\n", lowFrom[f], o1s[o],
                   pairs[k][0], pairs[k][1], st==TASK2RUN_DONE ? "DONE" : a->fault ? a->fault : "?", a->detection.observedWidthMm,
                   a->detection.irFaceLevelAdc);
            assert(st==TASK2RUN_DONE && Task2Route_IsParked(&a->geometry,&physical));
            assert(a->detection.observedWidthMm >= 1000.0f);
        }
        puts("PASS a lower reading still on the face does not end O2 early");
    }
    {
        /* The IR reads 675 for 150 mm past O2's end: not present, so it does not extend the proof, and above
         * the fixed absent level, so the end comes late and wide. No return fits that width; the robot plans
         * around the face it proved instead and parks. */
        const char pairs[][2] = {{'L','R'},{'R','L'}};
        for (unsigned k = 0; k < 2; ++k) {
            Reset(0); physicalO1 = 600; physicalGap = 600; physicalWidth = 500; irOvershootMm = 150; irPastEndAdc = 675;
            assert(Task2Run_Start(&run,23,pairs[k][0]=='L' ? TASK2_COMMAND_ARROW_LEFT : TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
            RunTo(TASK2_APP_RANGE2);
            char line[16], ack[16];
            snprintf(line,sizeof line,"A 23 2 %c",pairs[k][1]); snprintf(ack,sizeof ack,"ACK 23 2 %c\n",pairs[k][1]);
            Send(line,ack);
            Task2App *a = &run.course.app;
            Task2RunStatus st=TASK2RUN_RUNNING;
            for(unsigned n=0;n<12000 && st==TASK2RUN_RUNNING;++n) st=Tick();
            bool narrowed = false;
            for(unsigned i=0;i<TASK2_DIAG_RECORDS;++i)
                narrowed |= a->diag.records[i].what && strcmp(a->diag.records[i].what,"EDGE_RETURN_PROVEN")==0 && a->diag.records[i].ok;
            printf("proven-face fallback: %c%c -> %s, narrowed %d, planned width %.0f\n", pairs[k][0], pairs[k][1],
                   st==TASK2RUN_DONE ? "DONE" : a->fault ? a->fault : "?", narrowed, a->detection.observedWidthMm);
            assert(narrowed && st==TASK2RUN_DONE && Task2Route_IsParked(&a->geometry,&physical));
        }
        puts("PASS a late, wide IR end falls back to the proven face and the run parks");
    }
    {
        /* Floor, 14 Oct 23:30–23:34: O1 at 576, O2 1060 wide with its face nearer than the 600 mm the gap is held
         * to, same-side arrows. The robot stops close to the face, so it backs up 100 mm before the IR traverse.
         * The front guard sees the face near ahead; a reverse moves away from it and must run its full length. */
        const char pairs[][2] = {{'L','L'},{'R','R'}};
        const float gaps[] = {550, 560, 580};
        for (unsigned k = 0; k < 2; ++k) for (unsigned g = 0; g < 3; ++g) {
            Reset(0); physicalO1 = 576; physicalGap = gaps[g]; physicalWidth = 1060; irSharp = true;
            assert(Task2Run_Start(&run,26,pairs[k][0]=='L' ? TASK2_COMMAND_ARROW_LEFT : TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
            RunTo(TASK2_APP_RANGE2);
            char line[16], ack[16];
            snprintf(line,sizeof line,"A 26 2 %c",pairs[k][1]); snprintf(ack,sizeof ack,"ACK 26 2 %c\n",pairs[k][1]);
            Send(line,ack);
            Task2App *a = &run.course.app;
            Task2RunStatus st=TASK2RUN_RUNNING;
            for(unsigned n=0;n<15000 && st==TASK2RUN_RUNNING;++n) st=Tick();
            printf("floor same-side, gap %.0f W 1060 %c%c -> %s\n", gaps[g], pairs[k][0], pairs[k][1],
                   st==TASK2RUN_DONE ? "DONE" : a->fault ? a->fault : "?");
            assert(st==TASK2RUN_DONE && Task2Route_IsParked(&a->geometry,&physical));
        }
        puts("PASS a same-side back-up runs its full length with O2's face near ahead");
    }
    {
        /* The same on a legal course: O1 600, gap 600, the ultrasonic reading 35, 50 or 70 mm short in round 2, so
         * the robot ends nearer the face than planned. Every back-up runs its checked 100 mm before the replan. */
        const char pairs[][2] = {{'L','L'},{'R','R'}};
        const float widths[] = {1000, 1500}, shorts[] = {35, 50, 70};
        for (unsigned k = 0; k < 2; ++k) for (unsigned w = 0; w < 2; ++w) for (unsigned u = 0; u < 3; ++u) {
            Reset(0); physicalO1 = 600; physicalGap = 600; physicalWidth = widths[w]; ussShortMm = shorts[u];
            assert(Task2Run_Start(&run,27,pairs[k][0]=='L' ? TASK2_COMMAND_ARROW_LEFT : TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
            RunTo(TASK2_APP_RANGE2);
            char line[16], ack[16];
            snprintf(line,sizeof line,"A 27 2 %c",pairs[k][1]); snprintf(ack,sizeof ack,"ACK 27 2 %c\n",pairs[k][1]);
            Send(line,ack);
            Task2App *a = &run.course.app;
            Task2RunStatus st=TASK2RUN_RUNNING;
            bool wasBacking = false; Task2RoutePose from = physical; float travelled = 0.0f; unsigned backups = 0U, short_ = 0U;
            for(unsigned n=0;n<15000 && st==TASK2RUN_RUNNING;++n) {
                bool backing = a->backingUp;
                if (backing && !wasBacking) from = physical;
                if (!backing && wasBacking) {     /* the reverse has ended; the next move has not started yet */
                    travelled = hypotf(physical.xMm - from.xMm, physical.yMm - from.yMm); ++backups;
                    short_ += travelled < 99.0f;
                }
                wasBacking = backing;
                st=Tick();
            }
            printf("legal course, USS %.0f short, W %.0f %c%c -> %s, back-ups %u (last %.1f mm)\n", shorts[u], widths[w],
                   pairs[k][0], pairs[k][1], st==TASK2RUN_DONE ? "DONE" : a->fault ? a->fault : "?", backups, travelled);
            assert(st==TASK2RUN_DONE && Task2Route_IsParked(&a->geometry,&physical) && short_ == 0U);
        }
        puts("PASS on a legal course with a short ultrasonic, every same-side back-up runs its full length");
    }
    {
        /* The same back-ups with no echo at all while reversing. A reverse moves inward, so a missing echo then is
         * no end: no edge comes from it, the next forward turn is not braked by one, and O2 is never planned
         * narrower than it is. */
        const char pairs[][2] = {{'L','L'},{'R','R'}};
        const float widths[] = {1000, 1500}, shorts[] = {35, 50, 70};
        for (unsigned k = 0; k < 2; ++k) for (unsigned w = 0; w < 2; ++w) for (unsigned u = 0; u < 3; ++u) {
            Reset(0); physicalO1 = 600; physicalGap = 600; physicalWidth = widths[w]; ussShortMm = shorts[u];
            noEchoReversing = true;
            assert(Task2Run_Start(&run,28,pairs[k][0]=='L' ? TASK2_COMMAND_ARROW_LEFT : TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
            RunTo(TASK2_APP_RANGE2);
            char line[16], ack[16];
            snprintf(line,sizeof line,"A 28 2 %c",pairs[k][1]); snprintf(ack,sizeof ack,"ACK 28 2 %c\n",pairs[k][1]);
            Send(line,ack);
            Task2App *a = &run.course.app;
            Task2RunStatus st=TASK2RUN_RUNNING;
            bool wasBacking = false; Task2RoutePose from = physical; unsigned backups = 0U, short_ = 0U;
            for(unsigned n=0;n<15000 && st==TASK2RUN_RUNNING;++n) {
                bool backing = a->backingUp;
                if (backing && !wasBacking) from = physical;
                if (!backing && wasBacking) {
                    ++backups;
                    short_ += hypotf(physical.xMm - from.xMm, physical.yMm - from.yMm) < 99.0f;
                }
                wasBacking = backing;
                st=Tick();
            }
            printf("no echo while reversing, USS %.0f short, W %.0f %c%c -> %s, back-ups %u, planned width %.0f\n", shorts[u],
                   widths[w], pairs[k][0], pairs[k][1], st==TASK2RUN_DONE ? "DONE" : a->fault ? a->fault : "?", backups,
                   a->detection.observedWidthMm);
            assert(st==TASK2RUN_DONE && Task2Route_IsParked(&a->geometry,&physical) && short_ == 0U && backups >= 1U);
            assert(a->detection.observedWidthMm >= widths[w] - 1.0f);
        }
        puts("PASS no echo during a recovery reverse makes no edge and never narrows O2");
    }
#if TASK2_DIAGNOSTICS
    {
        /* The run trace follows a whole run: a sample every 50 ms from the start, in time order, through every
         * phase to the finish, with the pose it reports. */
        Reset(0); physicalGap = 1050; physicalWidth = 500;
        assert(Task2Run_Start(&run,25,TASK2_COMMAND_ARROW_LEFT,&io,now)==NULL);
        RunTo(TASK2_APP_RANGE2);
        Send("A 25 2 R","ACK 25 2 R\n");
        Task2RunStatus st=TASK2RUN_RUNNING;
        for(unsigned n=0;n<12000 && st==TASK2RUN_RUNNING;++n) st=Tick();
        assert(st==TASK2RUN_DONE);
        const Task2Trace *tr = &run.course.app.trace;
        assert(tr->count > 100U && tr->dropped == 0U);
        bool phases[TASK2_APP_FINISHED + 1] = {false};
        unsigned moving = 0U;
        for (unsigned i = 0U; i < tr->count; ++i) {
            const Task2TraceSample *x = &tr->samples[(tr->head + i) % TASK2_TRACE_SAMPLES];
            /* Every 50 ms; the final sample may follow its predecessor sooner. */
            if (i && !(x->state & TASK2_TRACE_STATE_FINAL))
                assert(x->t10Ms - tr->samples[(tr->head + i - 1U) % TASK2_TRACE_SAMPLES].t10Ms == TASK2_TRACE_PERIOD_MS / 10U);
            if (i + 1U < tr->count) assert(!(x->state & TASK2_TRACE_STATE_FINAL));
            if (x->phase <= TASK2_APP_FINISHED) phases[x->phase] = true;
            moving += (x->flags & TASK2_TRACE_MOVING) != 0U;
        }
        const Task2TraceSample *last = &tr->samples[(tr->head + tr->count - 1U) % TASK2_TRACE_SAMPLES];
        assert(phases[TASK2_APP_RANGE2] && phases[TASK2_APP_ROUTE] && moving > tr->count / 2U);
        assert(fabsf(last->xMm - run.course.app.pose.xMm) < 50.0f && fabsf(last->yMm - run.course.app.pose.yMm) < 50.0f);
        static char lines[TASK2_DIAG_PARTS][TASK2_DIAG_LINE_SIZE];
        assert(last->state & TASK2_TRACE_STATE_FINAL);
        assert(Task2Run_Trace(&run, "D R", lines, TASK2_DIAG_PARTS) == 1U && strncmp(lines[0], "DR 25 INDEX v=2 ", 16) == 0);
        printf("run trace: %u samples over %.1f s, %u moving, last pose %d %d\n", (unsigned)tr->count,
               tr->count * TASK2_TRACE_PERIOD_MS / 1000.0, moving, last->xMm, last->yMm);
        puts("PASS the run trace samples a whole run every 50 ms and is served as D R");
    }
#endif
    {
        /* Same-side: a darker part of a wide face reads 600 beyond where the face is proven. Asking to look again
         * from the proven face sets no end; O2 is never planned shorter than the face actually seen (the last
         * present reading). Below 700 once the face is seen counts as its end, so a part of the face reading
         * that low is taken as the end: a known limit of the present/absent rule, the same as without the
         * look-again request. */
        const char pairs[][2] = {{'L','L'},{'R','R'}};
        const float widths[] = {1200, 1500, 2000}, lowFrom[] = {500, 600}, gaps[] = {600, 750}, o1s[] = {600, 1050};
        unsigned runs = 0, done = 0, stops = 0;
        for (unsigned k = 0; k < 2; ++k) for (unsigned w = 0; w < 3; ++w) for (unsigned f = 0; f < 2; ++f)
        for (unsigned g = 0; g < 2; ++g) for (unsigned o = 0; o < 2; ++o) {
            if (lowFrom[f] >= widths[w] / 2) continue;
            Reset(0); physicalO1 = o1s[o]; physicalGap = gaps[g]; physicalWidth = widths[w];
            irFaceAdc = 1300; irLowFromMm = lowFrom[f]; irLowAdc = 600; irPastEndAdc = 300;
            assert(Task2Run_Start(&run,24,pairs[k][0]=='L' ? TASK2_COMMAND_ARROW_LEFT : TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
            RunTo(TASK2_APP_RANGE2);
            char line[16], ack[16];
            snprintf(line,sizeof line,"A 24 2 %c",pairs[k][1]); snprintf(ack,sizeof ack,"ACK 24 2 %c\n",pairs[k][1]);
            Send(line,ack);
            Task2App *a = &run.course.app;
            Task2RunStatus st=TASK2RUN_RUNNING;
            expectFault = true;
            for(unsigned n=0;n<15000 && st==TASK2RUN_RUNNING;++n) st=Tick();
            expectFault = false;
            ++runs;
            bool ok = st==TASK2RUN_DONE && Task2Route_IsParked(&a->geometry,&physical);
            done += ok; stops += st==TASK2RUN_FAULT;
            bool shortened = a->detection.detected && 0.5f * a->detection.observedWidthMm + 1.0f < lowFrom[f];
            if (!ok || shortened)
                printf("dark face part: %c%c W %.0f from %.0f gap %.0f O1 %.0f -> %s, width %.0f, rewinds %u\n", pairs[k][0], pairs[k][1],
                       widths[w], lowFrom[f], gaps[g], o1s[o], st==TASK2RUN_DONE ? "DONE" : a->fault ? a->fault : "?",
                       a->detection.observedWidthMm, a->traverseRewinds);
            assert(!(ok && shortened));
        }
        printf("dark face part: %u/%u finished and parked, %u stopped, none planned shorter than the face seen\n", done, runs, stops);
        puts("PASS a darker part of a same-side face is never planned shorter than the face seen");
    }
    {
        /* Measured IR curve, noisy readings, open-room background or an object behind O2's end; the ultrasonic
         * reads short so the traverse runs farther from the face than planned. The end must never be
         * short of the real one (a short end turns the robot into O2). At gap 1500 the planned traverse
         * passes 63..71 cm from the face, past the Sharp's 59 cm "present" level: those runs are printed,
         * not checked, until the departure brings the IR nearer. */
        const char pairs[][2] = {{'L','R'},{'R','L'}};
        const float gaps[] = {600, 1050, 1500}, widths[] = {300, 600, 800}, shorts[] = {0, 100, 200}, behind[] = {0, 70, 90};
        unsigned runs = 0, done = 0, near = 0, nearDone = 0; float worstShort = -1e9f, worstLong = -1e9f;
        for (unsigned k = 0; k < 2; ++k) for (unsigned g = 0; g < 3; ++g) for (unsigned w = 0; w < 3; ++w)
        for (unsigned u = 0; u < 3; ++u) for (unsigned b = 0; b < 3; ++b) {
            Reset(0); physicalGap = gaps[g]; physicalWidth = widths[w]; ussShortMm = shorts[u];
            irSharp = true; irBehindCm = behind[b]; irNoise = 1U + 7U * runs;
            assert(Task2Run_Start(&run,21,pairs[k][0]=='L' ? TASK2_COMMAND_ARROW_LEFT : TASK2_COMMAND_ARROW_RIGHT,&io,now)==NULL);
            RunTo(TASK2_APP_RANGE2);
            char line[16], ack[16];
            snprintf(line,sizeof line,"A 21 2 %c",pairs[k][1]); snprintf(ack,sizeof ack,"ACK 21 2 %c\n",pairs[k][1]);
            Send(line,ack);
            Task2App *a = &run.course.app;
            Task2RunStatus st=TASK2RUN_RUNNING;
            for(unsigned n=0;n<15000 && st==TASK2RUN_RUNNING;++n) st=Tick();
            ++runs;
            bool ok = st==TASK2RUN_DONE && Task2Route_IsParked(&a->geometry,&physical);
            done += ok;
            if (gaps[g] < 1500) { ++near; nearDone += ok || gaps[g] > 600; }
            float shortBy = physicalWidth - a->detection.observedWidthMm;
            if (gaps[g] < 1500 && a->detection.detected && a->detection.edgeMethod != TASK2_EDGE_IR_INITIAL_ABSENCE_BOUND) {
                if (shortBy > worstShort) worstShort = shortBy;
                if (-shortBy > worstLong) worstLong = -shortBy;
            }
            if (!ok || shortBy > 0.0f)
                printf("sharp IR %c%c gap %.0f width %.0f short %.0f behind %.0f -> %s, observed %.0f, level %u\n",
                       pairs[k][0], pairs[k][1], gaps[g], widths[w], shorts[u], behind[b],
                       st==TASK2RUN_DONE ? "DONE" : a->fault ? a->fault : "?", a->detection.observedWidthMm,
                       a->detection.irFaceLevelAdc);
        }
        printf("sharp IR: %u/%u finished and parked; gaps 600 and 1050: observed width at most %.0f mm short, at most %.0f mm long\n",
               done, runs, worstShort, worstLong);
        assert(nearDone == near && worstShort <= 0.0f);
        puts("PASS the measured IR model finishes every gap-600 run and never ends O2's face early at gaps 600 and 1050");
    }
    Task2Estimate estimates;Task2Estimate_Init(&estimates,&task2RouteConfig);
    assert(Task2Estimate_SetFace(&estimates,1,1050,20));assert(Task2Estimate_SetGap(&estimates,600,20,TASK2_ESTIMATE_CAMERA));
    Task2Estimate_SetSpan(&estimates,350);Task2RouteConfig geometry=task2RouteConfig;
    assert(Task2Estimate_Apply(&estimates,&geometry));assert(geometry.obstacle2WidthMm==500 && estimates.cameraSpan.bound);
    Task2Estimate_SetSpan(&estimates,600);assert(Task2Estimate_Apply(&estimates,&geometry));assert(geometry.obstacle2WidthMm==600);
    assert(fabsf(geometry.obstacle2EndSpaceMm+geometry.obstacle2WidthMm/2-750)<0.01f);
    assert(!Task2Estimate_SetGap(&estimates,NAN,20,TASK2_ESTIMATE_ULTRASONIC));
    /* A 600 mm course read a little short is still the course; well outside its uncertainty it is not. */
    assert(Task2Estimate_SetFace(&estimates,1,592,20) && estimates.obstacle1Face.mm==600);
    assert(Task2Estimate_SetFace(&estimates,2,600+100+585,20) && estimates.faceGap.mm==600);
    assert(Task2Estimate_SetFace(&estimates,2,600+100+1530,20) && estimates.faceGap.mm==1500);
    /* The floor's short gap reading (549 for 600) is still the course; well beyond the bias allowance it is not. */
    assert(Task2Estimate_SetFace(&estimates,1,614,20) && Task2Estimate_SetFace(&estimates,2,614+100+549,20) &&
           estimates.faceGap.mm==600);
    assert(!Task2Estimate_SetFace(&estimates,2,614+100+490,20) && !Task2Estimate_SetFace(&estimates,1,510,20));
    assert(!Task2Estimate_SetGap(&estimates,490,20,TASK2_ESTIMATE_ULTRASONIC) && Task2Estimate_SetGap(&estimates,540,20,TASK2_ESTIMATE_ULTRASONIC));
    puts("PASS replaceable estimates and camera lower-bound semantics");
}
