#include "Task2MotionTest.h"
#include "RobotTestFixture.h"
#include "Task2ObstacleSensors.h"
#include <stdarg.h>
#include <string.h>

static RobotTestFixture *robot;
static uint32_t tick, runStartTick;
static unsigned scenario, starts, rangeReads, rangeWaits;
static float rangeMm = 1090, leftFraction, rightFraction;
static bool rangeOk = true, mutateAtReady;
static char oledRows[8][64];
static Task2RoutePose idealPose;
static Task2RouteReport idealReport;

static uint32_t sensorTick, sensorDue;
static bool sensorPending;
static Task2ObstacleObservation pendingObservation;

bool Task2ObstacleSensors_Init(const Task2ObstacleSensorConfig *config)
{
    (void)config;
    sensorTick = sensorDue = 0U;
    sensorPending = false;
    return true;
}
void Task2ObstacleSensors_Stop(void) { sensorPending = false; }
uint32_t Task2ObstacleSensors_Poll(const Task2ObstacleSensorConfig *c, uint32_t now,
    const Task2RoutePose *pose, Task2ObstacleObservation out[TASK2_SENSOR_COUNT])
{
    uint32_t mask = 0;
    if (sensorPending && now >= sensorDue)
    {
        out[0] = pendingObservation;
        mask = 1;
        sensorPending = false;
    }
    if (!sensorPending && (!sensorTick || now - sensorTick >= c->samplePeriodMs))
    {
        sensorTick = now;
        const Task2SensorMount *m = &c->ultrasonic;
        float x=idealPose.xMm+m->aheadMm*cosf(idealPose.yawRad)-m->leftMm*sinf(idealPose.yawRad);
        float y=idealPose.yMm+m->aheadMm*sinf(idealPose.yawRad)+m->leftMm*cosf(idealPose.yawRad);
        float angle=idealPose.yawRad+m->yawDeg*PI/180;
        const Task2RouteConfig *r=&task2MotionTestRoute.config;
        float face=r->parkToObstacle1Mm+100+r->obstacleFaceGapMm;
        float range=INFINITY;
        for(unsigned i=0;i<=120;++i)
        {
            float a=angle+c->ultrasonicBeamHalfAngleDeg*PI/180*(2.0f*i/120-1);
            if(cosf(a)<=0.05f || face<=x)continue;
            float d=(face-x)/cosf(a);
            if(fabsf(y+d*sinf(a))<=r->obstacle2WidthMm/2)range=fminf(range,d);
        }
        pendingObservation=(Task2ObstacleObservation){.valid=isfinite(range),
            .value=isfinite(range)?range:0,.pose=*pose};
        if(scenario==4)pendingObservation.valid=false;
        if(scenario==5)
        {
            pendingObservation.valid=true;
            pendingObservation.value=(face-x)/fmaxf(cosf(angle),0.05f);
        }
        if(scenario==6)pendingObservation.fault=true;
        sensorDue=now+(pendingObservation.valid?5:50);
        sensorPending=true;
    }
    return mask;
}

static float mmPerCount(void)
{
    return PI * motionControllerConfig.kinematics->rearWheelDiameterMm /
        motionControllerConfig.kinematics->rearEncoderCountsPerRev;
}

bool RobotTestFixture_ReadUltrasonicMm(float *distance)
{
    assert(robot == NULL && starts == 0);
    ++rangeReads;
    *distance = rangeMm;
    return rangeOk;
}

bool RobotTestFixture_InitMotionController(RobotTestFixture *f, const MotionControllerConfig *c)
{
    *f = (RobotTestFixture){0};
    robot = f;
    f->leftWheelController.motor = &f->leftRearWheel;
    f->rightWheelController.motor = &f->rightRearWheel;
    assert(SteeringController_Init(&f->steeringController, &f->steeringServo, &steeringCalibration));
    gyroDps = 0; imuOk = true;
    return MotionController_Init(&f->motionController, &f->leftWheelController,
        &f->rightWheelController, &f->steeringController, &f->imu, c) == 0;
}

uint32_t HAL_GetTick(void)
{
    ++tick;
    if (robot && scenario != 2 && starts > 0)
    {
        WheelSpeedController *l = &robot->leftWheelController, *r = &robot->rightWheelController;
        leftFraction += l->targetSpeedCps * 0.001f;
        rightFraction += r->targetSpeedCps * 0.001f;
        int dl = (int)leftFraction, dr = (int)rightFraction;
        leftFraction -= dl; rightFraction -= dr;
        robot->leftRearWheel.state.encCount += dl;
        robot->rightRearWheel.state.encCount += dr;
        l->measuredSpeedCps = l->targetSpeedCps;
        r->measuredSpeedCps = r->targetSpeedCps;
        float ds = 0.0005f * (l->targetSpeedCps + r->targetSpeedCps) * mmPerCount();
        float curvature = robot->motionController.targetCurvaturePerMm;
        float middle = idealPose.yawRad + 0.5f * ds * curvature;
        idealPose.xMm += ds * cosf(middle);
        idealPose.yMm += ds * sinf(middle);
        idealPose.yawRad += ds * curvature;
        gyroDps = curvature * 0.5f * (l->targetSpeedCps + r->targetSpeedCps) * mmPerCount() * 180 / PI;
        if (ds > 0)
            (void)Task2Route_CheckPose(&task2MotionTestRoute.config, &idealPose, &idealReport);
        if (scenario == 3 && tick - runStartTick > 2000)
            imuOk = false;
    }
    return tick;
}
void HAL_Delay(uint32_t ms) { for (uint32_t i=0; i<ms; ++i) (void)HAL_GetTick(); }
void OLED_Clear(void) { memset(oledRows, 0, sizeof(oledRows)); }
void OLED_Refresh_Gram(void) { }
void OLED_Printf(uint8_t x, uint8_t y, const char *fmt, ...)
{
    (void)x; assert(y < 8);
    va_list args; va_start(args,fmt); vsnprintf(oledRows[y],sizeof(oledRows[y]),fmt,args); va_end(args);
}
void SW1_WaitForPressAndRelease(void)
{
    if (strstr(oledRows[0],"Measure O1 range"))
    {
        assert(robot == NULL && starts == 0);
        ++rangeWaits;
    }
    else
    {
        assert(robot && task2MotionTestState == TASK2_TEST_READY);
        assert(strstr(oledRows[0],"Task2 stopped ready"));
        assert(task2MotionTestRoute.report.valid);
        near(robot->motionController.arcYawRatePID.kp,270,0);
        near(robot->motionController.arcYawRatePID.ki,150,0);
        assert(!robot->motionController.config->useMatchedYawRateReferenceFilter);
        ++starts;
        runStartTick = tick;
        if (mutateAtReady) task2RouteConfig.firstArrow = -task2RouteConfig.firstArrow;
    }
    HAL_Delay(1000); /* Manual waiting must not count towards motion timing. */
}
int SW1_ReadState(void) { return scenario == 1 && tick-runStartTick > 2000; }

static void resetHardware(unsigned nextScenario)
{
    scenario = nextScenario;
    robot = NULL;
    tick = runStartTick = starts = rangeReads = rangeWaits = 0;
    leftFraction = rightFraction = 0;
    rangeOk = true; mutateAtReady = false;
    idealPose = (Task2RoutePose){TASK2_ROUTE_START_X_MM,0,0};
    idealReport = (Task2RouteReport){.minimumObstacleGapMm=INFINITY,
        .minimumSideMarginMm=INFINITY,.minimumParkWallMarginMm=INFINITY,
        .minimumParkBackMarginMm=INFINITY,.minimumParkSideMarginMm=INFINITY};
}

static void checkCompleted(void)
{
    assert(starts == 1 && task2MotionTestState == TASK2_TEST_COMPLETE);
    assert(task2MotionTestResult.passed && task2MotionTestResult.completedSteps == task2MotionTestRoute.stepCount);
    assert(task2MotionTestResult.arrow1Recognised && task2MotionTestResult.obstacle1ExitPoseEstimateValid);
    assert(Task2Route_Obstacle1CameraVisible(&task2MotionTestRoute.config,
        &task2MotionTestResult.recognitionPoseEstimate));
    if(idealReport.minimumObstacleGapMm<=0 || idealReport.minimumSideMarginMm<=0 || idealReport.minimumParkWallMarginMm<=0)
        fprintf(stderr,"Collision layout %.0f %.0f %.0f %d %d: gap %.1f side %.1f park %.1f, final %.1f %.1f %.1f\n", task2MotionTestRoute.config.parkToObstacle1Mm,task2MotionTestRoute.config.obstacleFaceGapMm,task2MotionTestRoute.config.obstacle2WidthMm,task2MotionTestRoute.config.firstArrow,task2MotionTestRoute.config.secondArrow,idealReport.minimumObstacleGapMm,idealReport.minimumSideMarginMm,idealReport.minimumParkWallMarginMm,idealPose.xMm,idealPose.yMm,idealPose.yawRad*180/PI);
    assert(idealReport.minimumObstacleGapMm > 0 && idealReport.minimumSideMarginMm > 0 &&
        idealReport.minimumParkWallMarginMm > 0);
    near(task2MotionTestResult.obstacle1ExitPoseEstimate.yMm,task2MotionTestRoute.config.firstArrow*363.28f,20);
    near(task2MotionTestResult.obstacle1ExitPoseEstimate.yawRad,-task2MotionTestRoute.config.firstArrow*19*PI/180,0.04f);
    assert(task2MotionTestResult.arrow2Recognised && task2MotionTestResult.obstacle2EdgeStopped);
    assert(Task2Route_ObstacleCameraVisible(&task2MotionTestRoute.config,
        &task2MotionTestResult.arrow2RecognitionPoseEstimate, 1U));
    assert(task2MotionTestResult.arrow2RecognitionElapsedMs > task2MotionTestResult.recognitionElapsedMs);
    near(task2MotionTestRoute.steps[TASK2_ROUTE_O2_APPROACH_STEP].speedCps,
        task2MotionTestRoute.config.cruiseSpeedCps, 0);
    near(task2MotionTestStepResults[TASK2_ROUTE_O2_APPROACH_STEP].measuredDistanceMm,
        task2MotionTestRoute.steps[TASK2_ROUTE_O2_APPROACH_STEP].distanceMm, 10);
    assert(task2MotionTestDetection.detected && task2MotionTestDetection.observedWidthMm>=300);
    assert(task2MotionTestLogCount > 1 && !task2MotionTestResult.logTruncated);
    assert(task2MotionTestLog[task2MotionTestLogCount-1].mode == MOTIONCONTROLLER_IDLE);
    for (unsigned i=0; i<task2MotionTestRoute.stepCount; ++i)
        if (i != TASK2_ROUTE_O2_DEPARTURE_STEP && task2MotionTestRoute.steps[i].radiusMm != 0)
            assert(task2MotionTestStepResults[i].yawPredictedReached);
    bool accelerated = false, obstacle2Accelerated = false;
    for (unsigned i=0; i<task2MotionTestLogCount; ++i)
    {
        const Task2MotionTestSample *s=&task2MotionTestLog[i];
        if (s->timeMs < task2MotionTestResult.recognitionElapsedMs)
            assert(s->targetSpeedCps <= task2MotionTestRoute.config.approachSpeedCps+0.5f);
        if (s->stepIndex == 0 && s->timeMs > task2MotionTestResult.recognitionElapsedMs &&
            s->targetSpeedCps > task2MotionTestRoute.config.approachSpeedCps+100)
            accelerated=true;
        if (s->stepIndex == TASK2_ROUTE_O2_APPROACH_STEP)
        {
            if (s->timeMs < task2MotionTestResult.arrow2RecognitionElapsedMs)
                assert(s->targetSpeedCps <= task2MotionTestRoute.config.approachSpeedCps+0.5f);
            else if (s->targetSpeedCps > task2MotionTestRoute.config.approachSpeedCps+100)
                obstacle2Accelerated=true;
        }
        if (s->stepIndex == TASK2_ROUTE_O2_DEPARTURE_STEP)
            assert(s->targetSpeedCps <= task2MotionTestRoute.config.obstacle2DepartureSpeedCps+0.5f);
    }
    assert(accelerated);
    /* Short approaches can finish before exceeding the approach speed. */
    if (task2MotionTestRoute.steps[TASK2_ROUTE_O2_APPROACH_STEP].distanceMm >= 300)
        assert(obstacle2Accelerated);
    const Task2RouteConfig *c=&task2MotionTestRoute.config;
    float centre=c->bodyCenterAheadOfRearAxleMm+0.5f*c->frontProtrusionMm;
    float length=c->bodyLengthMm+c->frontProtrusionMm;
    float cx=idealPose.xMm+centre*cosf(idealPose.yawRad), cy=idealPose.yMm+centre*sinf(idealPose.yawRad);
    float ex=0.5f*(length*fabsf(cosf(idealPose.yawRad))+c->bodyWidthMm*fabsf(sinf(idealPose.yawRad)));
    float ey=0.5f*(length*fabsf(sinf(idealPose.yawRad))+c->bodyWidthMm*fabsf(cosf(idealPose.yawRad)));
    assert(cx-ex > -500 && cx+ex < 0 && fabsf(cy)+ey < 300);
}

static void testSensorEvents(void)
{
    Task2ObstacleSensorConfig config=task2ObstacleSensorConfig;
    config.ultrasonic=(Task2SensorMount){0,0,0};
    Task2RouteConfig route=task2RouteConfig;
    route.secondArrow=1;
    float face=route.parkToObstacle1Mm+100+route.obstacleFaceGapMm;
    Task2ObstacleObservation o={.pose={face-500,0,40*PI/180}};
    Task2ObstacleDetection state={0};
    for(unsigned i=0;i<10;++i)Task2ObstacleDetection_Observe(&state,&config,&route,0,&o,true);
    assert(!state.detected && !state.armed[0]);
    o.valid=true;o.value=500/cosf(o.pose.yawRad);
    for(unsigned i=0;i<config.presentSamples;++i)Task2ObstacleDetection_Observe(&state,&config,&route,0,&o,false);
    assert(state.armed[0]);
    o.valid=false;
    for(unsigned i=0;i<config.absentSamples-1;++i)Task2ObstacleDetection_Observe(&state,&config,&route,0,&o,true);
    assert(!state.detected);
    o.valid=true;Task2ObstacleDetection_Observe(&state,&config,&route,0,&o,true);
    assert(state.absent[0]==0);
    o.valid=false;
    for(unsigned i=0;i<config.absentSamples;++i)Task2ObstacleDetection_Observe(&state,&config,&route,0,&o,true);
    assert(state.detected && state.source==TASK2_SENSOR_ULTRASONIC);
    near(state.observedWidthMm,2*(500*tanf(25*PI/180)+config.edgeUncertaintyMm),0.1f);
    float width=state.observedWidthMm;
    state=(Task2ObstacleDetection){0};config.ultrasonic.leftMm=50;
    o.valid=true;o.value=600;
    for(unsigned i=0;i<3;++i)Task2ObstacleDetection_Observe(&state,&config,&route,0,&o,false);
    o.valid=false;
    for(unsigned i=0;i<3;++i)Task2ObstacleDetection_Observe(&state,&config,&route,0,&o,true);
    assert(state.detected && state.observedWidthMm>width+80);
    config.rightIR=(Task2SensorMount){0,0,0};state=(Task2ObstacleDetection){0};
    o.valid=true;o.value=400;
    for(unsigned i=0;i<3;++i)Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);
    assert(!state.armed[2] && !state.detected && state.absent[2]==0);
    o.value=700;
    for(unsigned i=0;i<2;++i)Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,false);
    assert(!state.armed[2]);
    o.value=699;Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,false);
    assert(state.present[2]==0);
    o.value=700;
    for(unsigned i=0;i<3;++i)Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,false);
    assert(state.armed[2]);
    const float noisyPresent[]={900,750,850,700,800};
    for(unsigned i=0;i<5;++i){o.value=noisyPresent[i];Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);}
    o.value=401;
    for(unsigned i=0;i<5;++i)Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);
    assert(!state.detected && state.absent[2]==0);
    o.value=400;
    for(unsigned i=0;i<2;++i)Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);
    assert(!state.detected && state.absent[2]==2);
    o.value=401;Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);
    assert(state.absent[2]==0 && state.armed[2]);
    o.value=400;
    for(unsigned i=0;i<3;++i)Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);
    assert(state.detected && state.source==TASK2_SENSOR_RIGHT_IR);
    near(state.observedWidthMm,2*(500*tanf(40*PI/180)+config.edgeUncertaintyMm),0.1f);
    state=(Task2ObstacleDetection){0};
    o.valid=false;Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);
    assert(state.fault && !state.detected);
    state=(Task2ObstacleDetection){0};o.valid=true;o.value=4096;
    Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);
    assert(state.fault && !state.detected);
    assert(Task2ObstacleDetection_ValidConfig(&config));
    config.irAbsentAdc=700;assert(!Task2ObstacleDetection_ValidConfig(&config));
    config.irAbsentAdc=400;config.irPresentAdc=4096;
    assert(!Task2ObstacleDetection_ValidConfig(&config));
    config.irPresentAdc=700;
    config.ultrasonic.aheadMm=NAN;
    assert(!Task2ObstacleDetection_ValidConfig(&config));
    puts("PASS: acquired-before-lost, fresh-sample confirmation, dropout reset, configurable mounts, IR hysteresis and faults");
}

static void testTraverseFallback(void)
{
    Task2Route route;
    Task2RouteConfig config=task2RouteConfig;
    config.parkToObstacle1Mm=600;config.obstacleFaceGapMm=600;
    config.obstacle2WidthMm=500;config.firstArrow=-1;config.secondArrow=1;
    assert(Task2Route_Create(&config,&route));
    Task2RoutePose stopped={1050,-300,80*PI/180};
    assert(Task2Route_ReplanObstacle2(&route,&stopped,500));
    assert(route.stepCount==14);
    task2MotionTestRoute=route;
    resetHardware(0);
    idealPose=stopped;
    RobotTestFixture fixture;
    assert(RobotTestFixture_InitMotionController(&fixture,&task2MotionTestControllerConfig));
    starts=1;
    for(unsigned i=5;i<route.stepCount;++i)
    {
        const Task2RouteStep *step=&route.steps[i];
        assert((step->radiusMm==0 ? MotionController_MoveStraight(&fixture.motionController,step->distanceMm,step->speedCps)
            : MotionController_MoveArc(&fixture.motionController,step->distanceMm,step->radiusMm,step->speedCps))==0);
        unsigned n=0;
        while(MotionController_IsBusy(&fixture.motionController))
        {
            HAL_Delay(10);
            assert(MotionController_Update(&fixture.motionController,0.01f)==0);
            assert(++n<10000);
        }
    }
    assert(idealReport.minimumObstacleGapMm>0 && idealReport.minimumSideMarginMm>0 && idealReport.minimumParkWallMarginMm>0);
    assert(cosf(idealPose.yawRad)<0);
    for(unsigned f=0;f<2;++f)for(int side=-1;side<=1;side+=2)
    {
        float forward=f ? 210.0f : -37.0f;
        float x=idealPose.xMm+forward*cosf(idealPose.yawRad)-side*100*sinf(idealPose.yawRad);
        float y=idealPose.yMm+forward*sinf(idealPose.yawRad)+side*100*cosf(idealPose.yawRad);
        assert(x>-500 && x<0 && fabsf(y)<300);
    }
    puts("PASS: front/back traverse fallback through real stopped controller, end-space clearance and parking");
}

int main(void)
{
    testSensorEvents();
    const Task2RouteConfig original=task2RouteConfig;
    const Task2MotionTestConfig originalTest=task2MotionTestConfig;
    task2MotionTestConfig.measureObstacle1AtStartup=false;
    const float positions[]={600,1050}, gaps[]={600,750,1050,1500}, widths[]={300,350,400,500};
    unsigned runs=0, shortReturns=0, alignedReturns=0, traverseReturns=0;
    for(unsigned p=0;p<2;++p)for(unsigned g=0;g<4;++g)for(unsigned w=0;w<4;++w)
    for(int left=-1;left<=1;left+=2)for(int right=-1;right<=1;right+=2)
    {
        task2RouteConfig=original;
        task2RouteConfig.parkToObstacle1Mm=positions[p]; task2RouteConfig.obstacleFaceGapMm=gaps[g];
        task2RouteConfig.obstacle2WidthMm=widths[w]; task2RouteConfig.firstArrow=left; task2RouteConfig.secondArrow=right;
        task2RouteConfig.recognitionCameraRangeMm=1000;
        resetHardware(0); Task2MotionTestRun();
        if(!task2MotionTestResult.passed)fprintf(stderr,"Failed %u: %u %s, width %.1f, stop %.1f %.1f %.1f\n",runs,task2MotionTestResult.status,task2MotionTestRoute.report.reason,task2MotionTestDetection.observedWidthMm,task2MotionTestResult.obstacle2StoppedPose.xMm,task2MotionTestResult.obstacle2StoppedPose.yMm,task2MotionTestResult.obstacle2StoppedPose.yawRad*180/PI);
        checkCompleted(); ++runs;
        if(task2MotionTestRoute.stepCount==8)++shortReturns;
        if(task2MotionTestRoute.stepCount==11)++alignedReturns;
        if(task2MotionTestRoute.stepCount==14)++traverseReturns;
    }
    assert(shortReturns+alignedReturns+traverseReturns==runs);
    printf("Return variants: %u direct, %u aligned, %u traverses\n",shortReturns,alignedReturns,traverseReturns);
    puts("PASS: 128 sensor-driven stopped routes, arrows, 600-1500mm gaps, widths, acquisition, observed-edge replanning and footprints");
    task2RouteConfig=original;
    testTraverseFallback();
    task2MotionTestConfig=originalTest;
    resetHardware(0); rangeMm=1090; mutateAtReady=true; Task2MotionTestRun();
    checkCompleted(); assert(rangeReads==1 && rangeWaits==1);
    near(task2MotionTestRoute.config.parkToObstacle1Mm,1050,0);
    assert(task2MotionTestRoute.config.firstArrow==original.firstArrow);
    const float invalidRanges[]={NAN,INFINITY,0,19,4001,639,1090};
    for(unsigned i=0;i<7;++i)
    {
        task2RouteConfig=original; resetHardware(0); rangeMm=invalidRanges[i]; rangeOk=i!=6;
        Task2MotionTestRun(); assert(!task2MotionTestResult.passed && starts==0 && robot==NULL);
        assert(rangeReads==1);
    }
    task2RouteConfig=original; task2MotionTestConfig.measureObstacle1AtStartup=false;
    task2RouteConfig.firstArrow=0; resetHardware(0); Task2MotionTestRun();
    assert(!task2MotionTestResult.passed && robot==NULL && starts==0);
    for(unsigned s=1;s<=6;++s)
    {
        task2RouteConfig=original; resetHardware(s); Task2MotionTestRun();
        assert(!task2MotionTestResult.passed && starts==1);
        assert(robot->motionController.mode==MOTIONCONTROLLER_IDLE);
        if(s==1)assert(task2MotionTestResult.cancelled);
        if(s==2)assert(task2MotionTestResult.timedOut);
        if(s>=3)assert(task2MotionTestResult.status!=0);
    }
    puts("PASS: measured asymmetric frame, one startup range, frozen configuration, invalid input, cancellation, timeout and IMU fault");
    return 0;
}
