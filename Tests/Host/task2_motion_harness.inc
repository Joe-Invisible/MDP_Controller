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
static bool physicalCrossings[2][2];

static uint32_t sensorTick, sensorDue;
static bool sensorPending;
static unsigned irReads;
static float irStraightPreviewMm;
static unsigned maxStartupChecks, maxReturnChecks;
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
    if (task2MotionTestStepIndex == TASK2_ROUTE_O2_APPROACH_STEP ||
        (task2MotionTestRoute.obstacle2IRDeparture && task2MotionTestStepIndex == TASK2_ROUTE_O2_DEPARTURE_STEP))
        assert(!c->sideIREnabled);
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
        float sensorWidth = scenario == 12 ? 300.0f : r->obstacle2WidthMm;
        float face=r->parkToObstacle1Mm+100+r->obstacleFaceGapMm;
        float range=INFINITY;
        for(unsigned i=0;i<=120;++i)
        {
            float a=angle+c->ultrasonicBeamHalfAngleDeg*PI/180*(2.0f*i/120-1);
            if(cosf(a)<=0.05f || face<=x)continue;
            float d=(face-x)/cosf(a);
            if(fabsf(y+d*sinf(a))<=sensorWidth/2)range=fminf(range,d);
        }
        pendingObservation=(Task2ObstacleObservation){.valid=isfinite(range),
            .value=isfinite(range)?range:0,.pose=*pose};
        if(scenario==4)pendingObservation.valid=false;
        if(scenario==5 || scenario==9)
        {
            pendingObservation.valid=true;
            pendingObservation.value=(face-x)/fmaxf(cosf(angle),0.05f);
        }
        if(scenario==6)pendingObservation.fault=true;
        sensorDue=now+(pendingObservation.valid?5:50);
        sensorPending=true;
        if (c->sideIREnabled)
        {
            ++irReads;
            const Task2SensorMount *mounts[]={&c->leftIR,&c->rightIR};
            for(unsigned i=0;i<2;++i)
            {
                const Task2SensorMount *ir=mounts[i];
                float sx=idealPose.xMm+ir->aheadMm*cosf(idealPose.yawRad)-ir->leftMm*sinf(idealPose.yawRad);
                float sy=idealPose.yMm+ir->aheadMm*sinf(idealPose.yawRad)+ir->leftMm*cosf(idealPose.yawRad);
                float ray=idealPose.yawRad+ir->yawDeg*PI/180;
                bool onFace=cosf(ray)>0.05f && face>sx &&
                    fabsf(sy+(face-sx)*tanf(ray))<=sensorWidth/2;
                float adc = scenario==13 ? 550 : scenario==7 ? 300 : onFace || scenario==5 || scenario==11 ? 900 : 300;
                if (scenario == 10 && cosf(ray) > 0.05f && face > sx)
                {
                    float beyond = r->secondArrow * (sy + (face-sx)*tanf(ray)) - r->obstacle2WidthMm/2;
                    /* Gradual cone loss reaches 400 about 125 mm beyond the centre-ray edge. */
                    adc = fmaxf(300, fminf(900, 900 - 4 * beyond));
                }
                out[i+1]=(Task2ObstacleObservation){.valid=scenario!=8,.fault=scenario==8,
                    .value=adc,.pose=*pose};
            }
            mask|=6U;
        }
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
        float previousX = idealPose.xMm;
        float curvature = robot->motionController.targetCurvaturePerMm;
        float middle = idealPose.yawRad + 0.5f * ds * curvature;
        idealPose.xMm += ds * cosf(middle);
        idealPose.yMm += ds * sinf(middle);
        idealPose.yawRad += ds * curvature;
        gyroDps = curvature * 0.5f * (l->targetSpeedCps + r->targetSpeedCps) * mmPerCount() * 180 / PI;
        if (ds > 0)
        {
            for (unsigned obstacle = 0; obstacle < 2; ++obstacle)
            {
                float centre = task2MotionTestRoute.config.parkToObstacle1Mm + 50 +
                    (obstacle ? 100 + task2MotionTestRoute.config.obstacleFaceGapMm : 0);
                if ((previousX < centre && idealPose.xMm >= centre) ||
                    (previousX > centre && idealPose.xMm <= centre))
                {
                    unsigned returning = cosf(idealPose.yawRad) < 0;
                    int side = returning ? -task2MotionTestRoute.config.secondArrow : obstacle
                        ? task2MotionTestRoute.config.secondArrow : task2MotionTestRoute.config.firstArrow;
                    assert(side * idealPose.yMm > 0);
                    physicalCrossings[obstacle][returning] = true;
                }
            }
        }
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
#if TASK2_ROUTE_EXHAUSTIVE_CHECKS
        assert(task2MotionTestRoute.report.clearanceFromStep==0 &&
            task2MotionTestRoute.report.recognitionPreviewValid);
#else
        assert(task2MotionTestRoute.report.clearanceFromStep==TASK2_ROUTE_O2_APPROACH_STEP &&
            !task2MotionTestRoute.report.recognitionPreviewValid);
        assert(task2MotionTestRoute.report.minimumObstacleGapMm>=task2MotionTestRoute.config.obstacleBufferMm);
#endif
        if(task2MotionTestRoute.report.plannerPoseChecks>maxStartupChecks)
            maxStartupChecks=task2MotionTestRoute.report.plannerPoseChecks;
        near(robot->motionController.arcYawRatePID.kp,200,0);
        near(robot->motionController.arcYawRatePID.ki,100,0);
        assert(!robot->motionController.config->useMatchedYawRateReferenceFilter);
        irStraightPreviewMm = task2MotionTestRoute.steps[TASK2_ROUTE_O2_IR_STRAIGHT_STEP].distanceMm;
        ++starts;
        runStartTick = tick;
        if (mutateAtReady) task2RouteConfig.firstArrow = -task2RouteConfig.firstArrow;
    }
    HAL_Delay(1000); /* Manual waiting must not count towards motion timing. */
}
int SW1_ReadState(void)
{
    return (scenario == 1 && tick-runStartTick > 2000) ||
        (scenario == 11 && task2MotionTestStepIndex == TASK2_ROUTE_O2_IR_STRAIGHT_STEP &&
         fabsf(robot->motionController.travelledDistanceMm) >
            task2MotionTestRoute.steps[TASK2_ROUTE_O2_IR_STRAIGHT_STEP].distanceMm + 20);
}

static void resetHardware(unsigned nextScenario)
{
    scenario = nextScenario;
    robot = NULL;
    tick = runStartTick = starts = rangeReads = rangeWaits = 0;
    leftFraction = rightFraction = 0;
    irReads = 0U;
    memset(physicalCrossings, 0, sizeof(physicalCrossings));
    rangeOk = true; mutateAtReady = false;
    idealPose = (Task2RoutePose){TASK2_ROUTE_START_X_MM,0,0};
    idealReport = (Task2RouteReport){.minimumObstacleGapMm=INFINITY,
        .minimumSideMarginMm=INFINITY,.minimumParkWallMarginMm=INFINITY,
        .minimumParkBackMarginMm=INFINITY,.minimumParkSideMarginMm=INFINITY};
}

static void checkCompleted(void)
{
    if(task2MotionTestRoute.report.plannerPoseChecks>maxReturnChecks)
        maxReturnChecks=task2MotionTestRoute.report.plannerPoseChecks;
    if(task2MotionTestState != TASK2_TEST_COMPLETE)fprintf(stderr,"FLAGS arrows%d/%d edge%d\n",task2MotionTestResult.arrow1Recognised,task2MotionTestResult.arrow2Recognised,task2MotionTestResult.obstacle2EdgeStopped);
    if(task2MotionTestState != TASK2_TEST_COMPLETE)fprintf(stderr,"FAIL status%d elapsed%u steps%u/%u timedout%d cancelled%d final %.1f %.1f %.3f\n",task2MotionTestResult.status,task2MotionTestResult.elapsedMs,task2MotionTestResult.completedSteps,task2MotionTestRoute.stepCount,task2MotionTestResult.timedOut,task2MotionTestResult.cancelled,idealPose.xMm,idealPose.yMm,idealPose.yawRad);
    if(task2MotionTestState != TASK2_TEST_COMPLETE)fprintf(stderr,"STOP width%.2f edge method%d edge %.2f %.2f stop %.2f %.2f %.2f quarter %.2f %.2f %.2f\n",task2MotionTestDetection.observedWidthMm,task2MotionTestDetection.edgeMethod,task2MotionTestDetection.edgePose.xMm,task2MotionTestDetection.edgePose.yMm,task2MotionTestResult.obstacle2StoppedPose.xMm,task2MotionTestResult.obstacle2StoppedPose.yMm,task2MotionTestResult.obstacle2StoppedPose.yawRad,task2MotionTestResult.obstacle2ParallelStoppedPose.xMm,task2MotionTestResult.obstacle2ParallelStoppedPose.yMm,task2MotionTestResult.obstacle2ParallelStoppedPose.yawRad);
    assert(starts == 1 && task2MotionTestState == TASK2_TEST_COMPLETE);
    assert(task2MotionTestResult.passed && task2MotionTestResult.completedSteps == task2MotionTestRoute.stepCount);
    for (unsigned obstacle=0; obstacle<2; ++obstacle)
        assert(physicalCrossings[obstacle][0] && physicalCrossings[obstacle][1]);
#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
    assert(!task2MotionTestRoute.report.planningBudgetExceeded);
    assert(task2MotionTestRoute.report.plannerPoseChecks <= TASK2_ROUTE_RUNTIME_POSE_CHECK_LIMIT);
#endif
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
    if(task2MotionTestRoute.obstacle2IRDeparture)
    {
        assert(!task2MotionTestRoute.obstacle2ReturnPending && !task2MotionTestRoute.obstacle2ParallelReturnPending);
        assert(task2MotionTestRoute.report.obstacleBufferExceptionUsed);
        near(task2MotionTestResult.obstacle2ParallelStoppedPose.yawRad,0,0.04f);
        assert(task2MotionTestDetection.source==(task2MotionTestRoute.config.secondArrow==1
            ? TASK2_SENSOR_RIGHT_IR : TASK2_SENSOR_LEFT_IR));
    }
    assert(task2MotionTestLogCount > 1 && !task2MotionTestResult.logTruncated);
    assert(task2MotionTestLog[task2MotionTestLogCount-1].mode == MOTIONCONTROLLER_IDLE);
    for (unsigned i=0; i<task2MotionTestRoute.stepCount; ++i)
        if (i != TASK2_ROUTE_O2_DEPARTURE_STEP && task2MotionTestRoute.steps[i].radiusMm != 0)
        {
            if(!task2MotionTestStepResults[i].yawPredictedReached)
                fprintf(stderr,"Yaw exit step %u length %.2f R%.0f, layout %.0f %.0f %.0f %d %d\n",i,task2MotionTestRoute.steps[i].distanceMm,task2MotionTestRoute.steps[i].radiusMm,task2MotionTestRoute.config.parkToObstacle1Mm,task2MotionTestRoute.config.obstacleFaceGapMm,task2MotionTestRoute.config.obstacle2WidthMm,task2MotionTestRoute.config.firstArrow,task2MotionTestRoute.config.secondArrow);
            assert(task2MotionTestStepResults[i].yawPredictedReached);
        }
    bool accelerated = false, obstacle2Accelerated = false;
    unsigned freshIRReads=0;
    for (unsigned i=0; i<task2MotionTestLogCount; ++i)
    {
        const Task2MotionTestSample *s=&task2MotionTestLog[i];
        if(s->irFresh)
        {
            ++freshIRReads;
            assert(s->irObserved && s->irValid && s->irAdc<=4095);
            assert(task2MotionTestRoute.config.secondArrow*s->routeYawRad*180/PI >=
                task2MotionTestSensorConfig.irAcquisitionMinYawDeg-0.01f);
            assert(task2MotionTestRoute.obstacle2IRDeparture
                ? s->stepIndex==TASK2_ROUTE_O2_IR_STRAIGHT_STEP : s->stepIndex>=TASK2_ROUTE_O2_DEPARTURE_STEP);
        }
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
        if (s->stepIndex == TASK2_ROUTE_O2_DEPARTURE_STEP ||
            (task2MotionTestRoute.obstacle2IRDeparture && s->stepIndex==TASK2_ROUTE_O2_IR_STRAIGHT_STEP &&
             task2MotionTestRoute.steps[s->stepIndex].radiusMm==0))
            assert(s->targetSpeedCps <= task2MotionTestRoute.config.obstacle2DepartureSpeedCps+0.5f);
    }
    assert(freshIRReads >= task2MotionTestSensorConfig.absentSamples || !task2MotionTestRoute.obstacle2IRDeparture);
    if(task2MotionTestDetection.edgeMethod==TASK2_EDGE_IR_INITIAL_ABSENCE_BOUND)
        assert(!task2MotionTestDetection.armed[task2MotionTestRoute.config.secondArrow==1 ? TASK2_SENSOR_RIGHT_IR : TASK2_SENSOR_LEFT_IR]);
    assert(freshIRReads==irReads); /* No selected ADC conversion is lost between motion-log entries. */
    if(task2MotionTestRoute.obstacle2IRDeparture && task2MotionTestRoute.steps[5].radiusMm==0)
    {
        float heading=0;
        for(unsigned i=0;i<=4;++i)heading+=task2MotionTestStepResults[i].measuredYawRad;
        near(heading,task2MotionTestRoute.config.secondArrow*PI/2,0.04f);
        near(task2MotionTestStepResults[5].measuredYawRad,0,0.02f);
    }
    assert(accelerated);
    /* Short approaches can finish before exceeding the approach speed. */
    float recognitionTravel=hypotf(task2MotionTestResult.arrow2RecognitionPoseEstimate.xMm-
        task2MotionTestResult.obstacle1ExitPoseEstimate.xMm,task2MotionTestResult.arrow2RecognitionPoseEstimate.yMm-
        task2MotionTestResult.obstacle1ExitPoseEstimate.yMm);
    if (task2MotionTestRoute.steps[TASK2_ROUTE_O2_APPROACH_STEP].distanceMm - recognitionTravel >= 300)
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
    config.irAcquisitionMinYawDeg=0; /* Isolate ADC hysteresis tests from the mounting yaw gate. */
    Task2RouteConfig route=task2RouteConfig;
    route.firstArrow=route.secondArrow=1;
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
    o.pose.yawRad=20*PI/180;
    o.valid=true;o.value=400;
    for(unsigned i=0;i<3;++i)Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);
    assert(!state.armed[2] && !state.detected && state.absent[2]==0);
    o.value=700;
    state=(Task2ObstacleDetection){0};
    for(unsigned i=0;i<5;++i)Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,false);
    assert(!state.armed[2] && !state.irObserved[1]);
    for(unsigned i=0;i<2;++i)Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);
    assert(!state.armed[2]);
    o.value=699;Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);
    assert(state.present[2]==0);
    o.value=700;
    for(unsigned i=0;i<3;++i)Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);
    assert(state.armed[2]);
    const float noisyPresent[]={900,750,850,700,800};
    for(unsigned i=0;i<5;++i){o.value=noisyPresent[i];Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);}
    o.value=401;
    o.pose.yawRad=40*PI/180;
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
    /* Run02's early side return cannot arm IR; the genuine face near 85 deg can. */
    state=(Task2ObstacleDetection){0};config.rightIR=(Task2SensorMount){135,-33.5,-90};
    config.irAcquisitionMinYawDeg=80;
    o=(Task2ObstacleObservation){.valid=true,.value=900,.pose={face-478.43f,-174.20f,37.44f*PI/180}};
    for(unsigned i=0;i<5;++i)Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);
    assert(!state.armed[2] && state.present[2]==0);
    o.pose=(Task2RoutePose){face-375.55f,11.50f,75*PI/180};
    for(unsigned i=0;i<5;++i)Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);
    assert(!state.armed[2] && !state.irObserved[1]);
    o.pose.yawRad=80*PI/180;
    for(unsigned i=0;i<2;++i)Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);
    assert(state.irObserved[1] && !state.armed[2]);
    o.pose=(Task2RoutePose){face-375.55f,11.50f,85*PI/180};
    for(unsigned i=0;i<3;++i)Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);
    assert(state.armed[2] && state.irObserved[1] && state.lastIRAdc[1]==900);
    o.value=300;o.pose=(Task2RoutePose){face-345,80,100*PI/180};
    for(unsigned i=0;i<3;++i)Task2ObstacleDetection_Observe(&state,&config,&route,2,&o,true);
    assert(state.detected && state.source==TASK2_SENSOR_RIGHT_IR);
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
    /* Isolate return planning from a synthetic stop; this pose is supplied,
     * rather than reached by the production IR startup prefix. */
    route=(Task2Route){.config=config,.stepCount=5};
    Task2RoutePose stopped={1050,-300,80*PI/180};
    bool replanned=Task2Route_ReplanObstacle2(&route,&stopped,500,TASK2_ROUTE_O2_DEPARTURE_STEP);
#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
    if (!replanned)
    {
        assert(route.report.planningBudgetExceeded && !route.report.valid);
        assert(route.report.plannerPoseChecks==TASK2_ROUTE_RUNTIME_POSE_CHECK_LIMIT);
        assert(!strcmp(route.report.reason,"PLANNING_BUDGET"));
        puts("PASS: difficult synthetic return rejects at the MCU work limit");
        return;
    }
#endif
    assert(replanned);
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

static void testRecordedQuarterReturn(void)
{
    for (int side=-1; side<=1; side+=2)
    {
        Task2RouteConfig c=task2RouteConfig;
        c.parkToObstacle1Mm=1050.73999f;c.firstArrow=-side;c.secondArrow=side;
        /* Treat the inferred 647 mm obstacle as real for this regression,
         * keeping the logged 750 mm side boundary. */
        c.obstacle2WidthMm=647.003174f;c.obstacle2EndSpaceMm=750-c.obstacle2WidthMm/2;
        Task2Route route={.config=c,.stepCount=6,.obstacle2IRDeparture=true,.obstacle2ReturnPending=true};
        Task2RoutePose stop={1859.04907f,side*217.813293f,side*1.57820058f};
        /* The buffer exception still rejects body overlap and side-wall loss. */
        Task2Route rejected = route;
        Task2RoutePose collision = {c.parkToObstacle1Mm + 150 + c.obstacleFaceGapMm, 0, side*PI/2};
        assert(!Task2Route_ReplanObstacle2(&rejected,&collision,647.003174f,5));
        assert(!strcmp(rejected.report.reason,"NO_IR_QUARTER_TURN"));
        rejected = route;
        Task2RoutePose wall = stop;
        wall.yMm = side*700;
        assert(!Task2Route_ReplanObstacle2(&rejected,&wall,647.003174f,5));
        assert(!strcmp(rejected.report.reason,"NO_IR_QUARTER_TURN"));
        assert(Task2Route_ReplanObstacle2(&route,&stop,647.003174f,5));
        assert(route.obstacle2ParallelReturnPending && route.stepCount==7);
        near(route.steps[6].radiusMm,-side*275,0);
        task2MotionTestRoute=route;resetHardware(0);idealPose=stop;
        RobotTestFixture fixture;
        assert(RobotTestFixture_InitMotionController(&fixture,&task2MotionTestControllerConfig));
        starts=1;
        for(unsigned i=6;i<route.stepCount;++i)
        {
            Task2RouteStep step=route.steps[i];
            assert((step.radiusMm==0 ? MotionController_MoveStraight(&fixture.motionController,step.distanceMm,step.speedCps)
                : MotionController_MoveArc(&fixture.motionController,step.distanceMm,step.radiusMm,step.speedCps))==0);
            unsigned updates=0;
            while(MotionController_IsBusy(&fixture.motionController))
            {
                HAL_Delay(10);assert(MotionController_Update(&fixture.motionController,0.01f)==0);
                assert(++updates<10000);
            }
            if(i==6)
            {
                near(idealPose.yawRad,0,0.04f);
                assert(Task2Route_ReplanParallelReturn(&route,&idealPose,647.003174f,6));
                assert(!route.obstacle2ParallelReturnPending && route.obstacle2DirectRearArc && route.obstacle2ParkReturnPending);
                task2MotionTestRoute=route;
            }
            else if(route.obstacle2ParkReturnPending && i+1==route.stepCount)
            {
                assert(Task2Route_ReplanParkingReturn(&route,&idealPose,647.003174f,i));
                task2MotionTestRoute=route;
            }
        }
        assert(physicalCrossings[1][0] && physicalCrossings[1][1]);
        assert(idealReport.minimumObstacleGapMm>0 && idealReport.minimumSideMarginMm>35 && idealReport.minimumParkWallMarginMm>0);
        float rear=c.bodyCenterAheadOfRearAxleMm-c.bodyLengthMm/2;
        float front=c.bodyCenterAheadOfRearAxleMm+c.bodyLengthMm/2+c.frontProtrusionMm;
        for(unsigned f=0;f<2;++f)for(int sign=-1;sign<=1;sign+=2)
        {
            float ahead=f?front:rear;
            float x=idealPose.xMm+ahead*cosf(idealPose.yawRad)-sign*c.bodyWidthMm/2*sinf(idealPose.yawRad);
            float y=idealPose.yMm+ahead*sinf(idealPose.yawRad)+sign*c.bodyWidthMm/2*cosf(idealPose.yawRad);
            assert(x>-500 && x<0 && fabsf(y)<300);
        }
    }
    puts("PASS: run03 stop and its mirror, R275 measured quarter stop, direct calculated rear arc and parking");
}

static void testStraightIREdgeMethods(void)
{
    Task2ObstacleSensorConfig config = task2ObstacleSensorConfig;
    Task2RouteConfig route = task2RouteConfig;
    route.parkToObstacle1Mm = 1042.85095f;
    route.firstArrow = TASK2_ARROW_LEFT; route.secondArrow = TASK2_ARROW_RIGHT;
    const Task2ObstacleObservation samples[] = {
        {.valid=true,.value=1095,.pose={1781.97882697f,-77.94357809f,-1.572672010f}},
        {.valid=true,.value=1075,.pose={1781.97882697f,-77.94357809f,-1.572672010f}},
        {.valid=true,.value=1069,.pose={1781.97882697f,-77.94357809f,-1.572672010f}},
        {.valid=true,.value=1081,.pose={1781.97079030f,-81.50504763f,-1.573433760f}},
        {.valid=true,.value=968,.pose={1781.90909346f,-95.61891977f,-1.576901560f}},
        {.valid=true,.value=775,.pose={1781.78125245f,-112.23865779f,-1.580075030f}},
        {.valid=true,.value=725,.pose={1781.62457065f,-128.92411756f,-1.580297710f}},
        {.valid=true,.value=631,.pose={1781.46333791f,-145.60953398f,-1.580620530f}},
        {.valid=true,.value=575,.pose={1781.29652747f,-162.22893605f,-1.581045630f}},
        {.valid=true,.value=473,.pose={1781.12165987f,-178.91420744f,-1.581506970f}},
        {.valid=true,.value=381,.pose={1780.92859000f,-195.53332500f,-1.583319310f}},
        {.valid=true,.value=357,.pose={1780.71510667f,-212.21814729f,-1.583862070f}},
        {.valid=true,.value=387,.pose={1780.49991363f,-228.90296262f,-1.583524230f}}
    };
    Task2ObstacleDetection state = {0};
    /* No arc acquisition, even at 90 degrees, until the owner starts the traverse. */
    Task2ObstacleObservation arc = samples[0];
    for (unsigned i=0; i<10; ++i)
        Task2ObstacleDetection_Observe(&state,&config,&route,TASK2_SENSOR_LEFT_IR,&arc,true);
    assert(!state.irObserved[0] && !state.armed[TASK2_SENSOR_LEFT_IR]);
    Task2ObstacleDetection_BeginIRTraverse(&state,&samples[0].pose);
    for (unsigned i=0; i<sizeof(samples)/sizeof(samples[0]); ++i)
        Task2ObstacleDetection_Observe(&state,&config,&route,TASK2_SENSOR_LEFT_IR,&samples[i],true);
    assert(state.detected && state.edgeMethod==TASK2_EDGE_IR_TREND);
    assert(state.observedWidthMm>=490 && state.observedWidthMm<570);
    assert(state.edgePose.yMm > state.firstAbsentPose[TASK2_SENSOR_LEFT_IR].yMm + 90);
    printf("PASS: run08 ADC replay uses decline onset: width %.1f mm (threshold model was 751.4 mm)\n",state.observedWidthMm);
    for (int side=-1; side<=1; side+=2)
    {
        route.firstArrow=-side;route.secondArrow=side;
        Task2ObstacleSensor sensor=side==1 ? TASK2_SENSOR_RIGHT_IR : TASK2_SENSOR_LEFT_IR;
        float face=route.parkToObstacle1Mm+100+route.obstacleFaceGapMm;
        Task2ObstacleObservation o={.valid=true,.value=900,.pose={face-400,side*100,side*PI/2}};
        state=(Task2ObstacleDetection){0};
        Task2ObstacleDetection_BeginIRTraverse(&state,&o.pose);
        const float noise[]={1000,900,1100,950,1050,900,1000,920,1010};
        for(unsigned i=0;i<sizeof(noise)/sizeof(noise[0]);++i)
        {
            o.value=noise[i];o.pose.yMm+=side*10;
            Task2ObstacleDetection_Observe(&state,&config,&route,sensor,&o,true);
        }
        assert(state.armed[sensor] && !state.irTrendCandidateValid && !state.detected);
        const float declining[]={900,780,630,500,430};
        for(unsigned i=0;i<5;++i)
        {
            o.value=declining[i];o.pose.yMm+=side*10;
            Task2ObstacleDetection_Observe(&state,&config,&route,sensor,&o,true);
        }
        assert(state.irTrendCandidateValid && !state.detected);
        const float rebound[]={800,1000,1000};
        for(unsigned i=0;i<3;++i)
        {
            o.value=rebound[i];o.pose.yMm+=side*10;
            Task2ObstacleDetection_Observe(&state,&config,&route,sensor,&o,true);
        }
        assert(!state.irTrendCandidateValid && !state.detected);
        /* Already absent: three fresh conversions suffice, with no preceding presence. */
        state=(Task2ObstacleDetection){0};o.value=300;
        Task2ObstacleDetection_BeginIRTraverse(&state,&o.pose);
        for(unsigned i=0;i<3;++i)Task2ObstacleDetection_Observe(&state,&config,&route,sensor,&o,true);
        assert(state.detected && !state.armed[sensor] && state.edgeMethod==TASK2_EDGE_IR_INITIAL_ABSENCE_BOUND);
    }
    puts("PASS: noise, rebound rejection, no arc IR acquisition and mirrored initial-absence bounds");
}

static void testRun08RearBypass(void)
{
    for (int side=-1;side<=1;side+=2)
    {
        Task2RouteConfig c=task2RouteConfig;
        c.parkToObstacle1Mm=1042.85095f;c.firstArrow=-side;c.secondArrow=side;
        Task2Route route={.config=c,.stepCount=7,.obstacle2IRDeparture=true,.obstacle2ParallelReturnPending=true};
        Task2RoutePose p={2032.1499f,side*527.745667f,side*.00917601585f};
        assert(Task2Route_ReplanParallelReturn(&route,&p,751.408508f,6));
        assert(route.obstacle2ParkReturnPending && route.stepCount==9);
        assert(route.steps[7].radiusMm==0 && route.steps[7].distanceMm<150);
        assert(fabsf(route.steps[8].radiusMm)>530 && fabsf(route.steps[8].radiusMm)<580);
        /* Rear bypass is planned without a parking suffix. */
        for(unsigned i=7;i<route.stepCount;++i)
        {
            const Task2RouteStep *step=&route.steps[i];
            float yaw=p.yawRad;
            if(step->radiusMm==0) {p.xMm+=step->distanceMm*cosf(yaw);p.yMm+=step->distanceMm*sinf(yaw);}
            else {float end=yaw+step->distanceMm/step->radiusMm;p.xMm+=step->radiusMm*(sinf(end)-sinf(yaw));p.yMm+=step->radiusMm*(cosf(yaw)-cosf(end));p.yawRad=end;}
        }
        assert(Task2Route_ReplanParkingReturn(&route,&p,751.408508f,8));
        assert(!route.obstacle2ParkReturnPending && route.stepCount==13);
    }
    puts("PASS: run08 751 mm bound and mirror: short positioning straight, calculated rear arc, separate parking");
}

static void testRun10SustainedLowerIR(void)
{
    const Task2ObstacleObservation recorded[] = {
        {.valid=true,.value=882,.pose={1857.30145586f,118.14909938f,1.575795050f}},
        {.valid=true,.value=869,.pose={1857.30145586f,118.14909938f,1.575795050f}},
        {.valid=true,.value=876,.pose={1857.30145586f,118.14909938f,1.575795050f}},
        {.valid=true,.value=870,.pose={1857.28970928f,120.58934344f,1.575424910f}},
        {.valid=true,.value=879,.pose={1857.21619497f,133.31811882f,1.577718500f}},
        {.valid=true,.value=785,.pose={1857.07262692f,150.06964181f,1.581014630f}},
        {.valid=true,.value=273,.pose={1856.89343000f,166.62295500f,1.582228060f}},
        {.valid=true,.value=267,.pose={1856.69716890f,183.17607856f,1.583076360f}},
        {.valid=true,.value=280,.pose={1856.48929064f,199.92693253f,1.583335040f}}
    };
    Task2ObstacleSensorConfig config=task2ObstacleSensorConfig;
    Task2RouteConfig route=task2RouteConfig;
    route.parkToObstacle1Mm=1041.3075f;
    for(int side=-1;side<=1;side+=2)
    {
        route.firstArrow=-side;route.secondArrow=side;
        Task2ObstacleSensor sensor=side==1 ? TASK2_SENSOR_RIGHT_IR : TASK2_SENSOR_LEFT_IR;
        Task2SensorMount mirrored={135,side*-33.5f,side*-90.0f};
        if(side==1)config.rightIR=mirrored;else config.leftIR=mirrored;
        Task2ObstacleObservation o=recorded[0];o.pose.yMm*=side;o.pose.yawRad*=side;
        Task2ObstacleDetection state={0};
        Task2ObstacleDetection_BeginIRTraverse(&state,&o.pose);
        for(unsigned i=0;i<sizeof(recorded)/sizeof(recorded[0]);++i)
        {
            o=recorded[i];o.pose.yMm*=side;o.pose.yawRad*=side;
            Task2ObstacleDetection_Observe(&state,&config,&route,sensor,&o,true);
            if(i+1<sizeof(recorded)/sizeof(recorded[0]))assert(!state.detected);
        }
        assert(state.detected && state.edgeMethod==TASK2_EDGE_IR_TREND);
        assert(state.irFilteredAdc==273 && state.irDeclineSamples==3);
        assert(state.absent[sensor]==3);
        near(state.observedWidthMm,554.2737f,0.1f);
        assert(hypotf(state.edgePose.xMm-state.firstAbsentPose[sensor].xMm,
            state.edgePose.yMm-state.firstAbsentPose[sensor].yMm)>40);
        /* A substantial drop can settle above the absence threshold too. */
        o=recorded[0];o.pose.yMm*=side;o.pose.yawRad*=side;
        state=(Task2ObstacleDetection){0};
        Task2ObstacleDetection_BeginIRTraverse(&state,&o.pose);
        const float plateau[]={900,900,900,600,600,600,600};
        for(unsigned i=0;i<7;++i)
        {
            o.value=plateau[i];o.pose.yMm+=side*10;
            Task2ObstacleDetection_Observe(&state,&config,&route,sensor,&o,true);
        }
        assert(state.armed[sensor] && state.irTrendCandidateValid && !state.detected);
        Task2RoutePose onset=state.irTrendCandidatePose;
        for(unsigned i=0;i<3;++i)
        {
            o.value=300;o.pose.yMm+=side*10;
            Task2ObstacleDetection_Observe(&state,&config,&route,sensor,&o,true);
        }
        assert(state.detected && state.edgeMethod==TASK2_EDGE_IR_TREND);
        near(state.edgePose.yMm,onset.yMm,0);
    }
    puts("PASS: run10 and mirror use onset width 554.3 mm, equal low medians confirm decline, braking confirmation unchanged");
}

int main(void)
{
    setbuf(stdout, NULL);
    testSensorEvents();
    testStraightIREdgeMethods();
    testRun10SustainedLowerIR();
    testRun08RearBypass();
    const Task2RouteConfig tuned=task2RouteConfig;
    task2RouteConfig.cruiseSpeedCps=5000;
    task2RouteConfig.approachSpeedCps=2000;
    task2RouteConfig.obstacle2DepartureSpeedCps=1000;
    const Task2RouteConfig original=task2RouteConfig;
    const Task2MotionTestConfig originalTest=task2MotionTestConfig;
    task2MotionTestConfig.measureObstacle1AtStartup=false;
    const float positions[]={600,1050}, gaps[]={600,750,1050,1500}, widths[]={300,350,400,500};
    unsigned runs=0, rejected=0, stoppedRejected=0, shortReturns=0, alignedReturns=0, traverseReturns=0, irReturns=0;
    for(unsigned p=0;p<2;++p)for(unsigned g=0;g<4;++g)for(unsigned w=0;w<4;++w)
    for(int left=-1;left<=1;left+=2)for(int right=-1;right<=1;right+=2)
    {
        task2RouteConfig=original;
        task2RouteConfig.parkToObstacle1Mm=positions[p]; task2RouteConfig.obstacleFaceGapMm=gaps[g];
        task2RouteConfig.obstacle2WidthMm=widths[w]; task2RouteConfig.firstArrow=left; task2RouteConfig.secondArrow=right;
        task2RouteConfig.recognitionCameraRangeMm=1000;
        resetHardware(0); Task2MotionTestRun();
        if (!task2MotionTestResult.passed && starts == 0 && robot == NULL &&
            task2MotionTestResult.status == MOTIONCONTROLLER_STATUS_PROFILE_ERROR &&
            !strcmp(task2MotionTestRoute.report.reason, "NO_O2_APPROACH_OR_RETURN"))
        {
            assert(g==3 && (w==0 || (w==1 && left==1 && right==-1)));
            assert(!(p == 1 && g == 2 && w == 3)); /* Current layout is required. */
            ++rejected;
            continue; /* Full 90-degree turn plus a bounded initial absence must fit the approach limits. */
        }
        if (!task2MotionTestResult.passed && g == 3 && starts == 1 &&
            task2MotionTestResult.obstacle2EdgeStopped &&
            !strcmp(task2MotionTestRoute.report.reason, "NO_OBSERVED_EDGE_RETURN"))
        {
            assert(robot->motionController.mode==MOTIONCONTROLLER_IDLE);
            assert(idealReport.minimumObstacleGapMm>0 && idealReport.minimumSideMarginMm>0);
            ++stoppedRejected;
            continue; /* Reference-edge screen cannot guarantee an actual brake-stop return. */
        }
        if(!task2MotionTestResult.passed)fprintf(stderr,"Failed layout %.0f %.0f %.0f %d %d: %s, checks %u, max %u/%u\n",positions[p],gaps[g],widths[w],left,right,task2MotionTestRoute.report.reason,task2MotionTestRoute.report.plannerPoseChecks,maxStartupChecks,maxReturnChecks);
        checkCompleted(); ++runs;
        if(task2MotionTestRoute.obstacle2IRDeparture)++irReturns;
        else if(task2MotionTestRoute.stepCount==8)++shortReturns;
        else if(task2MotionTestRoute.stepCount==11)++alignedReturns;
        else if(task2MotionTestRoute.stepCount==14)++traverseReturns;
    }
    assert(shortReturns+alignedReturns+traverseReturns+irReturns==runs);
    assert(runs==122 && rejected==6 && stoppedRejected==0);
    printf("Return variants: %u direct, %u aligned, %u traverses, %u IR-directed\n",shortReturns,alignedReturns,traverseReturns,irReturns);
    printf("PASS: 128 layouts: %u completed, %u rejected before motors, %u rejected while stopped at edge\n", runs, rejected, stoppedRejected);
    unsigned fastCompleted=0, fastRejected=0;
    for(unsigned g=0;g<2;++g)for(unsigned w=0;w<2;++w)for(int side=-1;side<=1;side+=2)
    {
        task2RouteConfig=original;task2RouteConfig.obstacleFaceGapMm=g ? 1050 : 600;
        task2RouteConfig.obstacle2WidthMm=w ? 500 : 300;
        task2RouteConfig.firstArrow=-side;task2RouteConfig.secondArrow=side;
        task2RouteConfig.cruiseSpeedCps=7000;task2RouteConfig.approachSpeedCps=3000;
        task2RouteConfig.obstacle2DepartureSpeedCps=2000;
        resetHardware(0);Task2MotionTestRun();
        if (!task2MotionTestResult.passed && g == 1 && starts == 0 && robot == NULL &&
            !strcmp(task2MotionTestRoute.report.reason, "NO_O2_APPROACH_OR_RETURN"))
        {
            ++fastRejected;
            continue; /* Reject a high-speed guard that cannot fit the side corridor. */
        }
        if(!task2MotionTestResult.passed)fprintf(stderr,"Fast IR failure %.0f %.0f %d: %s\n",task2RouteConfig.obstacleFaceGapMm,task2RouteConfig.obstacle2WidthMm,side,task2MotionTestRoute.report.reason);
        checkCompleted(); ++fastCompleted;
    }
    assert(fastCompleted==8 && fastRejected==0);
    printf("PASS: fast mirrored layouts: %u completed, %u rejected before motors\n", fastCompleted, fastRejected);
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
        if(s==5)assert(task2MotionTestResult.timedOut && task2MotionTestDetection.armed[original.secondArrow==1 ? TASK2_SENSOR_RIGHT_IR : TASK2_SENSOR_LEFT_IR]);
        else if(s>=3)assert(task2MotionTestResult.status!=0);
    }
    task2RouteConfig=original;resetHardware(7);Task2MotionTestRun();checkCompleted();
    assert(task2MotionTestDetection.edgeMethod==TASK2_EDGE_IR_INITIAL_ABSENCE_BOUND);
    task2RouteConfig=original;resetHardware(8);Task2MotionTestRun();
    assert(!task2MotionTestResult.passed && task2MotionTestDetection.fault);
    assert(robot->motionController.mode==MOTIONCONTROLLER_IDLE);
    task2RouteConfig=original;resetHardware(9);Task2MotionTestRun();checkCompleted();
    puts("PASS: initially absent IR, IR ADC fault, and ultrasonic background cannot terminate opposite-arrow departure");
    task2RouteConfig=tuned;resetHardware(0);Task2MotionTestRun();checkCompleted();
    puts("PASS: current published tuning and arrow configuration");
    for (int side=-1; side<=1; side+=2)
    {
        task2RouteConfig=original;
        task2RouteConfig.parkToObstacle1Mm=1050;
        task2RouteConfig.obstacleFaceGapMm=1050;
        task2RouteConfig.obstacle2WidthMm=500;
        task2RouteConfig.firstArrow=-side;task2RouteConfig.secondArrow=side;
        resetHardware(10);Task2MotionTestRun();
        assert(task2MotionTestResult.obstacle2EdgeStopped && task2MotionTestDetection.detected);
        assert(task2MotionTestRoute.steps[5].sensorTerminated);
        assert(task2MotionTestStepResults[5].measuredDistanceMm > irStraightPreviewMm + 20);
        unsigned absent=0;
        for (unsigned i=0;i<task2MotionTestLogCount;++i)
        {
            const Task2MotionTestSample *sample=&task2MotionTestLog[i];
            if(sample->irFresh && sample->irAdc<=400)++absent;
        }
        assert(absent>=3);
        /* Replay with no disappearance, cancel only after passing the old preview cap. */
        resetHardware(11);Task2MotionTestRun();
        assert(task2MotionTestResult.cancelled && !task2MotionTestResult.obstacle2EdgeStopped);
        assert(robot->motionController.mode==MOTIONCONTROLLER_IDLE);
        assert(fabsf(robot->motionController.travelledDistanceMm) > task2MotionTestRoute.steps[5].distanceMm);
    }
    puts("PASS: mirrored gradual IR disappearance beyond nominal edge, continuous-straight cancellation and no-absence timeout");
    for (int side=-1;side<=1;side+=2)
    {
        task2RouteConfig=original;task2RouteConfig.parkToObstacle1Mm=1050;task2RouteConfig.obstacleFaceGapMm=1050;
        task2RouteConfig.obstacle2WidthMm=500;task2RouteConfig.firstArrow=-side;task2RouteConfig.secondArrow=side;
        resetHardware(12);Task2MotionTestRun();checkCompleted();
        assert(task2MotionTestDetection.edgeMethod==TASK2_EDGE_IR_INITIAL_ABSENCE_BOUND);
        assert(task2MotionTestDetection.observedWidthMm>=300 && task2MotionTestDetection.observedWidthMm<=580);
        resetHardware(13);Task2MotionTestRun();
        assert(!task2MotionTestResult.passed && task2MotionTestDetection.irAcquisitionTimedOut);
        assert(!task2MotionTestDetection.detected && robot->motionController.mode==MOTIONCONTROLLER_IDLE);
    }
    puts("PASS: unknown actual 300 mm O2 with nominal 500 mm planning in both mirrors; ambiguous ADC timeout");
    testRecordedQuarterReturn();
    puts("PASS: measured asymmetric frame, one startup range, frozen configuration, invalid input, cancellation, timeout and IMU fault");
    printf("Planner pose checks: max startup %u, max return %u\n",maxStartupChecks,maxReturnChecks);
    return 0;
}
