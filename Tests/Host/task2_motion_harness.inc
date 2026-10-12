#include "Task2MotionTest.h"
#include "RobotTestFixture.h"
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
        .minimumSideMarginMm=INFINITY,.minimumParkWallMarginMm=INFINITY};
}

static void checkCompleted(void)
{
    assert(starts == 1 && task2MotionTestState == TASK2_TEST_COMPLETE);
    assert(task2MotionTestResult.passed && task2MotionTestResult.completedSteps == 8);
    assert(task2MotionTestResult.arrow1Recognised && task2MotionTestResult.obstacle1ExitPoseEstimateValid);
    assert(Task2Route_Obstacle1CameraVisible(&task2MotionTestRoute.config,
        &task2MotionTestResult.recognitionPoseEstimate));
    assert(idealReport.minimumObstacleGapMm > 0 && idealReport.minimumSideMarginMm > 0 &&
        idealReport.minimumParkWallMarginMm > 0);
    assert(fabsf(task2MotionTestResult.obstacle1ExitPoseEstimate.yMm) < 20);
    assert(fabsf(task2MotionTestResult.obstacle1ExitPoseEstimate.yawRad) < 0.04f);
    assert(task2MotionTestLogCount > 1 && !task2MotionTestResult.logTruncated);
    assert(task2MotionTestLog[task2MotionTestLogCount-1].mode == MOTIONCONTROLLER_IDLE);
    for (unsigned i=0; i<8; ++i)
        if (task2MotionTestRoute.steps[i].radiusMm != 0)
            assert(task2MotionTestStepResults[i].yawPredictedReached);
    bool accelerated = false;
    for (unsigned i=0; i<task2MotionTestLogCount; ++i)
    {
        const Task2MotionTestSample *s=&task2MotionTestLog[i];
        if (s->timeMs < task2MotionTestResult.recognitionElapsedMs)
            assert(s->targetSpeedCps <= task2MotionTestRoute.config.approachSpeedCps+0.5f);
        if (s->stepIndex == 0 && s->timeMs > task2MotionTestResult.recognitionElapsedMs &&
            s->targetSpeedCps > task2MotionTestRoute.config.approachSpeedCps+100)
            accelerated=true;
    }
    assert(accelerated);
    const Task2RouteConfig *c=&task2MotionTestRoute.config;
    float centre=c->bodyCenterAheadOfRearAxleMm+0.5f*c->frontProtrusionMm;
    float length=c->bodyLengthMm+c->frontProtrusionMm;
    float cx=idealPose.xMm+centre*cosf(idealPose.yawRad), cy=idealPose.yMm+centre*sinf(idealPose.yawRad);
    float ex=0.5f*(length*fabsf(cosf(idealPose.yawRad))+c->bodyWidthMm*fabsf(sinf(idealPose.yawRad)));
    float ey=0.5f*(length*fabsf(sinf(idealPose.yawRad))+c->bodyWidthMm*fabsf(cosf(idealPose.yawRad)));
    assert(cx-ex > -500 && cx+ex < 0 && fabsf(cy)+ey < 300);
}

int main(void)
{
    const Task2RouteConfig original=task2RouteConfig;
    const Task2MotionTestConfig originalTest=task2MotionTestConfig;
    task2MotionTestConfig.measureObstacle1AtStartup=false;
    const float positions[]={600,1050}, gaps[]={1050,1100}, widths[]={300,350,400,500};
    unsigned runs=0;
    for(unsigned p=0;p<2;++p)for(unsigned g=0;g<2;++g)for(unsigned w=0;w<4;++w)
    for(int left=-1;left<=1;left+=2)for(int right=-1;right<=1;right+=2)
    {
        task2RouteConfig=original;
        task2RouteConfig.parkToObstacle1Mm=positions[p]; task2RouteConfig.obstacleFaceGapMm=gaps[g];
        task2RouteConfig.obstacle2WidthMm=widths[w]; task2RouteConfig.firstArrow=left; task2RouteConfig.secondArrow=right;
        task2RouteConfig.recognitionCameraRangeMm= p ? 500 : 1000;
        resetHardware(0); Task2MotionTestRun();
        if(!task2MotionTestResult.passed)fprintf(stderr,"Failed %u: %u %s\n",runs,task2MotionTestResult.status,task2MotionTestRoute.report.reason);
        checkCompleted(); ++runs;
    }
    puts("PASS: 64 stopped routes, both arrows, distances/widths, 500/1000mm recognition, acceleration, yaw exits and footprints");
    task2RouteConfig=original;
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
    for(unsigned s=1;s<=3;++s)
    {
        task2RouteConfig=original; resetHardware(s); Task2MotionTestRun();
        assert(!task2MotionTestResult.passed && starts==1);
        assert(robot->motionController.mode==MOTIONCONTROLLER_IDLE);
        if(s==1)assert(task2MotionTestResult.cancelled);
        if(s==2)assert(task2MotionTestResult.timedOut);
        if(s==3)assert(task2MotionTestResult.status!=0);
    }
    puts("PASS: measured asymmetric frame, one startup range, frozen configuration, invalid input, cancellation, timeout and IMU fault");
    return 0;
}
