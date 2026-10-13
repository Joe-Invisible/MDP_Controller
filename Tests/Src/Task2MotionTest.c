#include "Task2MotionTest.h"
#include "RobotTestFixture.h"
#include "Task2ObstacleSensors.h"
#include "oledutils.h"
#include "userbutton.h"
#include <math.h>

#define TASK2_PI 3.14159265358979323846f

/* Startup pose: rear axle x=-250, chassis tail 213 mm from park back wall.
 * No camera/RPi direction input yet: configured arrows simulate recognition. */
Task2MotionTestConfig task2MotionTestConfig = {
    .measureObstacle1AtStartup = true,
    .controlPeriodMs = 10U,
    .logPeriodMs = 100U,
    .startClearanceMs = 400U,
    .timeoutMs = 60000U,
    .brakeTimeoutMs = 3000U,
};
/* IR positions below are editable assumptions. Enable IR only after checking
 * mounts and thresholds. Ultrasonic +210 mm is the measured position. */
Task2ObstacleSensorConfig task2ObstacleSensorConfig = {
    .ultrasonicEnabled = true, .sideIREnabled = false,
    .ultrasonic = {210.0f, 0.0f, 0.0f},
    .leftIR = {110.0f, 100.0f, 90.0f},
    .rightIR = {110.0f, -100.0f, -90.0f},
    .ultrasonicBeamHalfAngleDeg = 15.0f,
    .ultrasonicRangeToleranceMm = 80.0f,
    .irPresentVoltageV = 0.70f, .irAbsentVoltageV = 0.55f,
    .edgeUncertaintyMm = 20.0f,
    .samplePeriodMs = 60U, .acquisitionTimeoutMs = 1500U,
    .presentSamples = 3U, .absentSamples = 3U,
};
Task2ObstacleSensorConfig task2MotionTestSensorConfig;
Task2ObstacleDetection task2MotionTestDetection;
Task2MotionTestState task2MotionTestState;
Task2Route task2MotionTestRoute;
MotionControllerConfig task2MotionTestControllerConfig;
Task2MotionTestStartupRange task2MotionTestStartupRange;
Task2MotionTestResult task2MotionTestResult;
Task2MotionTestStepResult task2MotionTestStepResults[TASK2_ROUTE_STEP_COUNT];
Task2MotionTestSample task2MotionTestLog[TASK2_MOTION_TEST_LOG_CAPACITY];
uint32_t task2MotionTestLogCount;
uint32_t task2MotionTestStepIndex;

static RobotTestFixture fixture;
static Task2RoutePose estimatedPose;
static float completedTravelMm, completedYawRad, previousStepTravelMm, previousStepYawRad;
static MotionControllerArcConfig testArcConfig;
static MotionControllerArcFeedforwardPoint negativeFeedforwardPoints[9];
/* Preserve the finite-curvature calibration used by the previous stopped run. */
static const MotionControllerArcFeedforwardPoint positiveFeedforwardPoints[] = {
    {0.0000005f, -0.05208333f}, {0.000054f, -5.625f}, {0.000183f, -11.25f}, {0.000436f, -16.875f},
    {1.0f / 1500.0f, -22.5f}, {1.0f / 1000.0f, -27.7f},
    {1.0f / 500.0f, -47.5f}, {1.0f / 300.0f, -75.0f}, {1.0f / 275.0f, -81.7f},
};

static bool ConfigureController(void)
{
    task2MotionTestControllerConfig = motionControllerConfig;
    MotionControllerConfig *c = &task2MotionTestControllerConfig;
    c->straightYawRateKp = c->arcYawRateKp = 270.0f;
    c->straightYawRateKi = c->arcYawRateKi = 150.0f;
    c->straightYawRateKd = c->arcYawRateKd = 0.0f;
    c->straightHeadingKpPerSec = c->arcHeadingKpPerSec = 0.0f;
    c->useMatchedYawRateReferenceFilter = false;
    c->arcYawRateFilterTauSec = 0.10f;
    c->straightAccelerationMmps2 = c->straightDecelerationMmps2 = 3000.0f;
    c->arcAccelerationMmps2 = c->arcDecelerationMmps2 = 2000.0f;
    testArcConfig = arcMotionConfig;
    testArcConfig.positivePoints = positiveFeedforwardPoints;
    testArcConfig.positivePointCount = sizeof(positiveFeedforwardPoints) / sizeof(positiveFeedforwardPoints[0]);
    c->arcConfig = &testArcConfig;
    uint32_t count = arcMotionConfig.negativePointCount;
    if (!count || !arcMotionConfig.negativePoints ||
        count >= sizeof(negativeFeedforwardPoints) / sizeof(negativeFeedforwardPoints[0]))
        return false;
    for (uint32_t i = 0U; i < count; ++i)
        negativeFeedforwardPoints[i] = arcMotionConfig.negativePoints[i];
    const MotionControllerArcFeedforwardPoint *near = &negativeFeedforwardPoints[count - 1U];
    negativeFeedforwardPoints[count] = (MotionControllerArcFeedforwardPoint){
        -0.0000005f, near->rawSteeringCommand * -0.0000005f / near->curvaturePerMm};
    testArcConfig.negativePoints = negativeFeedforwardPoints;
    testArcConfig.negativePointCount = count + 1U;
    return true;
}

static bool MeasureStartingRange(void)
{
    if (!task2MotionTestConfig.measureObstacle1AtStartup)
        return true;
    OLED_Clear();
    OLED_Printf(0, 0, "Measure O1 range");
    OLED_Printf(0, 2, "Park +x, stationary");
    OLED_Printf(0, 4, "SW1: measure once");
    OLED_Refresh_Gram();
    SW1_WaitForPressAndRelease();
    HAL_Delay(task2MotionTestConfig.startClearanceMs);
    task2MotionTestStartupRange.attempted = true;
    float range;
    if (!RobotTestFixture_ReadUltrasonicMm(&range) || !isfinite(range) || range < 20.0f || range > 4000.0f)
        return false;
    task2MotionTestStartupRange.sensorToObstacleMm = range;
    task2MotionTestStartupRange.parkToObstacle1Mm =
        TASK2_ROUTE_START_X_MM + task2MotionTestSensorConfig.ultrasonic.aheadMm +
        range * cosf(task2MotionTestSensorConfig.ultrasonic.yawDeg * TASK2_PI / 180.0f);
    task2MotionTestStartupRange.valid = true;
    task2RouteConfig.parkToObstacle1Mm = task2MotionTestStartupRange.parkToObstacle1Mm;
    return true;
}

static MotionControllerStatus LaunchStep(void)
{
    const Task2RouteStep *step = &task2MotionTestRoute.steps[task2MotionTestStepIndex];
    previousStepTravelMm = previousStepYawRad = 0.0f;
    MotionControllerStatus status = step->radiusMm == 0.0f
        ? MotionController_MoveStraight(&fixture.motionController, step->distanceMm, step->speedCps)
        : MotionController_MoveArc(&fixture.motionController, step->distanceMm, step->radiusMm, step->speedCps);
    if (status == MOTIONCONTROLLER_STATUS_OK && task2MotionTestStepIndex == 0U)
        status = MotionController_RequestCruiseSpeed(&fixture.motionController,
            task2MotionTestRoute.config.approachSpeedCps);
    return status;
}

static void UpdatePose(void)
{
    const MotionController *c = &fixture.motionController;
    float travel = fabsf(c->travelledDistanceMm);
    float yaw = c->yawDeg * TASK2_PI / 180.0f;
    float ds = travel - previousStepTravelMm;
    float middleHeading = completedYawRad + 0.5f * (previousStepYawRad + yaw);
    estimatedPose.xMm += ds * cosf(middleHeading);
    estimatedPose.yMm += ds * sinf(middleHeading);
    estimatedPose.yawRad = completedYawRad + yaw;
    previousStepTravelMm = travel;
    previousStepYawRad = yaw;
    task2MotionTestResult.measuredTravelMm = completedTravelMm + travel;
    task2MotionTestResult.measuredYawRad = completedYawRad + yaw;
    task2MotionTestResult.finalPoseEstimate = estimatedPose;
}

static void LogSample(uint32_t elapsedMs)
{
    if (task2MotionTestLogCount >= TASK2_MOTION_TEST_LOG_CAPACITY)
    {
        task2MotionTestResult.logTruncated = true;
        return;
    }
    const MotionController *c = &fixture.motionController;
    task2MotionTestLog[task2MotionTestLogCount++] = (Task2MotionTestSample){
        .timeMs = elapsedMs, .stepIndex = task2MotionTestStepIndex, .mode = c->mode,
        .stepTravelMm = c->travelledDistanceMm,
        .routeTravelMm = task2MotionTestResult.measuredTravelMm,
        .routeYawRad = task2MotionTestResult.measuredYawRad,
        .targetSpeedCps = c->targetSpeedCps, .measuredCentreSpeedMmps = c->measuredCentreSpeedMmps,
        .targetYawRateRadPerSec = c->arcTargetYawRateRadPerSec, .yawRateDps = c->yawRateDps,
        .headingErrorRad = c->arcHeadingErrorRad,
        .steeringFeedforward = c->arcSteeringFeedforwardCommand,
        .steeringCorrection = c->arcSteeringCorrectionCommand,
        .steeringCommand = fixture.steeringController.command,
        .leftTargetCps = fixture.leftWheelController.targetSpeedCps,
        .rightTargetCps = fixture.rightWheelController.targetSpeedCps,
        .leftMeasuredCps = fixture.leftWheelController.measuredSpeedCps,
        .rightMeasuredCps = fixture.rightWheelController.measuredSpeedCps,
        .accelerationLimitMmps2 = c->motionProfile.accelerationMmps2,
        .decelerationLimitMmps2 = c->motionProfile.decelerationMmps2,
        .ultrasonicArmed = task2MotionTestDetection.armed[TASK2_SENSOR_ULTRASONIC],
        .ultrasonicAbsentSamples = task2MotionTestDetection.absent[TASK2_SENSOR_ULTRASONIC],
        .lastUltrasonicRangeMm = task2MotionTestDetection.lastUltrasonicRangeMm,
        .obstacle2EdgeDetected = task2MotionTestDetection.detected,
    };
}

__attribute__((noinline)) void Task2MotionTestFinished(void)
{
    OLED_Clear();
    OLED_Printf(0, 0, "Task2 %s", task2MotionTestResult.passed ? "DONE" : "INCOMPLETE");
    OLED_Printf(0, 1, "Time: %lu ms", (unsigned long)task2MotionTestResult.elapsedMs);
    OLED_Printf(0, 2, "Steps: %lu/%lu", (unsigned long)task2MotionTestResult.completedSteps,
        (unsigned long)task2MotionTestRoute.stepCount);
    OLED_Printf(0, 3, "Status: %u", (unsigned)task2MotionTestResult.status);
    OLED_Printf(0, 4, "%s", task2MotionTestResult.timedOut ? "TIMEOUT" :
        task2MotionTestResult.cancelled ? "CANCELLED" : task2MotionTestResult.logTruncated ? "LOG TRUNCATED" : "Stopped primitives");
    if (!task2MotionTestRoute.report.valid && task2MotionTestRoute.report.reason != NULL)
        OLED_Printf(0, 5, "%s", task2MotionTestRoute.report.reason);
    OLED_Refresh_Gram();
}

void Task2MotionTestRun(void)
{
    task2MotionTestState = TASK2_TEST_IDLE;
    task2MotionTestSensorConfig = task2ObstacleSensorConfig;
    task2MotionTestDetection = (Task2ObstacleDetection){0};
    task2MotionTestRoute = (Task2Route){0};
    task2MotionTestControllerConfig = (MotionControllerConfig){0};
    task2MotionTestStartupRange = (Task2MotionTestStartupRange){0};
    task2MotionTestResult = (Task2MotionTestResult){.status = MOTIONCONTROLLER_STATUS_NOT_INITIALIZED};
    task2MotionTestStepIndex = task2MotionTestLogCount = 0U;
    for (unsigned i = 0U; i < TASK2_ROUTE_STEP_COUNT; ++i)
        task2MotionTestStepResults[i] = (Task2MotionTestStepResult){0};
    completedTravelMm = completedYawRad = previousStepTravelMm = previousStepYawRad = 0.0f;
    estimatedPose = (Task2RoutePose){TASK2_ROUTE_START_X_MM, 0.0f, 0.0f};
    if (!Task2ObstacleDetection_ValidConfig(&task2MotionTestSensorConfig) ||
        task2MotionTestConfig.controlPeriodMs == 0U || task2MotionTestConfig.logPeriodMs == 0U ||
        task2MotionTestConfig.timeoutMs == 0U || task2MotionTestConfig.brakeTimeoutMs == 0U)
    {
        task2MotionTestResult.status = MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
        goto finished;
    }
    if (!MeasureStartingRange())
    {
        task2MotionTestResult.status = MOTIONCONTROLLER_STATUS_INVALID_STATE;
        goto finished;
    }
    if (!Task2Route_Create(&task2RouteConfig, &task2MotionTestRoute))
    {
        task2MotionTestResult.status = MOTIONCONTROLLER_STATUS_PROFILE_ERROR;
        goto finished;
    }
    if (!Task2ObstacleSensors_Init(&task2MotionTestSensorConfig))
    {
        task2MotionTestResult.status = MOTIONCONTROLLER_STATUS_INVALID_STATE;
        goto finished;
    }
    if (!ConfigureController())
    {
        task2MotionTestResult.status = MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION;
        goto finished;
    }
    if (!RobotTestFixture_InitMotionController(&fixture, &task2MotionTestControllerConfig))
        goto finished;
    fixture.leftWheelController.pid.kp = fixture.rightWheelController.pid.kp = 0.03f;
    fixture.leftWheelController.pid.ki = fixture.rightWheelController.pid.ki = 0.0f;
    fixture.leftWheelController.pid.kd = fixture.rightWheelController.pid.kd = 0.0f;
    task2MotionTestState = TASK2_TEST_READY;
    OLED_Clear();
    OLED_Printf(0, 0, "Task2 stopped ready");
    OLED_Printf(0, 1, "Path: %.2fm", (double)(task2MotionTestRoute.totalTravelMm / 1000.0f));
    OLED_Printf(0, 2, "O1:%c O2:%c W:%g", task2MotionTestRoute.config.firstArrow == TASK2_ARROW_LEFT ? 'L' : 'R',
        task2MotionTestRoute.config.secondArrow == TASK2_ARROW_LEFT ? 'L' : 'R', (double)task2MotionTestRoute.config.obstacle2WidthMm);
    OLED_Printf(0, 3, "O1 face: %.0fmm", (double)task2MotionTestRoute.config.parkToObstacle1Mm);
    OLED_Printf(0, 4, "SW1: start");
    OLED_Printf(0, 5, "SW1 running: cancel");
    OLED_Refresh_Gram();
    SW1_WaitForPressAndRelease();
    HAL_Delay(task2MotionTestConfig.startClearanceMs);
    uint32_t startMs = HAL_GetTick(), previousUpdateMs = startMs, lastLogMs = startMs, brakeStartMs = startMs;
    bool acquiring = false, edgeBraking = false;
    uint32_t acquireStartMs = startMs;
    task2MotionTestResult.status = LaunchStep();
    task2MotionTestState = TASK2_TEST_RUNNING;
    while (task2MotionTestResult.status == MOTIONCONTROLLER_STATUS_OK &&
        (task2MotionTestState == TASK2_TEST_RUNNING || task2MotionTestState == TASK2_TEST_BRAKING))
    {
        uint32_t nowMs = HAL_GetTick();
        if (nowMs - previousUpdateMs < task2MotionTestConfig.controlPeriodMs)
        {
            HAL_Delay(1U);
            continue;
        }
        if (task2MotionTestState == TASK2_TEST_RUNNING &&
            (nowMs - startMs >= task2MotionTestConfig.timeoutMs || SW1_ReadState() == SW1_Enabled))
        {
            task2MotionTestResult.timedOut = nowMs - startMs >= task2MotionTestConfig.timeoutMs;
            task2MotionTestResult.cancelled = !task2MotionTestResult.timedOut;
            if (acquiring)
            {
                task2MotionTestResult.status = MotionController_Stop(&fixture.motionController);
                break;
            }
            task2MotionTestResult.status = MotionController_Brake(&fixture.motionController);
            task2MotionTestState = TASK2_TEST_BRAKING;
            brakeStartMs = nowMs;
        }
        if (task2MotionTestState == TASK2_TEST_BRAKING && nowMs - brakeStartMs >= task2MotionTestConfig.brakeTimeoutMs)
        {
            task2MotionTestResult.status = MOTIONCONTROLLER_STATUS_INVALID_STATE;
            break;
        }
        if (task2MotionTestResult.status != MOTIONCONTROLLER_STATUS_OK)
            break;
        if (!acquiring)
        {
            task2MotionTestResult.status = MotionController_Update(&fixture.motionController,
                (nowMs - previousUpdateMs) / 1000.0f);
            UpdatePose();
        }
        if (task2MotionTestState == TASK2_TEST_RUNNING &&
            task2MotionTestStepIndex >= TASK2_ROUTE_O2_APPROACH_STEP &&
            task2MotionTestStepIndex <= TASK2_ROUTE_O2_DEPARTURE_STEP && !edgeBraking)
        {
            Task2ObstacleObservation observations[TASK2_SENSOR_COUNT];
            uint32_t fresh = Task2ObstacleSensors_Poll(&task2MotionTestSensorConfig, nowMs,
                &estimatedPose, observations);
            for (unsigned sensor = 0U; sensor < TASK2_SENSOR_COUNT; ++sensor)
                if (fresh & (1U << sensor))
                    Task2ObstacleDetection_Observe(&task2MotionTestDetection,
                        &task2MotionTestSensorConfig, &task2MotionTestRoute.config,
                        (Task2ObstacleSensor)sensor, &observations[sensor],
                        task2MotionTestStepIndex == TASK2_ROUTE_O2_DEPARTURE_STEP && !acquiring);
            if (task2MotionTestDetection.fault)
                task2MotionTestResult.status = MOTIONCONTROLLER_STATUS_INVALID_STATE;
            if (task2MotionTestStepIndex == TASK2_ROUTE_O2_APPROACH_STEP &&
                Task2Route_ObstacleCameraVisible(&task2MotionTestRoute.config, &estimatedPose, 1U))
                task2MotionTestResult.arrow2Recognised = true;
            if (task2MotionTestDetection.detected && !edgeBraking)
            {
                task2MotionTestResult.status = MotionController_Brake(&fixture.motionController);
                edgeBraking = true;
                brakeStartMs = nowMs;
            }
        }
        if (edgeBraking && nowMs - brakeStartMs >= task2MotionTestConfig.brakeTimeoutMs)
            task2MotionTestResult.status = MOTIONCONTROLLER_STATUS_INVALID_STATE;
        if (acquiring && task2MotionTestState == TASK2_TEST_RUNNING)
        {
            if (task2MotionTestDetection.armed[TASK2_SENSOR_ULTRASONIC])
            {
                acquiring = false;
                task2MotionTestResult.status = LaunchStep();
            }
            else if (nowMs - acquireStartMs >= task2MotionTestSensorConfig.acquisitionTimeoutMs)
                task2MotionTestResult.status = MOTIONCONTROLLER_STATUS_INVALID_STATE;
            previousUpdateMs = nowMs;
            continue;
        }
        if (task2MotionTestResult.status != MOTIONCONTROLLER_STATUS_OK)
            break;
        if (task2MotionTestStepIndex == 0U && task2MotionTestState == TASK2_TEST_RUNNING &&
            !task2MotionTestResult.arrow1Recognised &&
            Task2Route_Obstacle1CameraVisible(&task2MotionTestRoute.config, &estimatedPose))
        {
            task2MotionTestResult.status = MotionController_RequestCruiseSpeed(&fixture.motionController,
                task2MotionTestRoute.config.cruiseSpeedCps);
            if (task2MotionTestResult.status != MOTIONCONTROLLER_STATUS_OK)
                break;
            task2MotionTestResult.arrow1Recognised = true;
            task2MotionTestResult.recognitionElapsedMs = nowMs - startMs;
            task2MotionTestResult.recognitionPoseEstimate = estimatedPose;
        }
        if (!MotionController_IsBusy(&fixture.motionController))
        {
            if (task2MotionTestState == TASK2_TEST_BRAKING)
                break;
            const MotionController *c = &fixture.motionController;
            task2MotionTestStepResults[task2MotionTestStepIndex] = (Task2MotionTestStepResult){
                fabsf(c->travelledDistanceMm), c->yawDeg * TASK2_PI / 180.0f,
                c->terminalYawPredictedReached, c->terminalDistanceLimitReached};
            completedTravelMm = task2MotionTestResult.measuredTravelMm;
            completedYawRad = task2MotionTestResult.measuredYawRad;
            ++task2MotionTestResult.completedSteps;
            if (task2MotionTestStepIndex == 2U)
            {
                task2MotionTestResult.obstacle1ExitPoseEstimateValid = true;
                task2MotionTestResult.obstacle1ExitPoseEstimate = estimatedPose;
            }
            if (task2MotionTestStepIndex == TASK2_ROUTE_O2_DEPARTURE_STEP)
            {
                if (!edgeBraking || !task2MotionTestDetection.detected)
                {
                    task2MotionTestResult.status = MOTIONCONTROLLER_STATUS_INVALID_STATE;
                    break; /* Guard endpoint reached without an acquired, confirmed edge. */
                }
                task2MotionTestResult.obstacle2EdgeStopped = true;
                task2MotionTestResult.obstacle2StoppedPose = estimatedPose;
                task2MotionTestRoute.steps[4].distanceMm = fabsf(c->travelledDistanceMm);
                if (!Task2Route_ReplanObstacle2(&task2MotionTestRoute, &estimatedPose,
                    task2MotionTestDetection.observedWidthMm))
                {
                    task2MotionTestResult.status = MOTIONCONTROLLER_STATUS_PROFILE_ERROR;
                    break;
                }
                edgeBraking = false;
                Task2ObstacleSensors_Stop();
            }
            if (task2MotionTestResult.completedSteps == task2MotionTestRoute.stepCount)
                task2MotionTestState = TASK2_TEST_COMPLETE;
            else
            {
                ++task2MotionTestStepIndex;
                if (task2MotionTestStepIndex == TASK2_ROUTE_O2_DEPARTURE_STEP &&
                    !task2MotionTestDetection.armed[TASK2_SENSOR_ULTRASONIC])
                {
                    acquiring = true;
                    acquireStartMs = nowMs;
                }
                else
                    task2MotionTestResult.status = LaunchStep();
            }
        }
        previousUpdateMs = nowMs;
        if (nowMs - lastLogMs >= task2MotionTestConfig.logPeriodMs)
        {
            LogSample(nowMs - startMs);
            lastLogMs = nowMs;
        }
    }
    task2MotionTestResult.elapsedMs = HAL_GetTick() - startMs;
    if (task2MotionTestState != TASK2_TEST_COMPLETE)
        MotionController_Stop(&fixture.motionController);
    task2MotionTestResult.passed = task2MotionTestState == TASK2_TEST_COMPLETE &&
        task2MotionTestResult.status == MOTIONCONTROLLER_STATUS_OK &&
        task2MotionTestResult.arrow1Recognised && task2MotionTestResult.arrow2Recognised &&
        task2MotionTestResult.obstacle2EdgeStopped;
    if (task2MotionTestLogCount == TASK2_MOTION_TEST_LOG_CAPACITY)
    {
        --task2MotionTestLogCount;
        task2MotionTestResult.logTruncated = true;
    }
    LogSample(task2MotionTestResult.elapsedMs);
finished:
    Task2ObstacleSensors_Stop();
    if (!task2MotionTestResult.passed)
        task2MotionTestState = task2MotionTestResult.cancelled ? TASK2_TEST_CANCELLED : TASK2_TEST_FAILED;
    Task2MotionTestFinished();
}
