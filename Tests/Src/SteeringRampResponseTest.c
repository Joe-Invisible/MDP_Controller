#include "SteeringRampResponseTest.h"
#include "RobotTestFixture.h"
#include "oledutils.h"
#include "userbutton.h"
#include <math.h>
#include <string.h>

#define CONTROL_PERIOD_MS 10U
#define RUN_TIMEOUT_MS 6000U
#define BRAKE_TIMEOUT_MS 3000U
#define CENTRE_TIMEOUT_MS 3000U
#define INITIAL_SETTLING_SEC 0.5f
#define SOFTWARE_SLEW_LIMIT 480.0f
#define WHEEL_KP 0.03f
#define PROFILE_ACCELERATION_MMPS2 2000.0f
/* Conservative circumscribed body radius used by the existing fusion test,
 * plus clearance. This is a planned footprint, not a bound on tracking error. */
#define FOOTPRINT_PADDING_MM (181.0f + 100.0f)

/* Change only this table to adjust the sweep. Rate is nominal at speedCps.
 * Each row measures both entry and exit under the same wheel-speed setting. */
const SteeringRampTrial steeringRampTrials[STEERING_RAMP_TRIAL_COUNT] = {
    {4500.0f,  50.0f, 525.0f, 350.0f, 300.0f, 350.0f, 150.0f},
    {4500.0f, 100.0f, 525.0f, 350.0f, 300.0f, 350.0f, 150.0f},
    {4500.0f, 200.0f, 525.0f, 350.0f, 300.0f, 350.0f, 150.0f},
};

static RobotTestFixture fixture;
static MotionControllerArcConfig rampArcConfig;
static MotionControllerArcFeedforwardPoint positivePoints[STEERING_RAMP_KNOT_CAPACITY];
SteeringRampProfile steeringRampProfiles[STEERING_RAMP_TRIAL_COUNT];
MotionControllerConfig steeringRampMotionConfig;
SteeringRampResponseSample steeringRampLog[STEERING_RAMP_LOG_CAPACITY];
SteeringRampResponseResult steeringRampResults[STEERING_RAMP_TRIAL_COUNT];
uint32_t steeringRampLogCount;
uint32_t steeringRampResultCount;
uint32_t steeringRampLogPeriodMs;
bool steeringRampFinished;
bool steeringRampBatchComplete;
_Static_assert(sizeof(SteeringRampResponseSample) == 64U,
               "Recheck the target RAM budget when adding sample fields");

static bool ConfigureExperiment(void)
{
    /* The pushed branch still lacks the three measured loose-turn points.
     * Keep this snapshot local: do not silently change other tests' tuning. */
    static const MotionControllerArcFeedforwardPoint loosePoints[] = {
        {0.000054f, -5.625f},
        {0.000183f, -11.25f},
        {0.000436f, -16.875f},
    };
    rampArcConfig = *motionControllerConfig.arcConfig;
    uint32_t count = sizeof(loosePoints) / sizeof(loosePoints[0]);
    const float lastLooseCurvature = loosePoints[count - 1U].curvaturePerMm;
    memcpy(positivePoints, loosePoints, sizeof(loosePoints));
    for (uint32_t i = 0U; i < rampArcConfig.positivePointCount; ++i)
    {
        const MotionControllerArcFeedforwardPoint point = rampArcConfig.positivePoints[i];
        if (point.curvaturePerMm <= lastLooseCurvature)
            continue;
        if (count >= STEERING_RAMP_KNOT_CAPACITY)
            return false;
        positivePoints[count++] = point;
    }
    rampArcConfig.positivePoints = positivePoints;
    rampArcConfig.positivePointCount = count;
    steeringRampMotionConfig = motionControllerConfig;
    steeringRampMotionConfig.arcConfig = &rampArcConfig;
    steeringRampMotionConfig.useLegacyStraightSteering = false;
    steeringRampMotionConfig.straightYawRateKp = 0.0f;
    steeringRampMotionConfig.straightYawRateKi = 0.0f;
    steeringRampMotionConfig.straightYawRateKd = 0.0f;
    steeringRampMotionConfig.straightHeadingKpPerSec = 0.0f;
    steeringRampMotionConfig.arcYawRateKp = 0.0f;
    steeringRampMotionConfig.arcYawRateKi = 0.0f;
    steeringRampMotionConfig.arcYawRateKd = 0.0f;
    steeringRampMotionConfig.arcHeadingKpPerSec = 0.0f;
    steeringRampMotionConfig.arcYawRateFilterTauSec = 0.1f;
    steeringRampMotionConfig.useMatchedYawRateReferenceFilter = true;
    steeringRampMotionConfig.straightAccelerationMmps2 = PROFILE_ACCELERATION_MMPS2;
    steeringRampMotionConfig.arcAccelerationMmps2 = PROFILE_ACCELERATION_MMPS2;
    steeringRampMotionConfig.straightDecelerationMmps2 = PROFILE_ACCELERATION_MMPS2;
    steeringRampMotionConfig.arcDecelerationMmps2 = PROFILE_ACCELERATION_MMPS2;
    return true;
}

static void EstimateFootprint(const SteeringRampProfile *profile,
                              SteeringRampResponseResult *result)
{
    float x = 0.0f, y = 0.0f;
    /* Midpoint integration at <=1 mm resolution is adequate for a display. */
    uint32_t steps = (uint32_t)ceilf(profile->totalDistanceMm);
    float ds = profile->totalDistanceMm / (float)steps;
    for (uint32_t i = 0U; i < steps; ++i)
    {
        MotionPathSample sample;
        SteeringRampProfile_Evaluate(profile, ((float)i + 0.5f) * ds, &sample);
        x += ds * cosf(sample.desiredYawRad);
        y += ds * sinf(sample.desiredYawRad);
        result->minXmm = fminf(result->minXmm, x);
        result->maxXmm = fmaxf(result->maxXmm, x);
        result->minYmm = fminf(result->minYmm, y);
        result->maxYmm = fmaxf(result->maxYmm, y);
    }
}

static bool PrepareSteering(void)
{
    SteeringController_StartCentre(&fixture.steeringController);
    uint32_t start = HAL_GetTick();
    while (SteeringController_IsCentring(&fixture.steeringController))
    {
        HAL_Delay(CONTROL_PERIOD_MS);
        SteeringController_UpdateCentre(&fixture.steeringController,
                                         CONTROL_PERIOD_MS * 0.001f);
        if (HAL_GetTick() - start >= CENTRE_TIMEOUT_MS)
            return false;
    }
    SteeringController_SetRawCommand(&fixture.steeringController,
        steeringRampMotionConfig.straightSteeringFeedforwardCommand);
    HAL_Delay((uint32_t)(INITIAL_SETTLING_SEC * 1000.0f));
    return true;
}

static void ShowPreparation(uint32_t index)
{
    const SteeringRampProfile *profile = &steeringRampProfiles[index];
    const SteeringRampResponseResult *result = &steeringRampResults[index];
    const MotionControllerConfig *config = fixture.motionController.config;
    const SteeringRampKnot *end = &profile->knots[profile->knotCount - 1U];
    OLED_Clear();
    OLED_Printf(0, 0, "Ramp %lu/%u %.0f u/s", (unsigned long)(index + 1U),
                STEERING_RAMP_TRIAL_COUNT, profile->trial.nominalRawRatePerSec);
    OLED_Printf(0, 1, "%.0fCPS %.1f -> %.1f", profile->trial.speedCps,
                profile->knots[0].rawCommand, end->rawCommand);
    OLED_Printf(0, 2, "Travel %.0f mm", profile->totalDistanceMm);
    OLED_Printf(0, 3, "Area %.0fx%.0f mm",
                result->maxXmm - result->minXmm + 2.0f * FOOTPRINT_PADDING_MM,
                result->maxYmm - result->minYmm + 2.0f * FOOTPRINT_PADDING_MM);
    OLED_Printf(0, 4, "S P%.0f I%.0f D%.0f H%.1f", config->straightYawRateKp,
                config->straightYawRateKi, config->straightYawRateKd,
                config->straightHeadingKpPerSec);
    OLED_Printf(0, 5, "A P%.0f I%.0f D%.0f H%.1f", config->arcYawRateKp,
                config->arcYawRateKi, config->arcYawRateKd, config->arcHeadingKpPerSec);
    OLED_Printf(0, 6, "Wheel P%.2f; 10ms", fixture.leftWheelController.pid.kp);
    OLED_Printf(0, 7, "Reset pose; SW1 start");
    OLED_Refresh_Gram();
}

static void LogSample(uint32_t index, uint32_t elapsedMs, float commandRate,
                      bool terminal)
{
    SteeringRampResponseResult *result = &steeringRampResults[index];
    uint32_t slot;
    if (terminal && result->sampleCount == STEERING_RAMP_SAMPLES_PER_TRIAL)
        slot = steeringRampLogCount - 1U;
    else
    {
        slot = steeringRampLogCount++;
        ++result->sampleCount;
    }
    const MotionController *controller = &fixture.motionController;
    const SteeringRampProfile *profile = &steeringRampProfiles[index];
    SteeringRampPhase phase = SteeringRampProfile_Phase(profile, controller->travelledDistanceMm);
    if (controller->mode == MOTIONCONTROLLER_PROFILE_PREPARING)
        phase = STEERING_RAMP_PREPARING;
    else if (controller->mode == MOTIONCONTROLLER_BRAKING)
        phase = STEERING_RAMP_STOPPING;
    if (terminal)
        phase = STEERING_RAMP_FINISHED;
    steeringRampLog[slot] = (SteeringRampResponseSample){
        .timeMs = elapsedMs, .trialIndex = (uint16_t)index,
        .mode = (uint8_t)controller->mode, .phase = (uint8_t)phase,
        .progressMm = controller->travelledDistanceMm,
        .rawReference = SteeringRampProfile_RawCommand(profile, controller->travelledDistanceMm),
        .rawCommand = fixture.steeringController.command,
        .rawCommandRatePerSec = commandRate,
        .curvaturePerMm = controller->targetCurvaturePerMm,
        .measuredCentreSpeedMmps = controller->measuredCentreSpeedMmps,
        .yawRateDps = controller->yawRateDps,
        .filteredYawRateDps = controller->filteredYawRateDps,
        .yawDeg = controller->yawDeg,
        .leftTargetCps = fixture.leftWheelController.targetSpeedCps,
        .rightTargetCps = fixture.rightWheelController.targetSpeedCps,
        .leftMeasuredCps = fixture.leftWheelController.measuredSpeedCps,
        .rightMeasuredCps = fixture.rightWheelController.measuredSpeedCps,
        .wheelSyncCorrectionCps = controller->wheelSyncCorrectionCps,
    };
}

static void RunTrial(uint32_t index)
{
    const SteeringRampProfile *profile = &steeringRampProfiles[index];
    SteeringRampResponseResult *result = &steeringRampResults[index];
    result->firstSample = steeringRampLogCount;
    MotionPathProfile path = {
        .signedDistanceMm = profile->totalDistanceMm,
        .maxSpeedCps = profile->trial.speedCps,
        .steeringSettlingTimeSec = INITIAL_SETTLING_SEC,
        .evaluate = SteeringRampProfile_Evaluate, .context = profile,
        .steeringCommandRatePerSec = SOFTWARE_SLEW_LIMIT,
        /* Distance termination: yaw must not change the duration of the test. */
        .terminalYawPriority = false,
    };
    result->status = MotionController_FollowProfile(&fixture.motionController, &path);
    if (result->status != MOTIONCONTROLLER_STATUS_OK)
    {
        result->outcome = STEERING_RAMP_CONTROLLER_ERROR;
        return;
    }
    uint32_t start = HAL_GetTick(), previous = start, abortTime = 0U;
    bool aborting = false;
    float previousCommand = fixture.steeringController.command;
    float commandRate = 0.0f;
    LogSample(index, 0U, commandRate, false);
    while (MotionController_IsBusy(&fixture.motionController))
    {
        uint32_t now = HAL_GetTick();
        uint32_t interval = now - previous;
        if (interval < CONTROL_PERIOD_MS)
            continue;
        previous = now;
        result->maxUpdateIntervalMs = interval > result->maxUpdateIntervalMs
                                         ? interval : result->maxUpdateIntervalMs;
        SteeringRampOutcome abortReason = STEERING_RAMP_NOT_RUN;
        if (SW1_ReadState() == SW1_Enabled)
            abortReason = STEERING_RAMP_CANCELLED;
        else if (now - start >= RUN_TIMEOUT_MS)
            abortReason = STEERING_RAMP_TIMED_OUT;
        else if (result->sampleCount >= STEERING_RAMP_SAMPLES_PER_TRIAL - 1U)
            abortReason = STEERING_RAMP_LOG_FULL;
        if (!aborting && abortReason != STEERING_RAMP_NOT_RUN)
        {
            result->outcome = abortReason;
            MotionController_Brake(&fixture.motionController);
            abortTime = now;
            aborting = true;
        }
        if (aborting && now - abortTime >= BRAKE_TIMEOUT_MS)
        {
            result->outcome = STEERING_RAMP_BRAKE_TIMED_OUT;
            MotionController_Stop(&fixture.motionController);
            break;
        }
        result->status = MotionController_Update(&fixture.motionController, interval * 0.001f);
        commandRate = (fixture.steeringController.command - previousCommand) /
                      (interval * 0.001f);
        previousCommand = fixture.steeringController.command;
        if (result->status != MOTIONCONTROLLER_STATUS_OK ||
            fabsf(fixture.motionController.arcSteeringCorrectionCommand) > 0.0001f)
        {
            result->outcome = STEERING_RAMP_CONTROLLER_ERROR;
            MotionController_Stop(&fixture.motionController);
            break;
        }
        /* Reserve one terminal sample even if braking outlasts the log budget. */
        if (MotionController_IsBusy(&fixture.motionController) &&
            result->sampleCount < STEERING_RAMP_SAMPLES_PER_TRIAL - 1U)
            LogSample(index, now - start, commandRate, false);
    }
    result->elapsedMs = HAL_GetTick() - start;
    result->measuredDistanceMm = fixture.motionController.travelledDistanceMm;
    result->measuredYawDeg = fixture.motionController.yawDeg;
    if (result->outcome == STEERING_RAMP_NOT_RUN)
        result->outcome = STEERING_RAMP_COMPLETED;
    LogSample(index, result->elapsedMs, commandRate, true);
}

__attribute__((noinline)) void SteeringRampResponseTestFinished(void)
{
    /* Debugger hook: all batch state has been finalized before this call. */
    __asm volatile ("" ::: "memory");
}

void SteeringRampResponseTestRun(void)
{
    steeringRampLogCount = 0U;
    steeringRampResultCount = 0U;
    steeringRampLogPeriodMs = CONTROL_PERIOD_MS;
    steeringRampFinished = false;
    steeringRampBatchComplete = false;
    memset(steeringRampResults, 0, sizeof(steeringRampResults));
    memset(steeringRampProfiles, 0, sizeof(steeringRampProfiles));
    if (!ConfigureExperiment() ||
        !RobotTestFixture_InitMotionController(&fixture, &steeringRampMotionConfig))
    {
        steeringRampResults[0].outcome = STEERING_RAMP_PREPARATION_ERROR;
        steeringRampResultCount = 1U;
        goto finished;
    }
    fixture.leftWheelController.pid.kp = fixture.rightWheelController.pid.kp = WHEEL_KP;
    fixture.leftWheelController.pid.ki = fixture.rightWheelController.pid.ki = 0.0f;
    fixture.leftWheelController.pid.kd = fixture.rightWheelController.pid.kd = 0.0f;
    /* Validate the entire sweep before asking the operator to start any run. */
    for (uint32_t i = 0U; i < STEERING_RAMP_TRIAL_COUNT; ++i)
    {
        if (!SteeringRampProfile_Build(&steeringRampProfiles[i], &fixture.motionController,
                                      &steeringRampTrials[i]))
        {
            steeringRampResults[i].outcome = STEERING_RAMP_PREPARATION_ERROR;
            steeringRampResultCount = i + 1U;
            goto finished;
        }
        EstimateFootprint(&steeringRampProfiles[i], &steeringRampResults[i]);
    }
    for (uint32_t i = 0U; i < STEERING_RAMP_TRIAL_COUNT; ++i)
    {
        steeringRampResultCount = i + 1U;
        MotionControllerStatus status = MotionController_Init(&fixture.motionController,
            &fixture.leftWheelController, &fixture.rightWheelController,
            &fixture.steeringController, &fixture.imu, &steeringRampMotionConfig);
        steeringRampResults[i].status = status;
        if (status != MOTIONCONTROLLER_STATUS_OK || !PrepareSteering())
        {
            steeringRampResults[i].outcome = STEERING_RAMP_PREPARATION_ERROR;
            break;
        }
        ShowPreparation(i);
        SW1_WaitForPressAndRelease();
        RunTrial(i);
        if (steeringRampResults[i].outcome != STEERING_RAMP_COMPLETED)
            break;
    }
    steeringRampBatchComplete =
        steeringRampResultCount == STEERING_RAMP_TRIAL_COUNT &&
        steeringRampResults[STEERING_RAMP_TRIAL_COUNT - 1U].outcome == STEERING_RAMP_COMPLETED;
finished:
    steeringRampFinished = true;
    OLED_Clear();
    OLED_Printf(0, 0, "Ramp batch %s", steeringRampBatchComplete ? "DONE" : "INVALID");
    for (uint32_t i = 0U; i < steeringRampResultCount; ++i)
        OLED_Printf(0, (uint8_t)(i + 1U), "Run%lu code%u %.2fs", (unsigned long)(i + 1U),
                    (unsigned)steeringRampResults[i].outcome,
                    steeringRampResults[i].elapsedMs * 0.001f);
    OLED_Printf(0, 5, "Export whole batch");
    OLED_Refresh_Gram();
    SteeringRampResponseTestFinished();
}
