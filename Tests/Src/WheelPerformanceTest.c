#include "WheelPerformanceTest.h"

#include "RobotTestFixture.h"
#include "oledutils.h"
#include "userbutton.h"
#include <math.h>
#include <string.h>

#define CONTROL_PERIOD_MS 10U
#define LOG_PERIOD_MS 20U
#define MAX_CONTROL_GAP_MS 50U
#define STATIONARY_SAMPLES 5U

/* Each direction gets the complete table, twice. Change repetitions if needed.
 * The 100% points provide the speed ceiling; 80/90% show approach to saturation.
 */
static const float speedPwmTable[] = {80.0f, 90.0f, 100.0f};
static const float accelerationTable[] = {
    500.0f, 1000.0f, 1500.0f, 3000.0f, 6000.0f, 10000.0f
};

volatile WheelPerformanceConfig wheelPerformanceConfig = {
    .experiment = WHEEL_PERFORMANCE_TOP_SPEED,
    .repetitions = 2U,
    .waitBetweenTrials = true,
    .batteryVoltage = 0.0f,
    .steeringRawCommand = 0.0f,
    .maxDriveDistanceMm = 2000.0f,
    .speedDriveMs = 1500U,
    .steadyWindowMs = 500U,
    .accelerationTargetCps = 5000.0f,
    .accelerationHoldMs = 200U,
    .brakeTimeoutMs = 1000U
};

WheelPerformanceConfig wheelPerformanceRunConfig;
/* Startup does not initialize .ccmram: explicitly cleared in TestRun().
 * This avoids consuming the main SRAM used by FreeRTOS and existing tests.
 */
WheelPerformanceSample wheelPerformanceTrace[WHEEL_PERFORMANCE_TRACE_CAPACITY]
    __attribute__((section(".ccmram"), aligned(4)));
WheelPerformanceTrial wheelPerformanceTrials[WHEEL_PERFORMANCE_TRIAL_CAPACITY];
volatile uint32_t wheelPerformanceTraceCount;
volatile uint32_t wheelPerformanceTrialCount;
volatile uint32_t wheelPerformanceStatus;
float wheelPerformanceMmPerCount;

_Static_assert(sizeof(WheelPerformanceSample) == 20U, "Unexpected trace layout");
_Static_assert(sizeof(wheelPerformanceTrace) <= 64U * 1024U, "CCM trace too large");

static RobotTestFixture fixture;
static int16_t previousLeftEncoder;
static int16_t previousRightEncoder;
static float leftSpeedCps;
static float rightSpeedCps;
static float leftDistanceMm;
static float rightDistanceMm;
static float leftPathMm;
static float rightPathMm;

static void BrakeBoth(void)
{
    DCMotor_Brake(&fixture.leftRearWheel);
    DCMotor_Brake(&fixture.rightRearWheel);
}

static void ShowMessage(const char *message)
{
    OLED_Clear();
    OLED_Printf(0, 0, "%s", message);
    OLED_Refresh_Gram();
}

static uint32_t TableCount(void)
{
    return wheelPerformanceRunConfig.experiment == WHEEL_PERFORMANCE_TOP_SPEED
        ? sizeof(speedPwmTable) / sizeof(speedPwmTable[0])
        : sizeof(accelerationTable) / sizeof(accelerationTable[0]);
}

static uint32_t DriveLogPeriodMs(void)
{
    /* Fast ramps need every control sample; speed plateaus and braking use 50 Hz. */
    return wheelPerformanceRunConfig.experiment == WHEEL_PERFORMANCE_ACCELERATION
        ? CONTROL_PERIOD_MS : LOG_PERIOD_MS;
}

static uint32_t DriveDurationMs(float accelerationMmps2)
{
    if (wheelPerformanceRunConfig.experiment == WHEEL_PERFORMANCE_TOP_SPEED)
        return wheelPerformanceRunConfig.speedDriveMs;

    return (uint32_t)ceilf(1000.0f *
        wheelPerformanceRunConfig.accelerationTargetCps *
        wheelPerformanceMmPerCount / accelerationMmps2) +
        wheelPerformanceRunConfig.accelerationHoldMs;
}

static bool ValidateConfiguration(void)
{
    const WheelPerformanceConfig *c = &wheelPerformanceRunConfig;
    if ((c->experiment != WHEEL_PERFORMANCE_TOP_SPEED &&
         c->experiment != WHEEL_PERFORMANCE_ACCELERATION) ||
        c->repetitions == 0U || c->repetitions > 4U ||
        !isfinite(c->batteryVoltage) || c->batteryVoltage < 0.0f ||
        !isfinite(c->steeringRawCommand) || fabsf(c->steeringRawCommand) > 100.0f ||
        !isfinite(c->maxDriveDistanceMm) || c->maxDriveDistanceMm <= 0.0f ||
        !isfinite(c->accelerationTargetCps) || c->accelerationTargetCps <= 0.0f ||
        c->accelerationTargetCps > 30000.0f ||
        c->speedDriveMs < c->steadyWindowMs || c->steadyWindowMs < 200U ||
        c->speedDriveMs > 10000U || c->brakeTimeoutMs < 100U ||
        c->brakeTimeoutMs > 5000U || c->accelerationHoldMs > 2000U ||
        !isfinite(wheelPerformanceMmPerCount) || wheelPerformanceMmPerCount <= 0.0f)
        return false;

    uint32_t trialCount = 2U * c->repetitions * TableCount();
    if (trialCount > WHEEL_PERFORMANCE_TRIAL_CAPACITY)
        return false;

    /* Reserve worst-case traces INCLUDING stopping and phase-boundary samples.
     * Refuse a sweep that cannot fit instead of silently truncating it.
     */
    uint32_t samplesPerDirection = 0U;
    for (uint32_t i = 0; i < TableCount(); ++i) {
        float acceleration = c->experiment == WHEEL_PERFORMANCE_ACCELERATION
            ? accelerationTable[i] : 0.0f;
        uint32_t driveLogPeriod = DriveLogPeriodMs();
        samplesPerDirection +=
            (DriveDurationMs(acceleration) + driveLogPeriod - 1U) / driveLogPeriod +
            (c->brakeTimeoutMs + LOG_PERIOD_MS - 1U) / LOG_PERIOD_MS + 4U;
    }
    return 2U * c->repetitions * samplesPerDirection <=
        WHEEL_PERFORMANCE_TRACE_CAPACITY;
}

static void ObserveWheels(uint32_t dtMs)
{
    int16_t left = DCMotor_GetEncoderCount(&fixture.leftRearWheel);
    int16_t right = DCMotor_GetEncoderCount(&fixture.rightRearWheel);
    int16_t dl = (int16_t)((uint16_t)left - (uint16_t)previousLeftEncoder);
    int16_t dr = (int16_t)((uint16_t)right - (uint16_t)previousRightEncoder);
    previousLeftEncoder = left;
    previousRightEncoder = right;
    leftSpeedCps = 1000.0f * dl / dtMs;
    rightSpeedCps = 1000.0f * dr / dtMs;
    leftDistanceMm += dl * wheelPerformanceMmPerCount;
    rightDistanceMm += dr * wheelPerformanceMmPerCount;
    /* Distance guard counts absolute wheel travel, not signed centre distance. */
    leftPathMm += fabsf(dl * wheelPerformanceMmPerCount);
    rightPathMm += fabsf(dr * wheelPerformanceMmPerCount);
}

static bool RecordSample(uint32_t timeMs, uint32_t dtMs, uint32_t trial,
                         uint32_t phase, float targetCps,
                         float leftPwm, float rightPwm)
{
    if (wheelPerformanceTraceCount >= WHEEL_PERFORMANCE_TRACE_CAPACITY) {
        wheelPerformanceStatus = WHEEL_TEST_LOG_FULL;
        return false;
    }
    if (!isfinite(leftSpeedCps) || !isfinite(rightSpeedCps) ||
        fabsf(leftSpeedCps) > 32767.0f || fabsf(rightSpeedCps) > 32767.0f) {
        wheelPerformanceStatus = WHEEL_TEST_TELEMETRY_RANGE;
        return false;
    }
    WheelPerformanceSample *s = &wheelPerformanceTrace[wheelPerformanceTraceCount];
    *s = (WheelPerformanceSample) {
        .timeMs = timeMs, .dtMs = (uint16_t)dtMs,
        .trial = (uint8_t)trial, .phase = (uint8_t)phase,
        .targetCps = (int16_t)lroundf(targetCps),
        .leftCps = (int16_t)lroundf(leftSpeedCps),
        .rightCps = (int16_t)lroundf(rightSpeedCps),
        .leftPwmX100 = (int16_t)lroundf(leftPwm * 100.0f),
        .rightPwmX100 = (int16_t)lroundf(rightPwm * 100.0f),
        .leftMode = (uint8_t)fixture.leftRearWheel.state.mode,
        .rightMode = (uint8_t)fixture.rightRearWheel.state.mode
    };
    ++wheelPerformanceTraceCount;
    return true;
}

static bool InitWheelControllers(void)
{
    const WheelSpeedBrakeConfig brakeConfig = {
        .map = &rearWheelBrakeMap,
        .engageOverspeedCps = 200.0f, .releaseOverspeedCps = 100.0f,
        .fullDemandOverspeedCps = 800.0f
    };
    return RobotTestFixture_InitWheelControllers(&fixture) &&
        WheelSpeedController_ConfigureBrake(&fixture.leftWheelController, &brakeConfig) &&
        WheelSpeedController_ConfigureBrake(&fixture.rightWheelController, &brakeConfig);
}

static float EstimateDriveDistanceMm(int direction, float value)
{
    if (wheelPerformanceRunConfig.experiment == WHEEL_PERFORMANCE_TOP_SPEED) {
        /* Existing steady-speed model, applied for the full drive duration.
         * Startup acceleration is deliberately not subtracted. This is a
         * model estimate, not a bound on an as-yet unmeasured maximum speed.
         * Use the faster wheel so runway planning covers either wheel's travel.
         */
        float leftSlope = direction > 0 ? leftCalibration.forwardSlope
                                        : leftCalibration.reverseSlope;
        float leftOffset = direction > 0 ? leftCalibration.forwardOffset
                                         : leftCalibration.reverseOffset;
        float rightSlope = direction > 0 ? rightCalibration.forwardSlope
                                         : rightCalibration.reverseSlope;
        float rightOffset = direction > 0 ? rightCalibration.forwardOffset
                                          : rightCalibration.reverseOffset;
        float leftCps = fmaxf(0.0f, leftSlope * (value - leftOffset));
        float rightCps = fmaxf(0.0f, rightSlope * (value - rightOffset));
        return fmaxf(leftCps, rightCps) * wheelPerformanceMmPerCount *
            (wheelPerformanceRunConfig.speedDriveMs / 1000.0f);
    }

    float speedMmps = wheelPerformanceRunConfig.accelerationTargetCps *
        wheelPerformanceMmPerCount;
    float rampTimeSec = speedMmps / value;
    float holdTimeSec = DriveDurationMs(value) / 1000.0f - rampTimeSec;
    return 0.5f * speedMmps * rampTimeSec +
        speedMmps * fmaxf(0.0f, holdTimeSec);
}

static void PrepareTrial(uint32_t trialIndex, int direction, float value,
                         float estimatedDriveMm)
{
    OLED_Clear();
    OLED_Printf(0, 0, "Trial %lu/%lu %s", (unsigned long)(trialIndex + 1U),
        (unsigned long)(2U * wheelPerformanceRunConfig.repetitions * TableCount()),
        direction > 0 ? "FWD" : "REV");
    bool rawPwm = wheelPerformanceRunConfig.experiment == WHEEL_PERFORMANCE_TOP_SPEED;
    OLED_Printf(0, 1, rawPwm ? "PWM %.0f%%" : "Accel %.0f mm/s2", value);
    OLED_Printf(0, 2, "Drive ~%.0f mm", estimatedDriveMm);
    OLED_Printf(0, 3, "%s", rawPwm ? "Model estimate" : "Ramp + hold estimate");
    OLED_Printf(0, 4, "Guard %.0f mm", wheelPerformanceRunConfig.maxDriveDistanceMm);
    OLED_Printf(0, 5, "Plus stopping space");
    if (estimatedDriveMm >= wheelPerformanceRunConfig.maxDriveDistanceMm)
        OLED_Printf(0, 6, "Guard may cut run");
    OLED_Printf(0, 7, "%s", wheelPerformanceRunConfig.waitBetweenTrials
        ? "Reposition; SW1" : "Auto start");
    OLED_Refresh_Gram();
    if (wheelPerformanceRunConfig.waitBetweenTrials)
        SW1_WaitForPressAndRelease();
    else
        HAL_Delay(500U);
    /* No controller updates across a repositioning gap; counts reset afterward. */
}

static bool StopAndRecord(uint32_t trialIndex, uint32_t trialStart,
                          uint32_t *lastControl, uint32_t *lastLog)
{
    BrakeBoth();
    uint32_t stopStart = HAL_GetTick();
    uint32_t stable = 0U;
    while ((HAL_GetTick() - stopStart) < wheelPerformanceRunConfig.brakeTimeoutMs) {
        uint32_t now = HAL_GetTick();
        uint32_t dtMs = now - *lastControl;
        if (dtMs < CONTROL_PERIOD_MS)
            continue;
        *lastControl = now;
        ObserveWheels(dtMs);
        if (dtMs > MAX_CONTROL_GAP_MS && wheelPerformanceStatus == WHEEL_TEST_RUNNING)
            wheelPerformanceStatus = WHEEL_TEST_TIMING_FAULT;
        bool stationary = fabsf(leftSpeedCps) < WHEEL_STATIONARY_THRESHOLD_CPS &&
            fabsf(rightSpeedCps) < WHEEL_STATIONARY_THRESHOLD_CPS;
        stable = stationary ? stable + 1U : 0U;
        if ((now - *lastLog) >= LOG_PERIOD_MS || stable >= STATIONARY_SAMPLES) {
            *lastLog = now;
            RecordSample(now - trialStart, dtMs, trialIndex, WHEEL_TEST_BRAKE,
                         0.0f, 100.0f, 100.0f);
        }
        if (stable >= STATIONARY_SAMPLES) {
            DCMotor_Neutral(&fixture.leftRearWheel);
            DCMotor_Neutral(&fixture.rightRearWheel);
            return true;
        }
    }
    /* Keep brakes engaged on timeout; do not start another trial. */
    if (wheelPerformanceStatus == WHEEL_TEST_RUNNING)
        wheelPerformanceStatus = WHEEL_TEST_STOP_TIMEOUT;
    return false;
}

static bool RunTrial(int direction, uint32_t repetition, uint32_t tableIndex)
{
    uint32_t index = wheelPerformanceTrialCount;
    bool rawPwm = wheelPerformanceRunConfig.experiment == WHEEL_PERFORMANCE_TOP_SPEED;
    float value = rawPwm ? speedPwmTable[tableIndex] : accelerationTable[tableIndex];
    float estimatedDriveMm = EstimateDriveDistanceMm(direction, value);
    PrepareTrial(index, direction, value, estimatedDriveMm);

    Servo_SetSteering(&fixture.steeringServo, wheelPerformanceRunConfig.steeringRawCommand);
    HAL_Delay(300U);
    if (!rawPwm && !InitWheelControllers()) {
        wheelPerformanceStatus = WHEEL_TEST_INIT_FAILED;
        BrakeBoth();
        return false;
    }

    previousLeftEncoder = DCMotor_GetEncoderCount(&fixture.leftRearWheel);
    previousRightEncoder = DCMotor_GetEncoderCount(&fixture.rightRearWheel);
    leftSpeedCps = rightSpeedCps = 0.0f;
    leftDistanceMm = rightDistanceMm = leftPathMm = rightPathMm = 0.0f;
    WheelPerformanceTrial *trial = &wheelPerformanceTrials[index];
    *trial = (WheelPerformanceTrial) {
        .direction = direction > 0 ? 1U : 2U, .repetition = repetition + 1U,
        .pwmPercent = rawPwm ? direction * value : 0.0f,
        .accelerationMmps2 = rawPwm ? 0.0f : value,
        .targetCps = rawPwm ? 0.0f : direction * wheelPerformanceRunConfig.accelerationTargetCps,
        .estimatedDriveDistanceMm = estimatedDriveMm,
        .firstSample = wheelPerformanceTraceCount, .status = WHEEL_TEST_RUNNING
    };
    ++wheelPerformanceTrialCount;

    uint32_t start = HAL_GetTick();
    uint32_t lastControl = start;
    uint32_t lastLog = start;
    uint32_t driveDuration = DriveDurationMs(rawPwm ? 0.0f : value);
    if (rawPwm) {
        DCMotor_SetPWM(&fixture.leftRearWheel, direction * value);
        DCMotor_SetPWM(&fixture.rightRearWheel, direction * value);
    }

    /* Trace records the command applied DURING the preceding encoder interval.
     * New ramp commands are computed after observation/logging, avoiding a
     * one-control-cycle shift between target and measured response.
     */
    uint32_t intervalPhase = rawPwm ? WHEEL_TEST_DRIVE : WHEEL_TEST_RAMP;
    RecordSample(0U, 0U, index, intervalPhase, 0.0f,
                 rawPwm ? direction * value : 0.0f, rawPwm ? direction * value : 0.0f);
    while (wheelPerformanceStatus == WHEEL_TEST_RUNNING) {
        uint32_t now = HAL_GetTick();
        uint32_t dtMs = now - lastControl;
        if (dtMs < CONTROL_PERIOD_MS)
            continue;
        lastControl = now;
        ObserveWheels(dtMs);
        float target = rawPwm ? 0.0f : fixture.leftWheelController.targetSpeedCps;
        float lpwm = rawPwm ? direction * value : fixture.leftWheelController.outputPWM;
        float rpwm = rawPwm ? direction * value : fixture.rightWheelController.outputPWM;
        if (fixture.leftRearWheel.state.mode == DCMOTOR_MODE_BRAKE)
            lpwm = fixture.leftRearWheel.state.activeDutyCycle;
        if (fixture.rightRearWheel.state.mode == DCMOTOR_MODE_BRAKE)
            rpwm = fixture.rightRearWheel.state.activeDutyCycle;
        if (now - lastLog >= DriveLogPeriodMs()) {
            lastLog = now;
            if (!RecordSample(now - start, dtMs, index, intervalPhase, target, lpwm, rpwm))
                break;
        }
        if (dtMs > MAX_CONTROL_GAP_MS) {
            wheelPerformanceStatus = WHEEL_TEST_TIMING_FAULT;
            break;
        }
        if (SW1_ReadState() == SW1_Enabled) {
            wheelPerformanceStatus = WHEEL_TEST_BUTTON_ABORT;
            break;
        }
        if (fmaxf(leftPathMm, rightPathMm) >= wheelPerformanceRunConfig.maxDriveDistanceMm) {
            wheelPerformanceStatus = WHEEL_TEST_DISTANCE_LIMIT;
            break;
        }
        uint32_t elapsed = now - start;
        if (elapsed >= driveDuration)
            break;
        if (!rawPwm) {
            float magnitude = fminf(wheelPerformanceRunConfig.accelerationTargetCps,
                value * (elapsed / 1000.0f) / wheelPerformanceMmPerCount);
            target = direction * magnitude;
            intervalPhase = magnitude >= wheelPerformanceRunConfig.accelerationTargetCps
                ? WHEEL_TEST_HOLD : WHEEL_TEST_RAMP;
            WheelSpeedController_SetTarget(&fixture.leftWheelController, target);
            WheelSpeedController_SetTarget(&fixture.rightWheelController, target);
            WheelSpeedController_Update(&fixture.leftWheelController, dtMs / 1000.0f);
            WheelSpeedController_Update(&fixture.rightWheelController, dtMs / 1000.0f);
        }
    }

    trial->driveMs = HAL_GetTick() - start;
    trial->leftDriveDistanceMm = leftDistanceMm;
    trial->rightDriveDistanceMm = rightDistanceMm;
    bool stopped = StopAndRecord(index, start, &lastControl, &lastLog);
    trial->leftFinalDistanceMm = leftDistanceMm;
    trial->rightFinalDistanceMm = rightDistanceMm;
    trial->sampleCount = wheelPerformanceTraceCount - trial->firstSample;
    trial->status = wheelPerformanceStatus == WHEEL_TEST_RUNNING
        ? WHEEL_TEST_COMPLETE : wheelPerformanceStatus;
    return stopped && wheelPerformanceStatus == WHEEL_TEST_RUNNING;
}

__attribute__((noinline)) void WheelPerformanceTest_Complete(void)
{
    __asm volatile ("" ::: "memory");
}

void WheelPerformanceTestRun(void)
{
    memset(wheelPerformanceTrace, 0, sizeof(wheelPerformanceTrace));
    memset(wheelPerformanceTrials, 0, sizeof(wheelPerformanceTrials));
    memset(&fixture, 0, sizeof(fixture));
    wheelPerformanceTraceCount = wheelPerformanceTrialCount = 0U;
    wheelPerformanceStatus = WHEEL_TEST_NOT_STARTED;
    ShowMessage("Set config; SW1");
    SW1_WaitForPressAndRelease();
    wheelPerformanceRunConfig = wheelPerformanceConfig;
    wheelPerformanceMmPerCount = 3.14159265358979323846f *
        kinematics.rearWheelDiameterMm / kinematics.rearEncoderCountsPerRev;
    if (!ValidateConfiguration()) {
        wheelPerformanceStatus = WHEEL_TEST_INVALID_CONFIG;
        ShowMessage("Invalid config");
        WheelPerformanceTest_Complete();
        return;
    }
    if (!RobotTestFixture_InitRearWheels(&fixture) ||
        !RobotTestFixture_InitFrontWheels(&fixture)) {
        wheelPerformanceStatus = WHEEL_TEST_INIT_FAILED;
        /* Only touch a peripheral whose fixture initialization has reached it. */
        if (fixture.leftRearWheel.config.pwmHtim) DCMotor_Brake(&fixture.leftRearWheel);
        if (fixture.rightRearWheel.config.pwmHtim) DCMotor_Brake(&fixture.rightRearWheel);
        ShowMessage("Init failed");
        WheelPerformanceTest_Complete();
        return;
    }
    wheelPerformanceStatus = WHEEL_TEST_RUNNING;
    bool keepRunning = true;
    for (uint32_t repetition = 0; repetition < wheelPerformanceRunConfig.repetitions && keepRunning;
         ++repetition) {
        for (int direction = 1; direction >= -1 && keepRunning; direction -= 2) {
            for (uint32_t i = 0; i < TableCount() && keepRunning; ++i)
                keepRunning = RunTrial(direction, repetition, i);
        }
    }
    if (wheelPerformanceStatus == WHEEL_TEST_RUNNING)
        wheelPerformanceStatus = WHEEL_TEST_COMPLETE;
    OLED_Clear();
    OLED_Printf(0, 0, "Sweep status %lu", (unsigned long)wheelPerformanceStatus);
    OLED_Printf(0, 1, "%lu trials", (unsigned long)wheelPerformanceTrialCount);
    OLED_Printf(0, 2, "%lu samples", (unsigned long)wheelPerformanceTraceCount);
    OLED_Printf(0, 3, "Export once now");
    OLED_Refresh_Gram();
    WheelPerformanceTest_Complete();
}
