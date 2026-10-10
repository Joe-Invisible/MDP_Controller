/* Host simulation exercises the real harness and WheelSpeedController.
 * It tests sequencing, logs and fault handling, not physical robot performance.
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "../../../Tests/Src/WheelPerformanceTest.c"

static uint32_t tick;
static uint32_t abortAt;
static bool refuseInit;
static bool preventStop;
static bool injectTimingGap;
static uint32_t buttons;
static uint32_t rawFullTrials;
static uint32_t preparedScreens;
static char oledRows[8][32];

static void Advance(void)
{
    ++tick;
    DCMotor *motors[] = {&fixture.leftRearWheel, &fixture.rightRearWheel};
    for (unsigned i = 0; i < 2; ++i) {
        DCMotor *m = motors[i];
        float magnitude = fmaxf(0.0f, fabsf(m->command) - 53.0f);
        float desired = copysignf(magnitude * (i == 0 ? 200.0f : 190.0f), m->command);
        float tau = m->state.mode == DCMOTOR_MODE_BRAKE ? 0.025f : 0.08f;
        if (m->state.mode == DCMOTOR_MODE_BRAKE)
            desired = preventStop ? 1500.0f : 0.0f;
        m->speed += (desired - m->speed) * (0.001f / tau);
        m->position += m->speed * 0.001;
    }
}

uint32_t HAL_GetTick(void)
{
    if (injectTimingGap && fixture.leftRearWheel.state.mode == DCMOTOR_MODE_DRIVE && tick % 200U == 0U) {
        for (unsigned i = 0; i < 100; ++i) Advance();
        injectTimingGap = false;
    }
    Advance();
    return tick;
}
void HAL_Delay(uint32_t ms) { for (uint32_t i = 0; i < ms; ++i) Advance(); }
void DCMotor_SetPWM(DCMotor *m, float pwm)
{
    m->command = pwm;
    m->state.activeDutyCycle = fabsf(pwm);
    m->state.mode = pwm ? DCMOTOR_MODE_DRIVE : DCMOTOR_MODE_NEUTRAL;
    if (fabsf(pwm) == 100.0f && m->wheel == 0) ++rawFullTrials;
}
void DCMotor_SetBrakePWM(DCMotor *m, float pwm)
{
    m->command = 0.0f;
    m->state.mode = DCMOTOR_MODE_BRAKE;
    m->state.activeDutyCycle = pwm;
}
void DCMotor_Brake(DCMotor *m) { DCMotor_SetBrakePWM(m, 100); }
void DCMotor_Neutral(DCMotor *m) { DCMotor_SetPWM(m, 0); }
int16_t DCMotor_GetEncoderCount(DCMotor *m)
{
    return (int16_t)(uint16_t)(int64_t)llround(m->position);
}
bool RobotTestFixture_InitRearWheels(RobotTestFixture *f)
{
    f->leftRearWheel.config.pwmHtim = &tick;
    f->rightRearWheel.config.pwmHtim = &tick;
    f->leftRearWheel.wheel = 0;
    f->rightRearWheel.wheel = 1;
    /* Start close to encoder wrap to exercise signed/modulo deltas. */
    f->leftRearWheel.position = f->rightRearWheel.position = 32700;
    return !refuseInit;
}
bool RobotTestFixture_InitFrontWheels(RobotTestFixture *f) { (void)f; return true; }
bool RobotTestFixture_InitWheelControllers(RobotTestFixture *f)
{
    return WheelSpeedController_Init(&f->leftWheelController, &f->leftRearWheel,
        WHEELSPEEDCONTROLLER_KP, WHEELSPEEDCONTROLLER_KI, -30, 30, &leftCalibration) &&
        WheelSpeedController_Init(&f->rightWheelController, &f->rightRearWheel,
        WHEELSPEEDCONTROLLER_KP, WHEELSPEEDCONTROLLER_KI, -30, 30, &rightCalibration);
}
void Servo_SetSteering(Servo *s, float value) { s->raw = value; }
SW1_State_t SW1_ReadState(void)
{
    return abortAt && tick >= abortAt ? SW1_Enabled : SW1_Idle;
}
void SW1_WaitForPressAndRelease(void)
{
    ++buttons;
    /* Simulate manual repositioning: it must not pollute the next trial. */
    fixture.leftRearWheel.position += 12345;
    fixture.rightRearWheel.position -= 7000;
    HAL_Delay(250);
}
void OLED_Clear(void) { memset(oledRows, 0, sizeof(oledRows)); }
void OLED_Printf(uint8_t x, uint8_t y, const char *fmt, ...)
{
    assert(x == 0 && y < 8);
    va_list args;
    va_start(args, fmt);
    vsnprintf(oledRows[y], sizeof(oledRows[y]), fmt, args);
    va_end(args);
    assert(strlen(oledRows[y]) <= 21); /* 128 px / 6 px per font cell */
}
void OLED_Refresh_Gram(void)
{
    if (strncmp(oledRows[2], "Drive ~", 7) == 0) {
        ++preparedScreens;
        assert(strncmp(oledRows[4], "Guard ", 6) == 0);
        assert(strcmp(oledRows[5], "Plus stopping space") == 0);
        assert(strcmp(oledRows[7], wheelPerformanceRunConfig.waitBetweenTrials
            ? "Reposition; SW1" : "Auto start") == 0);
    }
}

static void Reset(uint32_t kind)
{
    tick = abortAt = buttons = rawFullTrials = preparedScreens = 0;
    refuseInit = preventStop = injectTimingGap = false;
    wheelPerformanceConfig = (WheelPerformanceConfig) {
        .experiment = kind, .repetitions = 2, .waitBetweenTrials = true,
        .batteryVoltage = 11.3f, .steeringRawCommand = 0,
        .maxDriveDistanceMm = 2000, .speedDriveMs = 1500,
        .steadyWindowMs = 500, .accelerationTargetCps = 5000,
        .accelerationHoldMs = 200, .brakeTimeoutMs = 1000
    };
}

static void CheckCompleted(uint32_t trials)
{
    assert(wheelPerformanceStatus == WHEEL_TEST_COMPLETE);
    assert(wheelPerformanceTrialCount == trials);
    assert(wheelPerformanceTraceCount > 0);
    assert(wheelPerformanceTraceCount < WHEEL_PERFORMANCE_TRACE_CAPACITY);
    assert(buttons == (wheelPerformanceRunConfig.waitBetweenTrials ? trials + 1 : 1));
    assert(preparedScreens == trials);
    uint32_t end = 0;
    bool forward = false, reverse = false;
    for (uint32_t i = 0; i < trials; ++i) {
        WheelPerformanceTrial *t = &wheelPerformanceTrials[i];
        assert(t->status == WHEEL_TEST_COMPLETE);
        assert(t->firstSample == end);
        assert(t->sampleCount > 0);
        assert(t->estimatedDriveDistanceMm > 0);
        end += t->sampleCount;
        WheelPerformanceSample *last = &wheelPerformanceTrace[end - 1];
        assert(last->phase == WHEEL_TEST_BRAKE);
        assert(abs(last->leftCps) < 100 && abs(last->rightCps) < 100);
        for (uint32_t j = t->firstSample; j < end; ++j) {
            WheelPerformanceSample *s = &wheelPerformanceTrace[j];
            assert(s->trial == i);
            assert(abs(s->leftCps) < 15000 && abs(s->rightCps) < 15000);
            if (j > t->firstSample)
                assert(s->timeMs > wheelPerformanceTrace[j - 1].timeMs);
            if (s->phase != WHEEL_TEST_BRAKE && s->timeMs > 200) {
                if (t->direction == 1) { assert(s->leftCps > 0); forward = true; }
                else { assert(s->leftCps < 0); reverse = true; }
            }
        }
    }
    assert(forward && reverse && end == wheelPerformanceTraceCount);
    assert(fixture.leftRearWheel.state.mode == DCMOTOR_MODE_NEUTRAL);
    assert(fixture.rightRearWheel.state.mode == DCMOTOR_MODE_NEUTRAL);
}

int main(int argc, char **argv)
{
    /* One completed sweep for checking the actual GDB export scripts. */
    if (argc > 1) {
        uint32_t kind = (uint32_t)atoi(argv[1]);
        Reset(kind);
        WheelPerformanceTestRun();
        CheckCompleted(kind == 1 ? 12 : 24);
        return 0;
    }
    Reset(1); WheelPerformanceTestRun(); CheckCompleted(12);
    assert(rawFullTrials == 4);
    assert(fabsf(EstimateDriveDistanceMm(1, 100) - 1894.77f) < 1.0f);
    assert(EstimateDriveDistanceMm(1, 100) > EstimateDriveDistanceMm(1, 80));
    assert(EstimateDriveDistanceMm(1, 100) != EstimateDriveDistanceMm(-1, 100));
    Reset(2); WheelPerformanceTestRun(); CheckCompleted(24);
    assert(fabsf(EstimateDriveDistanceMm(1, 500) - 567.0f) < 1.0f);
    assert(fabsf(EstimateDriveDistanceMm(1, 10000) - 154.0f) < 1.0f);
    assert(EstimateDriveDistanceMm(1, 500) > EstimateDriveDistanceMm(1, 10000));
    Reset(1); wheelPerformanceConfig.waitBetweenTrials = false;
    WheelPerformanceTestRun(); CheckCompleted(12);
    Reset(1); wheelPerformanceConfig.repetitions = 4;
    wheelPerformanceConfig.speedDriveMs = 10000;
    WheelPerformanceTestRun();
    assert(wheelPerformanceStatus == WHEEL_TEST_INVALID_CONFIG);
    assert(wheelPerformanceTrialCount == 0);
    Reset(1); wheelPerformanceConfig.maxDriveDistanceMm = 10;
    WheelPerformanceTestRun();
    assert(wheelPerformanceStatus == WHEEL_TEST_DISTANCE_LIMIT);
    assert(wheelPerformanceTrialCount == 1);
    assert(wheelPerformanceTrials[0].sampleCount > 0);
    assert(preparedScreens == 1);
    Reset(1); abortAt = 1500; WheelPerformanceTestRun();
    assert(wheelPerformanceStatus == WHEEL_TEST_BUTTON_ABORT);
    assert(wheelPerformanceTrialCount == 1);
    Reset(1); preventStop = true; WheelPerformanceTestRun();
    assert(wheelPerformanceStatus == WHEEL_TEST_STOP_TIMEOUT);
    assert(fixture.leftRearWheel.state.mode == DCMOTOR_MODE_BRAKE);
    Reset(1); refuseInit = true; WheelPerformanceTestRun();
    assert(wheelPerformanceStatus == WHEEL_TEST_INIT_FAILED);
    assert(wheelPerformanceTrialCount == 0);
    Reset(1); injectTimingGap = true; WheelPerformanceTestRun();
    assert(wheelPerformanceStatus == WHEEL_TEST_TIMING_FAULT);
    wheelPerformanceTraceCount = WHEEL_PERFORMANCE_TRACE_CAPACITY;
    assert(!RecordSample(0, 10, 0, WHEEL_TEST_DRIVE, 0, 0, 0));
    assert(wheelPerformanceStatus == WHEEL_TEST_LOG_FULL);
    printf("Sweep simulation: complete logs, repeats, reverse, encoder wrap, "
           "repositioning, configuration and fault handling passed.\n");
    return 0;
}
