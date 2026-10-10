#ifndef WHEEL_PERFORMANCE_TEST_H
#define WHEEL_PERFORMANCE_TEST_H

#include <stdbool.h>
#include <stdint.h>

#define WHEEL_PERFORMANCE_TOP_SPEED 1U
#define WHEEL_PERFORMANCE_ACCELERATION 2U
#define WHEEL_PERFORMANCE_DECELERATION 3U
#define WHEEL_PERFORMANCE_TRACE_CAPACITY 3000U
#define WHEEL_PERFORMANCE_TRIAL_CAPACITY 64U

typedef enum {
    WHEEL_TEST_NOT_STARTED = 0,
    WHEEL_TEST_RUNNING,
    WHEEL_TEST_COMPLETE,
    WHEEL_TEST_BUTTON_ABORT,
    WHEEL_TEST_DISTANCE_LIMIT,
    WHEEL_TEST_STOP_TIMEOUT,
    WHEEL_TEST_LOG_FULL,
    WHEEL_TEST_INVALID_CONFIG,
    WHEEL_TEST_INIT_FAILED,
    WHEEL_TEST_TIMING_FAULT,
    WHEEL_TEST_TELEMETRY_RANGE,
    WHEEL_TEST_SPEED_NOT_SETTLED
} WheelPerformanceStatus;

typedef enum {
    WHEEL_TEST_DRIVE = 1,
    WHEEL_TEST_RAMP,
    WHEEL_TEST_HOLD,
    WHEEL_TEST_BRAKE,
    WHEEL_TEST_PREP_ACCEL,
    WHEEL_TEST_PREP_HOLD,
    WHEEL_TEST_DECEL,
    WHEEL_TEST_ZERO_SETTLE
} WheelPerformancePhase;

/* Edit defaults in .c, or change this object in GDB before the FIRST SW1 press.
 * The entire configuration is frozen for the sweep and exported with the data.
 * Battery voltage is a manually entered measurement; there is no battery ADC.
 */
typedef struct {
    uint32_t experiment;
    uint32_t repetitions;
    bool waitBetweenTrials;
    float batteryVoltage;
    float steeringRawCommand;
    float maxDriveDistanceMm;
    uint32_t speedDriveMs;
    uint32_t steadyWindowMs;
    float accelerationTargetCps;
    uint32_t accelerationHoldMs;
    uint32_t brakeTimeoutMs;
    float decelerationStartCps;
    float preparationAccelerationMmps2;
    uint32_t preparationHoldMs;
    uint32_t preparationTimeoutMs;
    float wheelKp;
    float wheelKi;
} WheelPerformanceConfig;

/* 20 bytes/sample, 60,000 bytes for the whole sweep, in CPU-accessible CCM RAM.
 * Speeds are rounded to 1 CPS; PWM is signed and stored in hundredths of a %.
 * Motor modes distinguish positive braking PWM from forward propulsion.
 * timeMs is relative to this trial, not the whole sweep (repositioning excluded).
 */
typedef struct {
    uint32_t timeMs;
    uint16_t dtMs;
    uint8_t trial;
    uint8_t phase;
    int16_t targetCps;
    int16_t leftCps;
    int16_t rightCps;
    int16_t leftPwmX100;
    int16_t rightPwmX100;
    uint8_t leftMode;
    uint8_t rightMode;
} WheelPerformanceSample;

typedef struct {
    uint32_t direction; /* 1 forward, 2 reverse */
    uint32_t repetition;
    float pwmPercent;
    float accelerationMmps2;
    float targetCps;
    float estimatedDriveDistanceMm; /* Nominal controlled travel; excludes unprofiled stopping. */
    uint32_t firstSample;
    uint32_t sampleCount;
    uint32_t driveMs;
    float leftDriveDistanceMm;
    float rightDriveDistanceMm;
    float leftFinalDistanceMm;
    float rightFinalDistanceMm;
    uint32_t status;
    float decelerationMmps2;
    float estimatedSlowdownDistanceMm;
    uint32_t decelerationStartMs;
    uint32_t zeroTargetMs;
    float leftDecelerationStartCps;
    float rightDecelerationStartCps;
    float leftDecelerationStartDistanceMm;
    float rightDecelerationStartDistanceMm;
} WheelPerformanceTrial;

extern volatile WheelPerformanceConfig wheelPerformanceConfig;
extern WheelPerformanceConfig wheelPerformanceRunConfig;
extern WheelPerformanceSample wheelPerformanceTrace[WHEEL_PERFORMANCE_TRACE_CAPACITY];
extern WheelPerformanceTrial wheelPerformanceTrials[WHEEL_PERFORMANCE_TRIAL_CAPACITY];
extern volatile uint32_t wheelPerformanceTraceCount;
extern volatile uint32_t wheelPerformanceTrialCount;
extern volatile uint32_t wheelPerformanceStatus;
extern float wheelPerformanceMmPerCount;

void WheelPerformanceTestRun(void);
/* Set a breakpoint here. It runs once, after the entire sweep (or an abort).
 * Logs remain intact until reset. Do not reset before exporting.
 */
void WheelPerformanceTest_Complete(void);

#endif
