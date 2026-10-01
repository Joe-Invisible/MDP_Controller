/*
 * Single owner of periodic ultrasonic and IR acquisition.
 * MotionTask owns UART replies; this task never sends protocol messages.
 * Snapshot critical sections contain copies only, never driver or OLED calls.
 */
#include "SensorTask.h"

#include "SensorConfig.h"
#include "SideIRSensorConfig.h"
#include "UltrasonicSensorConfig.h"
#include "cmsis_os.h"
#include "tim.h"

#if SENSOR_OLED_DIAGNOSTICS
#include "OLEDManager.h"
#endif

typedef struct
{
    GP2Y0A21YK leftIR;
    GP2Y0A21YK rightIR;
    bool leftIRReady;
    bool rightIRReady;
    bool ultrasonicPending;
    uint32_t lastTriggerMs;
    uint32_t lastIRMs;
    uint32_t ultrasonicSequence;
    uint32_t irSequence;
#if SENSOR_OLED_DIAGNOSTICS
    OLED_Handle_t ultrasonicDisplay;
    OLED_Handle_t irDisplay;
    bool ultrasonicDisplayReady;
    bool irDisplayReady;
    uint32_t lastDisplayMs;
#endif
} SensorTaskState;

/* Zero initialization exposes NOT_READY before the first publication. */
static UltrasonicReading latestUltrasonic;
static IRPairReading latestIR;

UltrasonicReading SensorTask_GetUltrasonic(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    UltrasonicReading copy = latestUltrasonic;
    __set_PRIMASK(mask);
    return copy;
}

IRPairReading SensorTask_GetIR(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    IRPairReading copy = latestIR;
    __set_PRIMASK(mask);
    return copy;
}

static void SensorTask_PublishUltrasonic(UltrasonicReading reading)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    latestUltrasonic = reading;
    __set_PRIMASK(mask);
}

static void SensorTask_PublishIR(IRPairReading reading)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    latestIR = reading;
    __set_PRIMASK(mask);
}

static IRReading SensorTask_ReadIR(GP2Y0A21YK *sensor, bool initialized)
{
    IRReading reading = { .status = SENSOR_ERROR };

    /* Init validates fixed configuration. A failed side stays ERROR until reboot. */
    if (initialized)
    {
        GP2Y0A21YK_Measurement measurement;
        GP2Y0A21YK_Status result = GP2Y0A21YK_Read(sensor, &measurement);
        if (result == GP2Y0A21YK_OK)
        {
            reading.adc = measurement.rawAdc;
            reading.millivolts = (uint16_t)(measurement.voltageV * 1000.0f + 0.5f);
            reading.status = reading.adc == 0U ||
                reading.adc >= GP2Y0A21YK_ADC_FULL_SCALE_COUNT
                ? SENSOR_SATURATED : SENSOR_OK;
        }
        else if (result == GP2Y0A21YK_ERROR_ADC_TIMEOUT)
        {
            reading.status = SENSOR_TIMEOUT;
        }
    }

    /* Timestamp this acquisition/failed attempt, not a later UART query. */
    reading.capturedMs = HAL_GetTick();
    return reading;
}

static void SensorTask_ServiceIR(SensorTaskState *state)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - state->lastIRMs) < SENSOR_IR_PERIOD_MS)
    {
        return;
    }

    /* Skip missed periods instead of bursting conversions after a delay. */
    state->lastIRMs = now;
    IRPairReading pair = { .sequence = ++state->irSequence };
    /* Shared ADC1: serial access through the existing driver's bounded reads. */
    pair.left = SensorTask_ReadIR(&state->leftIR, state->leftIRReady);
    pair.right = SensorTask_ReadIR(&state->rightIR, state->rightIRReady);
    SensorTask_PublishIR(pair);
}

static void SensorTask_ServiceUltrasonic(SensorTaskState *state)
{
    HCSR04_Update(&hcsr04);
    if (state->ultrasonicPending && !HCSR04_IsBusy(&hcsr04))
    {
        UltrasonicReading reading = {
            .sequence = ++state->ultrasonicSequence,
            .capturedMs = HAL_GetTick(),
            .status = HCSR04_GetState(&hcsr04) == HCSR04_STATE_TIMEOUT
                ? SENSOR_TIMEOUT : SENSOR_ERROR
        };
        if (HCSR04_HasMeasurement(&hcsr04))
        {
            reading.capturedMs = hcsr04.measurementTickMs;
            reading.pulseUs = HCSR04_GetPulseWidthUs(&hcsr04);
            reading.distanceMm = HCSR04_GetDistanceMm(&hcsr04);
            float rawMm = (float)reading.pulseUs * HCSR04_MM_PER_US;
            reading.status = rawMm >= SENSOR_ULTRASONIC_MIN_MM &&
                rawMm <= SENSOR_ULTRASONIC_MAX_MM
                ? SENSOR_OK : SENSOR_OUT_OF_RANGE;
        }
        SensorTask_PublishUltrasonic(reading);
        state->ultrasonicPending = false;
    }

    uint32_t now = HAL_GetTick();
    if (!state->ultrasonicPending &&
        (uint32_t)(now - state->lastTriggerMs) >= SENSOR_ULTRASONIC_PERIOD_MS)
    {
        state->lastTriggerMs = now;
        state->ultrasonicPending = HCSR04_Trigger(&hcsr04);
        if (!state->ultrasonicPending)
        {
            UltrasonicReading failed = {
                .sequence = ++state->ultrasonicSequence,
                .capturedMs = now,
                .status = SENSOR_ERROR
            };
            SensorTask_PublishUltrasonic(failed);
        }
    }
}

#if SENSOR_OLED_DIAGNOSTICS
static void SensorTask_UpdateDisplay(SensorTaskState *state)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - state->lastDisplayMs) < SENSOR_OLED_PERIOD_MS)
    {
        return;
    }
    state->lastDisplayMs = now;

    /* Diagnostic failures never prevent acquisition or snapshot publication. */
    if (state->ultrasonicDisplayReady)
    {
        UltrasonicReading reading = SensorTask_GetUltrasonic();
        SensorStatus status = SensorReading_Status(&reading, now);
        if (status == SENSOR_OK)
        {
            (void)OLED_Post(&state->ultrasonicDisplay, "%.0f mm",
                            (double)reading.distanceMm);
        }
        else
        {
            (void)OLED_Post(&state->ultrasonicDisplay, "%s",
                            SensorReading_StatusName(status));
        }
    }
    if (state->irDisplayReady)
    {
        IRPairReading pair = SensorTask_GetIR();
        SensorStatus left = SensorReading_IRStatus(&pair.left, now);
        SensorStatus right = SensorReading_IRStatus(&pair.right, now);
        if (left == SENSOR_OK && right == SENSOR_OK)
        {
            (void)OLED_Post(&state->irDisplay, "L%u R%u",
                            (unsigned)pair.left.adc, (unsigned)pair.right.adc);
        }
        else
        {
            (void)OLED_Post(&state->irDisplay, "L:%s R:%s",
                            SensorReading_StatusName(left),
                            SensorReading_StatusName(right));
        }
    }
}
#endif

static void SensorTask_Initialize(SensorTaskState *state)
{
    HCSR04_Init(&hcsr04, USS_TRIG_GPIO_Port, USS_TRIG_Pin,
                &htim12, TIM_CHANNEL_2);
    state->leftIRReady = GP2Y0A21YK_Init(&state->leftIR, &sideIRLeftConfig)
        == GP2Y0A21YK_OK;
    state->rightIRReady = GP2Y0A21YK_Init(&state->rightIR, &sideIRRightConfig)
        == GP2Y0A21YK_OK;

    uint32_t now = HAL_GetTick();
    state->lastTriggerMs = now - SENSOR_ULTRASONIC_PERIOD_MS;
    state->lastIRMs = now - SENSOR_IR_PERIOD_MS;
#if SENSOR_OLED_DIAGNOSTICS
    state->ultrasonicDisplay.slot = OLED_INVALID_SLOT;
    state->irDisplay.slot = OLED_INVALID_SLOT;
    state->ultrasonicDisplayReady =
        OLED_Register(&state->ultrasonicDisplay, "USS") == OLED_OK;
    state->irDisplayReady = OLED_Register(&state->irDisplay, "IR raw") == OLED_OK;
    state->lastDisplayMs = now - SENSOR_OLED_PERIOD_MS;
#endif
}

void SensorTask(void *argument)
{
    (void)argument;
    SensorTaskState state = {0};
    SensorTask_Initialize(&state);

    uint32_t delayTicks = (osKernelGetTickFreq() * SENSOR_TASK_POLL_MS + 999U) / 1000U;
    if (delayTicks == 0U)
    {
        delayTicks = 1U;
    }
    for (;;)
    {
        SensorTask_ServiceUltrasonic(&state);
        SensorTask_ServiceIR(&state);
#if SENSOR_OLED_DIAGNOSTICS
        SensorTask_UpdateDisplay(&state);
#endif
        osDelay(delayTicks);
    }
}
