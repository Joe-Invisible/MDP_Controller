/* Test the actual task helpers with fake acquisition. No board access.
 * cc -std=c11 -Wall -Wextra -Werror -Ihosttests/sensor_task_stubs -IApps/Inc
 *    -IPeripheralDrivers/Inc hosttests/sensor_task_test.c Apps/Src/SensorReading.c
 *    -o /tmp/sensor_task_test
 * Repeat with -DSENSOR_OLED_DIAGNOSTICS=0 to exercise the display-free build.
 */
#include <assert.h>
#include <string.h>
#include "../Apps/Src/SensorTask.c"

uint32_t testTick, testMask, testPolarity;
HAL_StatusTypeDef testStartResult;
void (*testBeforeDisable)(void);
TIM_HandleTypeDef htim12;
HCSR04_HandleTypeDef hcsr04;
const GP2Y0A21YK_Config sideIRLeftConfig = { .channel = 1 };
const GP2Y0A21YK_Config sideIRRightConfig = { .channel = 2 };
static bool initOK[2], triggerOK;
static GP2Y0A21YK_Status readStatus[2];
static unsigned reads[2], triggers, posts;
static uint16_t raw[2];
static bool registerOK;

GP2Y0A21YK_Status GP2Y0A21YK_Init(GP2Y0A21YK *sensor, const GP2Y0A21YK_Config *cfg)
{
    sensor->config = *cfg;
    sensor->initialized = initOK[cfg->channel - 1];
    return sensor->initialized ? GP2Y0A21YK_OK : GP2Y0A21YK_ERROR_INVALID_ARGUMENT;
}
GP2Y0A21YK_Status GP2Y0A21YK_Read(GP2Y0A21YK *sensor, GP2Y0A21YK_Measurement *m)
{
    assert(testMask == 0); /* Never wait for the ADC with interrupts masked. */
    assert(sensor->initialized);
    unsigned side = sensor->config.channel - 1;
    reads[side]++;
    testTick += 2; /* Bounded conversion delay; timestamp must be after it. */
    m->rawAdc = raw[side];
    m->voltageV = 1.0f;
    return readStatus[side];
}
void HCSR04_Init(HCSR04_HandleTypeDef *s, GPIO_TypeDef *p, uint16_t pin,
                TIM_HandleTypeDef *t, uint32_t ch)
{
    (void)p; (void)pin; (void)t; (void)ch;
    memset(s, 0, sizeof(*s));
}
bool HCSR04_IsBusy(const HCSR04_HandleTypeDef *s)
{
    return s->state == HCSR04_STATE_WAITING_RISING || s->state == HCSR04_STATE_WAITING_FALLING;
}
bool HCSR04_Trigger(HCSR04_HandleTypeDef *s)
{
    assert(testMask == 0);
    triggers++;
    s->state = triggerOK ? HCSR04_STATE_WAITING_RISING : HCSR04_STATE_IDLE;
    s->triggerTickMs = testTick;
    return triggerOK;
}
void HCSR04_Update(HCSR04_HandleTypeDef *s)
{
    if (HCSR04_IsBusy(s) && (uint32_t)(testTick - s->triggerTickMs) >= HCSR04_TIMEOUT_MS)
        s->state = HCSR04_STATE_TIMEOUT;
}
bool HCSR04_HasMeasurement(const HCSR04_HandleTypeDef *s) { return s->state == HCSR04_STATE_READY; }
uint32_t HCSR04_GetPulseWidthUs(const HCSR04_HandleTypeDef *s) { return s->pulseWidthUs; }
float HCSR04_GetDistanceMm(const HCSR04_HandleTypeDef *s) { return s->pulseWidthUs * HCSR04_MM_PER_US - 5; }
HCSR04_State HCSR04_GetState(const HCSR04_HandleTypeDef *s) { return s->state; }

#if SENSOR_OLED_DIAGNOSTICS
OLED_Status_t OLED_Register(OLED_Handle_t *h, const char *name)
{
    (void)name;
    h->slot = registerOK ? 0 : OLED_INVALID_SLOT;
    return registerOK ? OLED_OK : OLED_ERROR_FULL;
}
OLED_Status_t OLED_Post(OLED_Handle_t *h, const char *fmt, ...)
{
    (void)fmt;
    assert(testMask == 0 && h->slot != OLED_INVALID_SLOT);
    posts++;
    return OLED_OK;
}
#endif

static SensorTaskState Reset(void)
{
    testTick = 0; testMask = 0; triggers = 0; posts = 0;
    initOK[0] = initOK[1] = triggerOK = registerOK = true;
    readStatus[0] = readStatus[1] = GP2Y0A21YK_OK;
    reads[0] = reads[1] = 0;
    raw[0] = 1200; raw[1] = 1500;
    memset(&latestIR, 0, sizeof(latestIR));
    memset(&latestUltrasonic, 0, sizeof(latestUltrasonic));
    SensorTaskState state = {0};
    return state;
}

int main(void)
{
    SensorTaskState s = Reset();
    assert(SensorTask_GetIR().left.status == SENSOR_NOT_READY);
    initOK[0] = false;
    SensorTask_Initialize(&s);
    SensorTask_ServiceIR(&s);
    IRPairReading pair = SensorTask_GetIR();
    assert(reads[0] == 0 && reads[1] == 1);
    assert(pair.left.status == SENSOR_ERROR && pair.left.adc == 0);
    assert(pair.right.status == SENSOR_OK && pair.right.capturedMs == 2);
    assert(SensorReading_IRStatus(&pair.right, 252) == SENSOR_OK);
    assert(SensorReading_IRStatus(&pair.right, 253) == SENSOR_STALE);
    testTick = 49; SensorTask_ServiceIR(&s);
    assert(reads[1] == 1);
    testTick = 50; readStatus[1] = GP2Y0A21YK_ERROR_ADC_TIMEOUT;
    SensorTask_ServiceIR(&s);
    pair = SensorTask_GetIR();
    assert(pair.sequence == 2 && pair.right.status == SENSOR_TIMEOUT && pair.right.adc == 0);

    s = Reset(); SensorTask_Initialize(&s);
    raw[0] = 0; raw[1] = 4095;
    SensorTask_ServiceIR(&s);
    pair = SensorTask_GetIR();
    assert(pair.left.status == SENSOR_SATURATED && pair.right.status == SENSOR_SATURATED);
    s.lastIRMs = UINT32_MAX - 24; testTick = 25;
    SensorTask_ServiceIR(&s);
    assert(reads[0] == 2 && reads[1] == 2);
    testMask = 1; (void)SensorTask_GetIR(); (void)SensorTask_GetUltrasonic();
    assert(testMask == 1); testMask = 0;

    s = Reset(); SensorTask_Initialize(&s);
    SensorTask_ServiceUltrasonic(&s);
    assert(triggers == 1 && s.ultrasonicPending);
    hcsr04.state = HCSR04_STATE_READY; hcsr04.pulseWidthUs = 1200;
    hcsr04.measurementTickMs = 10; testTick = 20;
    SensorTask_ServiceUltrasonic(&s);
    UltrasonicReading u = SensorTask_GetUltrasonic();
    assert(u.sequence == 1 && u.status == SENSOR_OK && u.capturedMs == 10);
    testTick = 69; SensorTask_ServiceUltrasonic(&s); assert(triggers == 1);
    testTick = 70; SensorTask_ServiceUltrasonic(&s); assert(triggers == 2);
    testTick = 120; SensorTask_ServiceUltrasonic(&s);
    u = SensorTask_GetUltrasonic();
    assert(u.status == SENSOR_TIMEOUT && u.pulseUs == 0 && u.distanceMm == 0);
    testTick = 140; triggerOK = false; SensorTask_ServiceUltrasonic(&s);
    assert(SensorTask_GetUltrasonic().status == SENSOR_ERROR);
    assert(!s.ultrasonicPending);

    s = Reset(); SensorTask_Initialize(&s); SensorTask_ServiceUltrasonic(&s);
    hcsr04.state = HCSR04_STATE_READY; hcsr04.pulseWidthUs = 10;
    SensorTask_ServiceUltrasonic(&s);
    assert(SensorTask_GetUltrasonic().status == SENSOR_OUT_OF_RANGE);
#if SENSOR_OLED_DIAGNOSTICS
    s = Reset(); registerOK = false; SensorTask_Initialize(&s);
    SensorTask_ServiceIR(&s); SensorTask_UpdateDisplay(&s);
    assert(posts == 0 && SensorTask_GetIR().left.status == SENSOR_OK);
    s = Reset(); SensorTask_Initialize(&s); SensorTask_UpdateDisplay(&s);
    assert(posts == 2);
    testTick = 199; SensorTask_UpdateDisplay(&s); assert(posts == 2);
    testTick = 200; SensorTask_UpdateDisplay(&s); assert(posts == 4);
#endif
    return 0;
}
