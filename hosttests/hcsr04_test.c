/* Host-only fake HAL exercises the actual driver, never accesses hardware.
 * cc -Wall -Wextra -Werror -Ihosttests/hcsr04_stubs -IPeripheralDrivers/Inc
 *    hosttests/hcsr04_test.c PeripheralDrivers/Src/hcsr04.c -o /tmp/hcsr04_test
 */
#include "hcsr04.h"
#include <assert.h>
#include <math.h>
uint32_t testTick, testMask, testPolarity;
HAL_StatusTypeDef testStartResult=HAL_OK;
void (*testBeforeDisable)(void);
static HCSR04_HandleTypeDef sensor;
static TIM_HandleTypeDef timer={.Channel=HAL_TIM_ACTIVE_CHANNEL_2, .period=65535};
static void Capture(uint32_t count) {
    timer.capture=count;
    HCSR04_HandleInputCapture(&sensor, &timer);
}
static void CompleteDuringTimeoutCheck(void) { Capture(1100); }
int main(void) {
    HCSR04_Init(&sensor, 0, 1, &timer, TIM_CHANNEL_2);
    assert(!HCSR04_HasMeasurement(&sensor));
    assert(HCSR04_Trigger(&sensor));
    assert(!HCSR04_Trigger(&sensor));
    timer.Channel=HAL_TIM_ACTIVE_CHANNEL_1;
    Capture(100);
    assert(sensor.state==HCSR04_STATE_WAITING_RISING);
    timer.Channel=HAL_TIM_ACTIVE_CHANNEL_2;
    Capture(65000);
    testTick=10;
    Capture(464); /* Counter wrap, 1000 us pulse. */
    assert(HCSR04_HasMeasurement(&sensor));
    assert(HCSR04_GetPulseWidthUs(&sensor)==1000);
    assert(sensor.measurementTickMs==10);
    assert(fabsf(HCSR04_GetDistanceMm(&sensor)-166.5f)<0.001f);
    testTick=UINT32_MAX-10;
    assert(HCSR04_Trigger(&sensor));
    testTick=38; HCSR04_Update(&sensor);
    assert(HCSR04_IsBusy(&sensor));
    testTick=39; HCSR04_Update(&sensor);
    assert(sensor.state==HCSR04_STATE_TIMEOUT);
    assert(!HCSR04_HasMeasurement(&sensor));
    assert(testMask==0);
    assert(HCSR04_Trigger(&sensor));
    Capture(100);
    testTick+=50;
    testBeforeDisable=CompleteDuringTimeoutCheck;
    HCSR04_Update(&sensor);
    assert(HCSR04_HasMeasurement(&sensor)); /* Do not overwrite completed echo. */
    testStartResult=HAL_ERROR;
    assert(!HCSR04_Trigger(&sensor));
    assert(!HCSR04_HasMeasurement(&sensor));
    sensor.pulseWidthUs=1;
    assert(HCSR04_GetDistanceMm(&sensor)==0.0f);
    return 0;
}
