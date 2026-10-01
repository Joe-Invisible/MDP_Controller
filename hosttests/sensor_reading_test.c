/* cc -Wall -Wextra -Werror -IApps/Inc hosttests/sensor_reading_test.c Apps/Src/SensorReading.c -o /tmp/sensor_reading_test */
#include "SensorReading.h"
#include <assert.h>
#include <string.h>
int main(void) {
    char reply[SENSOR_REPLY_SIZE];
    UltrasonicReading r = {0};
    SensorReading_Format(&r, 9000, reply, sizeof(reply));
    assert(strcmp(reply, "U 0 NOT_READY - 0 0\n") == 0);
    r = (UltrasonicReading){.sequence=300, .capturedMs=1000, .pulseUs=1200,
        .distanceMm=200.8f, .status=SENSOR_OK};
    SensorReading_Format(&r, 1020, reply, sizeof(reply));
    assert(strcmp(reply, "U 300 OK 200.8 1200 20\n") == 0);
    assert(SensorReading_Status(&r, 1250) == SENSOR_OK);
    SensorReading_Format(&r, 1251, reply, sizeof(reply));
    assert(strcmp(reply, "U 300 STALE - 1200 251\n") == 0);
    r.status=SENSOR_TIMEOUT; r.pulseUs=0; r.distanceMm=0;
    SensorReading_Format(&r, 1020, reply, sizeof(reply));
    assert(strcmp(reply, "U 300 TIMEOUT - 0 20\n") == 0);
    r.status=SENSOR_ERROR;
    assert(SensorReading_Status(&r, 1020) == SENSOR_ERROR);
    r.status=SENSOR_OK; r.capturedMs=UINT32_MAX-9;
    assert(SensorReading_Status(&r, 20) == SENSOR_OK);
    assert(SensorReading_Status(&r, 241) == SENSOR_STALE);
    IRPairReading pair = {.sequence=512,
        .left={.capturedMs=1000, .adc=1200, .millivolts=967, .status=SENSOR_OK},
        .right={.capturedMs=1005, .status=SENSOR_TIMEOUT}};
    SensorReading_FormatIR(&pair, 1020, reply, sizeof(reply));
    assert(strcmp(reply, "I 512 OK 1200 967 20 TIMEOUT 0 0 15\n") == 0);
    SensorReading_FormatIR(&pair, 1300, reply, sizeof(reply));
    assert(strcmp(reply, "I 512 STALE 1200 967 300 STALE 0 0 295\n") == 0);
    return 0;
}
