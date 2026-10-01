#ifndef SENSOR_READING_H
#define SENSOR_READING_H

#include <stddef.h>
#include <stdint.h>
#include "SensorConfig.h"

#define SENSOR_REPLY_SIZE 128U

typedef enum
{
    SENSOR_NOT_READY,
    SENSOR_OK,
    SENSOR_TIMEOUT,
    SENSOR_ERROR,
    SENSOR_OUT_OF_RANGE,
    SENSOR_STALE,
    SENSOR_SATURATED
} SensorStatus;

typedef struct
{
    uint32_t sequence;
    uint32_t capturedMs;
    uint32_t pulseUs;
    float distanceMm;
    SensorStatus status;
} UltrasonicReading;

/* ADC OK means successful acquisition, not a calibrated/valid distance. */
typedef struct
{
    uint32_t capturedMs;
    uint16_t adc;
    uint16_t millivolts;
    SensorStatus status;
} IRReading;

typedef struct
{
    uint32_t sequence;
    IRReading left;
    IRReading right;
} IRPairReading;

SensorStatus SensorReading_Status(const UltrasonicReading *reading, uint32_t now);
SensorStatus SensorReading_IRStatus(const IRReading *reading, uint32_t now);
const char *SensorReading_StatusName(SensorStatus status);

/* U <sample-seq> <status> <corrected-mm-or-dash> <echo-us> <age-ms>\n */
void SensorReading_Format(const UltrasonicReading *reading, uint32_t now,
                          char *reply, size_t size);
/* I seq L-status L-adc L-mV L-age R-status R-adc R-mV R-age\n */
void SensorReading_FormatIR(const IRPairReading *reading, uint32_t now,
                            char *reply, size_t size);

#endif /* SENSOR_READING_H */
