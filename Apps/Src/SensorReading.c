#include "SensorReading.h"

#include <stdio.h>

static SensorStatus SensorReading_CheckAge(SensorStatus status, uint32_t capturedMs,
                                           uint32_t now, uint32_t staleMs)
{
    if (status != SENSOR_NOT_READY && (uint32_t)(now - capturedMs) > staleMs)
    {
        return SENSOR_STALE;
    }
    return status;
}

SensorStatus SensorReading_Status(const UltrasonicReading *reading, uint32_t now)
{
    return SensorReading_CheckAge(reading->status, reading->capturedMs,
                                   now, SENSOR_ULTRASONIC_STALE_MS);
}

SensorStatus SensorReading_IRStatus(const IRReading *reading, uint32_t now)
{
    return SensorReading_CheckAge(reading->status, reading->capturedMs,
                                   now, SENSOR_IR_STALE_MS);
}

const char *SensorReading_StatusName(SensorStatus status)
{
    static const char *const names[] = {
        "NOT_READY", "OK", "TIMEOUT", "ERROR", "OUT_OF_RANGE", "STALE", "SATURATED"
    };
    return (unsigned)status < sizeof(names) / sizeof(names[0]) ? names[status] : "ERROR";
}

void SensorReading_Format(const UltrasonicReading *reading, uint32_t now,
                          char *reply, size_t size)
{
    SensorStatus status = SensorReading_Status(reading, now);
    char distance[16] = "-";
    if (status == SENSOR_OK)
    {
        (void)snprintf(distance, sizeof(distance), "%.1f", (double)reading->distanceMm);
    }
    (void)snprintf(reply, size, "U %lu %s %s %lu %lu\n",
        (unsigned long)reading->sequence, SensorReading_StatusName(status), distance,
        (unsigned long)reading->pulseUs,
        (unsigned long)(reading->status == SENSOR_NOT_READY ? 0U : now - reading->capturedMs));
}

static void SensorReading_FormatIRSide(const IRReading *reading, uint32_t now,
                                       char *out, size_t size)
{
    uint32_t age = reading->status == SENSOR_NOT_READY ? 0U : now - reading->capturedMs;
    SensorStatus status = SensorReading_IRStatus(reading, now);
    (void)snprintf(out, size, "%s %u %u %lu", SensorReading_StatusName(status),
                   (unsigned)reading->adc, (unsigned)reading->millivolts, (unsigned long)age);
}

void SensorReading_FormatIR(const IRPairReading *reading, uint32_t now,
                            char *reply, size_t size)
{
    char left[48];
    char right[48];
    SensorReading_FormatIRSide(&reading->left, now, left, sizeof(left));
    SensorReading_FormatIRSide(&reading->right, now, right, sizeof(right));
    (void)snprintf(reply, size, "I %lu %s %s\n",
                   (unsigned long)reading->sequence, left, right);
}
