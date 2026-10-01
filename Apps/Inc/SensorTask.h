#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include "SensorReading.h"

void SensorTask(void *argument);

/* Task context only. Short protected copies; never wait for an acquisition. */
UltrasonicReading SensorTask_GetUltrasonic(void);
IRPairReading SensorTask_GetIR(void);

#endif /* SENSOR_TASK_H */
