#ifndef TASK2_OBSTACLE_SENSORS_H
#define TASK2_OBSTACLE_SENSORS_H
#include "Task2ObstacleDetection.h"
bool Task2ObstacleSensors_Init(const Task2ObstacleSensorConfig *config);
/* A bit per fresh observation; repeated control ticks never repeat a sample. */
uint32_t Task2ObstacleSensors_Poll(const Task2ObstacleSensorConfig *config,
    uint32_t nowMs, const Task2RoutePose *pose,
    Task2ObstacleObservation observations[TASK2_SENSOR_COUNT]);
void Task2ObstacleSensors_Stop(void);
#endif
