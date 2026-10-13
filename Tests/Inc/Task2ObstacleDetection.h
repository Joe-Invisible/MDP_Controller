#ifndef TASK2_OBSTACLE_DETECTION_H
#define TASK2_OBSTACLE_DETECTION_H
#include "Task2Route.h"

typedef struct
{
    float aheadMm;
    float leftMm;
    float yawDeg; /* Beam direction relative to chassis +x. */
} Task2SensorMount;

typedef struct
{
    bool ultrasonicEnabled;
    bool sideIREnabled;
    Task2SensorMount ultrasonic;
    Task2SensorMount leftIR;
    Task2SensorMount rightIR;
    float ultrasonicBeamHalfAngleDeg; /* Explicit cone model, not calibrated accuracy. */
    float ultrasonicRangeToleranceMm;
    uint16_t irPresentAdc; /* Inclusive raw 12-bit ADC thresholds. */
    uint16_t irAbsentAdc;
    float edgeUncertaintyMm;
    uint32_t samplePeriodMs;
    uint32_t acquisitionTimeoutMs;
    uint32_t presentSamples;
    uint32_t absentSamples;
} Task2ObstacleSensorConfig;

typedef enum { TASK2_SENSOR_ULTRASONIC, TASK2_SENSOR_LEFT_IR, TASK2_SENSOR_RIGHT_IR,
    TASK2_SENSOR_COUNT } Task2ObstacleSensor;

typedef struct
{
    bool valid;
    bool fault; /* ADC/trigger fault is distinct from a completed no-echo timeout. */
    float value; /* mm for ultrasonic; raw ADC counts (0..4095) for IR. */
    Task2RoutePose pose; /* Pose when the measurement was triggered. */
} Task2ObstacleObservation;

typedef struct
{
    bool armed[TASK2_SENSOR_COUNT];
    uint32_t present[TASK2_SENSOR_COUNT];
    uint32_t absent[TASK2_SENSOR_COUNT];
    Task2RoutePose firstAbsentPose[TASK2_SENSOR_COUNT];
    bool detected;
    bool fault;
    Task2ObstacleSensor source;
    float lastUltrasonicRangeMm;
    float observedWidthMm;
    Task2RoutePose edgePose;
} Task2ObstacleDetection;

bool Task2ObstacleDetection_ValidConfig(const Task2ObstacleSensorConfig *config);
void Task2ObstacleDetection_Observe(Task2ObstacleDetection *state,
    const Task2ObstacleSensorConfig *config, const Task2RouteConfig *route,
    Task2ObstacleSensor sensor, const Task2ObstacleObservation *observation,
    bool departing);
#endif
