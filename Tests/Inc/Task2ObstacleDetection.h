#ifndef TASK2_OBSTACLE_DETECTION_H
#define TASK2_OBSTACLE_DETECTION_H
#include "Task2Route.h"

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
    float irAcquisitionMinYawDeg; /* Mirrored chassis yaw; side IR becomes eligible here. */
    uint16_t irTrendMinDropAdc; /* Median-filtered drop from the preceding plateau. */
    uint16_t irTrendReboundAdc; /* Small upward fluctuations do not cancel a decline. */
    uint32_t irTrendSamples;
    float irTrendMinTravelMm;
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

typedef enum
{
    TASK2_EDGE_THRESHOLD = 0,
    TASK2_EDGE_IR_TREND,
    TASK2_EDGE_IR_INITIAL_ABSENCE_BOUND,
} Task2EdgeMethod;

typedef struct
{
    bool armed[TASK2_SENSOR_COUNT];
    uint32_t present[TASK2_SENSOR_COUNT];
    uint32_t absent[TASK2_SENSOR_COUNT];
    Task2RoutePose firstAbsentPose[TASK2_SENSOR_COUNT];
    bool detected;
    bool fault;
    bool irAcquisitionTimedOut;
    Task2ObstacleSensor source;
    float lastUltrasonicRangeMm;
    uint16_t lastIRAdc[2]; /* Left, right; last ADC read, before geometry checks. */
    bool irObserved[2];
    bool irValid[2];
    bool irTraverseStarted;
    Task2RoutePose irTraverseStartPose;
    uint16_t irHistoryAdc[3];
    Task2RoutePose irHistoryPose[3];
    uint32_t irHistoryCount;
    uint16_t irFilteredAdc;
    uint16_t irPeakAdc;
    Task2RoutePose irPeakPose;
    uint32_t irDeclineSamples;
    bool irTrendCandidateValid;
    Task2RoutePose irTrendCandidatePose;
    Task2EdgeMethod edgeMethod;
    float observedWidthMm;
    Task2RoutePose edgePose;
} Task2ObstacleDetection;

bool Task2ObstacleDetection_ValidConfig(const Task2ObstacleSensorConfig *config);
/* Called once at the stopped-heading start of the sensor-terminated straight. */
void Task2ObstacleDetection_BeginIRTraverse(Task2ObstacleDetection *state,
    const Task2RoutePose *pose);
/* Ultrasonic may acquire on approach; IR observations require departing=true. */
void Task2ObstacleDetection_Observe(Task2ObstacleDetection *state,
    const Task2ObstacleSensorConfig *config, const Task2RouteConfig *route,
    Task2ObstacleSensor sensor, const Task2ObstacleObservation *observation,
    bool departing);
#endif
