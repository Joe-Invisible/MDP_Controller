#ifndef TASK2_OBSTACLE_DETECTION_H
#define TASK2_OBSTACLE_DETECTION_H
#include "Task2Route.h"
#define TASK2_USS_HISTORY_CAPACITY 15U

typedef struct
{
    bool ultrasonicEnabled;
    bool sideIREnabled;
    Task2SensorMount ultrasonic;
    Task2SensorMount leftIR;
    Task2SensorMount rightIR;
    float ultrasonicBeamHalfAngleDeg; /* Explicit cone model, not calibrated accuracy. */
    float ultrasonicRangeToleranceMm;
    uint32_t ultrasonicApproachMedianSamples;
    uint32_t ultrasonicDepartureMedianSamples;
    float ultrasonicJumpMm; /* Upward change after compensating for chassis motion. */
    float ultrasonicMaxDepartureYawDeg; /* Relative to the O2 face normal. */
    uint16_t irPresentAdc; /* Inclusive raw 12-bit ADC thresholds. */
    uint16_t irAbsentAdc;
    uint16_t irFaceDropAdc; /* Once armed, "gone" is also anything this far below the face level; 0 disables. */
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
    TASK2_EDGE_USS_RANGE_JUMP,
    TASK2_EDGE_USS_NO_ECHO,
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
    bool ultrasonicObserved;
    bool ultrasonicValid;
    bool ultrasonicDepartureStarted;
    bool ultrasonicHeadingExceeded;
    uint32_t ultrasonicHistoryCount;
    float ultrasonicHistoryResidualMm[TASK2_USS_HISTORY_CAPACITY];
    float ultrasonicFilteredResidualMm;
    float ultrasonicBaselineResidualMm;
    uint32_t ultrasonicJumpSamples;
    uint32_t ultrasonicNoEchoSamples;
    bool ultrasonicJumpCandidateValid;
    Task2RoutePose ultrasonicJumpPose;
    Task2RoutePose ultrasonicNoEchoPose;
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
    /* O2 half-length proven by armed, present face samples: the hit's lateral coordinate on the
     * face plane, less edge and pose uncertainty. A lower bound, unlike observedWidthMm. */
    float faceExtentLowerMm;
    float faceHitMm[8];        /* Latest consecutive face hits (at most 8 are compared). */
    uint32_t faceHitCount;     /* Consecutive qualifying samples; reset by any non-present one. */
    Task2RoutePose lastUltrasonicPose; /* Capture pose of lastUltrasonicRangeMm. */
    bool reacquire;               /* Same-side, unarmed: the IR reads no face beyond the proven face; back onto it. */
    uint16_t irFaceRecentAdc[3];  /* Latest consecutive present readings. */
    uint32_t irFaceRecentCount;
    uint16_t irFaceMedianAdc[8];  /* Medians of three, from the latest present readings once armed. */
    uint32_t irFaceMedianCount;
    uint16_t irFaceLevelAdc;      /* Highest of irFaceMedianAdc: the face level; 0 before any. */
} Task2ObstacleDetection;

bool Task2ObstacleDetection_ValidConfig(const Task2ObstacleSensorConfig *config);
/* Keep acquired O2 evidence; seed a shorter departure window from its baseline. */
void Task2ObstacleDetection_BeginUSSDeparture(Task2ObstacleDetection *state,
    const Task2ObstacleSensorConfig *config);
/* Called once at the stopped-heading start of the sensor-terminated straight. */
void Task2ObstacleDetection_BeginIRTraverse(Task2ObstacleDetection *state,
    const Task2RoutePose *pose);
/* USS: approach acquisition, LL/RR departure loss. IR: LR/RL straight only. */
void Task2ObstacleDetection_Observe(Task2ObstacleDetection *state,
    const Task2ObstacleSensorConfig *config, const Task2RouteConfig *route,
    Task2ObstacleSensor sensor, const Task2ObstacleObservation *observation,
    bool departing);
#endif
