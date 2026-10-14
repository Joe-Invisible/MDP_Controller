#ifndef TASK2_APP_H
#define TASK2_APP_H
#include "Task2Commands.h"
#include "Task2Estimate.h"
#include "SensorReading.h"

typedef struct {
    float distanceMm; /* Signed travel since primitive launch, including braking. */
    float yawRad;    /* Relative to primitive launch. */
    bool moving, braking;
} Task2MotionReading;
typedef struct {
    void *context;
    const char *(*moveStraight)(void *, float distanceMm, float speedCps);
    const char *(*moveArc)(void *, float distanceMm, float radiusMm, float speedCps);
    bool (*busy)(void *);
    void (*brake)(void *);
    const char *(*requestSpeed)(void *, float speedCps);
    const char *(*moveContinuous)(void *, float speedCps);
    Task2MotionReading (*motion)(void *);
    UltrasonicReading (*ultrasonic)(void *);
    IRPairReading (*infrared)(void *);
    float mmPerCount;
    float decelerationMmps2;
} Task2AppIo;

typedef enum {
    TASK2_APP_RANGE1, TASK2_APP_APPROACH1, TASK2_APP_WAIT1, TASK2_APP_SWERVE1,
    TASK2_APP_RANGE2, TASK2_APP_RECOGNITION2, TASK2_APP_WAIT2,
    TASK2_APP_ACQUIRE2, TASK2_APP_ROUTE, TASK2_APP_FINISHED, TASK2_APP_ABORTED
} Task2AppPhase;
typedef enum { TASK2_APP_RUNNING, TASK2_APP_DONE, TASK2_APP_FAULT } Task2AppStatus;
#define TASK2_APP_POSE_HISTORY 32U
typedef struct {
    const Task2AppIo *io;
    Task2AppPhase phase;
    const char *fault;
    Task2RouteConfig geometry;
    Task2ObstacleSensorConfig sensors;
    Task2Estimate estimates;
    Task2Route route;
    Task2ObstacleDetection detection;
    Task2RoutePose pose, motionOrigin;
    bool primitive, edgeBraking, irStarted;
    uint32_t step, phaseStartedMs, previousMs, activeMs, irStartedMs;
    uint32_t ultrasonicSequence, irSequence, lastUSSObservedMs, lastIRObservedMs;
    float pathMm, previousDistanceMm, previousYawRad;
    float faceWindow[TASK2_USS_HISTORY_CAPACITY];
    unsigned faceCount;
    struct { uint32_t ms; Task2RoutePose pose; } history[TASK2_APP_POSE_HISTORY];
    unsigned historyHead, historyCount;
} Task2App;

extern const Task2ObstacleSensorConfig task2AppSensorDefaults;
const char *Task2App_Start(Task2App *, const Task2AppIo *, uint32_t nowMs);
Task2AppStatus Task2App_Update(Task2App *, Task2Commands *, uint32_t nowMs);
void Task2App_Abort(Task2App *);
#endif
