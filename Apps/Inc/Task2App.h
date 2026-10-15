#ifndef TASK2_APP_H
#define TASK2_APP_H
#include "Task2Commands.h"
#include "Task2Estimate.h"
#include "SensorReading.h"
#include "Task2Diagnostics.h"

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
    uint32_t (*elapsedUs)(void *); /* Optional free-running microsecond clock for planning time. */
    /* Optional planning host: send a request, wait (bounded) for its reply. Returns the reply length,
     * 0 if none came in time, or TASK2_PLAN_CANCELLED if a stop arrived meanwhile. */
    size_t (*plan)(void *, uint16_t generation, const uint8_t *request, size_t length, uint8_t *reply, size_t capacity);
} Task2AppIo;
#define TASK2_PLAN_CANCELLED ((size_t)-1)

typedef enum {
    TASK2_APP_RANGE1, TASK2_APP_APPROACH1, TASK2_APP_WAIT1, TASK2_APP_SWERVE1,
    TASK2_APP_RANGE2, TASK2_APP_RECOGNITION2, TASK2_APP_WAIT2,
    TASK2_APP_ACQUIRE2, TASK2_APP_ROUTE, TASK2_APP_FINISHED, TASK2_APP_ABORTED
} Task2AppPhase;
typedef enum { TASK2_APP_RUNNING, TASK2_APP_DONE, TASK2_APP_FAULT } Task2AppStatus;
#define TASK2_APP_POSE_HISTORY 32U
/* Echoes kept for qualifying a nearer O2 face; the median of at least three must agree. */
#define TASK2_NEAR_FACE_SAMPLES 5U

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
    uint16_t planGeneration;   /* Each request to the planning host; a reply must match it. */
    bool planCancelled;        /* A stop arrived while waiting for the host: commit nothing. */
    uint8_t planSource;        /* Where the last planning call ran: 'L' here, 'R' on the host. */
    uint32_t planCalls;        /* Host-capable planning calls this run. */
    unsigned sensorConflicts;  /* Edges shorter than the proven face, widened to it. */
    unsigned obstacleShortfalls; /* Control ticks whose estimated O1/O2 gap was under the buffer; not a stop. */
    float rejectedRangeMm;     /* The last range refused as outside the course, for its fault record. */
    float cameraFaceMm, ussFaceMm; /* O2's face x at RANGE2 from the camera and from the ultrasonic. */
    uint32_t planHostUs;       /* Its time waiting for the host (transfer and host compute). */
    unsigned planHostAccepted, planHostRejected; /* Host routes used, and replies refused for a local plan. */
    unsigned planHostFailed;   /* Host plans that found no route, confirmed by planning here. */
    char planHostRefusal;      /* Why the last reply was refused: n count, c config, p prefix, l length, s sensor step,
                                * d departure kind, i indices, t buffer tier, x clearance, r unreadable or stale. */
    bool primitive, edgeBraking, irStarted;
    uint32_t step, phaseStartedMs, previousMs, activeMs, irStartedMs;
    uint32_t ultrasonicSequence, irSequence, lastUSSObservedMs, lastIRObservedMs;
    float pathMm, previousDistanceMm, previousYawRad;
    float provenHalfMm; /* Largest O2 half-length proven this run; sets the side limit. */
    bool edgeMeasured;  /* O2's end has been measured: the side limit is the proven one only. */
    bool sameSideDeparture; /* Same-side departure committed: also the proven limit only. */
    bool guardBraking;  /* Same-side departure braked for front clearance to O2's face. */
    unsigned departureExtensions;
    bool backingUp;     /* Reversing before a same-side IR traverse; plan it at the stop. */
    bool approachBraking; /* The O2 approach braked: the ultrasonic saw the face nearer than planned. */
    bool faceCorrected;   /* A near echo moved O2's face estimate closer at the end of the approach. */
    bool departureBackingUp; /* Reversing after a corrected face so the departure fits; plan it at the stop. */
    bool reacquireBraking;   /* Braking to back onto the proven face (same-side IR saw none beyond it). */
    float nearFaceMm;     /* Face x from the latest fresh echo at the end of the approach (NAN if none). */
    float nearFaceHistoryMm[TASK2_NEAR_FACE_SAMPLES]; /* Face x of recent echoes on the approach and while */
    uint32_t nearFaceHistoryMs[TASK2_NEAR_FACE_SAMPLES]; /* acquiring, each from its own capture pose. */
    unsigned nearFaceCount;
    uint16_t irHistoryAdc[TASK2_DIAG_IR_HISTORY]; /* The end-finding IR's latest readings, a ring. */
    unsigned irHistoryCount;
    bool rewinding;     /* Reversing along O2's face so the side IR starts on the proven part. */
    unsigned traverseRewinds;
    float rewindMm;
    float frozenRangeMm;       /* USS range held, within the frozen tolerance... */
    float frozenExpectedMm;    /* ...and the face range expected where it was first read. */
    bool ussFrozen;
    uint8_t lastUSSStatus;      /* SensorStatus, sequence and capture time of the latest fresh USS reading. */
    uint32_t lastUSSSequence, lastUSSCapturedMs;
    float faceWindow[TASK2_USS_HISTORY_CAPACITY];
    unsigned faceCount;
    struct { uint32_t ms; Task2RoutePose pose; } history[TASK2_APP_POSE_HISTORY];
    unsigned historyHead, historyCount;
#if TASK2_DIAGNOSTICS
    Task2Diagnostics diag;
    Task2Trace trace;
    uint32_t traceStartMs, traceLastMs;
    UltrasonicReading traceUss;   /* Sensor snapshot at the start of the latest update, for its trace sample. */
    IRPairReading traceIr;
#endif
} Task2App;

extern const Task2ObstacleSensorConfig task2AppSensorDefaults;
const char *Task2App_Start(Task2App *, const Task2AppIo *, uint32_t nowMs);
Task2AppStatus Task2App_Update(Task2App *, Task2Commands *, uint32_t nowMs);
void Task2App_Abort(Task2App *);
/* A fault from outside the app (watchdog, controller): record it with the run's state before the abort. */
void Task2App_NoteFault(Task2App *, const char *reason);
/* Shortest range of the USS cone to O2's face plane from robot pose p; NAN if the cone misses it. */
float Task2App_FaceRange(const Task2App *, const Task2RoutePose *p);
/* Feed each fresh USS reading with the face range expected at its capture pose. True once the range has
 * held while the expected range changed: the sensor is frozen. Invalid readings restart the comparison. */
bool Task2App_FrozenUSS(Task2App *, bool valid, float rangeMm, float expectedMm);
#endif
