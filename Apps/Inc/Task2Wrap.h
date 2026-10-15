#ifndef INC_TASK2WRAP_H_
#define INC_TASK2WRAP_H_

#include <stdbool.h>
#include <stdint.h>

#include "Task2App.h"
#include "Task2Commands.h"
#include "Task2Diagnostics.h"

/*
 * Task 2 as a fixed sequence of R275 moves placed by measurement, without a route planner.
 *
 *  1. O1: a standing ultrasonic range, then an encoder move to O1_STOP from it. The live ultrasonic only
 *     guards (O1 clearly nearer than the encoders say: brake, range again standing, finish the approach).
 *  2. Swerve past O1 on arrow 1's side: to +57 deg, 50 mm, to -40 deg, 100 mm, to 0 deg. It ends square,
 *     about 100 mm off the start line on O1's side, close enough for the ultrasonic to see a 300 mm O2.
 *  3. O2: a standing ultrasonic range, then a move to O2_STOP from its face. A reverse never brings the tail
 *     within O1_TAIL_MIN of O1. With no echo: range again, back off in steps, and only then move sideways
 *     or assume the nearest O2 the rules allow.
 *  4. 90 deg towards arrow 2. The O2-side IR finds O2's near end (backing up to it if the face is already
 *     out of view); the robot stops with its rear axle END_SHORT before the end.
 *  5. 180 deg round that end, along the back face to the far end, the same stop, 90 deg towards home.
 *  6. Home to the start line by odometry. The final S onto it starts only once the tail is past O1; when that
 *     S would not finish before the car park, a lane beside O1 comes first. Park on odometry with the body
 *     centred in the car park; the ultrasonic only guards.
 *
 * Every arc ends on a heading in the run's frame (start heading 0), so heading errors do not add up.
 * Nothing here stops the run on a navigation doubt: a missing measurement has a stated fallback, recorded
 * in the run's decision records. Only motion faults (from MotionTask) and broken I/O end a run.
 *
 * Frame: rear-axle centre, x forward from the start pose, y to the left, yaw anticlockwise; the start pose is
 * (TASK2_ROUTE_START_X_MM, 0, 0) so the trace reads like the planner's. Distances mm, speeds mm/s.
 */

/* 1: the run starts with the robot's rear against the car park's back wall; 0: with its front at the
 * entrance. Sets where the car park is behind the start pose. */
#ifndef TASK2_WRAP_START_REAR_FLUSH
#define TASK2_WRAP_START_REAR_FLUSH 1
#endif

#define TASK2_WRAP_MOVES 12U
#define TASK2_WRAP_RANGE_SAMPLES 5U

typedef enum {
    TASK2_WRAP_RANGE1, TASK2_WRAP_APPROACH1, TASK2_WRAP_REMEASURE1, TASK2_WRAP_WAIT1, TASK2_WRAP_SWERVE,
    TASK2_WRAP_RANGE2, TASK2_WRAP_STANDOFF, TASK2_WRAP_WAIT2, TASK2_WRAP_TURN_OUT, TASK2_WRAP_NEAR_PROBE,
    TASK2_WRAP_NEAR_SEARCH, TASK2_WRAP_NEAR_SETTLE, TASK2_WRAP_UTURN, TASK2_WRAP_FAR_SEARCH,
    TASK2_WRAP_FAR_SETTLE, TASK2_WRAP_TURN_HOME, TASK2_WRAP_HOME, TASK2_WRAP_PARK, TASK2_WRAP_PARK_FINISH,
    TASK2_WRAP_FINISHED, TASK2_WRAP_ABORTED
} Task2WrapPhase;

typedef enum { TASK2_WRAP_ARC_TO, TASK2_WRAP_STRAIGHT, TASK2_WRAP_TO_X, TASK2_WRAP_S_TO_Y } Task2WrapMoveKind;
typedef struct {
    uint8_t kind;      /* Task2WrapMoveKind */
    int8_t turn;       /* ARC_TO: +1 anticlockwise, -1 clockwise. S_TO_Y: 0 heading +x, 1 heading -x. */
    float value;       /* ARC_TO heading (rad); STRAIGHT signed distance; TO_X / S_TO_Y the target x / y. */
    float speedMmps;
} Task2WrapMove;

/* Standing range: fresh readings in a window, or none by its deadline. */
typedef struct {
    float values[TASK2_WRAP_RANGE_SAMPLES];
    unsigned count;
    uint32_t deadlineMs;
    bool active;
} Task2WrapRange;

/* A side-IR search along O2: the state wanted on three samples in a row, and where the first was taken.
 * A search for the face's end only counts once the face itself has been seen. */
typedef struct {
    bool want;         /* true: find the face; false: find where it ends */
    int8_t direction;  /* +1 forward, -1 reverse */
    unsigned run, faceRun;  /* the wanted state's run, and present samples in a row before the face counts */
    float firstIrYMm, firstSpeedMmps;
    bool found, braking, slowed, seenFace, exhausted;
    float slowAtYMm;   /* Axle y where a cruise search drops to search speed; NAN: none. */
    float limitIrYMm;  /* The search gives up once the sensor passes this y along its motion; NAN: none. */
    bool present;      /* Hysteresis state between the absent and present levels. */
    uint32_t lastIrMs; /* When the last valid sample arrived (or the search began). */
    bool noData;       /* Ran out because the IR gave no valid samples, not by distance. */
} Task2WrapSearch;

typedef struct {
    const Task2AppIo *io;
    Task2WrapPhase phase;
    const char *fault;
    int arrow1, arrow2;              /* +1 left, -1 right, 0 not known yet */
    Task2RoutePose pose, motionOrigin;
    bool primitive;
    float previousDistanceMm, previousYawRad, pathMm;
    float arcTargetRad;              /* Heading the running arc should end on; NAN for other moves. */
    Task2WrapMove moves[TASK2_WRAP_MOVES];
    unsigned moveCount, moveNext;
    Task2WrapRange range;
    Task2WrapSearch search;
    uint32_t phaseStartedMs, previousMs, activeMs;
    uint32_t ultrasonicSequence, irSequence;
    struct { uint32_t ms; Task2RoutePose pose; } history[TASK2_APP_POSE_HISTORY];
    unsigned historyHead, historyCount;
    /* Measurements. */
    float startFrontXMm, o1RangeMm, o1FaceXMm, o2FaceXMm, nearEndYMm, farEndYMm;
    float lastUssMm;
    uint8_t lastUssStatus;
    uint32_t lastUssSequence, lastUssCapturedMs;
    unsigned guardRun, backoffSteps, probeSamples, probePresent, rangeTries;
    bool guardBraked, remeasured, o2Assumed, sideStepped, nearExhausted, farExhausted, nearReacquired, singleS;
    uint16_t irAdc[2];
    uint16_t irHistoryAdc[TASK2_DIAG_IR_HISTORY];
    unsigned irHistoryCount, arcMisses;
#if TASK2_DIAGNOSTICS
    Task2Diagnostics diag;
    Task2Trace trace;
    uint32_t traceStartMs, traceLastMs;
    UltrasonicReading traceUss;
    IRPairReading traceIr;
#endif
} Task2Wrap;

const char *Task2Wrap_Start(Task2Wrap *, const Task2AppIo *, uint32_t nowMs);
Task2AppStatus Task2Wrap_Update(Task2Wrap *, Task2Commands *, uint32_t nowMs);
void Task2Wrap_Abort(Task2Wrap *);
/* A fault from outside (watchdog, controller): record it with the run's state before the abort. */
void Task2Wrap_NoteFault(Task2Wrap *, const char *reason);

#endif
