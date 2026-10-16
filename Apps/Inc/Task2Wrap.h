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
 *  1. O1: a standing ultrasonic range (taken while waiting for START when the readings then were steady),
 *     then, once arrow 1 is known (the robot waits at the start for it), an encoder move to O1_STOP from it. The live ultrasonic only guards (O1 clearly nearer than the
 *     encoders say: brake, range again standing, finish the approach).
 *  2. Swerve past O1 on arrow 1's side: three arcs to +63, -52 and 0 deg. It ends square, about 90 mm off the
 *     start line on O1's side, close enough for the ultrasonic to see a 300 mm O2. While beside O1 and nearly
 *     square, the ultrasonic already ranges O2 from afar; the last arc brakes if that says the front would end
 *     too near O2.
 *  3. O2: a standing ultrasonic range, then a move to O2_STOP from its face. A reverse never brings the tail
 *     within O1_TAIL_MIN of O1. With no echo: the camera's depth to O2's arrow (sent with arrow 2) if it comes
 *     within CAMERA_WAIT and fits the rules; else back off in steps, range again, and only then move sideways
 *     or assume the nearest O2 the rules allow.
 *  4. 90 deg towards arrow 2. The O2-side IR finds O2's near end (backing up to it if the face is already
 *     out of view); the robot stops with its rear axle END_SHORT before the end.
 *  5. 180 deg round that end, along the back face to the far end, the same stop, 90 deg towards home.
 *  6. Home by odometry, the first that fits with its clearances: one arc and a straight that ends at an angle
 *     with the body centred in the car park; a diagonal S onto the start line, then straight in; or past O1 in
 *     a lane beside it (an S out first if needed) and then in at an angle, or an S onto the start line and
 *     straight in. The ultrasonic only guards the last straight.
 *
 * Every arc ends on a heading in the run's frame (start heading 0), so heading errors do not add up.
 * Nothing here stops the run on a navigation doubt: a missing measurement has a stated fallback, recorded
 * in the run's decision records. Only motion faults (from MotionTask) and broken I/O end a run.
 *
 * Frame: rear-axle centre, x forward from the start pose, y to the left, yaw anticlockwise; the start pose is
 * (TASK2_ROUTE_START_X_MM, 0, 0) so the trace reads like the planner's. Distances mm, speeds mm/s.
 */

/* Gap from the robot's rear to the car park's back wall at the start (0: rear against it; 243: front on the
 * entrance line). Sets where the car park is behind the start pose, so where the robot parks. */
#ifndef TASK2_WRAP_START_BACK_GAP_MM
#define TASK2_WRAP_START_BACK_GAP_MM 210
#endif
/* 1: the back leg searches for O2's far end with the IR; 0: it drives to the far end mirrored from the near one
 * (O2 is centred on the start line by the rules), searching only when the near end was not measured. */
#ifndef TASK2_WRAP_FAR_BY_IR
#define TASK2_WRAP_FAR_BY_IR 1
#endif
/* 1: home ends with an arc and an angled straight into the car park wherever that fits, else one S (which may
 * finish inside the car park); 0: never the angled entry after the turn home (the one arc from O2's far end is
 * kept either way). 0 while TASK2_WRAP_PARK_CLEAR_MM is 100: no angled entry fits then, and searching for one along
 * the lane costs the STM about a second. */
#ifndef TASK2_WRAP_ANGLED_HOME
#define TASK2_WRAP_ANGLED_HOME 0
#endif
/* 1: at each O2 end the robot turns from where its search stopped when that is already within the turn's window
 * (floor: the stop lands 2-4 cm before the end, the target is 5 cm); 0: it always settles to END_SHORT first. */
#ifndef TASK2_WRAP_SETTLE_SKIP
#define TASK2_WRAP_SETTLE_SKIP 0
#endif
/* 1: home prefers a straight along the lane and then a diagonal S that lands exactly on the parking spot (no straight in,
 * one stop fewer), when that fits; otherwise the S that starts earliest and then the straight in. Experimental: the
 * span is solved for a start heading of exactly pi, so a heading error still leaves a straight in (about 7 mm per
 * 1.5 deg) or refuses the S; not used in any image. */
#ifndef TASK2_WRAP_EXACT_S
#define TASK2_WRAP_EXACT_S 0
#endif
/* 1: when the S home already ends with the body wholly inside the car park (80 mm to every wall and the entrance line),
 * it stays there instead of driving on to the centred spot. */
#ifndef TASK2_WRAP_PARK_WHERE_INSIDE
#define TASK2_WRAP_PARK_WHERE_INSIDE 1
#endif
/* 1: after the near end was found by reversing, the move forward to the U-turn spot runs at cruise, not settle speed. */
#ifndef TASK2_WRAP_FAST_REACQUIRE_SETTLE
#define TASK2_WRAP_FAST_REACQUIRE_SETTLE 0
#endif
/* 3: the three-arc swerve past O1; 5: arcs with two short straights between them. */
#ifndef TASK2_WRAP_SWERVE_ARCS
#define TASK2_WRAP_SWERVE_ARCS 3
#endif

/* Ultrasonic readings while no run is active, so a run can use O1's range from before START. Commanded motion
 * and unsteady or stale echoes make them unusable; the robot must still be left alone before START. */
#define TASK2_WRAP_PRESTART_SAMPLES 5U
typedef struct {
    float values[TASK2_WRAP_PRESTART_SAMPLES];
    uint32_t capturedMs[TASK2_WRAP_PRESTART_SAMPLES], sequences[TASK2_WRAP_PRESTART_SAMPLES];
    unsigned count, head;
    uint32_t sequence;
} Task2WrapPrestart;
/* What the samples gave at START: the median when trusted, with where it came from. */
typedef struct {
    float medianMm;                    /* median of five kept echoes, also when not trusted; NAN with fewer */
    float spreadMm;                    /* largest minus smallest echo; NAN with fewer than five */
    uint32_t newestSequence, newestCapturedMs, oldestAgeMs;
    unsigned count;                    /* echoes kept */
    const char *reject;                /* NULL when trusted, else why not */
} Task2WrapPrestartRange;

#define TASK2_WRAP_MOVES 12U
#define TASK2_WRAP_RANGE_SAMPLES 5U

typedef enum {
    TASK2_WRAP_RANGE1, TASK2_WRAP_APPROACH1, TASK2_WRAP_REMEASURE1, TASK2_WRAP_WAIT1, TASK2_WRAP_SWERVE,
    TASK2_WRAP_RANGE2, TASK2_WRAP_STANDOFF, TASK2_WRAP_WAIT2, TASK2_WRAP_TURN_OUT, TASK2_WRAP_NEAR_PROBE,
    TASK2_WRAP_NEAR_SEARCH, TASK2_WRAP_NEAR_SETTLE, TASK2_WRAP_UTURN, TASK2_WRAP_FAR_SEARCH,
    TASK2_WRAP_FAR_SETTLE, TASK2_WRAP_TURN_HOME, TASK2_WRAP_HOME, TASK2_WRAP_PARK, TASK2_WRAP_PARK_FINISH,
    TASK2_WRAP_FINISHED, TASK2_WRAP_ABORTED,
    TASK2_WRAP_WAIT1_START,            /* O1 ranged, arrow 1 not yet read: wait where the camera reads it well */
    TASK2_WRAP_STANDOFF_CHECK          /* at the standoff: range O2 again (accurate here) and correct the standoff */
} Task2WrapPhase;

/* ENTRY: into the car park at an angle if that fits from where the robot is when it runs. Otherwise, turn 2
 * (queued past O1): an S onto the start line and the straight park; turn 1 (queued earlier, from the lane scan):
 * the home route is planned again from there. */
/* DIAG: the diagonal S home, solved again from where the robot is when it runs (turn 1: to land exactly on the parking
 * spot, if that still fits; else home is planned again). */
typedef enum { TASK2_WRAP_ARC_TO, TASK2_WRAP_STRAIGHT, TASK2_WRAP_TO_X, TASK2_WRAP_S_TO_Y, TASK2_WRAP_ENTRY,
               TASK2_WRAP_DIAG } Task2WrapMoveKind;
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

#define TASK2_WRAP_FACE_SAMPLES 5U
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
    uint16_t faceAdc[TASK2_WRAP_FACE_SAMPLES];  /* The latest present samples on the face, for the level an end drops from. */
    unsigned faceCount;
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
    float o2SideFaceXMm;              /* O2's face from the ultrasonic beside O1 mid-swerve (agreeing echoes); NAN: none */
    float o2SideFaces[6];             /* each side echo's face */
    unsigned o2SideCount;
    Task2WrapPrestartRange prestart;  /* O1's range from before START (count 0: none offered) */
    float cameraO1Mm;                 /* O1's range from the front by the camera before START; NAN: none */
    float lastUssMm;
    uint8_t lastUssStatus;
    uint32_t lastUssSequence, lastUssCapturedMs;
    unsigned guardRun, backoffSteps, probeSamples, probePresent, rangeTries;
    bool guardBraked, remeasured, o2Assumed, sideStepped, nearExhausted, farExhausted, nearReacquired, singleS;
    bool cameraRejected;             /* the camera's O2 depth was outside the rules: not waited for again */
    bool standoffChecked;            /* O2 ranged again at the standoff */
    bool swerveBraked;               /* the swerve's last arc braked short of O2 */
    bool angled;                     /* parks at an angle: the last straight ends with the body centred in the car park */
    unsigned replans;                /* home routes planned again because an early entry no longer fitted */
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
/* O1's range from before START (Task2WrapPrestart_Range); call right after Start. An untrusted one is only
 * recorded, and O1 is ranged after START. */
void Task2Wrap_UsePrestart(Task2Wrap *, const Task2WrapPrestartRange *);
/* The camera's depth to O1's arrow before START (lens axis, mm; 0: none); call right after Start. When the ultrasonic
 * reads O1 much further than this, the approach stops short and ranges again. */
void Task2Wrap_UseCameraO1(Task2Wrap *, unsigned depthMm);
Task2AppStatus Task2Wrap_Update(Task2Wrap *, Task2Commands *, uint32_t nowMs);
void Task2Wrap_Abort(Task2Wrap *);
/* A fault from outside (watchdog, controller): record it with the run's state before the abort. */
void Task2Wrap_NoteFault(Task2Wrap *, const char *reason);

/* While no run is active: keep the latest ultrasonic samples; motion clears them. */
void Task2WrapPrestart_Update(Task2WrapPrestart *, const Task2AppIo *, bool moving, uint32_t nowMs);
/* The kept samples as a standing range now: trusted only if they are five, recent and steady. */
Task2WrapPrestartRange Task2WrapPrestart_Range(const Task2WrapPrestart *, uint32_t nowMs);
/* The median of the last five samples if they are steady, regardless of age (for reporting); NAN otherwise. */
float Task2WrapPrestart_Steady(const Task2WrapPrestart *);

#endif
