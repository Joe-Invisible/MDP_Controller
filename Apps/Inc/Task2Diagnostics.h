#ifndef INC_TASK2DIAGNOSTICS_H_
#define INC_TASK2DIAGNOSTICS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Planning and fault snapshots for the Pi's session log. Each planning decision and each fault of a run
 * stores one fixed-size record; nothing is sent until the Pi asks with "D T" (index) or "D T g" (one
 * record), and only while the wheels are stationary. The newest records are kept; the first fault of the
 * run is kept separately so later records cannot overwrite it.
 *
 * Units: mm, degrees, ms, us. Pose is the rear-axle centre in the course frame: x forward from the car
 * park, y to the left, yaw anticlockwise from +x.
 */

#ifndef TASK2_DIAGNOSTICS
#define TASK2_DIAGNOSTICS 1
#endif

/* Bump when a field or enum meaning changes, so old logs are read with the right key. */
#define TASK2_DIAG_FORMAT 4U
#ifndef TASK2_FIRMWARE_ID
#define TASK2_FIRMWARE_ID __DATE__ " " __TIME__
#endif

#define TASK2_DIAG_RECORDS 12U
#define TASK2_DIAG_STEPS 12U       /* A window of the route around its active step. */
#define TASK2_DIAG_IR_HISTORY 8U   /* Latest readings of the IR that looks for O2's end. */
#define TASK2_DIAG_PARTS 7U
#define TASK2_DIAG_LINE_SIZE 224U

typedef enum { TASK2_DIAG_PLAN, TASK2_DIAG_FAULT, TASK2_DIAG_STOP } Task2DiagKind;

typedef struct {
    float distanceMm, radiusMm, speedCps;
    bool sensorTerminated;
} Task2DiagStep;

typedef struct {
    uint16_t generation;       /* Run-local id, from 1 in capture order. */
    uint8_t kind;              /* Task2DiagKind */
    bool ok;                   /* The planning call succeeded (always false for a fault). */
    const char *what;          /* Decision site: static token. */
    const char *reason;        /* Route reason or fault token: static string or NULL. */
    uint32_t ms;               /* Course time of the capture. */
    uint8_t phase, step;       /* Task2AppPhase and route step when captured. */
    bool hasNext;              /* A movement outside the route launched with this decision (a reverse). */
    Task2DiagStep next;
    /* Robot state when captured. */
    float xMm, yMm, yawDeg;
    char arrow1, arrow2;       /* 'L', 'R' or '-'. */
    float obstacle1FaceMm, gapMm;
    float widthMm, widthUncertaintyMm;
    uint8_t widthSource;
    bool widthValid, widthBound;
    /* The app's bounds when captured. */
    float planWidthMm, endSpaceMm, sideLimitMm, provenHalfMm;
    bool edgeMeasured, provenOnly;
    /* What the planner was given and the policy it left: its start pose (which may not be the robot's,
     * e.g. a planned reverse), its own config and any reduced return buffers. */
    float fromXMm, fromYMm, fromYawDeg;
    float configWidthMm, configEndSpaceMm;
    bool returnFallbackActive;
    float returnObstacleBufferMm, returnBoundaryBufferMm;
    /* Outcome. */
    float obstacleGapMm, sideMarginMm, parkMarginMm;
    bool fallback, bufferException, poseAdjusted, returnCertified, exhausted;
    uint32_t candidateChecks;  /* The planner's last-candidate count. */
    uint32_t callChecks;       /* Every check in the call. */
    uint32_t callUs;           /* Whole-call time; 0 when no clock was given. */
    char planSource;           /* 'R' the host's route was used, 'L' planned here, '-' no host-capable call. */
    char planRefusal;          /* Why a host reply was refused (Task2AppIo codes), or '-'. */
    uint32_t hostWaitMs;       /* Time waiting for the host in that call. */
    uint8_t stepTotal;         /* The route's step count. */
    uint8_t stepFirst;         /* Route index of steps[0]. */
    uint8_t stepCount;         /* Steps captured; fewer than stepTotal means the route was cut. */
    Task2DiagStep steps[TASK2_DIAG_STEPS];
    /* Sensors. */
    float ussRangeMm;          /* Last valid range the detector kept. */
    bool ussValid;
    uint8_t ussStatus;         /* SensorStatus of the latest fresh reading. */
    uint32_t ussSequence, ussCapturedMs;
    uint32_t ussSinceObservedMs; /* Course time since the app last took a fresh reading. */
    uint16_t irAdc[2];         /* Left, right. */
    uint16_t irHistory[TASK2_DIAG_IR_HISTORY]; /* Oldest first; irHistoryCount are valid. */
    uint8_t irHistoryCount;
    uint16_t irFaceLevelAdc;   /* Face level the IR end was judged against; 0 before any. */
    uint8_t edgeMethod;
    bool detected;
    float observedWidthMm;
    float rejectedRangeMm;     /* A range reading refused as outside the course (a T2_*_RANGE fault). */
    float cameraFaceMm, ussFaceMm; /* O2's face x at RANGE2 from the camera and from the ultrasonic. */
    float frozenRangeMm, frozenExpectedMm;
    bool ussFrozen;
    uint8_t extensions, rewinds;
} Task2DiagRecord;

typedef struct {
    Task2DiagRecord records[TASK2_DIAG_RECORDS];
    Task2DiagRecord fault;     /* The run's first fault. */
    bool hasFault;
    uint8_t head, count;
    uint16_t nextGeneration;
    uint32_t overwritten;      /* Older records replaced by newer ones. */
} Task2Diagnostics;

void Task2Diag_Init(Task2Diagnostics *d);
/* A record to fill for this capture; its generation and kind are set. Never NULL. */
Task2DiagRecord *Task2Diag_Begin(Task2Diagnostics *d, Task2DiagKind kind);
/* Call once the record from Task2Diag_Begin is filled; keeps the first fault aside. */
void Task2Diag_End(Task2Diagnostics *d, const Task2DiagRecord *record);

/*
 * "D T" -> one index line; "D T g" -> TASK2_DIAG_PARTS lines for record g. Returns the number of lines
 * written to lines[] (0 if the request is malformed or names no record). Each line ends in '\n', fits in
 * TASK2_DIAG_LINE_SIZE and is marked "~" if it ever had to be cut, so a partial line is never mistaken for
 * a whole one.
 */
unsigned Task2Diag_Reply(const Task2Diagnostics *d, unsigned run, const char *request,
                         char lines[][TASK2_DIAG_LINE_SIZE], unsigned maxLines);

/*
 * Run trace: the robot's state every TASK2_TRACE_PERIOD_MS from the start of a run, the newest
 * TASK2_TRACE_SAMPLES kept, so a floor run can be followed sample by sample, not only at its decisions.
 * Sent only between runs (never while one is active, stopping or finishing), so the blocks of one fetch come
 * from one unchanging trace: "D R" -> one index line (n, blocks, per = samples a line, lines = lines a
 * block); "D R b" -> block b, up to TASK2_TRACE_BLOCK_LINES lines of TASK2_TRACE_PER_LINE samples, oldest
 * first. A sample is 20 bytes, little-endian, sent as hex:
 *   t (10 ms since the run started, u16), x, y (mm, s16), yaw (0.01 deg, s16), phase, step (u8),
 *   flags (u8, TASK2_TRACE_* bits), state (u8, TASK2_TRACE_STATE_* bits), ultrasonic (mm, u16),
 *   left IR, right IR (ADC, u16), IR face level (ADC, u16). 65535 = no valid reading at that tick.
 * Sensor values are the latest snapshot at the tick, not necessarily captured at that pose, and repeat when
 * no new reading has come; the flags are the state the previous update left. Only the newest 40 s are kept
 * (drop counts the rest), waits included. A final sample is taken at a fault, finish or abort.
 */
#define TASK2_TRACE_FORMAT 2U
#define TASK2_TRACE_SAMPLES 800U       /* 40 s */
#define TASK2_TRACE_PERIOD_MS 50U
#define TASK2_TRACE_SAMPLE_BYTES 20U
#define TASK2_TRACE_PER_LINE 4U
#define TASK2_TRACE_BLOCK_LINES 7U     /* Lines per "D R b" reply; the reply buffer holds TASK2_DIAG_PARTS. */
#define TASK2_TRACE_BLOCK_SAMPLES (TASK2_TRACE_PER_LINE * TASK2_TRACE_BLOCK_LINES)
_Static_assert(TASK2_TRACE_BLOCK_LINES <= TASK2_DIAG_PARTS, "a trace block fits the reply buffer");

#define TASK2_TRACE_USS_ARMED   0x01U
#define TASK2_TRACE_IR_ARMED    0x02U  /* The IR that looks for O2's end. */
#define TASK2_TRACE_DETECTED    0x04U  /* O2's end found. */
#define TASK2_TRACE_REACQUIRE   0x08U  /* Same-side IR asks to look again from the proven face. */
#define TASK2_TRACE_EDGE_BRAKE  0x10U
#define TASK2_TRACE_APPROACH_BRAKE 0x20U
#define TASK2_TRACE_FACE_CORRECTED 0x40U
#define TASK2_TRACE_MOVING      0x80U  /* The motion controller is busy. */
#define TASK2_TRACE_STATE_FINAL 0x01U  /* The run's last sample: taken at its fault, finish or abort. */

typedef struct {
    uint16_t t10Ms;
    int16_t xMm, yMm, yawCentiDeg;
    uint8_t phase, step, flags, state;
    uint16_t ussMm, irLeft, irRight, faceLevel;
} Task2TraceSample;

typedef struct {
    Task2TraceSample samples[TASK2_TRACE_SAMPLES];
    uint16_t head, count;
    uint32_t dropped;          /* Older samples replaced by newer ones. */
} Task2Trace;

void Task2Trace_Init(Task2Trace *t);
void Task2Trace_Add(Task2Trace *t, const Task2TraceSample *sample);
/* "D R" / "D R b"; returns the number of lines written (0: malformed request or no such block). */
unsigned Task2Trace_Reply(const Task2Trace *t, unsigned run, const char *request,
                          char lines[][TASK2_DIAG_LINE_SIZE], unsigned maxLines);

#endif
