#ifndef INC_TASK2COMMANDS_H_
#define INC_TASK2COMMANDS_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Task 2 messages between the RPi and the STM, beside the batch session.
 * Pure state and formatting: no HAL/OS includes, host-tested in
 * hosttests/task2_commands_test.c.
 *
 * A run starts with the numbered batch "r:T2" (CommandSession gives it the
 * usual SEQ, BUSY, duplicate, Q and S handling); r is the run number in every
 * line below.
 *
 *   RPi -> STM   A r n L|R     arrow on obstacle n (1 or 2)
 *   STM -> RPi   ACK r n L|R   stored (repeated for a resend, even later in the run)
 *                NAK r n ARROW the other side is already stored; it is kept
 *                NAK r n STATE wrong run, or obstacle n is not being approached
 *   RPi -> STM   A - 1 L|R|-   arrow 1 seen before the run starts ("-" clears it); only between runs
 *   STM -> RPi   ACK - 1 L|R|- stored, for planning only: the run's arrow 1 comes from its
 *                start line, "r:T2L" / "r:T2R" (or none with "r:T2"), never from the preview
 *                NAK - n STATE during a run, or n is not 1
 *   RPi -> STM   Z - 1 mm      camera depth to obstacle 1's arrow before the run (lens axis, from the
 *                              photos that decided the preview); only between runs
 *   STM -> RPi   ACK - Z mm u  stored, with u the steady ultrasonic range to obstacle 1 from the robot's
 *                              front in mm, or "-" (added by Task2Run); NAK - Z STATE during a run
 *   RPi -> STM   W r mm        obstacle 2 bullseye span (lower bound; see Task2Commands_Span)
 *   STM -> RPi   ACK r W mm    stored; NAK r W STATE after ROUND2 or for another run
 *   RPi -> STM   Q T           course state
 *   STM -> RPi   T r stage a1 a2   e.g. "T 12 WAIT2 L -"; "T - IDLE - -" before any run
 *   STM -> RPi   EV r APPROACH n | EV r WAIT n | EV r ROUND 2 | EV r SUMMARY x1 x2 w2 path
 *
 * The preview remains available through Task2Commands_Preview for future pre-planning;
 * the current app plans after START from fresh ranging and the actual run arrow.
 * A stored arrow never changes. The course reads it with Task2Commands_Arrow()
 * and never guesses: with no arrow at its decision point it stops in WAITn
 * until one arrives or S cancels the run.
 */

#define TASK2COMMANDS_REPLY_SIZE 48U
#define TASK2COMMANDS_OUTBOX 8U

typedef enum
{
    TASK2_STAGE_IDLE,
    TASK2_STAGE_APPROACH1,
    TASK2_STAGE_WAIT1,
    TASK2_STAGE_SWERVE1,
    TASK2_STAGE_APPROACH2, /* only once the camera is past obstacle 1's face */
    TASK2_STAGE_WAIT2,
    TASK2_STAGE_ROUND2, /* committed to the way round obstacle 2 */
    TASK2_STAGE_HOME,
    TASK2_STAGE_DONE,
    TASK2_STAGE_STOPPED,
    TASK2_STAGE_FAULT,
} Task2Stage;

/* Same values as Task2Route's Task2Arrow, so the course can cast. */
typedef enum
{
    TASK2_COMMAND_ARROW_RIGHT = -1,
    TASK2_COMMAND_ARROW_NONE = 0,
    TASK2_COMMAND_ARROW_LEFT = 1,
} Task2CommandArrow;

typedef struct
{
    bool started; /* a run has begun since boot */
    uint8_t run;
    Task2Stage stage;
    Task2CommandArrow arrows[2];
    Task2CommandArrow preview; /* arrow 1 seen before the run: for planning, never committed */
    bool hasSpan;
    uint16_t obstacle2SpanMm;
    bool hasDepth;
    uint16_t obstacle2DepthMm;
    uint16_t
        previewO1DepthMm; /* camera depth to obstacle 1 before the run; 0: none. Cleared by Begin */
    /* Events waiting to be sent, oldest first. MotionTask drains it every tick. */
    char outbox[TASK2COMMANDS_OUTBOX][TASK2COMMANDS_REPLY_SIZE];
    unsigned outboxHead, outboxCount;
    uint32_t outboxDropped;
} Task2Commands;

void Task2Commands_Init(Task2Commands *t);

/* Batch r's T2 command is starting with arrow1 from its start line (NONE for plain "T2"):
 * no arrow 2 or span; queues EV r APPROACH 1. The preview is cleared. */
void Task2Commands_Begin(Task2Commands *t, uint8_t run, Task2CommandArrow arrow1);

/*
 * Handles "A ...", "W ...", "Z ..." and "Q T". Returns false for any other line,
 * leaving it to CommandSession. When true, reply holds the line to send.
 */
bool Task2Commands_Receive(Task2Commands *t,
                           const char *line,
                           char reply[TASK2COMMANDS_REPLY_SIZE]);

/* Records the stage; queues the event for APPROACHn, WAITn and ROUND2 when it changes. */
void Task2Commands_SetStage(Task2Commands *t, Task2Stage stage);

Task2CommandArrow Task2Commands_Arrow(const Task2Commands *t, unsigned obstacle);

/* Arrow 1 previewed for the next run; NONE if there is none. */
Task2CommandArrow Task2Commands_Preview(const Task2Commands *t);

/*
 * Latest bullseye span for this run, if any: outer edge to outer edge of the
 * camera-measured bullseyes on obstacle 2's face. A lower bound on its width,
 * not the width itself (how far in the bullseyes sit is not in the rules).
 */
bool Task2Commands_Span(const Task2Commands *t, uint16_t *spanMm);

/*
 * Camera depth to obstacle 2's arrow for this run, if any: along the lens axis, from photos taken while the
 * robot stood still in APPROACH2. Sent by the Pi just before arrow 2.
 */
bool Task2Commands_Depth(const Task2Commands *t, uint16_t *depthMm);

/* Queues "EV r SUMMARY ..." for the log, just before DONE. NAN prints "-". */
void Task2Commands_Summary(Task2Commands *t,
                           float obstacle1FaceMm,
                           float obstacle2FaceMm,
                           float obstacle2WidthMm,
                           float pathMm);

/* Drops unsent events, e.g. stage events overtaken by a stop or fault. */
void Task2Commands_ClearEvents(Task2Commands *t);

/* Copies the oldest queued event into line and removes it; false when empty. */
bool Task2Commands_NextEvent(Task2Commands *t, char line[TASK2COMMANDS_REPLY_SIZE]);

const char *Task2Commands_StageName(Task2Stage stage);

#endif
