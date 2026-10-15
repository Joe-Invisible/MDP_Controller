/*
 * Task2Wrap against a deterministic fake robot: ideal motion, a geometric ultrasonic (nearest of O1, O2 and
 * the car park walls in a 7.5 deg half-beam) and side IRs that see O2 within 450 mm. Each case may script
 * IR faults, including invalid or frozen IR samples. Checks the run's outcome, its decision records and the robot's clearance to O1/O2.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "Task2Run.h"
#include "Task2Wrap.h"

#define PI 3.14159265358979323846f
#define MM_PER_COUNT (PI * 65.5f / 1560.0f)

/* Bad: the IR reports `badStatus` from arrow 2 on (for badForMs, or for good when 0). Frozen: its sequence stops
 * from arrow 2 on. Bad behind O2: `badStatus` while the robot is behind O2 (the back-leg search). Bad late on the
 * back leg: `badStatus` once the robot is behind O2 and within 300 mm of the start line, coming from the near end,
 * so the no-data brake at cruise carries it past the point where the search would slow down. */
typedef enum { IR_NORMAL, IR_ALWAYS_ABSENT, IR_BACK_FACE_LATE, IR_REBOUND, IR_BAD, IR_FROZEN, IR_BAD_BEHIND_O2,
               IR_BAD_LATE_BACK_LEG } IRFault;
typedef struct {
    float d1, gap, width;
    int arrow1, arrow2;
    IRFault ir;
    bool ussDeadAtO2;
    SensorStatus badStatus;
    uint32_t badForMs;
} Case;

static struct {
    Case c;
    float x, y, yaw;                  /* truth = belief here */
    bool active, braking;
    float distance, radius, speed, travelled, turned, v;
    float originYaw;
    uint32_t now, ussSeq, irSeq, ussCaptured, irCaptured;
    float ussMm; int ussStatus;
    uint16_t irLeft, irRight;
    float o1x, o2x, minO1, minO2;
    int lossCalls;
    bool reboundDone, seenPresent;
    uint32_t round2At;
    SensorStatus irStatus;
} sim;

static const char *Move(void *c, float d, float r, float cps) {
    (void)c;
    if (sim.active) return "T2_BUSY";
    sim.active = true; sim.braking = false;
    sim.distance = d; sim.radius = r; sim.speed = fminf(cps * MM_PER_COUNT, r != 0.0f ? 700.0f : 1e9f);
    sim.travelled = sim.turned = sim.v = 0.0f; sim.originYaw = sim.yaw;
    return NULL;
}
static const char *Straight(void *c, float d, float v) { return Move(c, d, 0.0f, v); }
static const char *Arc(void *c, float d, float r, float v) { return Move(c, d, r, v); }
static bool Busy(void *c) { (void)c; return sim.active; }
static void Brake(void *c) { (void)c; if (sim.active) sim.braking = true; }
static int speedRefused;
static const char *Speed(void *c, float cps) { (void)c; if (!sim.active || sim.braking) { ++speedRefused; return "T2_SPEED"; } sim.speed = fminf(sim.speed, cps * MM_PER_COUNT); return NULL; }
static const char *Continuous(void *c, float v) { (void)c; (void)v; return "REJECTED"; }
static Task2MotionReading Motion(void *c) {
    (void)c;
    float sgn = sim.distance >= 0.0f ? 1.0f : -1.0f;
    return (Task2MotionReading){sgn * sim.travelled, sim.yaw - sim.originYaw, sim.active, sim.active && sim.braking};
}
static UltrasonicReading Ultrasonic(void *c) {
    (void)c;
    return (UltrasonicReading){.sequence = sim.ussSeq, .capturedMs = sim.ussCaptured, .distanceMm = sim.ussMm,
        .status = (SensorStatus)sim.ussStatus};
}
static IRPairReading Infrared(void *c) {
    (void)c;
    IRPairReading r = {.sequence = sim.irSeq};
    r.left = (IRReading){.capturedMs = sim.irCaptured, .adc = sim.irLeft, .status = sim.irStatus};
    r.right = (IRReading){.capturedMs = sim.irCaptured, .adc = sim.irRight, .status = sim.irStatus};
    return r;
}
static Task2AppIo io = {.moveStraight = Straight, .moveArc = Arc, .busy = Busy, .brake = Brake,
    .requestSpeed = Speed, .moveContinuous = Continuous, .motion = Motion, .ultrasonic = Ultrasonic,
    .infrared = Infrared, .mmPerCount = MM_PER_COUNT, .decelerationMmps2 = 1700.0f};

static float RayBox(float ox, float oy, float dx, float dy, float x0, float x1, float y0, float y1) {
    float tmin = 0.0f, tmax = 1e9f;
    float o[2] = {ox, oy}, d[2] = {dx, dy}, lo[2] = {x0, y0}, hi[2] = {x1, y1};
    for (int i = 0; i < 2; ++i) {
        if (fabsf(d[i]) < 1e-9f) { if (o[i] < lo[i] || o[i] > hi[i]) return INFINITY; continue; }
        float t1 = (lo[i] - o[i]) / d[i], t2 = (hi[i] - o[i]) / d[i];
        if (t1 > t2) { float s = t1; t1 = t2; t2 = s; }
        tmin = fmaxf(tmin, t1); tmax = fminf(tmax, t2);
        if (tmin > tmax) return INFINITY;
    }
    return tmin;
}
/* Distance from the body rectangle to an axis-aligned box: corner samples both ways (enough for a test). */
static float BodyGap(float x0, float x1, float y0, float y1) {
    float best = INFINITY, c = cosf(sim.yaw), s = sinf(sim.yaw);
    for (int i = 0; i <= 10; ++i) for (int j = 0; j <= 4; ++j) {
        float f = -37.0f + 257.0f * i / 10.0f, l = -100.0f + 200.0f * j / 4.0f;
        float px = sim.x + f * c - l * s, py = sim.y + f * s + l * c;
        float dx = fmaxf(fmaxf(x0 - px, 0.0f), px - x1), dy = fmaxf(fmaxf(y0 - py, 0.0f), py - y1);
        best = fminf(best, dx == 0.0f && dy == 0.0f ? -1.0f : hypotf(dx, dy));
    }
    return best;
}
static void Physics(float dt) {
    if (!sim.active) return;
    float remaining = sim.radius != 0.0f ? fabsf(sim.distance) - sim.travelled : fabsf(sim.distance) - sim.travelled;
    if (sim.braking) sim.v = fmaxf(0.0f, sim.v - 1700.0f * dt);
    else sim.v = fminf(fminf(sim.speed, sim.v + 2330.0f * dt), sqrtf(fmaxf(0.0f, 2.0f * 1700.0f * remaining)));
    float ds = sim.v * dt;
    if (!sim.braking) ds = fminf(ds, fmaxf(0.0f, remaining));
    if (ds <= 0.0f && (sim.v <= 0.0f || remaining <= 0.0f)) { sim.active = false; return; }
    float sgn = sim.distance >= 0.0f ? 1.0f : -1.0f;
    float dq = sim.radius != 0.0f ? sgn * ds / sim.radius : 0.0f;
    float qm = sim.yaw + 0.5f * dq;
    sim.x += sgn * ds * cosf(qm); sim.y += sgn * ds * sinf(qm); sim.yaw += dq;
    sim.travelled += ds;
    float h = 0.5f * sim.c.width;
    sim.minO1 = fminf(sim.minO1, BodyGap(sim.o1x, sim.o1x + 100.0f, -50.0f, 50.0f));
    sim.minO2 = fminf(sim.minO2, BodyGap(sim.o2x, sim.o2x + 100.0f, -h, h));
}
static void Sensors(void) {
    float h = 0.5f * sim.c.width;
    if (sim.now % 70U == 0U) {
        float ox = sim.x + 220.0f * cosf(sim.yaw), oy = sim.y + 220.0f * sinf(sim.yaw), best = INFINITY;
        float back = -250.0f - 37.0f - 2.0f;
        for (int k = -4; k <= 4; ++k) {
            float a = sim.yaw + 7.5f * PI / 180.0f * (float)k / 4.0f, dx = cosf(a), dy = sinf(a);
            best = fminf(best, RayBox(ox, oy, dx, dy, sim.o1x, sim.o1x + 100.0f, -50.0f, 50.0f));
            best = fminf(best, RayBox(ox, oy, dx, dy, sim.o2x, sim.o2x + 100.0f, -h, h));
            best = fminf(best, RayBox(ox, oy, dx, dy, back - 30.0f, back, -300.0f, 300.0f));
        }
        bool dead = sim.c.ussDeadAtO2 && sim.x > sim.o1x;
        ++sim.ussSeq; sim.ussCaptured = sim.now;
        sim.ussStatus = dead || !(best < 2000.0f) ? SENSOR_TIMEOUT : SENSOR_OK;
        sim.ussMm = sim.ussStatus == SENSOR_OK ? best : 0.0f;
    }
    if (sim.now % 50U == 0U) {
        uint16_t adc[2];
        const Task2SensorMount *m[2] = {&task2AppSensorDefaults.leftIR, &task2AppSensorDefaults.rightIR};
        for (int i = 0; i < 2; ++i) {
            float ox = sim.x + m[i]->aheadMm * cosf(sim.yaw) - m[i]->leftMm * sinf(sim.yaw);
            float oy = sim.y + m[i]->aheadMm * sinf(sim.yaw) + m[i]->leftMm * cosf(sim.yaw);
            float a = sim.yaw + m[i]->yawDeg * PI / 180.0f;
            float t = RayBox(ox, oy, cosf(a), sinf(a), sim.o2x, sim.o2x + 100.0f, -h, h);
            bool hit = t <= 450.0f;
            bool behind = sim.x > sim.o2x + 100.0f;
            if (sim.c.ir == IR_ALWAYS_ABSENT) hit = false;
            /* Late back face: unseen until the sensor is past a quarter of the way from the first end. */
            if (sim.c.ir == IR_BACK_FACE_LATE && behind && (float)sim.c.arrow2 * oy > 0.25f * h) hit = false;
            adc[i] = hit ? 1500U : 250U;
        }
        /* Rebound: the O2-side IR reads present once more on the second absent sample after seeing the face. */
        int side = sim.c.arrow2 > 0 ? 1 : 0;
        if (sim.c.ir == IR_REBOUND) {
            if (adc[side] == 1500U) sim.seenPresent = true;
            else if (sim.seenPresent && !sim.reboundDone && ++sim.lossCalls == 2) { adc[side] = 1500U; sim.reboundDone = true; }
        }
        bool round2 = sim.round2At != 0U;
        if (sim.c.ir == IR_FROZEN && round2) return;
        bool bad = (sim.c.ir == IR_BAD && round2 && (!sim.c.badForMs || sim.now < sim.round2At + sim.c.badForMs)) ||
                   (sim.c.ir == IR_BAD_BEHIND_O2 && sim.x > sim.o2x + 100.0f) ||
                   (sim.c.ir == IR_BAD_LATE_BACK_LEG && sim.x > sim.o2x + 100.0f && (float)sim.c.arrow2 * sim.y < 300.0f);
        ++sim.irSeq;
        /* STALE: new sequence numbers on an old capture (a stuck acquisition); the others are reported as such. */
        if (bad && sim.c.badStatus == SENSOR_STALE) { sim.irStatus = SENSOR_OK; return; }
        sim.irStatus = bad ? sim.c.badStatus : SENSOR_OK;
        sim.irCaptured = sim.now;
        sim.irLeft = adc[0]; sim.irRight = adc[1];
    }
}

typedef struct { Task2AppStatus status; const char *fault; float minO1, minO2; float x, y; uint32_t ms; int speedRefused; char records[1024]; } Outcome;
static Outcome Run(Case c) {
    static Task2Wrap w;
    static Task2Commands t;
    memset(&sim, 0, sizeof(sim));
    sim.c = c; sim.x = -250.0f; sim.minO1 = sim.minO2 = INFINITY; sim.irStatus = SENSOR_OK;
    float entrance = -250.0f - 37.0f - 2.0f + 500.0f;          /* rear-flush start */
    sim.o1x = entrance + c.d1; sim.o2x = sim.o1x + 100.0f + c.gap;
    Task2Commands_Init(&t);
    Task2Commands_Begin(&t, 1U, (Task2CommandArrow)c.arrow1);
    assert(Task2Wrap_Start(&w, &io, 0U) == NULL);
    Task2AppStatus status = TASK2_APP_RUNNING;
    uint32_t approach2At = 0U;
    for (sim.now = 10U; sim.now < 90000U && status == TASK2_APP_RUNNING; sim.now += 10U) {
        for (int i = 0; i < 10; ++i) Physics(0.001f);
        Sensors();
        if (t.stage == TASK2_STAGE_APPROACH2 && !approach2At) approach2At = sim.now;
        if (t.stage == TASK2_STAGE_ROUND2 && !sim.round2At) sim.round2At = sim.now;
        if (approach2At && sim.now == approach2At + 700U) {
            char reply[TASK2COMMANDS_REPLY_SIZE];
            assert(Task2Commands_Receive(&t, c.arrow2 > 0 ? "A 1 2 L" : "A 1 2 R", reply));
        }
        status = Task2Wrap_Update(&w, &t, sim.now);
    }
    Outcome o = {status, w.fault, sim.minO1, sim.minO2, sim.x, sim.y, sim.now, speedRefused, {0}};
    speedRefused = 0;
    size_t n = 0;
    for (unsigned i = 0U; i < w.diag.count; ++i) {
        const Task2DiagRecord *r = &w.diag.records[(w.diag.head + i) % TASK2_DIAG_RECORDS];
        n += (size_t)snprintf(o.records + n, sizeof(o.records) - n, "%s:%s;", r->what, r->reason ? r->reason : "-");
    }
    return o;
}
static int failures;
static void Expect(bool ok, const char *name, const Outcome *o) {
    printf("%s %s  (O1 %.0f, O2 %.0f, end %.0f,%.0f, %.1f s%s) %s\n", ok ? "PASS" : "FAIL", name, o->minO1, o->minO2,
           o->x, o->y, (double)o->ms / 1000.0, o->speedRefused ? ", speed refused" : "", o->records);
    if (!ok) ++failures;
}

/* ---- the same robot through Task2Run, as MotionTask drives it ------------------------------------------ */
typedef enum { RUN_NORMAL, RUN_LATE_ARROW2, RUN_NO_ARROW1, RUN_STOP_IN_SWERVE, RUN_FAULT_GOING_HOME } RunScenario;
typedef struct {
    Task2RunStatus status; const char *fault; Task2Stage stage;
    char events[1024]; unsigned diagLines, traceLines, traceBlockLines;
    bool movedAfterStop; float minO1, minO2;
} RunOutcome;
static RunOutcome RunLevel(Case c, RunScenario scenario) {
    static Task2Run run;
    static char lines[TASK2_DIAG_PARTS][TASK2_DIAG_LINE_SIZE];
    RunOutcome o = {0};
    memset(&sim, 0, sizeof(sim));
    sim.c = c; sim.x = -250.0f; sim.minO1 = sim.minO2 = INFINITY; sim.irStatus = SENSOR_OK;
    float entrance = -250.0f - 37.0f - 2.0f + 500.0f;
    sim.o1x = entrance + c.d1; sim.o2x = sim.o1x + 100.0f + c.gap;
    Task2Run_Init(&run);
    assert(Task2Run_Start(&run, 1U, scenario == RUN_NO_ARROW1 ? TASK2_COMMAND_ARROW_NONE : (Task2CommandArrow)c.arrow1,
                          &io, 0U) == NULL);
    Task2RunStatus status = TASK2RUN_RUNNING;
    const char *fault = NULL;
    uint32_t approach2At = 0U, stoppedAt = 0U;
    size_t n = 0U;
    char line[TASK2COMMANDS_REPLY_SIZE], reply[TASK2COMMANDS_REPLY_SIZE];
    for (sim.now = 10U; sim.now < 90000U && status == TASK2RUN_RUNNING; sim.now += 10U) {
        for (int i = 0; i < 10; ++i) Physics(0.001f);
        Sensors();
        Task2Stage stage = run.commands.stage;
        if (stage == TASK2_STAGE_APPROACH2 && !approach2At) approach2At = sim.now;
        if (stage == TASK2_STAGE_ROUND2 && !sim.round2At) sim.round2At = sim.now;
        uint32_t arrowDelay = scenario == RUN_LATE_ARROW2 ? 3000U : 700U;
        if (approach2At && sim.now == approach2At + arrowDelay)
            assert(Task2Run_Receive(&run, c.arrow2 > 0 ? "A 1 2 L" : "A 1 2 R", reply) && strncmp(reply, "ACK", 3) == 0);
        if (scenario == RUN_NO_ARROW1 && sim.now == 2000U)
            assert(Task2Run_Receive(&run, c.arrow1 > 0 ? "A 1 1 L" : "A 1 1 R", reply) && strncmp(reply, "ACK", 3) == 0);
        if (scenario == RUN_STOP_IN_SWERVE && !stoppedAt && stage == TASK2_STAGE_SWERVE1 && sim.active) {
            Task2Run_Stop(&run); io.brake(io.context); stoppedAt = sim.now;
        }
        if (scenario == RUN_FAULT_GOING_HOME && stage == TASK2_STAGE_HOME && run.active) {
            Task2Run_FaultReason(&run, "MOTION_WATCHDOG"); io.brake(io.context);
            fault = "MOTION_WATCHDOG"; status = TASK2RUN_FAULT;
        }
        bool wasActive = sim.active;
        if (status == TASK2RUN_RUNNING) status = Task2Run_Update(&run, sim.active, sim.now, &fault);
        if (stoppedAt && !wasActive && sim.active) o.movedAfterStop = true;
        if (!sim.active) Task2Run_Stationary(&run);
        while (Task2Run_NextEvent(&run, line)) n += (size_t)snprintf(o.events + n, sizeof(o.events) - n, "%s", line);
        if (stoppedAt && sim.now > stoppedAt + 3000U) break;
    }
    for (int i = 0; i < 300; ++i) { Physics(0.01f); if (!sim.active) Task2Run_Stationary(&run); }
    while (Task2Run_NextEvent(&run, line)) n += (size_t)snprintf(o.events + n, sizeof(o.events) - n, "%s", line);
    o.status = status; o.fault = fault; o.stage = run.commands.stage; o.minO1 = sim.minO1; o.minO2 = sim.minO2;
    o.diagLines = Task2Run_Diagnostics(&run, "D T", lines, TASK2_DIAG_PARTS);
    o.traceLines = Task2Run_Trace(&run, "D R", lines, TASK2_DIAG_PARTS);
    o.traceBlockLines = Task2Run_Trace(&run, "D R 0", lines, TASK2_DIAG_PARTS);
    for (char *q = o.events; *q; ++q) if (*q == '\n') *q = '|';
    return o;
}
static void ExpectRun(bool ok, const char *name, const RunOutcome *o) {
    printf("%s run-level %s  (stage %s, diag %u, trace %u/%u) %s\n", ok ? "PASS" : "FAIL", name,
           Task2Commands_StageName(o->stage), o->diagLines, o->traceLines, o->traceBlockLines, o->events);
    if (!ok) ++failures;
}

int main(void) {
    static const int pairs[4][2] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    /* Every arrow pair at the minimum gap, narrow and the floor's O2: finishes, clears, parks centred. */
    for (int k = 0; k < 4; ++k) for (int wi = 0; wi < 2; ++wi) {
        Case c = {600.0f, 600.0f, wi ? 1060.0f : 300.0f, pairs[k][0], pairs[k][1], IR_NORMAL, false, SENSOR_OK, 0U};
        Outcome o = Run(c);
        char name[64]; snprintf(name, sizeof(name), "D1 600 G 600 W %.0f arrows %+d %+d", c.width, c.arrow1, c.arrow2);
        Expect(o.status == TASK2_APP_DONE && o.minO1 > 20.0f && o.minO2 > 20.0f && fabsf(o.y) < 30.0f &&
               !strstr(o.records, "EXHAUSTED") && !strstr(o.records, "NO_ECHO"), name, &o);
    }
    /* A narrow O2 the turn overshoots is re-acquired in reverse (both sides). */
    for (int s = -1; s <= 1; s += 2) {
        Outcome o = Run((Case){900.0f, 1000.0f, 300.0f, 1, s, IR_NORMAL, false, SENSOR_OK, 0U});
        Expect(o.status == TASK2_APP_DONE && strstr(o.records, "T2_IR_REACQUIRED") && o.minO2 > 20.0f,
               s > 0 ? "narrow O2 re-acquired, arrow 2 left" : "narrow O2 re-acquired, arrow 2 right", &o);
    }
    /* IR never sees O2: both searches are bounded and flagged, the run still finishes. */
    for (int s = -1; s <= 1; s += 2) {
        Outcome o = Run((Case){900.0f, 1000.0f, 1060.0f, 1, s, IR_ALWAYS_ABSENT, false, SENSOR_OK, 0U});
        Expect(o.status == TASK2_APP_DONE && strstr(o.records, "NEAR_END:T2_IR_EXHAUSTED") &&
               strstr(o.records, "FAR_END:T2_IR_EXHAUSTED"), s > 0 ? "IR always absent, left" : "IR always absent, right", &o);
    }
    /* Back face unseen at first: the loss is only taken after the face is acquired. */
    for (int s = -1; s <= 1; s += 2) {
        Outcome o = Run((Case){900.0f, 1000.0f, 1060.0f, -1, s, IR_BACK_FACE_LATE, false, SENSOR_OK, 0U});
        Expect(o.status == TASK2_APP_DONE && !strstr(o.records, "FAR_END:T2_IR_EXHAUSTED") && o.minO2 > 20.0f,
               s > 0 ? "back face late, left" : "back face late, right", &o);
    }
    /* A one-sample rebound during the near loss does not end the search early. */
    for (int s = -1; s <= 1; s += 2) {
        Outcome o = Run((Case){900.0f, 1000.0f, 1060.0f, 1, s, IR_REBOUND, false, SENSOR_OK, 0U});
        Expect(o.status == TASK2_APP_DONE && o.minO2 > 20.0f, s > 0 ? "rebound, left" : "rebound, right", &o);
    }
    /* Ultrasonic dead past O1 at the minimum gap: no forward sidestep into O2; nearest O2 assumed. */
    for (int k = 0; k < 4; ++k) {
        Outcome o = Run((Case){600.0f, 600.0f, 300.0f, pairs[k][0], pairs[k][1], IR_NORMAL, true, SENSOR_OK, 0U});
        Expect(o.status == TASK2_APP_DONE && o.minO2 > 0.0f && o.minO1 > 0.0f && strstr(o.records, "T2_O2_ASSUMED_NEAREST") &&
               !strstr(o.records, "SIDESTEP"), "USS dead at O2, minimum gap", &o);
    }
    /* Invalid IR from arrow 2 on, every status SensorTask reports, and a frozen sequence: the standing probe
     * gives up within its wait, the run is bounded and flagged, and it finishes parked. */
    static const SensorStatus badStatuses[] = {SENSOR_TIMEOUT, SENSOR_ERROR, SENSOR_OUT_OF_RANGE, SENSOR_STALE,
                                               SENSOR_SATURATED, SENSOR_NOT_READY};
    for (unsigned b = 0U; b <= sizeof(badStatuses) / sizeof(badStatuses[0]); ++b) for (int wi = 0; wi < 2; ++wi)
        for (int s = -1; s <= 1; s += 2) {
            bool frozen = b == sizeof(badStatuses) / sizeof(badStatuses[0]);
            Case c = {600.0f, 600.0f, wi ? 1060.0f : 300.0f, 1, s, frozen ? IR_FROZEN : IR_BAD, false,
                      frozen ? SENSOR_OK : badStatuses[b], 0U};
            Outcome o = Run(c);
            char name[96];
            snprintf(name, sizeof(name), "IR %s from arrow 2, W %.0f, arrow 2 %+d",
                     frozen ? "FROZEN" : SensorReading_StatusName(c.badStatus), c.width, s);
            Expect(o.status == TASK2_APP_DONE && strstr(o.records, "NEAR_END:T2_IR_NO_DATA") && o.ms < 40000U &&
                   o.minO1 > 0.0f && fabsf(o.y) < 30.0f, name, &o);
        }
    /* Invalid IR that recovers while the probe still waits: the same run as with a healthy IR. */
    for (int wi = 0; wi < 2; ++wi) for (int s = -1; s <= 1; s += 2) {
        Outcome o = Run((Case){600.0f, 600.0f, wi ? 1060.0f : 300.0f, -1, s, IR_BAD, false, SENSOR_TIMEOUT, 1300U});
        Expect(o.status == TASK2_APP_DONE && !strstr(o.records, "NO_DATA") && !strstr(o.records, "PROBE_SHORT") &&
               !strstr(o.records, "EXHAUSTED") && o.minO2 > 20.0f && fabsf(o.y) < 30.0f,
               wi ? "IR TIMEOUT 1.3 s then valid, W 1060" : "IR TIMEOUT 1.3 s then valid, W 300", &o);
    }
    /* Invalid IR from the back leg on: that search stops within its no-data time, flagged, and finishes. */
    for (int s = -1; s <= 1; s += 2) {
        Outcome o = Run((Case){600.0f, 600.0f, 1060.0f, 1, s, IR_BAD_BEHIND_O2, false, SENSOR_ERROR, 0U});
        Expect(o.status == TASK2_APP_DONE && strstr(o.records, "FAR_END:T2_IR_NO_DATA") &&
               !strstr(o.records, "NEAR_END:T2_IR") && o.minO2 > 0.0f && fabsf(o.y) < 30.0f,
               s > 0 ? "IR ERROR on the back leg, left" : "IR ERROR on the back leg, right", &o);
    }
    /* IR lost on the back leg just before its slow-down point: the no-data brake from cruise stops past that point,
     * and the run must not ask the braking or stopped controller to slow down (it refuses). */
    for (int wi = 0; wi < 2; ++wi) for (int s = -1; s <= 1; s += 2) {
        Outcome o = Run((Case){600.0f, 600.0f, wi ? 1060.0f : 600.0f, 1, s, IR_BAD_LATE_BACK_LEG, false, SENSOR_TIMEOUT, 0U});
        char name[96];
        snprintf(name, sizeof(name), "IR lost before the back-leg slow-down, W %.0f, arrow 2 %+d", wi ? 1060.0 : 600.0, s);
        Expect(o.status == TASK2_APP_DONE && strstr(o.records, "FAR_END:T2_IR_NO_DATA") && !o.speedRefused &&
               o.minO2 > 0.0f && fabsf(o.y) < 30.0f, name, &o);
    }
    /* Through Task2Run, as MotionTask drives it. */
    {
        RunOutcome o = RunLevel((Case){600.0f, 600.0f, 1060.0f, 1, -1, IR_NORMAL, false, SENSOR_OK, 0U}, RUN_NORMAL);
        ExpectRun(o.status == TASK2RUN_DONE && o.stage == TASK2_STAGE_DONE && strstr(o.events, "EV 1 SUMMARY") &&
                  strstr(o.events, "EV 1 ROUND 2") && o.diagLines >= 1U && o.traceLines == 1U && o.traceBlockLines >= 1U,
                  "normal, DONE then stationary reads", &o);
        o = RunLevel((Case){600.0f, 600.0f, 1060.0f, -1, 1, IR_NORMAL, false, SENSOR_OK, 0U}, RUN_LATE_ARROW2);
        ExpectRun(o.status == TASK2RUN_DONE && strstr(o.events, "EV 1 WAIT 2") && strstr(o.events, "EV 1 ROUND 2") &&
                  o.minO2 > 0.0f, "arrow 2 three seconds late: waits at O2, then continues", &o);
        o = RunLevel((Case){900.0f, 1000.0f, 300.0f, 1, 1, IR_NORMAL, false, SENSOR_OK, 0U}, RUN_NO_ARROW1);
        ExpectRun(o.status == TASK2RUN_DONE && strstr(o.events, "EV 1 WAIT 1") && o.minO1 > 0.0f,
                  "no arrow 1 at start: waits at O1, then continues", &o);
        o = RunLevel((Case){600.0f, 600.0f, 1060.0f, 1, 1, IR_NORMAL, false, SENSOR_OK, 0U}, RUN_STOP_IN_SWERVE);
        ExpectRun(o.stage == TASK2_STAGE_STOPPED && !o.movedAfterStop && o.status == TASK2RUN_RUNNING,
                  "S during the swerve: brakes, no further move, STOPPED when still", &o);
        o = RunLevel((Case){600.0f, 600.0f, 1060.0f, -1, -1, IR_NORMAL, false, SENSOR_OK, 0U}, RUN_FAULT_GOING_HOME);
        ExpectRun(o.stage == TASK2_STAGE_FAULT && o.diagLines >= 1U, "external fault going home: FAULT, records kept", &o);
    }
    printf(failures ? "%d FAILED\n" : "task2 wrap: PASS\n", failures);
    return failures != 0;
}
