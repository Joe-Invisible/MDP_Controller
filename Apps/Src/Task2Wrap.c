#include "Task2Wrap.h"

#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846f
#define DEG (PI / 180.0f)

/* Robot: rear axle to front (ultrasonic face) and tail. */
#define FRONT_MM 220.0f
#define TAIL_MM 37.0f
#define BODY_LENGTH_MM (FRONT_MM + TAIL_MM)
/* Joe's calibrated full-lock radius; nothing here uses another. */
#define RADIUS_MM 275.0f

/* Speeds. Cruise and arc speeds are the planner's (7000 counts/s); the IR speed is where the floor traces
 * show its end readings: about 40 mm late at 255 mm/s. */
#define CRUISE_CPS 7000.0f
#define APPROACH_CPS 3000.0f
#define IR_SEARCH_MMPS 400.0f
#define SETTLE_MMPS 300.0f

/* Distances. */
#define O1_STOP_MM 305.0f           /* ultrasonic to O1 before the swerve */
#define O2_STOP_MM 280.0f           /* ultrasonic to O2 before the 90 deg turn: its front corner clears by ~65 */
#define O1_TAIL_MIN_MM 75.0f        /* reversing never brings the tail closer to O1's back face */
#define END_SHORT_MM 50.0f          /* rear axle before each O2 end at its turn; walls are 500 past the ends */
#define LANE_MM 250.0f              /* home lane beside O1 when the final S cannot follow it directly */
#define O1_TAIL_PAST_MM 30.0f       /* the final S starts once the tail is this far past O1's face */
#define PARK_ENTRY_MARGIN_MM 50.0f  /* a single S must end with the front this far short of the car park */
#define PARK_DEPTH_MM 500.0f        /* rules: 60 x 50 cm inside; the smaller is taken for the depth */
#define RULE_MIN_GAP_MM 600.0f
#define SLOW_BEFORE_MM 300.0f       /* back leg: search speed this far before the predicted far end */
#define SEARCH_LIMIT_MM 2000.0f     /* longest O2 face search with no geometric limit */
#define SEARCH_OVERRUN_MM 150.0f    /* a limited search's move runs this far past its limit; it brakes at the limit */
#define BACKOFF_STEP_MM 50.0f
/* Back leg: the search gives up with the sensor this far past the far end predicted from the near one
 * (O2 is centred on the start line by the rules); a wall may stand 500 mm past the true end. */
#define FAR_SEARCH_PAST_MM 350.0f
#define RULE_MIN_LENGTH_MM 300.0f

/* Sensors. */
#define USS_NEAR_MM 300.0f
#define USS_NEAR_SHORT_MM 30.0f     /* floor traces: readings under 300 mm are about 30 mm short */
#define USS_LATENCY_S 0.05f         /* floor traces: about 30 mm long while closing at 600 mm/s */
#define GUARD_SHORT_MM 100.0f
#define GUARD_SAMPLES 3U
#define RANGE_WINDOW_MS 1000U
#define RANGE_MIN_ECHOES 3U
#define RANGE1_TRIES 3U
/* Rules: the car park to O1 and O1 to O2 are each 600-1500 mm. A standing range outside that, with these
 * margins for placement and sensor error, is a wrong echo. */
#define RULE_MAX_GAP_MM 1500.0f
#define RANGE_SHORT_MARGIN_MM 60.0f
#define RANGE_LONG_MARGIN_MM 150.0f
#define IR_LOSS_SPREAD_MM 25.0f     /* the beam's width keeps a face in view past its end */
#define IR_LAG_S 0.045f             /* half the sample period plus half the sensor's own update time */
#define IR_CONFIRM_SAMPLES 3U
#define IR_NO_DATA_MS 300U          /* six sample periods without a valid IR sample end a search */
#define IR_PROBE_WAIT_MS 1000U      /* the standing probe decides on what it has by then */
#define ARC_MISS_DEG 3.0f
#define ACTIVE_TIMEOUT_MS 120000U

static float Wrap(float a) {
    while (a > PI) a -= 2.0f * PI;
    while (a <= -PI) a += 2.0f * PI;
    return a;
}
static float Cps(const Task2Wrap *w, float mmps) { return mmps / w->io->mmPerCount; }
static float StartX(void) { return TASK2_ROUTE_START_X_MM; }
/* Car park entrance and back wall, from the start pose. */
static float EntranceX(void) {
    return TASK2_WRAP_START_REAR_FLUSH ? StartX() - TAIL_MM + PARK_DEPTH_MM : StartX() + FRONT_MM;
}
static float BackWallX(void) { return EntranceX() - PARK_DEPTH_MM; }

/* ---- decision records and trace ------------------------------------------------------------------------ */
static void Note(Task2Wrap *w, Task2DiagKind kind, const char *what, bool ok, const char *reason) {
#if TASK2_DIAGNOSTICS
    Task2DiagRecord *r = Task2Diag_Begin(&w->diag, kind);
    r->what = what; r->ok = ok; r->reason = reason;
    r->ms = w->previousMs; r->phase = (uint8_t)w->phase; r->step = (uint8_t)w->moveNext;
    r->xMm = w->pose.xMm; r->yMm = w->pose.yMm; r->yawDeg = w->pose.yawRad / DEG;
    r->arrow1 = w->arrow1 > 0 ? 'L' : w->arrow1 < 0 ? 'R' : '-';
    r->arrow2 = w->arrow2 > 0 ? 'L' : w->arrow2 < 0 ? 'R' : '-';
    r->obstacle1FaceMm = w->o1FaceXMm;
    r->gapMm = w->o2FaceXMm - w->o1FaceXMm - 100.0f;
    r->widthMm = fabsf(w->nearEndYMm - w->farEndYMm);
    r->observedWidthMm = r->widthMm;
    r->fromXMm = w->nearEndYMm; r->fromYMm = w->farEndYMm;   /* the two ends, belief y */
    r->ussRangeMm = w->lastUssMm; r->ussValid = isfinite(w->lastUssMm);
    r->ussStatus = w->lastUssStatus; r->ussSequence = w->lastUssSequence; r->ussCapturedMs = w->lastUssCapturedMs;
    r->irAdc[0] = w->irAdc[0]; r->irAdc[1] = w->irAdc[1];
    unsigned kept = w->irHistoryCount < TASK2_DIAG_IR_HISTORY ? w->irHistoryCount : TASK2_DIAG_IR_HISTORY;
    r->irHistoryCount = (uint8_t)kept;
    for (unsigned i = 0U; i < kept; ++i)
        r->irHistory[i] = w->irHistoryAdc[(w->irHistoryCount - kept + i) % TASK2_DIAG_IR_HISTORY];
    r->detected = isfinite(w->nearEndYMm);
    r->rewinds = (uint8_t)w->backoffSteps; r->extensions = (uint8_t)w->arcMisses;
    r->fallback = w->o2Assumed || w->nearExhausted || w->farExhausted;
    r->poseAdjusted = w->sideStepped; r->exhausted = w->nearExhausted || w->farExhausted;
    r->returnFallbackActive = !w->singleS;
    r->ussFaceMm = w->o2FaceXMm;
    Task2Diag_End(&w->diag, r);
#else
    (void)w; (void)kind; (void)what; (void)ok; (void)reason;
#endif
}

#if TASK2_DIAGNOSTICS
static int16_t Clamp16(float v) {
    return (int16_t)lroundf(fmaxf(-32767.0f, fminf(32767.0f, isfinite(v) ? v : 0.0f)));
}
static uint16_t IRValue(const IRReading *r, uint32_t now) {
    return SensorReading_IRStatus(r, now) == SENSOR_OK ? r->adc : 65535U;
}
static void TraceSample(Task2Wrap *w, uint32_t now, uint8_t state) {
    if (!w->io) return;
    const UltrasonicReading u = w->traceUss;
    const IRPairReading ir = w->traceIr;
    bool searching = w->phase == TASK2_WRAP_NEAR_SEARCH || w->phase == TASK2_WRAP_FAR_SEARCH;
    uint32_t t10 = (now - w->traceStartMs) / 10U;
    Task2TraceSample s = {
        .t10Ms = (uint16_t)(t10 > 65535U ? 65535U : t10),
        .xMm = Clamp16(w->pose.xMm), .yMm = Clamp16(w->pose.yMm),
        .yawCentiDeg = Clamp16(remainderf(w->pose.yawRad / DEG, 360.0f) * 100.0f),
        .phase = (uint8_t)w->phase, .step = (uint8_t)w->moveNext,
        .flags = (uint8_t)((w->phase == TASK2_WRAP_APPROACH1 || w->phase == TASK2_WRAP_PARK ? TASK2_TRACE_USS_ARMED : 0U) |
            (searching ? TASK2_TRACE_IR_ARMED : 0U) | (isfinite(w->nearEndYMm) ? TASK2_TRACE_DETECTED : 0U) |
            (searching && w->search.braking ? TASK2_TRACE_EDGE_BRAKE : 0U) |
            (w->guardBraked ? TASK2_TRACE_APPROACH_BRAKE : 0U) |
            (w->io->busy(w->io->context) ? TASK2_TRACE_MOVING : 0U)),
        .state = state,
        .ussMm = SensorReading_Status(&u, now) == SENSOR_OK && u.distanceMm < 65535.0f
            ? (uint16_t)lroundf(u.distanceMm) : 65535U,
        .irLeft = IRValue(&ir.left, now), .irRight = IRValue(&ir.right, now),
        .faceLevel = (uint16_t)(w->search.present ? 1U : 0U),
    };
    Task2Trace_Add(&w->trace, &s);
}
static void Trace(Task2Wrap *w, uint32_t now) {
    w->traceUss = w->io->ultrasonic(w->io->context);
    w->traceIr = w->io->infrared(w->io->context);
    if (now - w->traceLastMs < TASK2_TRACE_PERIOD_MS) return;
    w->traceLastMs = now;
    TraceSample(w, now, 0U);
}
#endif

static Task2AppStatus Fail(Task2Wrap *w, const char *reason) {
    if (!w->fault) {
        Note(w, TASK2_DIAG_FAULT, "FAIL", false, reason);
#if TASK2_DIAGNOSTICS
        TraceSample(w, w->previousMs, TASK2_TRACE_STATE_FINAL);
#endif
    }
    w->fault = reason;
    if (w->io) w->io->brake(w->io->context);
    return TASK2_APP_FAULT;
}

/* ---- pose ---------------------------------------------------------------------------------------------- */
static void RememberPose(Task2Wrap *w, uint32_t now) {
    w->history[w->historyHead].ms = now;
    w->history[w->historyHead].pose = w->pose;
    w->historyHead = (w->historyHead + 1U) % TASK2_APP_POSE_HISTORY;
    if (w->historyCount < TASK2_APP_POSE_HISTORY) ++w->historyCount;
}
/* The pose when a sensor sample was taken (sensors run on their own schedule). */
static bool CapturedPose(const Task2Wrap *w, uint32_t ms, Task2RoutePose *pose) {
    unsigned first = (w->historyHead + TASK2_APP_POSE_HISTORY - w->historyCount) % TASK2_APP_POSE_HISTORY;
    for (unsigned i = 0U; i < w->historyCount; ++i) {
        unsigned slot = (first + i) % TASK2_APP_POSE_HISTORY;
        int32_t age = (int32_t)(ms - w->history[slot].ms);
        if (age < 0) return false;
        if (i + 1U == w->historyCount) { *pose = w->history[slot].pose; return true; }
        unsigned next = (slot + 1U) % TASK2_APP_POSE_HISTORY;
        uint32_t span = w->history[next].ms - w->history[slot].ms;
        if ((uint32_t)age <= span) {
            float f = span ? (float)age / (float)span : 0.0f;
            Task2RoutePose p = w->history[slot].pose, q = w->history[next].pose;
            *pose = (Task2RoutePose){p.xMm + f * (q.xMm - p.xMm), p.yMm + f * (q.yMm - p.yMm),
                p.yawRad + f * (q.yawRad - p.yawRad)};
            return true;
        }
    }
    return false;
}
static bool UpdatePose(Task2Wrap *w) {
    if (!w->primitive) return true;
    Task2MotionReading r = w->io->motion(w->io->context);
    if (!isfinite(r.distanceMm) || !isfinite(r.yawRad)) return false;
    float ds = r.distanceMm - w->previousDistanceMm;
    float heading = w->motionOrigin.yawRad + 0.5f * (w->previousYawRad + r.yawRad);
    w->pose.xMm += ds * cosf(heading);
    w->pose.yMm += ds * sinf(heading);
    w->pose.yawRad = w->motionOrigin.yawRad + r.yawRad;
    w->pathMm += fabsf(ds);
    w->previousDistanceMm = r.distanceMm;
    w->previousYawRad = r.yawRad;
    return true;
}

/* ---- primitives ---------------------------------------------------------------------------------------- */
static void BeginPrimitive(Task2Wrap *w) {
    w->primitive = false;
    w->previousDistanceMm = w->previousYawRad = 0.0f;
    w->motionOrigin = w->pose;
    w->arcTargetRad = NAN;
}
static const char *Straight(Task2Wrap *w, float distanceMm, float speedMmps) {
    BeginPrimitive(w);
    if (fabsf(distanceMm) < 1.0f) return NULL;
    const char *fault = w->io->moveStraight(w->io->context, distanceMm, Cps(w, speedMmps));
    if (!fault) w->primitive = true;
    return fault;
}
/* Arc at the calibrated radius, turning (+1 anticlockwise, -1 clockwise) until the heading is targetRad. */
static const char *ArcTo(Task2Wrap *w, float targetRad, int turn) {
    BeginPrimitive(w);
    float angle = Wrap(targetRad - w->pose.yawRad);
    if (turn > 0 && angle < 0.0f) angle += 2.0f * PI;
    if (turn < 0 && angle > 0.0f) angle -= 2.0f * PI;
    if (fabsf(angle) > 1.75f * PI) angle = Wrap(angle);   /* a near-full turn is a heading error, not a request */
    if (fabsf(angle) < 0.5f * DEG) return NULL;
    w->arcTargetRad = targetRad;
    const char *fault = w->io->moveArc(w->io->context, RADIUS_MM * fabsf(angle),
                                       angle > 0.0f ? RADIUS_MM : -RADIUS_MM, CRUISE_CPS);
    if (!fault) w->primitive = true;
    return fault;
}
/* Two equal arcs (and a straight beyond 550 mm of offset) to the line y = targetY, keeping the heading. */
static float SSpan(float offsetMm) {
    float m = fabsf(offsetMm);
    float th = m <= 2.0f * RADIUS_MM ? acosf(1.0f - m / (2.0f * RADIUS_MM)) : 0.5f * PI;
    return 2.0f * RADIUS_MM * sinf(th);
}
static void Push(Task2Wrap *w, Task2WrapMove m) {
    if (w->moveCount < TASK2_WRAP_MOVES) w->moves[w->moveCount++] = m;
}
static void ClearMoves(Task2Wrap *w) { w->moveCount = w->moveNext = 0U; }
/* Expands an S into its arcs and straight, in place of moves[i]. */
static void ExpandS(Task2Wrap *w, unsigned i) {
    Task2WrapMove s = w->moves[i];
    float base = s.turn ? PI : 0.0f;
    float dy = s.value - w->pose.yMm, m = fabsf(dy);
    Task2WrapMove out[3]; unsigned n = 0U;
    if (m >= 5.0f) {
        float th = m <= 2.0f * RADIUS_MM ? acosf(1.0f - m / (2.0f * RADIUS_MM)) : 0.5f * PI;
        float straight = m <= 2.0f * RADIUS_MM ? 0.0f : m - 2.0f * RADIUS_MM;
        int first = (dy > 0.0f ? 1 : -1) * (s.turn ? -1 : 1);
        out[n++] = (Task2WrapMove){TASK2_WRAP_ARC_TO, (int8_t)first, base + (float)first * th, 0.0f};
        if (straight >= 1.0f) out[n++] = (Task2WrapMove){TASK2_WRAP_STRAIGHT, 0, straight, s.speedMmps};
        out[n++] = (Task2WrapMove){TASK2_WRAP_ARC_TO, (int8_t)-first, base, 0.0f};
    }
    unsigned tail = w->moveCount - i - 1U;
    if (w->moveCount - 1U + n > TASK2_WRAP_MOVES) n = 0U;
    memmove(&w->moves[i + n], &w->moves[i + 1U], tail * sizeof(Task2WrapMove));
    memcpy(&w->moves[i], out, n * sizeof(Task2WrapMove));
    w->moveCount = w->moveCount - 1U + n;
}
/* Launches the next queued move; returns false when the queue is done. */
static bool LaunchNext(Task2Wrap *w, const char **fault) {
    while (w->moveNext < w->moveCount) {
        Task2WrapMove *m = &w->moves[w->moveNext];
        if (m->kind == TASK2_WRAP_S_TO_Y) { ExpandS(w, w->moveNext); continue; }
        ++w->moveNext;
        switch (m->kind) {
        case TASK2_WRAP_ARC_TO: *fault = ArcTo(w, m->value, m->turn); break;
        case TASK2_WRAP_STRAIGHT: *fault = Straight(w, m->value, m->speedMmps); break;
        case TASK2_WRAP_TO_X: *fault = Straight(w, (m->value - w->pose.xMm) / cosf(w->pose.yawRad), m->speedMmps); break;
        default: *fault = "T2_STATE"; break;
        }
        if (*fault || w->primitive) return true;
    }
    return false;
}
/* A finished arc: note one that ended clearly off its heading (the next arc aims at an absolute heading). */
static void CheckArc(Task2Wrap *w) {
    if (!isfinite(w->arcTargetRad)) return;
    if (fabsf(Wrap(w->pose.yawRad - w->arcTargetRad)) > ARC_MISS_DEG * DEG) {
        ++w->arcMisses;
        Note(w, TASK2_DIAG_PLAN, "ARC_MISS", false, "T2_ARC_HEADING");
    }
    w->arcTargetRad = NAN;
}

/* ---- sensors ------------------------------------------------------------------------------------------- */
/* A fresh ultrasonic sample and the pose it was taken at; *echo is false for no echo / out of range. */
static bool FreshUSS(Task2Wrap *w, uint32_t now, float *rangeMm, bool *echo, Task2RoutePose *p) {
    UltrasonicReading r = w->io->ultrasonic(w->io->context);
    if (r.sequence == w->ultrasonicSequence) return false;
    w->ultrasonicSequence = r.sequence;
    SensorStatus status = SensorReading_Status(&r, now);
    w->lastUssStatus = (uint8_t)status; w->lastUssSequence = r.sequence; w->lastUssCapturedMs = r.capturedMs;
    if (status == SENSOR_NOT_READY || status == SENSOR_STALE || !CapturedPose(w, r.capturedMs, p)) return false;
    *echo = status == SENSOR_OK && isfinite(r.distanceMm) && r.distanceMm >= 20.0f && r.distanceMm <= 4000.0f;
    *rangeMm = *echo ? r.distanceMm : NAN;
    w->lastUssMm = *rangeMm;
    return true;
}
static void BeginRange(Task2Wrap *w, uint32_t now) {
    w->range = (Task2WrapRange){.deadlineMs = now + RANGE_WINDOW_MS, .active = true};
}
/* While the robot still moves, a standing range has not started: keep its whole window ahead. */
static void HoldRange(Task2Wrap *w, uint32_t now) {
    if (w->range.active) { w->range.count = 0U; w->range.deadlineMs = now + RANGE_WINDOW_MS; }
}
/* Standing range: 1 with the median of the window's echoes, -1 when fewer than RANGE_MIN_ECHOES came by the
 * deadline (a median of one or two is a single wild reading), 0 meanwhile. The window closes early once full. */
static int StandingRange(Task2Wrap *w, uint32_t now, float *medianMm) {
    float range; bool echo; Task2RoutePose p;
    if (FreshUSS(w, now, &range, &echo, &p) && echo && w->range.count < TASK2_WRAP_RANGE_SAMPLES)
        w->range.values[w->range.count++] = range;
    if (w->range.count < TASK2_WRAP_RANGE_SAMPLES && (int32_t)(now - w->range.deadlineMs) < 0) return 0;
    w->range.active = false;
    if (w->range.count < RANGE_MIN_ECHOES) return -1;
    float v[TASK2_WRAP_RANGE_SAMPLES];
    memcpy(v, w->range.values, w->range.count * sizeof(float));
    for (unsigned i = 1U; i < w->range.count; ++i) {
        float x = v[i]; unsigned j = i;
        while (j && v[j - 1U] > x) { v[j] = v[j - 1U]; --j; }
        v[j] = x;
    }
    *medianMm = v[w->range.count / 2U];
    return 1;
}
/* The IR on O2's side: right when going round O2's left. */
static const Task2SensorMount *EndIR(const Task2Wrap *w) {
    return w->arrow2 > 0 ? &task2AppSensorDefaults.rightIR : &task2AppSensorDefaults.leftIR;
}
/* A fresh sample from the O2-side IR: its ADC and the course y of the sensor when it was taken. */
static bool FreshIR(Task2Wrap *w, uint32_t now, uint16_t *adc, float *sensorY) {
    IRPairReading r = w->io->infrared(w->io->context);
    if (r.sequence == w->irSequence) return false;
    w->irSequence = r.sequence;
    w->irAdc[0] = r.left.adc; w->irAdc[1] = r.right.adc;
    IRReading s = w->arrow2 > 0 ? r.right : r.left;
    Task2RoutePose p;
    if (SensorReading_IRStatus(&s, now) != SENSOR_OK || !CapturedPose(w, s.capturedMs, &p)) return false;
    w->irHistoryAdc[w->irHistoryCount++ % TASK2_DIAG_IR_HISTORY] = s.adc;
    const Task2SensorMount *m = EndIR(w);
    *adc = s.adc;
    *sensorY = p.yMm + m->aheadMm * sinf(p.yawRad) + m->leftMm * cosf(p.yawRad);
    return true;
}
/* Present at or above the present level, absent at or below the absent level, unchanged between. */
static bool IRPresent(Task2Wrap *w, uint16_t adc) {
    if (adc >= task2AppSensorDefaults.irPresentAdc) w->search.present = true;
    else if (adc <= task2AppSensorDefaults.irAbsentAdc) w->search.present = false;
    return w->search.present;
}

/* ---- O2 face searches ---------------------------------------------------------------------------------- */
/* Sign of the robot's motion along y during the current search. */
static float SearchMotionY(const Task2Wrap *w) {
    return (float)w->search.direction * (sinf(w->pose.yawRad) >= 0.0f ? 1.0f : -1.0f);
}
static float IRSensorY(const Task2Wrap *w) {
    const Task2SensorMount *m = EndIR(w);
    return w->pose.yMm + m->aheadMm * sinf(w->pose.yawRad) + m->leftMm * cosf(w->pose.yawRad);
}
/* Starts a search moving `direction` (+1 forward) along the face for `want` (true: the face appears). It
 * gives up once the sensor passes limitIrY along its motion; the move itself reaches that limit plus a
 * braking margin, so it never ends first. With no limit (NAN) only the overall length bounds it. */
static const char *BeginSearch(Task2Wrap *w, bool want, int direction, float speedMmps, float slowAtY, float limitIrY) {
    w->search = (Task2WrapSearch){.want = want, .direction = (int8_t)direction, .slowAtYMm = slowAtY,
        .limitIrYMm = limitIrY, .firstIrYMm = NAN, .lastIrMs = w->previousMs};
    float length = isfinite(limitIrY) ? fminf(fabsf(limitIrY - IRSensorY(w)) + SEARCH_OVERRUN_MM, 2.0f * SEARCH_LIMIT_MM)
                                      : SEARCH_LIMIT_MM;
    return Straight(w, (float)direction * length, speedMmps);
}
/* Feeds the search; true once it has finished (found and braked, or ran out). */
static bool Search(Task2Wrap *w, uint32_t now, bool busy, const char **fault) {
    Task2WrapSearch *s = &w->search;
    if (!s->found && !s->exhausted && isfinite(s->limitIrYMm) &&
        (IRSensorY(w) - s->limitIrYMm) * SearchMotionY(w) >= 0.0f) {
        s->exhausted = s->braking = true;
        w->io->brake(w->io->context);
    }
    /* Only a running, unbraked move takes a new speed: the controller refuses one once braking or stopped. */
    if (busy && !s->found && !s->exhausted && !s->slowed && isfinite(s->slowAtYMm)) {
        float along = (w->pose.yMm - s->slowAtYMm) * sinf(w->pose.yawRad) * s->direction;
        if (along >= 0.0f && !w->io->motion(w->io->context).braking) {
            s->slowed = true;
            *fault = w->io->requestSpeed(w->io->context, Cps(w, IR_SEARCH_MMPS));
        }
    }
    uint16_t adc; float y;
    if (!s->found && !s->exhausted && FreshIR(w, now, &adc, &y)) {
        s->lastIrMs = now;
        bool present = IRPresent(w, adc);
        if (!s->want && !s->seenFace) {
            /* An end is only where a face, seen on three samples in a row, goes out of view. */
            s->faceRun = present ? s->faceRun + 1U : 0U;
            s->seenFace = s->faceRun >= IR_CONFIRM_SAMPLES;
        } else if (present == s->want) {
            if (++s->run == 1U) {
                s->firstIrYMm = y;
                s->firstSpeedMmps = s->slowed || !isfinite(s->slowAtYMm) ? IR_SEARCH_MMPS : CRUISE_CPS * w->io->mmPerCount;
            }
            if (s->run >= IR_CONFIRM_SAMPLES) {
                s->found = s->braking = true;
                w->io->brake(w->io->context);
            }
        } else {
            s->run = 0U; s->firstIrYMm = NAN;
        }
    }
    /* Driving blind is not searching: with no valid IR for a while, stop where the search is. */
    if (!s->found && !s->exhausted && now - s->lastIrMs > IR_NO_DATA_MS) {
        s->exhausted = s->braking = s->noData = true;
        w->io->brake(w->io->context);
    }
    return !busy;
}
/* The end's belief y from a finished search: the first sample of the confirming run, less the mean lag
 * (and, for a face going out of view, the beam's width) along the motion. NAN if the search ran out. */
static float SearchEnd(const Task2Wrap *w) {
    const Task2WrapSearch *s = &w->search;
    if (!s->found || !isfinite(s->firstIrYMm)) return NAN;
    float motionY = SearchMotionY(w);
    float late = s->firstSpeedMmps * IR_LAG_S + (s->want ? 0.0f : IR_LOSS_SPREAD_MM);
    return s->firstIrYMm - motionY * late;
}
/* After a search: put the rear axle END_SHORT before the end, on the side the robot is heading. */
static const char *SettleAtEnd(Task2Wrap *w, float endY) {
    float heading = sinf(w->pose.yawRad) >= 0.0f ? 1.0f : -1.0f;
    float target = endY - heading * END_SHORT_MM;
    return Straight(w, (target - w->pose.yMm) * heading, SETTLE_MMPS);
}

/* ---- phases -------------------------------------------------------------------------------------------- */
static void Enter(Task2Wrap *w, Task2WrapPhase phase, uint32_t now) {
    w->phase = phase;
    w->phaseStartedMs = now;
}
static void QueueSwerve(Task2Wrap *w) {
    float s = (float)w->arrow1;
    ClearMoves(w);
    Push(w, (Task2WrapMove){TASK2_WRAP_ARC_TO, (int8_t)w->arrow1, 57.0f * DEG * s, 0.0f});
    Push(w, (Task2WrapMove){TASK2_WRAP_STRAIGHT, 0, 50.0f, CRUISE_CPS * w->io->mmPerCount});
    Push(w, (Task2WrapMove){TASK2_WRAP_ARC_TO, (int8_t)-w->arrow1, -40.0f * DEG * s, 0.0f});
    Push(w, (Task2WrapMove){TASK2_WRAP_STRAIGHT, 0, 100.0f, CRUISE_CPS * w->io->mmPerCount});
    Push(w, (Task2WrapMove){TASK2_WRAP_ARC_TO, (int8_t)w->arrow1, 0.0f, 0.0f});
}
/* Furthest the robot may reverse before its tail comes within O1_TAIL_MIN of O1's back face. */
static float ReverseRoom(const Task2Wrap *w) {
    return fmaxf(0.0f, (w->pose.xMm - TAIL_MM) - (w->o1FaceXMm + 100.0f + O1_TAIL_MIN_MM));
}
/* Home: the final S starts once the tail is past O1; a single S from the home lane if it also ends before
 * the car park, otherwise a lane beside O1 first. */
static void QueueHome(Task2Wrap *w) {
    float cruise = CRUISE_CPS * w->io->mmPerCount;
    float sStartX = w->o1FaceXMm - O1_TAIL_PAST_MM - TAIL_MM;
    float singleEndFront = sStartX - SSpan(w->pose.yMm) - FRONT_MM;
    w->singleS = singleEndFront >= EntranceX() + PARK_ENTRY_MARGIN_MM;
    ClearMoves(w);
    if (!w->singleS)
        Push(w, (Task2WrapMove){TASK2_WRAP_S_TO_Y, 1, (w->pose.yMm >= 0.0f ? 1.0f : -1.0f) * LANE_MM, cruise});
    Push(w, (Task2WrapMove){TASK2_WRAP_TO_X, 0, sStartX, cruise});
    Push(w, (Task2WrapMove){TASK2_WRAP_S_TO_Y, 1, 0.0f, cruise});
    Note(w, TASK2_DIAG_PLAN, "HOME", true, w->singleS ? "T2_HOME_SINGLE_S" : "T2_HOME_LANE");
}
/* Rear axle x for the body centred in the car park depth, heading home. */
static float ParkX(void) {
    return BackWallX() + 0.5f * (PARK_DEPTH_MM - BODY_LENGTH_MM) + FRONT_MM;
}

static Task2AppStatus Finish(Task2Wrap *w, Task2Commands *t, uint32_t now) {
    Task2Commands_Summary(t, w->o1FaceXMm, w->o2FaceXMm, fabsf(w->nearEndYMm - w->farEndYMm), w->pathMm);
    Note(w, TASK2_DIAG_PLAN, "PARKED", true, NULL);
#if TASK2_DIAGNOSTICS
    TraceSample(w, now, TASK2_TRACE_STATE_FINAL);
#endif
    Enter(w, TASK2_WRAP_FINISHED, now);
    return TASK2_APP_DONE;
}

const char *Task2Wrap_Start(Task2Wrap *w, const Task2AppIo *io, uint32_t now) {
    memset(w, 0, sizeof(*w));
    w->io = io;
    w->pose.xMm = StartX();
    w->startFrontXMm = StartX() + FRONT_MM;
    w->o1RangeMm = w->o1FaceXMm = w->o2FaceXMm = w->nearEndYMm = w->farEndYMm = w->lastUssMm = NAN;
    w->arcTargetRad = NAN;
    w->singleS = true;
    w->phaseStartedMs = w->previousMs = now;
#if TASK2_DIAGNOSTICS
    Task2Diag_Init(&w->diag);
    Task2Trace_Init(&w->trace);
    w->traceStartMs = now;
    w->traceLastMs = now - TASK2_TRACE_PERIOD_MS;
#endif
    if (!io || !io->moveStraight || !io->moveArc || !io->busy || !io->brake || !io->requestSpeed ||
        !io->motion || !io->ultrasonic || !io->infrared || !isfinite(io->mmPerCount) || io->mmPerCount <= 0.0f)
        return "T2_IO";
    if (io->busy(io->context)) return "T2_BUSY";
    /* Every run requires new measurements, even if a reading was valid at START. */
    w->ultrasonicSequence = io->ultrasonic(io->context).sequence;
    w->irSequence = io->infrared(io->context).sequence;
    RememberPose(w, now);
    BeginRange(w, now);
    return NULL;
}
void Task2Wrap_NoteFault(Task2Wrap *w, const char *reason) {
    if (w->io && !w->fault && w->phase != TASK2_WRAP_FINISHED && w->phase != TASK2_WRAP_ABORTED) {
        Note(w, TASK2_DIAG_FAULT, "EXTERNAL", false, reason);
#if TASK2_DIAGNOSTICS
        TraceSample(w, w->previousMs, TASK2_TRACE_STATE_FINAL);
#endif
        w->fault = reason;
    }
}
void Task2Wrap_Abort(Task2Wrap *w) {
    if (w->io && !w->fault && w->phase != TASK2_WRAP_FINISHED && w->phase != TASK2_WRAP_ABORTED) {
        Note(w, TASK2_DIAG_STOP, "ABORT", false, NULL);
#if TASK2_DIAGNOSTICS
        TraceSample(w, w->previousMs, TASK2_TRACE_STATE_FINAL);
#endif
    }
    w->phase = TASK2_WRAP_ABORTED;
    w->primitive = false;
    w->io = NULL;
}

Task2AppStatus Task2Wrap_Update(Task2Wrap *w, Task2Commands *t, uint32_t now) {
    if (!w->io || w->phase == TASK2_WRAP_ABORTED) return TASK2_APP_FAULT;
    if (w->fault) return TASK2_APP_FAULT;
    if (w->phase == TASK2_WRAP_FINISHED) return TASK2_APP_DONE;
    if (w->phase != TASK2_WRAP_WAIT1 && w->phase != TASK2_WRAP_WAIT2) w->activeMs += now - w->previousMs;
    w->previousMs = now;
    if (w->activeMs >= ACTIVE_TIMEOUT_MS) return Fail(w, "T2_TIMEOUT");
    if (!UpdatePose(w)) return Fail(w, "ODOMETRY");
    RememberPose(w, now);
#if TASK2_DIAGNOSTICS
    Trace(w, now);
#endif
    if (!w->arrow1) w->arrow1 = (int)Task2Commands_Arrow(t, 1U);
    bool busy = w->io->busy(w->io->context);
    const char *fault = NULL;
    if (!busy && w->primitive) { w->primitive = false; CheckArc(w); }
    float cruise = CRUISE_CPS * w->io->mmPerCount;

    switch (w->phase) {
    case TASK2_WRAP_RANGE1: {
        float r; int got = StandingRange(w, now, &r);
        if (!got) break;
        float o1Min = EntranceX() + RULE_MIN_GAP_MM - w->startFrontXMm;
        if (got > 0 && (r < o1Min - RANGE_SHORT_MARGIN_MM ||
                        r > EntranceX() + RULE_MAX_GAP_MM - w->startFrontXMm + RANGE_LONG_MARGIN_MM)) {
            Note(w, TASK2_DIAG_PLAN, "RANGE1", false, "T2_O1_IMPLAUSIBLE");
            got = -1;
        }
        if (got < 0 && ++w->rangeTries < RANGE1_TRIES) {   /* standing still costs little; a wrong O1 costs the run */
            BeginRange(w, now);
            break;
        }
        if (got < 0) {
            /* Still no usable echo from O1: plan on the nearest O1 the rules allow; the guard finds a nearer one. */
            r = o1Min;
            Note(w, TASK2_DIAG_PLAN, "RANGE1", false, "T2_O1_NO_ECHO");
        }
        w->o1RangeMm = r;
        w->o1FaceXMm = w->startFrontXMm + r;
        if (got > 0) Note(w, TASK2_DIAG_PLAN, "RANGE1", true, NULL);
        w->guardRun = 0U;
        Enter(w, TASK2_WRAP_APPROACH1, now);
        fault = Straight(w, r - O1_STOP_MM, cruise);
        break;
    }
    case TASK2_WRAP_APPROACH1: {
        float range; bool echo; Task2RoutePose p;
        if (busy && !w->guardBraked && FreshUSS(w, now, &range, &echo, &p)) {
            float expected = w->o1FaceXMm - (p.xMm + FRONT_MM * cosf(p.yawRad));
            bool short_ = echo && range - cruise * USS_LATENCY_S < expected - GUARD_SHORT_MM;
            w->guardRun = short_ ? w->guardRun + 1U : 0U;
            if (w->guardRun >= GUARD_SAMPLES) {
                w->guardBraked = true;
                w->io->brake(w->io->context);
                Note(w, TASK2_DIAG_PLAN, "O1_GUARD", true, "T2_O1_NEARER");
            }
        }
        if (busy) break;
        if (w->guardBraked && !w->remeasured) { BeginRange(w, now); Enter(w, TASK2_WRAP_REMEASURE1, now); break; }
        Enter(w, TASK2_WRAP_WAIT1, now);
        break;
    }
    case TASK2_WRAP_REMEASURE1: {
        if (busy) { HoldRange(w, now); break; }
        w->remeasured = true;
        float r; int got = StandingRange(w, now, &r);
        if (!got) break;
        if (got > 0) {
            w->o1FaceXMm = w->pose.xMm + FRONT_MM + r;
            fault = Straight(w, r - O1_STOP_MM, APPROACH_CPS * w->io->mmPerCount);
        }
        Note(w, TASK2_DIAG_PLAN, "RANGE1_AGAIN", got > 0, got > 0 ? NULL : "T2_O1_NO_ECHO");
        Enter(w, TASK2_WRAP_APPROACH1, now);
        break;
    }
    case TASK2_WRAP_WAIT1:
        if (!w->arrow1) {
            if (t->stage != TASK2_STAGE_WAIT1) Task2Commands_SetStage(t, TASK2_STAGE_WAIT1);
            break;
        }
        Task2Commands_SetStage(t, TASK2_STAGE_SWERVE1);
        QueueSwerve(w);
        Enter(w, TASK2_WRAP_SWERVE, now);
        (void)LaunchNext(w, &fault);
        break;
    case TASK2_WRAP_SWERVE:
        if (busy) break;
        if (LaunchNext(w, &fault)) break;
        Task2Commands_SetStage(t, TASK2_STAGE_APPROACH2);
        BeginRange(w, now);
        Enter(w, TASK2_WRAP_RANGE2, now);
        break;
    case TASK2_WRAP_RANGE2: {
        if (busy) { HoldRange(w, now); break; }
        if (LaunchNext(w, &fault)) { HoldRange(w, now); break; }
        float r; int got = StandingRange(w, now, &r);
        if (!got) break;
        if (got > 0) {
            if (r < USS_NEAR_MM) r += USS_NEAR_SHORT_MM;
            float face = w->pose.xMm + (FRONT_MM + r) * cosf(w->pose.yawRad), o1Back = w->o1FaceXMm + 100.0f;
            if (face < o1Back + RULE_MIN_GAP_MM - RANGE_SHORT_MARGIN_MM ||
                face > o1Back + RULE_MAX_GAP_MM + RANGE_LONG_MARGIN_MM) {
                Note(w, TASK2_DIAG_PLAN, "RANGE2", false, "T2_O2_IMPLAUSIBLE");
                got = -1;
            }
        }
        if (got > 0) {
            w->o2FaceXMm = w->pose.xMm + (FRONT_MM + r) * cosf(w->pose.yawRad);
            Note(w, TASK2_DIAG_PLAN, "RANGE2", true, w->backoffSteps || w->sideStepped ? "T2_O2_REACQUIRED" : NULL);
        } else {
            /* No echo: back off while the tail has room, then step towards the start line if even the nearest
             * O2 leaves room ahead for it; otherwise plan on the nearest O2 the rules allow. */
            float nearestFace = w->o1FaceXMm + 100.0f + RULE_MIN_GAP_MM;
            if (w->backoffSteps < 4U && ReverseRoom(w) >= BACKOFF_STEP_MM) {
                ++w->backoffSteps;
                Note(w, TASK2_DIAG_PLAN, "O2_NO_ECHO", false, "T2_O2_BACKOFF");
                fault = Straight(w, -BACKOFF_STEP_MM, SETTLE_MMPS);
                BeginRange(w, now);
                break;
            }
            float roomAhead = nearestFace - (w->pose.xMm + FRONT_MM);
            if (!w->sideStepped && fabsf(w->pose.yMm) >= 20.0f && roomAhead >= SSpan(w->pose.yMm) + 80.0f) {
                w->sideStepped = true;
                Note(w, TASK2_DIAG_PLAN, "O2_NO_ECHO", false, "T2_O2_SIDESTEP");
                ClearMoves(w);
                Push(w, (Task2WrapMove){TASK2_WRAP_S_TO_Y, 0, 0.0f, cruise});
                (void)LaunchNext(w, &fault);
                BeginRange(w, now);
                break;
            }
            w->o2Assumed = true;
            w->o2FaceXMm = nearestFace;
            Note(w, TASK2_DIAG_PLAN, "O2_NO_ECHO", false, "T2_O2_ASSUMED_NEAREST");
        }
        float move = w->o2FaceXMm - (w->pose.xMm + FRONT_MM) - O2_STOP_MM;
        if (move < 0.0f) move = fmaxf(move, -ReverseRoom(w));
        Enter(w, TASK2_WRAP_STANDOFF, now);
        fault = Straight(w, move, 500.0f);
        break;
    }
    case TASK2_WRAP_STANDOFF:
        if (busy) break;
        if (!w->arrow2) w->arrow2 = (int)Task2Commands_Arrow(t, 2U);
        if (!w->arrow2) {
            if (t->stage != TASK2_STAGE_WAIT2) Task2Commands_SetStage(t, TASK2_STAGE_WAIT2);
            Enter(w, TASK2_WRAP_WAIT2, now);
            break;
        }
        /* fall through */
    case TASK2_WRAP_WAIT2:
        if (!w->arrow2) w->arrow2 = (int)Task2Commands_Arrow(t, 2U);
        if (!w->arrow2) break;
        Task2Commands_SetStage(t, TASK2_STAGE_ROUND2);
        Enter(w, TASK2_WRAP_TURN_OUT, now);
        fault = ArcTo(w, 0.5f * PI * (float)w->arrow2, w->arrow2);
        break;
    case TASK2_WRAP_TURN_OUT:
        if (busy) break;
        Enter(w, TASK2_WRAP_NEAR_PROBE, now);
        w->probeSamples = w->probePresent = 0U;
        break;
    case TASK2_WRAP_NEAR_PROBE: {
        /* Standing: is O2's face in view? Three valid samples decide, or whatever the wait gathered. */
        uint16_t adc; float y;
        if (FreshIR(w, now, &adc, &y)) {
            w->probePresent += adc >= task2AppSensorDefaults.irPresentAdc ? 1U : 0U;
            ++w->probeSamples;
        }
        bool timedOut = now - w->phaseStartedMs >= IR_PROBE_WAIT_MS;
        if (w->probeSamples < IR_CONFIRM_SAMPLES && !timedOut) break;
        if (!w->probeSamples) {
            /* No valid IR at all: searching blind finds nothing. Take the face as running past the sensor, which
             * puts the U-turn furthest from O2 (a face out of view would put it at the shortest legal O2). */
            w->nearExhausted = true;
            w->nearEndYMm = IRSensorY(w);
            Note(w, TASK2_DIAG_PLAN, "NEAR_END", false, "T2_IR_NO_DATA");
            Enter(w, TASK2_WRAP_NEAR_SETTLE, now);
            fault = SettleAtEnd(w, w->nearEndYMm);
            break;
        }
        if (timedOut) Note(w, TASK2_DIAG_PLAN, "NEAR_PROBE", false, "T2_IR_PROBE_SHORT");
        /* A tie (one of two) counts as in view, as with no data. */
        bool inView = w->probePresent * 2U >= w->probeSamples;
        w->nearReacquired = !inView;
        Enter(w, TASK2_WRAP_NEAR_SEARCH, now);
        /* Out of view: O2 is narrower than the turn's reach; back along it until the sensor sees it,
         * never past the start line. */
        fault = inView ? BeginSearch(w, false, +1, IR_SEARCH_MMPS, NAN, NAN)
                       : BeginSearch(w, true, -1, IR_SEARCH_MMPS, NAN, 0.0f);
        /* The probe saw the face here on most of its samples: that is the face seen, so an end can follow
         * at once (the IR is often right at the end after this turn). */
        w->search.seenFace = inView;
        w->search.present = inView;
        break;
    }
    case TASK2_WRAP_NEAR_SEARCH:
        if (!Search(w, now, busy, &fault)) break;
        w->nearEndYMm = SearchEnd(w);
        if (!isfinite(w->nearEndYMm)) {
            /* No end seen: a face never found is taken as the shortest O2 the rules allow; a face that never
             * went out of view ends where the sensor is now. */
            w->nearExhausted = true;
            w->nearEndYMm = w->search.want ? 0.5f * RULE_MIN_LENGTH_MM * (float)w->arrow2 : IRSensorY(w);
            Note(w, TASK2_DIAG_PLAN, "NEAR_END", false, w->search.noData ? "T2_IR_NO_DATA" : "T2_IR_EXHAUSTED");
        } else {
            Note(w, TASK2_DIAG_PLAN, "NEAR_END", true, w->nearReacquired ? "T2_IR_REACQUIRED" : NULL);
        }
        Enter(w, TASK2_WRAP_NEAR_SETTLE, now);
        fault = SettleAtEnd(w, w->nearEndYMm);
        break;
    case TASK2_WRAP_NEAR_SETTLE:
        if (busy) break;
        Enter(w, TASK2_WRAP_UTURN, now);
        fault = ArcTo(w, -0.5f * PI * (float)w->arrow2, -w->arrow2);
        break;
    case TASK2_WRAP_UTURN: {
        if (busy) break;
        /* The far end is expected opposite the near one about the start line; cruise until near it. */
        float a = EndIR(w)->aheadMm;
        float slowAt = -w->nearEndYMm + (float)w->arrow2 * (a + SLOW_BEFORE_MM);
        float toSlow = (w->pose.yMm - slowAt) * (float)w->arrow2;
        float limit = -w->nearEndYMm - (float)w->arrow2 * FAR_SEARCH_PAST_MM;
        Enter(w, TASK2_WRAP_FAR_SEARCH, now);
        fault = toSlow > 0.0f ? BeginSearch(w, false, +1, cruise, slowAt, limit)
                              : BeginSearch(w, false, +1, IR_SEARCH_MMPS, NAN, limit);
        break;
    }
    case TASK2_WRAP_FAR_SEARCH:
        if (!Search(w, now, busy, &fault)) break;
        w->farEndYMm = SearchEnd(w);
        if (!isfinite(w->farEndYMm)) {
            w->farExhausted = true;
            w->farEndYMm = -w->nearEndYMm;
            Note(w, TASK2_DIAG_PLAN, "FAR_END", false, w->search.noData ? "T2_IR_NO_DATA" : "T2_IR_EXHAUSTED");
        } else {
            Note(w, TASK2_DIAG_PLAN, "FAR_END", true, NULL);
        }
        Enter(w, TASK2_WRAP_FAR_SETTLE, now);
        fault = SettleAtEnd(w, w->farEndYMm);
        break;
    case TASK2_WRAP_FAR_SETTLE:
        if (busy) break;
        Enter(w, TASK2_WRAP_TURN_HOME, now);
        fault = ArcTo(w, PI, -w->arrow2);
        break;
    case TASK2_WRAP_TURN_HOME:
        if (busy) break;
        Task2Commands_SetStage(t, TASK2_STAGE_HOME);
        QueueHome(w);
        Enter(w, TASK2_WRAP_HOME, now);
        (void)LaunchNext(w, &fault);
        break;
    case TASK2_WRAP_HOME:
        if (busy) break;
        if (LaunchNext(w, &fault)) break;
        w->guardRun = 0U; w->guardBraked = false;
        Enter(w, TASK2_WRAP_PARK, now);
        fault = Straight(w, (ParkX() - w->pose.xMm) / cosf(w->pose.yawRad), cruise);
        break;
    case TASK2_WRAP_PARK: {
        float range; bool echo; Task2RoutePose p;
        if (busy && !w->guardBraked && FreshUSS(w, now, &range, &echo, &p)) {
            float expected = (p.xMm - FRONT_MM) - BackWallX();
            bool short_ = echo && range - cruise * USS_LATENCY_S < expected - GUARD_SHORT_MM;
            w->guardRun = short_ ? w->guardRun + 1U : 0U;
            if (w->guardRun >= GUARD_SAMPLES) {
                w->guardBraked = true;
                w->io->brake(w->io->context);
                Note(w, TASK2_DIAG_PLAN, "PARK_GUARD", true, "T2_BACK_WALL_NEARER");
            }
        }
        if (busy) break;
        if (w->guardBraked) { BeginRange(w, now); Enter(w, TASK2_WRAP_PARK_FINISH, now); break; }
        return Finish(w, t, now);
    }
    case TASK2_WRAP_PARK_FINISH: {
        /* The guard stopped the robot: finish on a standing range to the back wall, never closer than the
         * centred gap. */
        if (busy) { HoldRange(w, now); break; }
        if (w->range.active) {
            float r; int got = StandingRange(w, now, &r);
            if (!got) break;
            float gap = 0.5f * (PARK_DEPTH_MM - BODY_LENGTH_MM);
            if (got > 0 && r > gap + 5.0f) { fault = Straight(w, r - gap, SETTLE_MMPS); break; }
        }
        return Finish(w, t, now);
    }
    default:
        return Fail(w, "T2_STATE");
    }
    if (fault) return Fail(w, fault);
    return TASK2_APP_RUNNING;
}
