#include "Task2Wrap.h"

#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846f
#define DEG (PI / 180.0f)

/* Robot: rear axle to front (ultrasonic face) and tail. */
#define FRONT_MM 220.0f
#define TAIL_MM 37.0f
#define BODY_LENGTH_MM (FRONT_MM + TAIL_MM)
#define HALF_WIDTH_MM 100.0f
/* Joe's calibrated full-lock radius; nothing here uses another. */
#define RADIUS_MM 275.0f

/* Speeds. Cruise and arc speeds are the planner's (7000 counts/s); the IR speed is where the floor traces
 * show its end readings: about 40 mm late at 255 mm/s. */
#define CRUISE_CPS 7000.0f
#ifndef TASK2_WRAP_IR_SEARCH_MMPS
#define TASK2_WRAP_IR_SEARCH_MMPS 400.0f
#endif
#define IR_SEARCH_MMPS TASK2_WRAP_IR_SEARCH_MMPS
#define SETTLE_MMPS 300.0f

/* Distances. */
/* The three-arc swerve: out to SWERVE_OUT_DEG, back to -SWERVE_BACK_DEG, then square, from O1_STOP. */
#ifndef TASK2_WRAP_SWERVE_OUT_DEG
#define TASK2_WRAP_SWERVE_OUT_DEG 63.0f
#endif
#ifndef TASK2_WRAP_SWERVE_BACK_DEG
#define TASK2_WRAP_SWERVE_BACK_DEG 52.0f
#endif
#ifndef TASK2_WRAP_SWERVE_STOP_MM
#define TASK2_WRAP_SWERVE_STOP_MM 295.0f
#endif
#if TASK2_WRAP_SWERVE_ARCS == 3
#define O1_STOP_MM TASK2_WRAP_SWERVE_STOP_MM   /* ultrasonic to O1 before the swerve */
#else
#define O1_STOP_MM 305.0f
#endif
#define O2_STOP_MM 280.0f           /* ultrasonic to O2 before the 90 deg turn: its front corner clears by ~65 */
#define STANDOFF_TOLERANCE_MM 10.0f   /* the standoff re-range corrects anything further off than this */
/* The standoff correction may bring the tail this close to O1: the turn out never moves the tail back, and a
 * standoff short of O2_STOP costs the turn out its clearance to O2 directly. */
#define STANDOFF_TAIL_MIN_MM 40.0f
#define O1_TAIL_MIN_MM 75.0f        /* reversing never brings the tail closer to O1's back face */
#define END_SHORT_MM 50.0f          /* rear axle before each O2 end at its turn; walls are 500 past the ends */
#define SETTLE_WINDOW_BEFORE_MM 90.0f  /* a stop this far before an end needs no settle (the U-turn keeps O2 >= ~63 from */
#define SETTLE_WINDOW_AFTER_MM 15.0f   /* 100 before to the end itself, and the wall >= ~65) ... down to this far before */
#define LANE_MM 250.0f              /* home lane beside O1 when the final S cannot follow it directly */
#define O1_TAIL_PAST_MM 30.0f       /* the final S starts once the tail is this far past O1's face */
#define PARK_ENTRY_MARGIN_MM 50.0f  /* a single S must end with the front this far short of the car park */
#define PARK_DEPTH_MM 500.0f        /* rules: inside 60 cm across x 50 cm deep */
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
/* A range from before START: five echoes within PRESTART_SPAN, the newest within PRESTART_FRESH of START,
 * all within PRESTART_SPREAD of each other. */
#define PRESTART_SPAN_MS 600U
#define PRESTART_FRESH_MS 200U
#define PRESTART_SPREAD_MM 20.0f
/* Rules: the car park to O1 and O1 to O2 are each 600-1500 mm. A standing range outside that, with these
 * margins for placement and sensor error, is a wrong echo. */
#define RULE_MAX_GAP_MM 1500.0f
#define RANGE_SHORT_MARGIN_MM 60.0f
#define RANGE_LONG_MARGIN_MM 150.0f
/* O2 from the camera when the ultrasonic has no echo: the Pi's depth to O2's arrow along the lens axis (sent with
 * arrow 2, from photos at the stopped pose), the lens LENS_AHEAD in front of the rear axle. Waited for up to
 * CAMERA_WAIT after the swerve, and only while the robot has not moved since. */
#define LENS_AHEAD_MM 123.0f
/* O1: the ultrasonic is distrusted when it reads more than this beyond the camera (an echo past a missed O1). */
#define O1_CAMERA_MARGIN_MM 150.0f
#define O1_CAMERA_MARGIN_FRACTION 0.25f
#define O1_SHORT_STOP_MM 150.0f   /* then approach to this much short of O1_STOP and range again standing */
#define CAMERA_WAIT_MS 2000U
#define SIDE_RANGE_MAX_YAW_DEG 12.0f  /* mid-swerve: O2 ranged from beside O1 only while this square */
#define SIDE_RANGE_MIN_Y_MM 200.0f    /* ... and this far off the start line (the beam passes O1) */
#define SIDE_RANGE_MIN_MM 300.0f      /* raw range limits for it (below 300 the near bias applies) */
#define SIDE_RANGE_MAX_MM 1500.0f
#define SIDE_SHORT_MARGIN_MM 120.0f   /* a side range may put O2 this much nearer than the rules: it only brakes earlier */
#define SWERVE_O2_CLEAR_MM 40.0f      /* the last swerve arc brakes when the body's front comes this near O2 (only
                                       * then: braking leaves it off square, which costs the turn out clearance) */
#define SIDE_AGREE_MM 60.0f           /* side echoes agreeing this closely place O2 */
#define IR_LOSS_SPREAD_MM 25.0f     /* the beam's width keeps a face in view past its end */
#define IR_LAG_S 0.045f             /* half the sample period plus half the sensor's own update time */
#define IR_CONFIRM_SAMPLES 3U
#define IR_NO_DATA_MS 300U          /* six sample periods without a valid IR sample end a search */
#define IR_PROBE_WAIT_MS 1000U      /* the standing probe decides on what it has by then */
#define ARC_MISS_DEG 3.0f
/* Home on one diagonal S: the body keeps HOME_CLEAR from O1 (centred within O1_Y_SLACK) and from O2 (between its
 * measured ends, widened by O2_Y_SLACK), checked every HOME_STEP along the path. ARC_MMPS only ranks candidates. */
#define HOME_CLEAR_MM 60.0f
/* Into the car park at an angle: the body keeps CARPARK_PATH_CLEAR from its walls on the way in and ends centred
 * with CARPARK_END_CLEAR to every wall, turned at most ENTRY_MAX_DEG. */
#ifndef TASK2_WRAP_PARK_WIDTH_MM
#define TASK2_WRAP_PARK_WIDTH_MM 600.0f   /* rules: inside 60 cm across the opening (diagram) x 50 cm deep */
#endif
#define PARK_WIDTH_MM TASK2_WRAP_PARK_WIDTH_MM
#define BODY_CENTRE_AHEAD_MM (0.5f * (FRONT_MM - TAIL_MM))
#ifndef TASK2_WRAP_PARK_CLEAR_MM
#define TASK2_WRAP_PARK_CLEAR_MM 100.0f /* the angled entry's clearance from the car park's walls (accumulated pose
                                         * error is not modelled: 60 mm let a 2.3 m angled entry touch a wall) */
#endif
#define CARPARK_PATH_CLEAR_MM TASK2_WRAP_PARK_CLEAR_MM
#define CARPARK_END_CLEAR_MM 80.0f
#ifndef TASK2_WRAP_S_WALL_CLEAR_MM
#define TASK2_WRAP_S_WALL_CLEAR_MM 100.0f  /* a home S that finishes inside the car park keeps this from its walls */
#endif
#define S_WALL_CLEAR_MM TASK2_WRAP_S_WALL_CLEAR_MM
#define PARK_INSIDE_MARGIN_MM 80.0f  /* parked where the S ends only when this far inside every wall and the entrance */
#define DIAG_COARSE 4U                /* the S start search tries every 4th start first */
#define EXACT_END_SLACK_MM 3.0f      /* an S solved to end at the parking spot may end this far past it (float) */
#define MOVE_STOP_S 0.3f             /* floor traces: the stop between two moves (for ranking routes only) */
#define ENTRY_MAX_DEG 40.0f
/* Heading error the straight in can carry (start placement, gyro, arc ends): its sideways drift grows along the
 * straight, so the car park walls need CARPARK_PATH_CLEAR plus that drift at each point of it. */
#define ENTRY_HEADING_SLACK_DEG 0.75f
#define O1_Y_SLACK_MM 20.0f
#define O2_Y_SLACK_MM 30.0f
#define HOME_STEP_MM 20.0f
#define ARC_MMPS 700.0f
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
    return StartX() - TAIL_MM - (float)TASK2_WRAP_START_BACK_GAP_MM + PARK_DEPTH_MM;
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
/* Replaces queued move i with the n moves in out (none if they would not fit the queue). */
static void Splice(Task2Wrap *w, unsigned i, const Task2WrapMove *out, unsigned n) {
    unsigned tail = w->moveCount - i - 1U;
    if (w->moveCount - 1U + n > TASK2_WRAP_MOVES) n = 0U;
    memmove(&w->moves[i + n], &w->moves[i + 1U], tail * sizeof(Task2WrapMove));
    memcpy(&w->moves[i], out, n * sizeof(Task2WrapMove));
    w->moveCount = w->moveCount - 1U + n;
}
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
    Splice(w, i, out, n);
}
static bool AngledEntry(const Task2Wrap *w, float *headingOut);
static bool DiagonalHomeFrom(const Task2Wrap *w, float x0, float y0, float q0, float *thetaOut, float *lengthOut);
static bool DiagonalExact(const Task2Wrap *w, float x0, float y0, float q0, float *thetaOut, float *lengthOut);
static void QueueHome(Task2Wrap *w);
#define HOME_REPLANS 3U
/* Launches the next queued move; returns false when the queue is done. */
static bool LaunchNext(Task2Wrap *w, const char **fault) {
    while (w->moveNext < w->moveCount) {
        Task2WrapMove *m = &w->moves[w->moveNext];
        if (m->kind == TASK2_WRAP_S_TO_Y) { ExpandS(w, w->moveNext); continue; }
        if (m->kind == TASK2_WRAP_DIAG) {
            float theta, length, speed = m->speedMmps;
            float side = w->pose.yMm >= 0.0f ? 1.0f : -1.0f;
            float q0 = PI + Wrap(w->pose.yawRad - PI);
            if ((m->turn == 1 && DiagonalExact(w, w->pose.xMm, w->pose.yMm, q0, &theta, &length)) ||
                DiagonalHomeFrom(w, w->pose.xMm, w->pose.yMm, q0, &theta, &length)) {
                Task2WrapMove out[3] = {{TASK2_WRAP_ARC_TO, (int8_t)side, PI + side * theta, 0.0f},
                                        {TASK2_WRAP_STRAIGHT, 0, length, speed},
                                        {TASK2_WRAP_ARC_TO, (int8_t)-side, PI, 0.0f}};
                Splice(w, w->moveNext, out, 3U);
            } else {
                /* Plan home again; at the limit QueueHome takes the lane past O1, never an unchecked S across it. */
                if (w->replans < HOME_REPLANS) ++w->replans;
                Note(w, TASK2_DIAG_PLAN, "HOME", false, "T2_DIAG_REPLAN");
                QueueHome(w);
            }
            continue;
        }
        if (m->kind == TASK2_WRAP_ENTRY) {
            float q1;
            if (TASK2_WRAP_ANGLED_HOME && AngledEntry(w, &q1)) {
                *m = (Task2WrapMove){TASK2_WRAP_ARC_TO, (int8_t)(w->pose.yMm >= 0.0f ? 1 : -1), q1, 0.0f};
                w->angled = true;
                Note(w, TASK2_DIAG_PLAN, "ENTRY", true, "T2_PARK_ANGLED");
            } else if (m->turn == 1 && w->replans < HOME_REPLANS) {
                /* Queued before O1, where an S onto the start line could cross it: plan home again from here. */
                ++w->replans;
                Note(w, TASK2_DIAG_PLAN, "HOME", false, "T2_ENTRY_REPLAN");
                QueueHome(w);
            } else {
                *m = (Task2WrapMove){TASK2_WRAP_S_TO_Y, 1, 0.0f, m->speedMmps};
            }
            continue;
        }
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
static void NoteFace(Task2WrapSearch *s, uint16_t adc) { s->faceAdc[s->faceCount++ % TASK2_WRAP_FACE_SAMPLES] = adc; }
/* The face level an end drops from: the highest of the latest present samples. */
static uint16_t FaceLevel(const Task2WrapSearch *s) {
    uint16_t level = 0U;
    unsigned n = s->faceCount < TASK2_WRAP_FACE_SAMPLES ? s->faceCount : TASK2_WRAP_FACE_SAMPLES;
    for (unsigned i = 0U; i < n; ++i) if (s->faceAdc[i] > level) level = s->faceAdc[i];
    return level;
}
/* The wanted state on this sample. For an end, something beyond it (floor: 500-800 for a metre past O2) can hold
 * the IR between the two levels, so a sample under the present level and irFaceDropAdc below the face is gone
 * too. */
static bool SearchHit(Task2WrapSearch *s, uint16_t adc, bool present) {
    if (s->want) return present;
    uint16_t drop = task2AppSensorDefaults.irFaceDropAdc;
    bool dropped = drop && adc < task2AppSensorDefaults.irPresentAdc && adc + drop <= FaceLevel(s);
    if (adc >= task2AppSensorDefaults.irPresentAdc) NoteFace(s, adc);
    return !present || dropped;
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
            if (present) NoteFace(s, adc);
        } else if (SearchHit(s, adc, present)) {
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
/* Put the rear axle END_SHORT before an end, on the side the robot is heading. */
static const char *MoveToEnd(Task2Wrap *w, float endY, float speedMmps) {
    float heading = sinf(w->pose.yawRad) >= 0.0f ? 1.0f : -1.0f;
    float target = endY - heading * END_SHORT_MM;
    return Straight(w, (target - w->pose.yMm) * heading, speedMmps);
}
/* The settle before a turn at an O2 end; none when the end was measured and the axle is already where that turn
 * fits (between SETTLE_WINDOW_BEFORE and SETTLE_WINDOW_AFTER of the end along the motion; END_SHORT nominally). */
static const char *SettleAtEnd(Task2Wrap *w, float endY, bool measured) {
#if TASK2_WRAP_SETTLE_SKIP
    float heading = sinf(w->pose.yawRad) >= 0.0f ? 1.0f : -1.0f, along = (w->pose.yMm - endY) * heading;
    if (measured && along >= -SETTLE_WINDOW_BEFORE_MM && along <= -SETTLE_WINDOW_AFTER_MM) return NULL;
#else
    (void)measured;
#endif
    return MoveToEnd(w, endY, SETTLE_MMPS);
}

/* ---- phases -------------------------------------------------------------------------------------------- */
static void Enter(Task2Wrap *w, Task2WrapPhase phase, uint32_t now) {
    w->phase = phase;
    w->phaseStartedMs = now;
}
static void QueueSwerve(Task2Wrap *w) {
    float s = (float)w->arrow1;
    ClearMoves(w);
#if TASK2_WRAP_SWERVE_ARCS == 3
    Push(w, (Task2WrapMove){TASK2_WRAP_ARC_TO, (int8_t)w->arrow1, TASK2_WRAP_SWERVE_OUT_DEG * DEG * s, 0.0f});
    Push(w, (Task2WrapMove){TASK2_WRAP_ARC_TO, (int8_t)-w->arrow1, -TASK2_WRAP_SWERVE_BACK_DEG * DEG * s, 0.0f});
#else
    Push(w, (Task2WrapMove){TASK2_WRAP_ARC_TO, (int8_t)w->arrow1, 57.0f * DEG * s, 0.0f});
    Push(w, (Task2WrapMove){TASK2_WRAP_STRAIGHT, 0, 50.0f, CRUISE_CPS * w->io->mmPerCount});
    Push(w, (Task2WrapMove){TASK2_WRAP_ARC_TO, (int8_t)-w->arrow1, -40.0f * DEG * s, 0.0f});
    Push(w, (Task2WrapMove){TASK2_WRAP_STRAIGHT, 0, 100.0f, CRUISE_CPS * w->io->mmPerCount});
#endif
    Push(w, (Task2WrapMove){TASK2_WRAP_ARC_TO, (int8_t)w->arrow1, 0.0f, 0.0f});
}
/* O2's face from the side echoes: the mean of the largest group agreeing within SIDE_AGREE (the nearer group on a
 * tie), once two agree; NAN before. A single wild echo then moves nothing. */
static float SideFace(const Task2Wrap *w) {
    unsigned best = 0U; float value = NAN;
    for (unsigned i = 0U; i < w->o2SideCount; ++i) {
        unsigned n = 0U; float sum = 0.0f;
        for (unsigned j = 0U; j < w->o2SideCount; ++j)
            if (fabsf(w->o2SideFaces[j] - w->o2SideFaces[i]) <= SIDE_AGREE_MM) { ++n; sum += w->o2SideFaces[j]; }
        float mean = sum / (float)n;
        if (n > best || (n == best && mean < value)) { best = n; value = mean; }
    }
    return best >= 2U ? value : NAN;
}
/* Whether O2's face at x is within the rules' gap behind O1 (with the ranging margins). */
static bool O2Plausible(const Task2Wrap *w, float faceX) {
    float o1Back = w->o1FaceXMm + 100.0f;
    return faceX >= o1Back + RULE_MIN_GAP_MM - RANGE_SHORT_MARGIN_MM && faceX <= o1Back + RULE_MAX_GAP_MM + RANGE_LONG_MARGIN_MM;
}
/* O2's face x from the camera's depth to its arrow (on the start line), at the current pose; NAN if unusable. */
static float CameraFaceX(const Task2Wrap *w, float depthMm) {
    float c = cosf(w->pose.yawRad), s = sinf(w->pose.yawRad);
    if (!isfinite(depthMm) || depthMm <= 0.0f || !(c > 0.5f)) return NAN;
    float lensX = w->pose.xMm + LENS_AHEAD_MM * c, lensY = w->pose.yMm + LENS_AHEAD_MM * s;
    /* depth = (X - lensX) cos(yaw) + (0 - lensY) sin(yaw) for the arrow at (X, 0). */
    return lensX + (depthMm + lensY * s) / c;
}
/* Furthest the robot may reverse before its tail comes within marginMm of O1's back face. */
static float ReverseRoomTo(const Task2Wrap *w, float marginMm) {
    return fmaxf(0.0f, (w->pose.xMm - TAIL_MM) - (w->o1FaceXMm + 100.0f + marginMm));
}
static float ReverseRoom(const Task2Wrap *w) { return ReverseRoomTo(w, O1_TAIL_MIN_MM); }
/* Rear axle x for the body centred in the car park depth, heading home. */
static float ParkX(void) {
    return BackWallX() + 0.5f * (PARK_DEPTH_MM - BODY_LENGTH_MM) + FRONT_MM;
}
/* Range the ultrasonic should read into the car park from pose p: along its axis to the back wall, or to a side wall
 * where the axis meets one first. */
static float ParkRayMm(const Task2RoutePose *p) {
    float c = cosf(p->yawRad), s = sinf(p->yawRad), hw = 0.5f * PARK_WIDTH_MM;
    float fx = p->xMm + FRONT_MM * c, fy = p->yMm + FRONT_MM * s, t = INFINITY;
    if (c < -1e-3f) t = (fx - BackWallX()) / -c;
    float side = s > 1e-3f ? (hw - fy) / s : s < -1e-3f ? (-hw - fy) / s : INFINITY;
    if (isfinite(side) && fx + side * c <= EntranceX()) t = fminf(t, side);
    return t;
}
/* Distance from the robot's body at (x, y, yaw) to the box [x0, x1] x [y0, y1]; -1 when they overlap. Overlap is
 * a separating-axis test on both shapes' axes (a long box can cross the body with no corner of either inside the
 * other). Once they are apart, the nearest points are a corner of one and the other shape, so the corner distances
 * both ways are exact. */
static float BodyBoxGap(float x, float y, float yaw, float x0, float x1, float y0, float y1) {
    static const float cf[4] = {FRONT_MM, FRONT_MM, -TAIL_MM, -TAIL_MM};
    static const float cl[4] = {HALF_WIDTH_MM, -HALF_WIDTH_MM, -HALF_WIDTH_MM, HALF_WIDTH_MM};
    float c = cosf(yaw), s = sinf(yaw), px[4], py[4], bx[4] = {x0, x1, x1, x0}, by[4] = {y0, y0, y1, y1};
    for (unsigned i = 0U; i < 4U; ++i) { px[i] = x + cf[i] * c - cl[i] * s; py[i] = y + cf[i] * s + cl[i] * c; }
    const float axes[4][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}, {c, s}, {-s, c}};
    bool separated = false;
    for (unsigned a = 0U; a < 4U && !separated; ++a) {
        float pmin = INFINITY, pmax = -INFINITY, bmin = INFINITY, bmax = -INFINITY;
        for (unsigned i = 0U; i < 4U; ++i) {
            float p = px[i] * axes[a][0] + py[i] * axes[a][1], b = bx[i] * axes[a][0] + by[i] * axes[a][1];
            pmin = fminf(pmin, p); pmax = fmaxf(pmax, p); bmin = fminf(bmin, b); bmax = fmaxf(bmax, b);
        }
        separated = pmax < bmin || bmax < pmin;
    }
    if (!separated) return -1.0f;
    float best = INFINITY;
    for (unsigned i = 0U; i < 4U; ++i) {
        float dx = fmaxf(fmaxf(x0 - px[i], px[i] - x1), 0.0f), dy = fmaxf(fmaxf(y0 - py[i], py[i] - y1), 0.0f);
        best = fminf(best, hypotf(dx, dy));
        float rx = (bx[i] - x) * c + (by[i] - y) * s, ry = -(bx[i] - x) * s + (by[i] - y) * c;
        float ex = fmaxf(fmaxf(-TAIL_MM - rx, rx - FRONT_MM), 0.0f), ey = fmaxf(fmaxf(-HALF_WIDTH_MM - ry, ry - HALF_WIDTH_MM), 0.0f);
        best = fminf(best, hypotf(ex, ey));
    }
    return best;
}
/* Whether the body at axle (x, y), any heading, keeps `clear` from the box: every body point lies within BODY_REACH
 * of the axle. Lets the clearance checks skip the exact test for boxes that are far away (the STM is slow at it). */
#define BODY_REACH_MM 242.0f   /* > hypot(FRONT, HALF_WIDTH), the front corners */
static bool FarFromBox(float x, float y, float x0, float x1, float y0, float y1, float clear) {
    float dx = fmaxf(fmaxf(x0 - x, x - x1), 0.0f), dy = fmaxf(fmaxf(y0 - y, y - y1), 0.0f);
    return dx * dx + dy * dy >= (BODY_REACH_MM + clear) * (BODY_REACH_MM + clear);
}
static bool BoxClear(float x, float y, float yaw, float x0, float x1, float y0, float y1, float clear) {
    return FarFromBox(x, y, x0, x1, y0, y1, clear) || BodyBoxGap(x, y, yaw, x0, x1, y0, y1) >= clear;
}
/* Whether the body at a pose keeps HOME_CLEAR from O1 and O2. */
static bool HomeClear(const Task2Wrap *w, float x, float y, float yaw) {
    float o2lo = fminf(w->nearEndYMm, w->farEndYMm) - O2_Y_SLACK_MM, o2hi = fmaxf(w->nearEndYMm, w->farEndYMm) + O2_Y_SLACK_MM;
    return BoxClear(x, y, yaw, w->o1FaceXMm, w->o1FaceXMm + 100.0f, -50.0f - O1_Y_SLACK_MM, 50.0f + O1_Y_SLACK_MM, HOME_CLEAR_MM) &&
           BoxClear(x, y, yaw, w->o2FaceXMm, w->o2FaceXMm + 100.0f, o2lo, o2hi, HOME_CLEAR_MM);
}
/* Whether the body keeps `clear` from the car park's back wall and side walls. */
static bool CarParkClear(float x, float y, float yaw, float clear) {
    float back = BackWallX(), entrance = EntranceX(), hw = 0.5f * PARK_WIDTH_MM;
    return BoxClear(x, y, yaw, back - 30.0f, back, -hw - 30.0f, hw + 30.0f, clear) &&
           BoxClear(x, y, yaw, back, entrance, hw, hw + 30.0f, clear) &&
           BoxClear(x, y, yaw, back, entrance, -hw - 30.0f, -hw, clear);
}
/* Whether every corner of the body lies inside the car park with `margin` to each wall. */
static bool InsideCarPark(float x, float y, float yaw, float margin) {
    static const float cf[4] = {FRONT_MM, FRONT_MM, -TAIL_MM, -TAIL_MM};
    static const float cl[4] = {HALF_WIDTH_MM, -HALF_WIDTH_MM, -HALF_WIDTH_MM, HALF_WIDTH_MM};
    float c = cosf(yaw), s = sinf(yaw), hw = 0.5f * PARK_WIDTH_MM;
    for (unsigned i = 0U; i < 4U; ++i) {
        float px = x + cf[i] * c - cl[i] * s, py = y + cf[i] * s + cl[i] * c;
        if (px < BackWallX() + margin || px > EntranceX() - margin || fabsf(py) > hw - margin) return false;
    }
    return true;
}
/* Whether the body keeps HOME_CLEAR from O1 and O2 and, unless wallClear < 0, wallClear from the car park's walls. */
static bool PathClearWalls(const Task2Wrap *w, float x, float y, float yaw, float wallClear) {
    return HomeClear(w, x, y, yaw) && (wallClear < 0.0f || CarParkClear(x, y, yaw, wallClear));
}
static bool PathClear(const Task2Wrap *w, float x, float y, float yaw, bool carPark) {
    return PathClearWalls(w, x, y, yaw, carPark ? CARPARK_PATH_CLEAR_MM : -1.0f);
}
/* Samples an arc of signed radius r (positive anticlockwise) from heading q0 to q1, checking each pose (against the
 * car park's walls with wallClear unless it is negative). */
static bool ArcClearWalls(const Task2Wrap *w, float *x, float *y, float q0, float q1, float r, float wallClear) {
    unsigned n = (unsigned)ceilf(fabsf(r * (q1 - q0)) / HOME_STEP_MM);
    if (!n) n = 1U;
    float q = q0;
    for (unsigned i = 1U; i <= n; ++i) {
        float qn = q0 + (q1 - q0) * (float)i / (float)n;
        *x += r * (sinf(qn) - sinf(q)); *y -= r * (cosf(qn) - cosf(q));
        q = qn;
        if (!PathClearWalls(w, *x, *y, q, wallClear)) return false;
    }
    return true;
}
static bool ArcClearFor(const Task2Wrap *w, float *x, float *y, float q0, float q1, float r, bool carPark) {
    return ArcClearWalls(w, x, y, q0, q1, r, carPark ? CARPARK_PATH_CLEAR_MM : -1.0f);
}
static bool ArcClear(const Task2Wrap *w, float *x, float *y, float q0, float q1, float r) {
    return ArcClearFor(w, x, y, q0, q1, r, false);
}
/* Rear axle on the car park centre line through its middle for a body heading q, centred in the car park. */
static void EntryTarget(float q, float *ax, float *ay) {
    *ax = BackWallX() + 0.5f * PARK_DEPTH_MM - BODY_CENTRE_AHEAD_MM * cosf(q);
    *ay = -BODY_CENTRE_AHEAD_MM * sinf(q);
}
/* For an arc from (x0, y0, q0) to heading q1 (radius side * RADIUS), the target's offset across the straight that
 * follows (positive to its left) and along it. */
static void EntryOffsets(float x0, float y0, float q0, float q1, float side, float *across, float *along) {
    float r = side * RADIUS_MM, ex = x0 + r * (sinf(q1) - sinf(q0)), ey = y0 - r * (cosf(q1) - cosf(q0)), tx, ty;
    EntryTarget(q1, &tx, &ty);
    float c = cosf(q1), s = sinf(q1);
    *along = (tx - ex) * c + (ty - ey) * s;
    *across = -(tx - ex) * s + (ty - ey) * c;
}
/* Into the car park from (x0, y0) heading home: an arc towards the start line to heading q1, then a straight that
 * ends with the body centred in the car park. q1 is solved exactly (the straight passes through the target); taken
 * only if the arc and straight keep their clearances from O1, O2 and the car park's walls and the body ends inside
 * with CARPARK_END_CLEAR, turned at most ENTRY_MAX_DEG. Needs O2's ends (the clearance from it). */
static bool AngledEntryFrom(const Task2Wrap *w, float x0, float y0, float q0, float *headingOut) {
    if (!isfinite(w->nearEndYMm) || !isfinite(w->farEndYMm) || !isfinite(w->o1FaceXMm) || !isfinite(w->o2FaceXMm))
        return false;
    float side = y0 >= 0.0f ? 1.0f : -1.0f, prevAcross = NAN, prevDeg = 0.0f;
    for (float deg = 0.0f; deg <= ENTRY_MAX_DEG + 0.01f; deg += 1.0f) {
        float across, along;
        EntryOffsets(x0, y0, q0, PI + side * deg * DEG, side, &across, &along);
        bool crossed = isfinite(prevAcross) && ((prevAcross <= 0.0f) != (across <= 0.0f));
        prevAcross = across;
        float lo = prevDeg, hi = deg;
        prevDeg = deg;
        if (!crossed) continue;
        /* The straight passes through the target between lo and hi: bisect for the heading. */
        float aLo, alongLo;
        EntryOffsets(x0, y0, q0, PI + side * lo * DEG, side, &aLo, &alongLo);
        for (unsigned k = 0U; k < 20U; ++k) {
            float mid = 0.5f * (lo + hi), aMid, alongMid;
            EntryOffsets(x0, y0, q0, PI + side * mid * DEG, side, &aMid, &alongMid);
            if ((aMid <= 0.0f) == (aLo <= 0.0f)) { lo = mid; aLo = aMid; } else hi = mid;
        }
        float q1 = PI + side * 0.5f * (lo + hi) * DEG, tx, ty, a2;
        EntryOffsets(x0, y0, q0, q1, side, &a2, &along);
        if (fabsf(a2) > 2.0f || along < 0.0f) continue;
        EntryTarget(q1, &tx, &ty);
        float x = x0, y = y0, c = cosf(q1), s = sinf(q1);
        if (fabsf(q1 - q0) > 0.5f * DEG && !ArcClearFor(w, &x, &y, q0, q1, side * RADIUS_MM, true)) continue;
        bool clear = true;
        unsigned n = (unsigned)ceilf(along / HOME_STEP_MM);
        float drift = tanf(ENTRY_HEADING_SLACK_DEG * DEG);
        for (unsigned i = 1U; i <= n && clear; ++i) {
            float d = along * (float)i / (float)n, px = x + d * c, py = y + d * s;
            clear = HomeClear(w, px, py, q1) && CarParkClear(px, py, q1, CARPARK_PATH_CLEAR_MM + d * drift);
        }
        if (!clear || !InsideCarPark(tx, ty, q1, CARPARK_END_CLEAR_MM)) continue;
        *headingOut = q1;
        return true;
    }
    return false;
}
static bool AngledEntry(const Task2Wrap *w, float *headingOut) {
    return AngledEntryFrom(w, w->pose.xMm, w->pose.yMm, PI + Wrap(w->pose.yawRad - PI), headingOut);
}
/* The angled parking straight from the measured pose: to the target's projection on the current heading. True (with
 * its length) only if it keeps its clearances (the car park's walls with the drift allowance) and the body ends
 * inside with CARPARK_END_CLEAR - 20: an arc that ended off its heading can make it miss. */
static bool ParkStraightOk(const Task2Wrap *w, float *lengthOut) {
    float q = w->pose.yawRad, c = cosf(q), s = sinf(q), tx, ty;
    EntryTarget(q, &tx, &ty);
    float along = (tx - w->pose.xMm) * c + (ty - w->pose.yMm) * s, drift = tanf(ENTRY_HEADING_SLACK_DEG * DEG);
    if (along < 0.0f) return false;
    unsigned n = (unsigned)ceilf(along / HOME_STEP_MM);
    for (unsigned i = 1U; i <= n; ++i) {
        float d = along * (float)i / (float)n, px = w->pose.xMm + d * c, py = w->pose.yMm + d * s;
        if (!HomeClear(w, px, py, q) || !CarParkClear(px, py, q, CARPARK_PATH_CLEAR_MM + d * drift)) return false;
    }
    if (!InsideCarPark(w->pose.xMm + along * c, w->pose.yMm + along * s, q, CARPARK_END_CLEAR_MM - 20.0f)) return false;
    *lengthOut = along;
    return true;
}
/* The earliest x along the current heading (TO_X drives it, so y follows the heading too) from which the angled
 * entry fits, searched every two HOME_STEPs down to the car park; false if none. The arc itself is solved again from
 * the actual pose when the straight ends (ENTRY). */
static bool AngledEntryOnLane(const Task2Wrap *w, float *startX) {
    float q0 = PI + Wrap(w->pose.yawRad - PI), c = cosf(q0), s = sinf(q0), q1;
    if (!(c < -0.5f)) return false;
    for (float x = w->pose.xMm; x > EntranceX(); x -= 2.0f * HOME_STEP_MM) {
        float d = (x - w->pose.xMm) / c, y = w->pose.yMm + d * s;
        /* The straight to x must itself keep its clearance. */
        if (d > 0.0f && !PathClear(w, x, y, q0, true)) return false;
        if (AngledEntryFrom(w, x, y, q0, &q1)) { *startX = x; return true; }
    }
    return false;
}
/* Whether the S that ExpandS would make from (x0, y0, heading q0 near home) to the line y = targetY keeps HOME_CLEAR:
 * its first arc runs from the measured heading to ExpandS's absolute heading, as ArcTo will drive it. */
static bool SClear(const Task2Wrap *w, float x0, float y0, float q0, float targetY) {
    float dy = targetY - y0, m = fabsf(dy);
    if (m < 5.0f) return true;
    float th = m <= 2.0f * RADIUS_MM ? acosf(1.0f - m / (2.0f * RADIUS_MM)) : 0.5f * PI;
    float straight = m <= 2.0f * RADIUS_MM ? 0.0f : m - 2.0f * RADIUS_MM;
    float first = dy > 0.0f ? -1.0f : 1.0f, q1 = PI + first * th, x = x0, y = y0;
    if (!ArcClear(w, &x, &y, q0, q1, first * RADIUS_MM)) return false;
    unsigned n = (unsigned)ceilf(straight / HOME_STEP_MM);
    for (unsigned i = 1U; i <= n; ++i) {
        float d = straight * (float)i / (float)n;
        if (!HomeClear(w, x + d * cosf(q1), y + d * sinf(q1), q1)) return false;
    }
    x += straight * cosf(q1); y += straight * sinf(q1);
    return ArcClear(w, &x, &y, q1, PI, -first * RADIUS_MM);
}
/* The x the diagonal S of angle th spans for a lateral offset m, with its straight's length (negative: too steep). */
static float SSpanAt(float m, float th, float *lengthOut) {
    *lengthOut = (m - 2.0f * RADIUS_MM * (1.0f - cosf(th))) / sinf(th);
    return 2.0f * RADIUS_MM * sinf(th) + *lengthOut * cosf(th);
}
/* Whether the diagonal S of angle th and straight `length` from (x0, y0, q0) keeps its clearances (O1/O2 HOME_CLEAR,
 * car-park walls S_WALL_CLEAR) and ends no further than endSlack past the parking spot, with the straight in after it. */
/* As ArcClearWalls (the same poses, integrated the same way), checking only every stride-th one and the last. */
static bool ArcClearStride(const Task2Wrap *w, float *x, float *y, float q0, float q1, float r, unsigned stride) {
    unsigned n = (unsigned)ceilf(fabsf(r * (q1 - q0)) / HOME_STEP_MM);
    if (!n) n = 1U;
    float q = q0;
    for (unsigned i = 1U; i <= n; ++i) {
        float qn = q0 + (q1 - q0) * (float)i / (float)n;
        *x += r * (sinf(qn) - sinf(q)); *y -= r * (cosf(qn) - cosf(q));
        q = qn;
        if ((i % stride == 0U || i == n) && !PathClearWalls(w, *x, *y, q, S_WALL_CLEAR_MM)) return false;
    }
    return true;
}
static bool DiagonalClearStride(const Task2Wrap *w, float x0, float y0, float q0, float side, float th, float length,
                                float endSlack, unsigned stride) {
    float x = x0, y = y0, q1 = PI + side * th;
    if (!ArcClearStride(w, &x, &y, q0, q1, side * RADIUS_MM, stride)) return false;
    unsigned n = (unsigned)ceilf(length / HOME_STEP_MM);
    for (unsigned i = 1U; i <= n; ++i) {
        float d = length * (float)i / (float)n;
        if ((i % stride == 0U || i == n) && !PathClearWalls(w, x + d * cosf(q1), y + d * sinf(q1), q1, S_WALL_CLEAR_MM))
            return false;
    }
    x += length * cosf(q1); y += length * sinf(q1);
    if (!ArcClearStride(w, &x, &y, q1, PI, -side * RADIUS_MM, stride)) return false;
    /* From the measured heading the S can end past the parking spot: then it does not fit. */
    if (x < ParkX() - endSlack) return false;
    /* The straight in that follows runs along the start line, so O1 must be behind it by then. */
    float rest = fmaxf(0.0f, x - ParkX());
    unsigned m2 = (unsigned)ceilf(rest / HOME_STEP_MM);
    for (unsigned i = 1U; i <= m2; ++i)
        if ((i % stride == 0U || i == m2) && !PathClearWalls(w, x - rest * (float)i / (float)m2, y, PI, S_WALL_CLEAR_MM))
            return false;
    return true;
}
static bool DiagonalClear(const Task2Wrap *w, float x0, float y0, float q0, float side, float th, float length,
                          float endSlack) {
    /* Every 4th pose first: most candidates fail somewhere along the way and are rejected ~4x sooner (the STM spends
     * most of its planning time here); then every pose, the full check. */
    return DiagonalClearStride(w, x0, y0, q0, side, th, length, endSlack, 4U) &&
           DiagonalClearStride(w, x0, y0, q0, side, th, length, endSlack, 1U);
}
/* Home on one diagonal S from (x0, y0) heading q0 near home: an arc of theta towards the start line, a straight, an
 * arc back square on the start line, at the latest where the body parks (the S may finish inside the car park, with
 * S_WALL_CLEAR from its walls). The quickest whose path, and the straight in after it, keeps its clearances; false
 * when none does (both ends of O2 must be known). */
static bool DiagonalHomeFrom(const Task2Wrap *w, float x0, float y0, float q0, float *thetaOut, float *lengthOut) {
    if (!isfinite(w->nearEndYMm) || !isfinite(w->farEndYMm) || !isfinite(w->o1FaceXMm) || !isfinite(w->o2FaceXMm))
        return false;
    float side = y0 >= 0.0f ? 1.0f : -1.0f, m = fabsf(y0);
    float cruise = CRUISE_CPS * w->io->mmPerCount;
    /* Each theta's time follows from its geometry alone: check the clearances cheapest first and take the first that
     * clears (the same answer as checking all and taking the cheapest; the clearance checks are the slow part). */
    struct { float th, length, cost; } c[43];
    unsigned nc = 0U;
    for (int deg = 6; deg <= 90; deg += 2) {
        float th = (float)deg * DEG, length = (m - 2.0f * RADIUS_MM * (1.0f - cosf(th))) / sinf(th);
        if (length < 0.0f) continue;
        float endX = x0 - 2.0f * RADIUS_MM * sinf(th) - length * cosf(th);
        if (endX < ParkX()) continue;
        float cost = 2.0f * RADIUS_MM * th / ARC_MMPS + (length + (endX - ParkX())) / cruise;
        unsigned k = nc++;
        for (; k > 0U && c[k - 1U].cost > cost; --k) c[k] = c[k - 1U];   /* stable: equal costs keep theta order */
        c[k].th = th; c[k].length = length; c[k].cost = cost;
    }
    for (unsigned k = 0U; k < nc; ++k) {
        if (!DiagonalClear(w, x0, y0, q0, side, c[k].th, c[k].length, 0.0f)) continue;
        *thetaOut = c[k].th; *lengthOut = c[k].length;
        return true;
    }
    return false;
}
/* The diagonal S from (x0, y0, q0) whose second arc ends at the parking spot itself, with no straight in after it:
 * theta solved so the S spans exactly x0 - ParkX (bisection), then the same clearances. */
static bool DiagonalExact(const Task2Wrap *w, float x0, float y0, float q0, float *thetaOut, float *lengthOut) {
    if (!isfinite(w->nearEndYMm) || !isfinite(w->farEndYMm) || !isfinite(w->o1FaceXMm) || !isfinite(w->o2FaceXMm))
        return false;
    float side = y0 >= 0.0f ? 1.0f : -1.0f, m = fabsf(y0), want = x0 - ParkX(), prevF = NAN, prevTh = 0.0f;
    for (int deg = 6; deg <= 90; deg += 2) {
        float th = (float)deg * DEG, length, f = SSpanAt(m, th, &length) - want;
        if (length < 0.0f) { prevF = NAN; continue; }
        if (isfinite(prevF) && (prevF > 0.0f) != (f > 0.0f)) {
            float lo = prevTh, hi = th, flo = prevF;
            for (unsigned i = 0U; i < 24U; ++i) {
                float mid = 0.5f * (lo + hi), lm, fm = SSpanAt(m, mid, &lm) - want;
                if ((fm > 0.0f) == (flo > 0.0f)) { lo = mid; flo = fm; } else hi = mid;
            }
            float t = 0.5f * (lo + hi), len;
            (void)SSpanAt(m, t, &len);
            if (len >= 0.0f && DiagonalClear(w, x0, y0, q0, side, t, len, EXACT_END_SLACK_MM)) {
                *thetaOut = t; *lengthOut = len;
                return true;
            }
        }
        prevF = f; prevTh = th;
    }
    return false;
}
/* The first x along the current heading (TO_X drives it) from which one diagonal S fits, searched every two
 * HOME_STEPs towards the car park; the straight to it must keep its clearance too. */
static bool DiagonalHome(const Task2Wrap *w, float *startX, float *thetaOut, float *lengthOut) {
    float q0 = PI + Wrap(w->pose.yawRad - PI), c = cosf(q0), s = sinf(q0), step = 2.0f * HOME_STEP_MM;
    if (!(c < -0.5f)) return false;
    /* Starts are tried every DIAG_COARSE steps first (each costs the STM up to 43 S checks); once one fits, the steps
     * just before it are tried, so the earliest start that fits near there is taken. */
    float lastMiss = NAN;
    unsigned k = 0U;
    for (float x = w->pose.xMm; x > EntranceX(); x -= step, ++k) {
        float d = (x - w->pose.xMm) / c, y = w->pose.yMm + d * s;
        if (d > 0.0f && !PathClear(w, x, y, q0, false)) return false;
        if (k % DIAG_COARSE != 0U) continue;
        if (!DiagonalHomeFrom(w, x, y, q0, thetaOut, lengthOut)) { lastMiss = x; continue; }
        for (float xf = isfinite(lastMiss) ? lastMiss - step : x; xf > x + 0.5f * step; xf -= step) {
            float df = (xf - w->pose.xMm) / c;
            float th, len;
            if (DiagonalHomeFrom(w, xf, w->pose.yMm + df * s, q0, &th, &len)) {
                *startX = xf; *thetaOut = th; *lengthOut = len;
                return true;
            }
        }
        *startX = x;
        return true;
    }
    return false;
}
/* Along the heading (TO_X) to where a diagonal S lands exactly at the parking spot, the quickest that fits; false if
 * none does. Each theta fixes the S's span, so its start: no scan of starts, and no straight in after it. */
static bool ExactHome(const Task2Wrap *w, float *startX, float *thetaOut, float *lengthOut) {
    float q0 = PI + Wrap(w->pose.yawRad - PI), c = cosf(q0), s = sinf(q0), cruise = CRUISE_CPS * w->io->mmPerCount;
    if (!(c < -0.5f)) return false;
    struct { float x0, cost; } cand[43];
    unsigned nc = 0U;
    for (int deg = 6; deg <= 90; deg += 2) {
        float th = (float)deg * DEG, x0 = w->pose.xMm, length = -1.0f;
        for (unsigned it = 0U; it < 4U; ++it) {   /* the start's y follows the heading, and the span follows y */
            float y = w->pose.yMm + (x0 - w->pose.xMm) / c * s;
            x0 = ParkX() + SSpanAt(fabsf(y), th, &length);
        }
        if (length < 0.0f || x0 > w->pose.xMm + 1.0f) continue;
        float d = (x0 - w->pose.xMm) / c;
        float cost = d / cruise + 2.0f * RADIUS_MM * th / ARC_MMPS + length / cruise + (d > 1.0f ? MOVE_STOP_S : 0.0f);
        unsigned k = nc++;
        for (; k > 0U && cand[k - 1U].cost > cost; --k) cand[k] = cand[k - 1U];
        cand[k].x0 = x0; cand[k].cost = cost;
    }
    for (unsigned k = 0U; k < nc; ++k) {
        float x0 = cand[k].x0, d = (x0 - w->pose.xMm) / c, y0 = w->pose.yMm + d * s;
        bool clear = true;
        unsigned n = (unsigned)ceilf(d / (2.0f * HOME_STEP_MM));
        for (unsigned i = 1U; i <= n && clear; ++i) {
            float t = d * (float)i / (float)n;
            clear = PathClear(w, w->pose.xMm + t * c, w->pose.yMm + t * s, q0, false);
        }
        if (clear && DiagonalExact(w, x0, y0, q0, thetaOut, lengthOut)) { *startX = x0; return true; }
    }
    return false;
}
/* Home: one diagonal S when it fits. Otherwise the final S starts once the tail is past O1; a single S from the
 * home lane if it also ends before the car park, otherwise a lane beside O1 first. */
static void QueueHome(Task2Wrap *w) {
    float cruise = CRUISE_CPS * w->io->mmPerCount;
    float theta, length;
    w->angled = false;
    float laneX;
    if (TASK2_WRAP_ANGLED_HOME && w->replans < HOME_REPLANS && AngledEntryOnLane(w, &laneX)) {
        /* Along the heading to where one arc and a straight end in the car park; the arc is solved again from where
         * the straight actually ends. */
        ClearMoves(w);
        if (laneX < w->pose.xMm - 1.0f) Push(w, (Task2WrapMove){TASK2_WRAP_TO_X, 0, laneX, cruise});
        Push(w, (Task2WrapMove){TASK2_WRAP_ENTRY, 1, 0.0f, cruise});
        w->singleS = true;
        Note(w, TASK2_DIAG_PLAN, "HOME", true, "T2_HOME_ANGLED");
        return;
    }
    float sX = w->pose.xMm;
    bool exact = TASK2_WRAP_EXACT_S && w->replans < HOME_REPLANS && ExactHome(w, &sX, &theta, &length);
    if (exact || (w->replans < HOME_REPLANS && DiagonalHome(w, &sX, &theta, &length))) {
        float side = w->pose.yMm >= 0.0f ? 1.0f : -1.0f;
        ClearMoves(w);
        if (sX < w->pose.xMm - 1.0f) {
            /* The S is solved again where the straight actually ends (turn 1: to land exactly on the spot). */
            Push(w, (Task2WrapMove){TASK2_WRAP_TO_X, 0, sX, cruise});
            Push(w, (Task2WrapMove){TASK2_WRAP_DIAG, (int8_t)(exact ? 1 : 0), 0.0f, cruise});
        } else {
            Push(w, (Task2WrapMove){TASK2_WRAP_ARC_TO, (int8_t)side, PI + side * theta, 0.0f});
            Push(w, (Task2WrapMove){TASK2_WRAP_STRAIGHT, 0, length, cruise});
            Push(w, (Task2WrapMove){TASK2_WRAP_ARC_TO, (int8_t)-side, PI, 0.0f});
        }
        w->singleS = true;
        Note(w, TASK2_DIAG_PLAN, "HOME", true, exact ? "T2_HOME_EXACT_S" : "T2_HOME_DIAGONAL");
        return;
    }
    float sStartX = w->o1FaceXMm - O1_TAIL_PAST_MM - TAIL_MM;
    float singleEndFront = sStartX - SSpan(w->pose.yMm) - FRONT_MM;
    w->singleS = singleEndFront >= EntranceX() + PARK_ENTRY_MARGIN_MM;
    ClearMoves(w);
    if (!w->singleS) {
        /* The lane beside O1: the nearest of LANE, LANE + 50, LANE + 100 whose S keeps its clearance from O1. */
        float side = w->pose.yMm >= 0.0f ? 1.0f : -1.0f, lane = LANE_MM;
        float q0 = PI + Wrap(w->pose.yawRad - PI);
        for (unsigned k = 0U; k < 3U && !SClear(w, w->pose.xMm, w->pose.yMm, q0, side * (LANE_MM + 50.0f * (float)k)); ++k)
            lane = LANE_MM + 50.0f * (float)(k + 1U);
        if (lane > LANE_MM + 100.0f) { lane = LANE_MM; Note(w, TASK2_DIAG_PLAN, "HOME_LANE", false, "T2_LANE_TIGHT"); }
        Push(w, (Task2WrapMove){TASK2_WRAP_S_TO_Y, 1, side * lane, cruise});
    }
    Push(w, (Task2WrapMove){TASK2_WRAP_TO_X, 0, sStartX, cruise});
    Push(w, (Task2WrapMove){TASK2_WRAP_ENTRY, 2, 0.0f, cruise});
    Note(w, TASK2_DIAG_PLAN, "HOME", true, w->singleS ? "T2_HOME_SINGLE_S" : "T2_HOME_LANE");
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
    w->prestart = (Task2WrapPrestartRange){.medianMm = NAN, .spreadMm = NAN};
    w->cameraO1Mm = w->o2SideFaceXMm = NAN;
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
    /* Every run requires new measurements, even if a reading was valid at START; the one exception is O1's
     * range from before START (Task2Wrap_UsePrestart), which is checked for age and steadiness. */
    w->ultrasonicSequence = io->ultrasonic(io->context).sequence;
    w->irSequence = io->infrared(io->context).sequence;
    RememberPose(w, now);
    BeginRange(w, now);
    return NULL;
}
void Task2Wrap_UseCameraO1(Task2Wrap *w, unsigned depthMm) {
    if (w->phase == TASK2_WRAP_RANGE1 && !w->fault && w->io && depthMm > 0U)
        w->cameraO1Mm = (float)depthMm - (FRONT_MM - LENS_AHEAD_MM);
}
/* Whether an ultrasonic range to O1 (from the front, mm) is so much further than the camera's that it is likely an echo
 * from beyond a missed O1. */
static bool O1BeyondCamera(float rangeMm, float cameraMm) {
    return isfinite(cameraMm) && cameraMm > 0.0f &&
           (!isfinite(rangeMm) || rangeMm > cameraMm + fmaxf(O1_CAMERA_MARGIN_MM, O1_CAMERA_MARGIN_FRACTION * cameraMm));
}
void Task2Wrap_UsePrestart(Task2Wrap *w, const Task2WrapPrestartRange *range) {
    if (w->phase == TASK2_WRAP_RANGE1 && !w->fault && w->io) w->prestart = *range;
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
    if (w->phase != TASK2_WRAP_WAIT1 && w->phase != TASK2_WRAP_WAIT2 && w->phase != TASK2_WRAP_WAIT1_START)
        w->activeMs += now - w->previousMs;
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
        float r; int got;
        bool prestart = w->prestart.count && !w->prestart.reject;
        if (w->prestart.count || w->prestart.reject) {
            /* The record shows the pre-START median and its newest echo, not a fresh post-START reading. */
            w->lastUssMm = w->prestart.medianMm;
            w->lastUssStatus = (uint8_t)(prestart ? SENSOR_OK : SENSOR_NOT_READY);
            w->lastUssSequence = w->prestart.newestSequence;
            w->lastUssCapturedMs = w->prestart.newestCapturedMs;
            if (!prestart) Note(w, TASK2_DIAG_PLAN, "PRESTART", false, w->prestart.reject);
            w->prestart.count = 0U;
            w->prestart.reject = NULL;
        }
        if (prestart) {
            r = w->prestart.medianMm; got = 1;
        } else {
            got = StandingRange(w, now, &r);
        }
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
        /* The camera saw O1 much nearer, or the ultrasonic had nothing usable: plan on the camera, stop short and range
         * again from there. */
        bool distrust = O1BeyondCamera(r, w->cameraO1Mm) || (got < 0 && isfinite(w->cameraO1Mm));
        if (distrust) {
            Note(w, TASK2_DIAG_PLAN, "RANGE1", false, got < 0 ? "T2_O1_CAMERA" : "T2_O1_CAMERA_NEARER");
            r = w->cameraO1Mm;
            w->guardBraked = true;
        }
        w->o1RangeMm = r;
        w->o1FaceXMm = w->startFrontXMm + r;
        if (got > 0 && !distrust) Note(w, TASK2_DIAG_PLAN, "RANGE1", true, prestart ? "T2_O1_PRESTART" : NULL);
        w->guardRun = 0U;
        if (!w->arrow1) {
            /* The camera reads arrow 1 well from here; at O1_STOP the arrow fills its view. */
            Enter(w, TASK2_WRAP_WAIT1_START, now);
            break;
        }
        Enter(w, TASK2_WRAP_APPROACH1, now);
        fault = Straight(w, distrust ? r - O1_STOP_MM - O1_SHORT_STOP_MM : r - O1_STOP_MM, cruise);
        break;
    }
    case TASK2_WRAP_WAIT1_START:
        if (!w->arrow1) {
            if (t->stage != TASK2_STAGE_WAIT1) Task2Commands_SetStage(t, TASK2_STAGE_WAIT1);
            break;
        }
        Note(w, TASK2_DIAG_PLAN, "ARROW1", true, "T2_ARROW1_AT_START");
        Enter(w, TASK2_WRAP_APPROACH1, now);
        fault = Straight(w, w->o1RangeMm - O1_STOP_MM - (w->guardBraked ? O1_SHORT_STOP_MM : 0.0f), cruise);
        break;
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
        /* What the camera's O1 is from here, if it was given. */
        float cameraLeft = isfinite(w->cameraO1Mm) ? w->startFrontXMm + w->cameraO1Mm - (w->pose.xMm + FRONT_MM) : NAN;
        bool useCamera = O1BeyondCamera(got > 0 ? r : NAN, cameraLeft);
        if (useCamera) { r = cameraLeft; got = 1; }
        if (got > 0) {
            w->o1FaceXMm = w->pose.xMm + FRONT_MM + r;
            fault = Straight(w, r - O1_STOP_MM, cruise);
        }
        Note(w, TASK2_DIAG_PLAN, "RANGE1_AGAIN", got > 0 && !useCamera,
             useCamera ? "T2_O1_CAMERA_NEARER" : got > 0 ? NULL : "T2_O1_NO_ECHO");
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
    case TASK2_WRAP_SWERVE: {
        float range; bool echo; Task2RoutePose p;
        if (busy && FreshUSS(w, now, &range, &echo, &p) && echo) {
            if (range < USS_NEAR_MM) range += USS_NEAR_SHORT_MM;
            float face = p.xMm + (FRONT_MM + range) * cosf(p.yawRad);
            if (fabsf(p.yawRad) < SIDE_RANGE_MAX_YAW_DEG * DEG && fabsf(p.yMm) >= SIDE_RANGE_MIN_Y_MM &&
                range >= SIDE_RANGE_MIN_MM && range <= SIDE_RANGE_MAX_MM &&
                face >= w->o1FaceXMm + 100.0f + RULE_MIN_GAP_MM - SIDE_SHORT_MARGIN_MM &&
                face <= w->o1FaceXMm + 100.0f + RULE_MAX_GAP_MM + RANGE_LONG_MARGIN_MM &&
                w->o2SideCount < sizeof w->o2SideFaces / sizeof w->o2SideFaces[0]) {
                w->o2SideFaces[w->o2SideCount++] = face;
                bool had = isfinite(w->o2SideFaceXMm);
                w->o2SideFaceXMm = SideFace(w);
                if (!had && isfinite(w->o2SideFaceXMm)) Note(w, TASK2_DIAG_PLAN, "O2_SIDE", true, NULL);
            }
        }
        if (busy && w->moveNext == w->moveCount && !w->swerveBraked && isfinite(w->o2SideFaceXMm)) {
            float front = w->pose.xMm + FRONT_MM * cosf(w->pose.yawRad) + HALF_WIDTH_MM * fabsf(sinf(w->pose.yawRad));
            if (front >= w->o2SideFaceXMm - SWERVE_O2_CLEAR_MM) {
                w->swerveBraked = true;
                w->io->brake(w->io->context);
                Note(w, TASK2_DIAG_PLAN, "SWERVE_GUARD", false, "T2_SWERVE_O2_NEAR");
            }
        }
        if (busy) break;
        if (LaunchNext(w, &fault)) break;
        Task2Commands_SetStage(t, TASK2_STAGE_APPROACH2);
        BeginRange(w, now);
        Enter(w, TASK2_WRAP_RANGE2, now);
        break;
    }
    case TASK2_WRAP_RANGE2: {
        if (busy) { HoldRange(w, now); break; }
        if (LaunchNext(w, &fault)) { HoldRange(w, now); break; }
        float r; int got = StandingRange(w, now, &r);
        if (!got) break;
        if (got > 0) {
            if (r < USS_NEAR_MM) r += USS_NEAR_SHORT_MM;
            if (!O2Plausible(w, w->pose.xMm + (FRONT_MM + r) * cosf(w->pose.yawRad))) {
                Note(w, TASK2_DIAG_PLAN, "RANGE2", false, "T2_O2_IMPLAUSIBLE");
                got = -1;
            }
        }
        uint16_t depth;
        bool still = !w->backoffSteps && !w->sideStepped;
        bool side = isfinite(w->o2SideFaceXMm);
        float cameraFace = got < 0 && !side && still && !w->cameraRejected && Task2Commands_Depth(t, &depth)
                               ? CameraFaceX(w, (float)depth) : NAN;
        if (isfinite(cameraFace) && !O2Plausible(w, cameraFace)) {
            w->cameraRejected = true;
            Note(w, TASK2_DIAG_PLAN, "RANGE2", false, "T2_O2_CAMERA_IMPLAUSIBLE");
            cameraFace = NAN;
        }
        if (got > 0) {
            /* The nearer of this close range and the one from beside O1 (a range read too far costs clearance). */
            float face = w->pose.xMm + (FRONT_MM + r) * cosf(w->pose.yawRad);
            w->o2FaceXMm = side ? fminf(face, w->o2SideFaceXMm) : face;
            Note(w, TASK2_DIAG_PLAN, "RANGE2", true, w->backoffSteps || w->sideStepped ? "T2_O2_REACQUIRED" : NULL);
        } else if (side) {
            w->o2FaceXMm = w->o2SideFaceXMm;
            Note(w, TASK2_DIAG_PLAN, "RANGE2", true, "T2_O2_SIDE_RANGE");
        } else if (isfinite(cameraFace)) {
            w->o2FaceXMm = cameraFace;
            Note(w, TASK2_DIAG_PLAN, "RANGE2", true, "T2_O2_CAMERA");
        } else if (still && !w->cameraRejected && !Task2Commands_Arrow(t, 2U) && now - w->phaseStartedMs < CAMERA_WAIT_MS) {
            /* The camera's depth comes with arrow 2: range again until it does. */
            BeginRange(w, now);
            break;
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
        fault = Straight(w, move, cruise);
        break;
    }
    case TASK2_WRAP_STANDOFF:
        if (busy) break;
        if (!w->standoffChecked) {
            /* RANGE2 is taken from just past O1, often a few cm from O2 where the ultrasonic is least accurate; from
             * the standoff it is accurate, and the turn out's clearance depends on it. */
            w->standoffChecked = true;
            BeginRange(w, now);
            Enter(w, TASK2_WRAP_STANDOFF_CHECK, now);
            break;
        }
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
    case TASK2_WRAP_STANDOFF_CHECK: {
        if (busy) { HoldRange(w, now); break; }
        if (w->range.active) {
            float r; int got = StandingRange(w, now, &r);
            if (!got) break;
            if (got > 0) {
                if (r < USS_NEAR_MM) r += USS_NEAR_SHORT_MM;
                float face = w->pose.xMm + (FRONT_MM + r) * cosf(w->pose.yawRad), o1Back = w->o1FaceXMm + 100.0f;
                /* Square on at about 25 cm the echo is O2: one nearer than the rules allow is still taken (the robot
                 * only backs off, within the tail room to O1), down to SIDE_SHORT_MARGIN below the minimum gap. */
                if (face >= o1Back + RULE_MIN_GAP_MM - SIDE_SHORT_MARGIN_MM && face <= o1Back + RULE_MAX_GAP_MM + RANGE_LONG_MARGIN_MM) {
                    /* The nearer of this face and RANGE2's: a range read too far is what costs the turn out its
                     * clearance; one read too near only leaves the robot a little further off. */
                    w->o2FaceXMm = isfinite(w->o2FaceXMm) ? fminf(face, w->o2FaceXMm) : face;
                    float error = (w->o2FaceXMm - w->pose.xMm) / cosf(w->pose.yawRad) - FRONT_MM - O2_STOP_MM;
                    if (fabsf(error) <= STANDOFF_TOLERANCE_MM) {
                        Note(w, TASK2_DIAG_PLAN, "STANDOFF", true, NULL);
                    } else {
                        float move = error < 0.0f ? fmaxf(error, -ReverseRoomTo(w, STANDOFF_TAIL_MIN_MM)) : error;
                        Note(w, TASK2_DIAG_PLAN, "STANDOFF", false,
                             move > error ? "T2_STANDOFF_CLAMPED" : "T2_STANDOFF_CORRECTED");
                        if (fabsf(move) >= 1.0f) {
                            fault = Straight(w, move, SETTLE_MMPS);
                            break;
                        }
                    }
                } else {
                    Note(w, TASK2_DIAG_PLAN, "STANDOFF", false, "T2_O2_IMPLAUSIBLE");
                }
            } else {
                Note(w, TASK2_DIAG_PLAN, "STANDOFF", false, "T2_STANDOFF_NO_ECHO");
            }
        }
        Enter(w, TASK2_WRAP_STANDOFF, now);
        break;
    }
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
            fault = SettleAtEnd(w, w->nearEndYMm, false);
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
#if TASK2_WRAP_FAST_REACQUIRE_SETTLE
        if (w->nearReacquired && !w->nearExhausted) { fault = MoveToEnd(w, w->nearEndYMm, cruise); break; }
#endif
        fault = SettleAtEnd(w, w->nearEndYMm, !w->nearExhausted);
        break;
    case TASK2_WRAP_NEAR_SETTLE:
        if (busy) break;
        Enter(w, TASK2_WRAP_UTURN, now);
        fault = ArcTo(w, -0.5f * PI * (float)w->arrow2, -w->arrow2);
        break;
    case TASK2_WRAP_UTURN: {
        if (busy) break;
#if !TASK2_WRAP_FAR_BY_IR
        if (!w->nearExhausted) {
            /* O2 is centred on the start line: its far end mirrors the near one. */
            w->farEndYMm = -w->nearEndYMm;
            Note(w, TASK2_DIAG_PLAN, "FAR_END", true, "T2_FAR_MIRRORED");
            Enter(w, TASK2_WRAP_FAR_SETTLE, now);
            fault = MoveToEnd(w, w->farEndYMm, cruise);
            break;
        }
#endif
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
        fault = SettleAtEnd(w, w->farEndYMm, !w->farExhausted);
        break;
    case TASK2_WRAP_FAR_SETTLE: {
        if (busy) break;
        /* The turn home and the entry turn the same way: one arc of 90 deg + theta into the straight in, when it fits. */
        float q1;
        if (AngledEntryFrom(w, w->pose.xMm, w->pose.yMm, PI + Wrap(w->pose.yawRad - PI), &q1)) {
            Task2Commands_SetStage(t, TASK2_STAGE_HOME);
            ClearMoves(w);
            Push(w, (Task2WrapMove){TASK2_WRAP_ARC_TO, (int8_t)(w->pose.yMm >= 0.0f ? 1 : -1), q1, 0.0f});
            w->angled = true;
            w->singleS = true;
            Note(w, TASK2_DIAG_PLAN, "HOME", true, "T2_HOME_ONE_ARC");
            Enter(w, TASK2_WRAP_HOME, now);
            (void)LaunchNext(w, &fault);
            break;
        }
        Enter(w, TASK2_WRAP_TURN_HOME, now);
        fault = ArcTo(w, PI, -w->arrow2);
        break;
    }
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
        if (w->angled) {
            /* Along the heading the arc actually ended on, to where the body is centred in the car park, if that
             * still fits; otherwise square up and plan home again from here. */
            float length, tx, ty;
            if (ParkStraightOk(w, &length) || w->replans >= HOME_REPLANS) {
                if (w->replans >= HOME_REPLANS && !ParkStraightOk(w, &length)) {
                    float c = cosf(w->pose.yawRad), s = sinf(w->pose.yawRad);
                    EntryTarget(w->pose.yawRad, &tx, &ty);
                    length = fmaxf(0.0f, (tx - w->pose.xMm) * c + (ty - w->pose.yMm) * s);
                }
                fault = Straight(w, length, cruise);
            } else {
                ++w->replans;
                w->angled = false;
                Note(w, TASK2_DIAG_PLAN, "HOME", false, "T2_ENTRY_RECHECK");
                Enter(w, TASK2_WRAP_TURN_HOME, now);
                fault = ArcTo(w, PI, Wrap(PI - w->pose.yawRad) >= 0.0f ? 1 : -1);
            }
        } else {
            float d = (ParkX() - w->pose.xMm) / cosf(w->pose.yawRad);
#if TASK2_WRAP_PARK_WHERE_INSIDE
            /* Already wholly inside with PARK_INSIDE_MARGIN to every wall and the entrance line: parked as it is. */
            if (InsideCarPark(w->pose.xMm, w->pose.yMm, w->pose.yawRad, PARK_INSIDE_MARGIN_MM)) d = 0.0f;
#endif
            if (fabsf(d) >= 5.0f) fault = Straight(w, d, cruise);   /* an exact S already ends on the spot */
        }
        break;
    case TASK2_WRAP_PARK: {
        float range; bool echo; Task2RoutePose p;
        /* Armed only once the whole body is inside the car park: a wrong echo can then only stop the robot where it
         * is already parked, never short of the car park. Until then odometry alone drives it in. */
        if (busy && !w->guardBraked && FreshUSS(w, now, &range, &echo, &p) && InsideCarPark(p.xMm, p.yMm, p.yawRad, 0.0f)) {
            float expected = ParkRayMm(&p);
            bool short_ = echo && range - cruise * USS_LATENCY_S < expected - GUARD_SHORT_MM;
            w->guardRun = short_ ? w->guardRun + 1U : 0U;
            if (w->guardRun >= GUARD_SAMPLES) {
                w->guardBraked = true;
                w->io->brake(w->io->context);
                Note(w, TASK2_DIAG_PLAN, "PARK_GUARD", true, "T2_BACK_WALL_NEARER");
            }
        }
        if (busy) break;
        if (w->guardBraked && !w->angled) { BeginRange(w, now); Enter(w, TASK2_WRAP_PARK_FINISH, now); break; }
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

void Task2WrapPrestart_Update(Task2WrapPrestart *p, const Task2AppIo *io, bool moving, uint32_t now) {
    UltrasonicReading r = io->ultrasonic(io->context);
    if (moving) { p->count = 0U; p->sequence = r.sequence; return; }
    if (r.sequence == p->sequence) return;
    p->sequence = r.sequence;
    if (SensorReading_Status(&r, now) != SENSOR_OK || !isfinite(r.distanceMm) || r.distanceMm < 20.0f ||
        r.distanceMm > 4000.0f) {
        p->count = 0U;
        return;
    }
    p->values[p->head] = r.distanceMm;
    p->capturedMs[p->head] = r.capturedMs;
    p->sequences[p->head] = r.sequence;
    p->head = (p->head + 1U) % TASK2_WRAP_PRESTART_SAMPLES;
    if (p->count < TASK2_WRAP_PRESTART_SAMPLES) ++p->count;
}
Task2WrapPrestartRange Task2WrapPrestart_Range(const Task2WrapPrestart *p, uint32_t now) {
    Task2WrapPrestartRange out = {.medianMm = NAN, .spreadMm = NAN, .count = p->count};
    if (!p->count) { out.reject = "T2_O1_PRESTART_NONE"; return out; }
    unsigned last = (p->head + TASK2_WRAP_PRESTART_SAMPLES - 1U) % TASK2_WRAP_PRESTART_SAMPLES;
    out.newestSequence = p->sequences[last];
    out.newestCapturedMs = p->capturedMs[last];
    if (p->count < TASK2_WRAP_PRESTART_SAMPLES) { out.reject = "T2_O1_PRESTART_FEW"; return out; }
    float v[TASK2_WRAP_PRESTART_SAMPLES];
    uint32_t oldest = 0U, newest = UINT32_MAX;
    for (unsigned i = 0U; i < TASK2_WRAP_PRESTART_SAMPLES; ++i) {
        uint32_t age = now - p->capturedMs[i];
        if (age > oldest) oldest = age;
        if (age < newest) newest = age;
        float x = p->values[i]; unsigned j = i;
        while (j && v[j - 1U] > x) { v[j] = v[j - 1U]; --j; }
        v[j] = x;
    }
    out.oldestAgeMs = oldest;
    out.spreadMm = v[TASK2_WRAP_PRESTART_SAMPLES - 1U] - v[0];
    out.medianMm = v[TASK2_WRAP_PRESTART_SAMPLES / 2U];
    if (oldest > PRESTART_SPAN_MS || newest > PRESTART_FRESH_MS) out.reject = "T2_O1_PRESTART_STALE";
    else if (out.spreadMm > PRESTART_SPREAD_MM) out.reject = "T2_O1_PRESTART_UNSTEADY";
    return out;
}
float Task2WrapPrestart_Steady(const Task2WrapPrestart *p) {
    if (p->count < TASK2_WRAP_PRESTART_SAMPLES) return NAN;
    float v[TASK2_WRAP_PRESTART_SAMPLES];
    for (unsigned i = 0U; i < TASK2_WRAP_PRESTART_SAMPLES; ++i) {
        float x = p->values[i]; unsigned j = i;
        while (j && v[j - 1U] > x) { v[j] = v[j - 1U]; --j; }
        v[j] = x;
    }
    return v[TASK2_WRAP_PRESTART_SAMPLES - 1U] - v[0] > PRESTART_SPREAD_MM ? NAN : v[TASK2_WRAP_PRESTART_SAMPLES / 2U];
}
