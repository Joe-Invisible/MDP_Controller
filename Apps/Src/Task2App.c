#include "Task2App.h"
#include "Task2Wire.h"
#ifdef TASK2_HOST_DEBUG
#include <stdio.h>
#endif
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846f
#define RANGE_TIMEOUT_MS 2500U
#define CAMERA_DEPTH_WAIT_MS 1500U
/* An echo this much nearer than O2's estimated face moves the estimate in; a farther one never moves
 * it out (at an angle the ultrasonic misses the face and times something beyond). Allows for the
 * ultrasonic reading short of the tape. */
#define NEAR_FACE_TOLERANCE_MM 60.0f
#define USS_FRESH_MS 300U
static const float departureBackupMm[] = {100.0f, 200.0f, 300.0f};  /* RANGE2's wait for the camera before using the ultrasonic. */
#define ACTIVE_TIMEOUT_MS 120000U
/* Rules: obstacle 2 is at least 300 mm long and at least 500 mm is clear past each end; past that
 * there may be a wall. The side limit is the proven half-length plus 500 mm, never assumed wider. */
#ifndef RULE_O2_MIN_LENGTH_MM
#define RULE_O2_MIN_LENGTH_MM 300.0f
#endif
#define RULE_O2_END_CLEARANCE_MM 500.0f
/* Same-side departure: a face still seen at the end of the probe lane extends it, at most this often. */
#define MAX_DEPARTURE_EXTENSIONS 6U
/* Extra width each extension plans past the proven half-length. */
#ifndef DEPARTURE_EXTENSION_MM
#define DEPARTURE_EXTENSION_MM 150.0f
#endif
/* Reverse distances tried, shortest first, when a same-side IR traverse needs room to turn. */
static const float traverseBackupMm[] = {100.0f, 200.0f, 300.0f};
/* A same-side traverse whose IR never saw the face reverses along it at most this often. */
#define MAX_TRAVERSE_REWINDS 2U
/* Proven face the IR crosses before the proven end, beyond the acquisition travel. */
#define REWIND_FACE_MARGIN_MM 20.0f
/* A USS range that holds within the tolerance while O2's face geometry says it should have changed by
 * at least the expected change, between capture poses, is frozen. */
#define USS_FROZEN_TOLERANCE_MM 3.0f
#define USS_FROZEN_EXPECTED_CHANGE_MM 30.0f
/* Allowance between an end's upper bound and the proven lower bound before the sensors disagree. */
#define SENSOR_CONFLICT_TOLERANCE_MM 50.0f
/* Trial plans are made here first so a rejected one leaves the live route untouched. */
static Task2Route planScratch;

/* Planning work and time before a call, so its record covers the whole call. */
typedef struct { uint32_t checks, us, calls; } PlanMark;
static uint32_t ElapsedUs(const Task2App *a) {
    return a->io && a->io->elapsedUs ? a->io->elapsedUs(a->io->context) : 0U;
}
static PlanMark Mark(const Task2App *a) {
    PlanMark m = {Task2Route_TotalPlanningChecks(), ElapsedUs(a), a->planCalls};
    return m;
}
#if TASK2_DIAGNOSTICS
static char ArrowLetter(Task2Arrow arrow) {
    return arrow == TASK2_ARROW_LEFT ? 'L' : arrow == TASK2_ARROW_RIGHT ? 'R' : '-';
}
#endif
/* One snapshot of the decision just made: inputs, bounds, the route it left and the sensors. */
static Task2DiagRecord *Note(Task2App *a, Task2DiagKind kind, const char *what, bool ok, const char *reason,
                              const Task2Route *route, const Task2RoutePose *from, const PlanMark *mark) {
#if TASK2_DIAGNOSTICS
    Task2DiagRecord *r = Task2Diag_Begin(&a->diag, kind);
    r->what = what; r->ok = ok; r->reason = reason ? reason : route->report.reason;
    r->ms = a->previousMs; r->phase = (uint8_t)a->phase; r->step = (uint8_t)a->step;
    r->xMm = a->pose.xMm; r->yMm = a->pose.yMm; r->yawDeg = a->pose.yawRad * 180.0f / PI;
    r->arrow1 = ArrowLetter(a->geometry.firstArrow); r->arrow2 = ArrowLetter(a->geometry.secondArrow);
    r->obstacle1FaceMm = a->estimates.obstacle1Face.mm; r->gapMm = a->estimates.faceGap.mm;
    const Task2Dimension *w = &a->estimates.obstacle2Width;
    r->widthMm = w->mm; r->widthUncertaintyMm = w->uncertaintyMm; r->widthSource = (uint8_t)w->source;
    r->widthValid = w->valid; r->widthBound = w->bound;
    r->planWidthMm = a->geometry.obstacle2WidthMm; r->endSpaceMm = a->geometry.obstacle2EndSpaceMm;
    r->sideLimitMm = 0.5f * a->geometry.obstacle2WidthMm + a->geometry.obstacle2EndSpaceMm;
    r->provenHalfMm = a->provenHalfMm; r->edgeMeasured = a->edgeMeasured;
    r->provenOnly = a->edgeMeasured || a->sameSideDeparture;
    if (!from) from = &a->pose;
    r->fromXMm = from->xMm; r->fromYMm = from->yMm; r->fromYawDeg = from->yawRad * 180.0f / PI;
    r->configWidthMm = route->config.obstacle2WidthMm; r->configEndSpaceMm = route->config.obstacle2EndSpaceMm;
    r->returnFallbackActive = route->returnFallbackActive;
    r->returnObstacleBufferMm = route->returnObstacleBufferMm;
    r->returnBoundaryBufferMm = route->returnBoundaryBufferMm;
    const Task2RouteReport *q = &route->report;
    r->obstacleGapMm = q->minimumObstacleGapMm; r->sideMarginMm = q->minimumSideMarginMm;
    r->parkMarginMm = q->minimumParkWallMarginMm;
    r->fallback = q->returnFallbackUsed; r->bufferException = q->obstacleBufferExceptionUsed;
    r->poseAdjusted = q->poseAdjustmentUsed; r->returnCertified = q->completeReturnCertified;
    r->exhausted = q->planningBudgetExceeded; r->candidateChecks = q->plannerPoseChecks;
    if (mark) {
        r->callChecks = Task2Route_TotalPlanningChecks() - mark->checks;
        r->callUs = a->io && a->io->elapsedUs ? ElapsedUs(a) - mark->us : 0U;
    }
    /* Host fields describe this decision only if a host-capable call was made since its mark. */
    bool hosted = mark && a->planCalls != mark->calls;
    r->planSource = hosted ? (char)a->planSource : '-';
    r->planRefusal = hosted && a->planHostRefusal ? a->planHostRefusal : '-';
    r->hostWaitMs = hosted ? a->planHostUs / 1000U : 0U;
    r->rejectedRangeMm = a->rejectedRangeMm;
    r->cameraFaceMm = a->cameraFaceMm;
    r->ussFaceMm = a->ussFaceMm;
    /* Keep the active step and what follows it when the route is longer than the record. */
    unsigned total = route->stepCount < TASK2_ROUTE_STEP_COUNT ? route->stepCount : TASK2_ROUTE_STEP_COUNT;
    unsigned first = total > TASK2_DIAG_STEPS && a->step > 1U ? a->step - 1U : 0U;
    if (total > TASK2_DIAG_STEPS && first > total - TASK2_DIAG_STEPS) first = total - TASK2_DIAG_STEPS;
    r->stepTotal = (uint8_t)total; r->stepFirst = (uint8_t)first;
    r->stepCount = (uint8_t)(total - first < TASK2_DIAG_STEPS ? total - first : TASK2_DIAG_STEPS);
    for (unsigned i = 0U; i < r->stepCount; ++i) {
        const Task2RouteStep *s = &route->steps[first + i];
        r->steps[i] = (Task2DiagStep){s->distanceMm, s->radiusMm, s->speedCps, s->sensorTerminated};
    }
    const Task2ObstacleDetection *d = &a->detection;
    r->ussRangeMm = d->lastUltrasonicRangeMm; r->ussValid = d->ultrasonicValid;
    r->ussStatus = a->lastUSSStatus; r->ussSequence = a->lastUSSSequence; r->ussCapturedMs = a->lastUSSCapturedMs;
    r->ussSinceObservedMs = a->previousMs - a->lastUSSObservedMs;
    r->irAdc[0] = d->lastIRAdc[0]; r->irAdc[1] = d->lastIRAdc[1];
    r->irFaceLevelAdc = d->irFaceLevelAdc;
    unsigned kept = a->irHistoryCount < TASK2_DIAG_IR_HISTORY ? a->irHistoryCount : TASK2_DIAG_IR_HISTORY;
    r->irHistoryCount = (uint8_t)kept;
    for (unsigned i = 0U; i < kept; ++i)
        r->irHistory[i] = a->irHistoryAdc[(a->irHistoryCount - kept + i) % TASK2_DIAG_IR_HISTORY];
    r->edgeMethod = (uint8_t)d->edgeMethod; r->detected = d->detected; r->observedWidthMm = d->observedWidthMm;
    r->frozenRangeMm = a->frozenRangeMm; r->frozenExpectedMm = a->frozenExpectedMm; r->ussFrozen = a->ussFrozen;
    r->extensions = (uint8_t)a->departureExtensions; r->rewinds = (uint8_t)a->traverseRewinds;
    Task2Diag_End(&a->diag, r);
    return r;
#else
    (void)a; (void)kind; (void)what; (void)ok; (void)reason; (void)route; (void)from; (void)mark;
    return NULL;
#endif
}
/* The reverse a recovery decision launches is not in the route; keep it with the decision's record. */
static void NoteNext(Task2App *a, Task2DiagRecord *r, const Task2RouteStep *s) {
#if TASK2_DIAGNOSTICS
    if (!r) return;
    r->hasNext = true;
    r->next = (Task2DiagStep){s->distanceMm, s->radiusMm, s->speedCps, s->sensorTerminated};
    Task2Diag_End(&a->diag, r);
#else
    (void)a; (void)r; (void)s;
#endif
}

const Task2ObstacleSensorConfig task2AppSensorDefaults = {
    .ultrasonicEnabled = true, .sideIREnabled = true,
    .ultrasonic = {220.0f, 0.0f, 0.0f},
    .leftIR = {155.0f, 37.5f, 90.0f}, .rightIR = {135.0f, -33.5f, -90.0f},
    .ultrasonicBeamHalfAngleDeg = 7.5f, .ultrasonicRangeToleranceMm = 80.0f,
    .ultrasonicApproachMedianSamples = 11U, .ultrasonicDepartureMedianSamples = 5U,
    .ultrasonicJumpMm = 250.0f, .ultrasonicMaxDepartureYawDeg = 45.0f,
    /* Sharp curve: 700 is about 59 cm, 400 about 113 cm. Once armed, the end is 300 below the face's own
     * level, kept between 400 and the present level. */
    .irPresentAdc = 700U, .irAbsentAdc = 400U, .irFaceDropAdc = 300U, .irAcquisitionMinYawDeg = 80.0f,
    .irTrendMinDropAdc = 200U, .irTrendReboundAdc = 50U,
    .irTrendSamples = 3U, .irTrendMinTravelMm = 20.0f, .edgeUncertaintyMm = 20.0f,
    .samplePeriodMs = SENSOR_ULTRASONIC_PERIOD_MS, .acquisitionTimeoutMs = 1500U,
    .presentSamples = 3U, .absentSamples = 3U,
};

/* The planner's side limit is W/2 + obstacle2EndSpaceMm in every configuration it derives; set the end
 * space so that sum is the side limit for the current planning width. Until O2's end is measured the
 * planner keeps its nominal width's limit (O2 is still ahead, near the centre line); once measured, or
 * once a same-side departure heads wide, only the proven half-length plus the rules' clearance is used,
 * even if that is narrower. */
static void ApplyArena(Task2App *a) {
    if (a->detection.faceExtentLowerMm > a->provenHalfMm) a->provenHalfMm = a->detection.faceExtentLowerMm;
    bool provenOnly = a->edgeMeasured || a->sameSideDeparture;
    float half = provenOnly ? a->provenHalfMm : fmaxf(a->provenHalfMm, 0.5f * task2RouteConfig.obstacle2WidthMm);
    float limit = half + RULE_O2_END_CLEARANCE_MM;
    a->geometry.obstacle2EndSpaceMm = limit - 0.5f * a->geometry.obstacle2WidthMm;
    a->route.config.obstacle2EndSpaceMm = limit - 0.5f * a->route.config.obstacle2WidthMm;
}
static void Enter(Task2App *a, Task2AppPhase phase, uint32_t now) {
    a->phase = phase;
    a->phaseStartedMs = now;
}
/*
 * Planning on a host. The request carries everything the planner call reads; the reply is the route
 * it left. Nothing in the reply is taken on trust: its config must be the one sent, the steps already
 * planned or driven must be unchanged, every step must be a drivable primitive, its return buffers one
 * of the planner's own tiers, and the new steps physically clear of this app's own geometry.
 * Anything else, or no reply in time, and the same call runs here instead.
 */
typedef enum { HOST_NONE, HOST_ACCEPTED, HOST_CANCELLED } HostResult;
/* The arena is about 4 m long: no plan from this course needs more per primitive, or in all. */
#define HOST_MAX_STEP_MM 5000.0f
#define HOST_MAX_SUFFIX_MM 12000.0f
/* A traverse's preview of O2: half its length plus the space beyond, at most this. */
#define HOST_MAX_PREVIEW_HALF_MM 5000.0f
static Task2WireRequest hostRequest;
static Task2WireReply hostReply;
static uint8_t hostOut[TASK2_WIRE_MAX_BYTES], hostIn[TASK2_WIRE_MAX_BYTES];

static size_t ConfigBytes(const Task2RouteConfig *c, uint8_t *out, size_t capacity) {
    Task2WireReply r = {0};
    r.route.config = *c;
    return Task2Wire_EncodeReply(&r, out, capacity);
}
static bool SameConfig(const Task2RouteConfig *a, const Task2RouteConfig *b) {
    static uint8_t x[TASK2_WIRE_MAX_BYTES], y[TASK2_WIRE_MAX_BYTES];
    size_t n = ConfigBytes(a, x, sizeof(x)), m = ConfigBytes(b, y, sizeof(y));
#ifdef TASK2_HOST_DEBUG
    if (!(n && n == m && memcmp(x, y, n) == 0)) {
        fprintf(stderr, "  SameConfig n %zu m %zu", n, m);
        for (size_t k = 0; k < n && k < m; ++k) if (x[k] != y[k]) { fprintf(stderr, " first diff at byte %zu", k); break; }
        fprintf(stderr, "\n");
    }
#endif
    return n && n == m && memcmp(x, y, n) == 0;
}
static bool SameStep(const Task2RouteStep *a, const Task2RouteStep *b) {
    return a->distanceMm == b->distanceMm && a->radiusMm == b->radiusMm &&
        a->speedCps == b->speedCps && a->sensorTerminated == b->sensorTerminated;
}
/* The route's orchestration state besides its steps; a planner call may change only what it changes. */
typedef struct {
    bool ir, lateral, pending, tangent, straightPark, directRear, parkPending, parallelPending, fallback;
    unsigned home;
    uint32_t corrections;
    float obstacleBuffer, boundaryBuffer, width;
} RouteState;
static RouteState StateOf(const Task2Route *r) {
    return (RouteState){r->obstacle2IRDeparture, r->obstacle2LateralDeparture, r->obstacle2ReturnPending,
        r->obstacle2TangentRearArc, r->obstacle2StraightParkReturn, r->obstacle2DirectRearArc,
        r->obstacle2ParkReturnPending, r->obstacle2ParallelReturnPending, r->returnFallbackActive,
        r->fallbackHomeStep, r->parkingHeadingCorrections, r->returnObstacleBufferMm, r->returnBoundaryBufferMm,
        r->returnWidthMm};
}
static bool SameBits(float a, float b) { return memcmp(&a, &b, sizeof(a)) == 0; }
static bool SameState(const RouteState *a, const RouteState *b) {
    return a->ir == b->ir && a->lateral == b->lateral && a->pending == b->pending && a->tangent == b->tangent &&
        a->straightPark == b->straightPark && a->directRear == b->directRear && a->parkPending == b->parkPending &&
        a->parallelPending == b->parallelPending && a->fallback == b->fallback && a->home == b->home &&
        a->corrections == b->corrections && SameBits(a->obstacleBuffer, b->obstacleBuffer) &&
        SameBits(a->boundaryBuffer, b->boundaryBuffer) && SameBits(a->width, b->width);
}
/* The state the planner leaves for this call: its normal result, or for a return its certified
 * fallback. Values only the planner can choose (an arc kind, a heading-only correction, the fallback's
 * home step and tier) are taken from the reply and then checked against what they must imply. */
static bool StateFollows(Task2WireCall call, const Task2Route *sent, const Task2Route *got, unsigned first,
                         float width) {
    RouteState was = StateOf(sent), now = StateOf(got), e = was;
    switch (call) {
    case TASK2_WIRE_REPLAN_DEPARTURE:
        return SameState(&now, &was);
    case TASK2_WIRE_REPLAN_IR_TRAVERSE:
        e.ir = e.pending = true;
        e.lateral = false;
        return SameState(&now, &e);
    case TASK2_WIRE_REPLAN_OBSTACLE2:
        e.pending = false;
        if (was.lateral || was.ir) {
            e.parallelPending = true;
            e.width = width;
        } else if (got->config.firstArrow == got->config.secondArrow) {
            e.tangent = now.tangent;
            e.parkPending = true;
        }
        break;
    case TASK2_WIRE_REPLAN_PARALLEL_RETURN:
        e.parallelPending = false;
        e.parkPending = true;
        e.tangent = now.tangent;
        e.directRear = got->steps[first].radiusMm != 0.0f;
        e.width = width;
        break;
    case TASK2_WIRE_REPLAN_PARKING_RETURN:
        if (now.parkPending && was.corrections >= 2U) return false;   /* a third heading correction */
        e.home = 0U;
        e.parkPending = now.parkPending;
        e.corrections = was.corrections + (now.parkPending ? 1U : 0U);
        e.straightPark = got->stepCount == first + 1U && got->steps[first].radiusMm == 0.0f;
        e.width = width;
        break;
    default:
        return false;
    }
    if (SameState(&now, &e)) return true;
    e = was;
    e.pending = e.parallelPending = e.parkPending = false;
    e.fallback = true;
    e.home = now.home;
    e.obstacleBuffer = now.obstacleBuffer;
    e.boundaryBuffer = now.boundaryBuffer;
    e.width = width;
    return now.home >= first && now.home <= got->stepCount && SameState(&now, &e);
}
/* Each speed the planner gives a step of this call: departures run at the departure speed, returns at
 * cruise, approach or departure speed. */
static bool PlannerSpeed(Task2WireCall call, const Task2RouteConfig *c, float speed) {
    if (call == TASK2_WIRE_REPLAN_DEPARTURE || call == TASK2_WIRE_REPLAN_IR_TRAVERSE)
        return speed == c->obstacle2DepartureSpeedCps;
    return speed == c->cruiseSpeedCps || speed == c->approachSpeedCps || speed == c->obstacle2DepartureSpeedCps;
}
static bool PlannerTier(float value, const float *tiers, unsigned count) {
    for (unsigned i = 0U; i < count; ++i)
        if (value == tiers[i]) return true;
    return false;
}
static char AcceptHostRoute(Task2WireCall call, const Task2RouteIRDeparture *ir, const Task2Route *sent,
                            const Task2Route *got, const Task2RouteConfig *config, const Task2RoutePose *from,
                            unsigned first, float width) {
    bool irCall = ir != NULL;
    static const float obstacleTiers[] = {100.0f, 75.0f, 50.0f, 25.0f, 1.0f};
    static const float boundaryTiers[] = {35.0f, 20.0f, 10.0f, 1.0f};
    if (got->stepCount > TASK2_ROUTE_STEP_COUNT || got->stepCount <= first || first > sent->stepCount)
        return 'n';
    /* A departure or traverse plans exactly the two departure steps after the approach. */
    if ((call == TASK2_WIRE_REPLAN_DEPARTURE || call == TASK2_WIRE_REPLAN_IR_TRAVERSE) &&
        (first != TASK2_ROUTE_O2_DEPARTURE_STEP || got->stepCount != TASK2_ROUTE_O2_IR_STRAIGHT_STEP + 1U))
        return 'n';
    /* A traverse plans with O2 at least as long as it reaches and a wider preview limit beyond it; the
     * proven limit stays the app's. Nothing else in the config may change. */
    Task2RouteConfig expected = config ? *config : sent->config;
    float previewHalf = 0.5f * got->config.obstacle2WidthMm + got->config.obstacle2EndSpaceMm;
    if (call == TASK2_WIRE_REPLAN_IR_TRAVERSE && isfinite(got->config.obstacle2WidthMm) &&
        isfinite(got->config.obstacle2EndSpaceMm) && got->config.obstacle2EndSpaceMm > 0.0f &&
        previewHalf <= HOST_MAX_PREVIEW_HALF_MM && got->config.obstacle2WidthMm >= expected.obstacle2WidthMm &&
        0.5f * got->config.obstacle2WidthMm + got->config.obstacle2EndSpaceMm >=
            0.5f * expected.obstacle2WidthMm + expected.obstacle2EndSpaceMm) {
        expected.obstacle2WidthMm = got->config.obstacle2WidthMm;
        expected.obstacle2EndSpaceMm = got->config.obstacle2EndSpaceMm;
    }
    if (!SameConfig(&got->config, &sent->config) && !SameConfig(&got->config, &expected)) {
#ifdef TASK2_HOST_DEBUG
        const float *g = (const float *)&got->config.obstacle2WidthMm, *t = (const float *)&sent->config.obstacle2WidthMm;
        for (unsigned k = 0U; k < 32U; ++k) if (memcmp(&g[k], &t[k], 4)) fprintf(stderr, "  config float %u got %g sent %g\n", k, g[k], t[k]);
        fprintf(stderr, "  arrows got %d %d sent %d %d\n", (int)got->config.firstArrow, (int)got->config.secondArrow, (int)sent->config.firstArrow, (int)sent->config.secondArrow);
#endif
        return 'c';
    }
    for (unsigned i = 0U; i < first; ++i)
        if (!SameStep(&got->steps[i], &sent->steps[i])) return 'p';
    /* Every new primitive is one this course could plan: bounded travel and at most one full turn. */
    float suffix = 0.0f;
    for (unsigned i = first; i < got->stepCount; ++i) {
        const Task2RouteStep *s = &got->steps[i];
        if (!(fabsf(s->distanceMm) <= HOST_MAX_STEP_MM) ||
            (s->radiusMm != 0.0f && !(fabsf(s->distanceMm) <= 2.0f * PI * fabsf(s->radiusMm))))
            return 'l';
        suffix += fabsf(s->distanceMm);
    }
    if (!(suffix <= HOST_MAX_SUFFIX_MM))
        return 'l';
    for (unsigned i = first; i < got->stepCount; ++i)
        if (!PlannerSpeed(call, &got->config, got->steps[i].speedCps)) return 'v';
    /* Only an IR departure or traverse ends a straight on the sensor, only at its own step, and only
     * going forward; an IR departure always does. */
    for (unsigned i = first; i < got->stepCount; ++i)
        if (got->steps[i].sensorTerminated &&
            (i != TASK2_ROUTE_O2_IR_STRAIGHT_STEP || !irCall || got->steps[i].radiusMm != 0.0f ||
             !(got->steps[i].distanceMm > 0.0f)))
            return 's';
    if ((call == TASK2_WIRE_REPLAN_DEPARTURE || call == TASK2_WIRE_REPLAN_IR_TRAVERSE) &&
        got->steps[TASK2_ROUTE_O2_IR_STRAIGHT_STEP].sensorTerminated != got->obstacle2IRDeparture)
        return 's';
    /* The kind of departure changes only when a traverse replaces it. */
    if (call == TASK2_WIRE_REPLAN_IR_TRAVERSE
        ? !got->obstacle2IRDeparture || got->obstacle2LateralDeparture
        : got->obstacle2IRDeparture != sent->obstacle2IRDeparture ||
          got->obstacle2LateralDeparture != sent->obstacle2LateralDeparture)
        return 'd';
    if (got->fallbackHomeStep > got->stepCount || got->parkingHeadingCorrections > 2U ||
        got->parkingHeadingCorrections < sent->parkingHeadingCorrections)
        return 'i';
    if (!StateFollows(call, sent, got, first, width))
        return 'f';
    /* The fallback's buffers are installed as sent: each must itself be one of the planner's tiers and no
     * more than the run's own buffer (comparisons with NaN fail). */
    if (got->returnFallbackActive &&
        (!PlannerTier(got->returnObstacleBufferMm, obstacleTiers, 5U) ||
         !(got->returnObstacleBufferMm <= got->config.obstacleBufferMm) ||
         !PlannerTier(got->returnBoundaryBufferMm, boundaryTiers, 4U) ||
         !(got->returnBoundaryBufferMm <= got->config.boundaryBufferMm)))
        return 't';
    return Task2Route_CertifySuffix(got, config ? config : &sent->config, from, first, width,
                                    ir ? ir->acquisitionTravelMm : 0.0f) ? 0 : 'x';
}

/* Ask the host to run `call` on `route`. Accepted: the route is replaced and *ok holds the call's
 * result. Cancelled: a stop arrived; nothing changes and nothing more may be planned. */
static HostResult PlanOnHost(Task2App *a, Task2WireCall call, Task2Route *route, const Task2RouteConfig *config,
                             const Task2RoutePose *pose, const Task2RouteIRDeparture *ir, float width,
                             uint32_t step, unsigned first, bool *ok) {
    ++a->planCalls;
    a->planSource = 'L';
    a->planHostUs = 0U;
    a->planHostRefusal = 0;
    if (!a->io || !a->io->plan) return HOST_NONE;
    /* Generations count across runs, so a late reply to an earlier run's request cannot match a new one. */
    static uint16_t generations;
    if (++generations == 0U) ++generations;
    a->planGeneration = generations;
    hostRequest = (Task2WireRequest){.call = (uint8_t)call, .generation = a->planGeneration, .pose = *pose,
        .hasIr = ir != NULL, .widthMm = width, .step = step};
    hostRequest.route = *route;
    hostRequest.config = config ? *config : route->config;
    if (ir) hostRequest.ir = *ir;
    size_t length = Task2Wire_EncodeRequest(&hostRequest, hostOut, sizeof(hostOut));
    if (!length) return HOST_NONE;
    uint32_t tag = Task2Wire_RequestTag(hostOut, length);
    uint32_t started = ElapsedUs(a);
    size_t got = a->io->plan(a->io->context, hostRequest.generation, hostOut, length, hostIn, sizeof(hostIn));
    a->planHostUs = ElapsedUs(a) - started;
    if (got == TASK2_PLAN_CANCELLED) {
        a->planCancelled = true;
        return HOST_CANCELLED;
    }
    if (!got)
        return HOST_NONE;
    /* A failed host plan is confirmed here before the run acts on it. */
    /* A reply belongs to this request only if it names its generation and its exact contents. */
    bool ours = Task2Wire_DecodeReply(hostIn, got, &hostReply) && hostReply.generation == hostRequest.generation &&
        hostReply.request == tag;
    if (ours && !hostReply.ok) {
        ++a->planHostFailed;
        return HOST_NONE;
    }
    if (!ours ||
        (a->planHostRefusal = AcceptHostRoute(call, ir, route, &hostReply.route, config, pose,
                                                               first, width)) != 0) {
        if (a->planHostRefusal == 0) a->planHostRefusal = 'r';   /* unreadable or stale */
#ifdef TASK2_HOST_DEBUG
        fprintf(stderr, "HOSTREFUSE call %u refusal %c first %u steps %u\n", (unsigned)call, a->planHostRefusal, first,
                (unsigned)hostReply.route.stepCount);
#endif
        ++a->planHostRejected;
        return HOST_NONE;
    }
    *route = hostReply.route;
    *ok = true;
    a->planSource = 'R';
    ++a->planHostAccepted;
    return HOST_ACCEPTED;
}

/* Each expensive planner call: on the host if it answers in time and its route checks out, else here. */
static bool PlanDeparture(Task2App *a, Task2Route *route, const Task2RouteConfig *c, const Task2RoutePose *pose,
                          const Task2RouteIRDeparture *ir) {
    bool ok = false;
    HostResult r = PlanOnHost(a, TASK2_WIRE_REPLAN_DEPARTURE, route, c, pose, ir, NAN, 0U,
                              TASK2_ROUTE_O2_DEPARTURE_STEP, &ok);
    return r == HOST_ACCEPTED ? ok : r == HOST_CANCELLED ? false : Task2Route_ReplanDeparture(route, c, pose, ir);
}
static bool PlanTraverse(Task2App *a, Task2Route *route, const Task2RouteConfig *c, const Task2RoutePose *pose,
                         const Task2RouteIRDeparture *ir) {
    bool ok = false;
    HostResult r = PlanOnHost(a, TASK2_WIRE_REPLAN_IR_TRAVERSE, route, c, pose, ir, NAN, 0U,
                              TASK2_ROUTE_O2_DEPARTURE_STEP, &ok);
    return r == HOST_ACCEPTED ? ok : r == HOST_CANCELLED ? false : Task2Route_ReplanIRTraverse(route, c, pose, ir);
}
static bool PlanEdgeReturn(Task2App *a, Task2Route *route, const Task2RoutePose *pose, float width, uint32_t step) {
    bool ok = false;
    HostResult r = PlanOnHost(a, TASK2_WIRE_REPLAN_OBSTACLE2, route, NULL, pose, NULL, width, step, step + 1U, &ok);
    return r == HOST_ACCEPTED ? ok : r == HOST_CANCELLED ? false : Task2Route_ReplanObstacle2(route, pose, width, step);
}
static bool PlanParallelReturn(Task2App *a, Task2Route *route, const Task2RoutePose *pose, float width, uint32_t step) {
    bool ok = false;
    HostResult r = PlanOnHost(a, TASK2_WIRE_REPLAN_PARALLEL_RETURN, route, NULL, pose, NULL, width, step, step + 1U, &ok);
    return r == HOST_ACCEPTED ? ok : r == HOST_CANCELLED ? false : Task2Route_ReplanParallelReturn(route, pose, width, step);
}
static bool PlanParkingReturn(Task2App *a, Task2Route *route, const Task2RoutePose *pose, float width, uint32_t step) {
    bool ok = false;
    HostResult r = PlanOnHost(a, TASK2_WIRE_REPLAN_PARKING_RETURN, route, NULL, pose, NULL, width, step, step + 1U, &ok);
    return r == HOST_ACCEPTED ? ok : r == HOST_CANCELLED ? false : Task2Route_ReplanParkingReturn(route, pose, width, step);
}

static Task2AppStatus Fail(Task2App *a, const char *reason);
#if TASK2_DIAGNOSTICS
static void TraceFinal(Task2App *a);
#endif
/* A record of the route with the margins the runtime check actually measured, not the planner's. */
static void NoteRuntime(Task2App *a, Task2DiagKind kind, const char *what, const char *reason,
                        const Task2RouteReport *runtime) {
#if TASK2_DIAGNOSTICS
    static Task2Route runtimeRecord;
    runtimeRecord = a->route;
    runtimeRecord.report.minimumObstacleGapMm = runtime->minimumObstacleGapMm;
    runtimeRecord.report.minimumSideMarginMm = runtime->minimumSideMarginMm;
    runtimeRecord.report.minimumParkWallMarginMm =
        fminf(runtime->minimumParkSideMarginMm, runtime->minimumParkBackMarginMm);
    Note(a, kind, what, kind != TASK2_DIAG_FAULT, reason, &runtimeRecord, NULL, NULL);
#else
    (void)a; (void)kind; (void)what; (void)reason; (void)runtime;
#endif
}
static Task2AppStatus FailMeasured(Task2App *a, const char *reason, const Task2RouteReport *runtime) {
    if (!a->fault) {
        NoteRuntime(a, TASK2_DIAG_FAULT, "FAIL", reason, runtime);
#if TASK2_DIAGNOSTICS
        TraceFinal(a);
#endif
        a->fault = reason;              /* Fail below records nothing more */
    }
    return Fail(a, reason);
}
static Task2AppStatus Fail(Task2App *a, const char *reason) {
    if (!a->fault) {
        Note(a, TASK2_DIAG_FAULT, "FAIL", false, reason, &a->route, NULL, NULL);
#if TASK2_DIAGNOSTICS
        TraceFinal(a);
#endif
    }
    a->fault = reason;
    if (a->io) a->io->brake(a->io->context);
    return TASK2_APP_FAULT;
}
static void RememberPose(Task2App *a, uint32_t now) {
    a->history[a->historyHead].ms = now;
    a->history[a->historyHead].pose = a->pose;
    a->historyHead = (a->historyHead + 1U) % TASK2_APP_POSE_HISTORY;
    if (a->historyCount < TASK2_APP_POSE_HISTORY) ++a->historyCount;
}
/* SensorTask owns acquisition. Associate asynchronous samples with captured poses,
 * never with the later UART/control tick. Tick differences are wrap safe. */
static bool CapturedPose(const Task2App *a, uint32_t ms, Task2RoutePose *pose) {
    unsigned first = (a->historyHead + TASK2_APP_POSE_HISTORY - a->historyCount) % TASK2_APP_POSE_HISTORY;
    for (unsigned i = 0U; i < a->historyCount; ++i) {
        unsigned slot = (first + i) % TASK2_APP_POSE_HISTORY;
        int32_t age = (int32_t)(ms - a->history[slot].ms);
        if (age < 0) return false;
        if (i + 1U == a->historyCount) { *pose = a->history[slot].pose; return true; }
        unsigned next = (slot + 1U) % TASK2_APP_POSE_HISTORY;
        uint32_t span = a->history[next].ms - a->history[slot].ms;
        if ((uint32_t)age <= span) {
            float f = span ? (float)age / span : 0.0f;
            Task2RoutePose p = a->history[slot].pose, q = a->history[next].pose;
            *pose = (Task2RoutePose){p.xMm + f * (q.xMm - p.xMm),
                p.yMm + f * (q.yMm - p.yMm), p.yawRad + f * (q.yawRad - p.yawRad)};
            return true;
        }
    }
    return false;
}
static bool UpdatePose(Task2App *a) {
    if (!a->primitive) return true;
    Task2MotionReading r = a->io->motion(a->io->context);
    if (!isfinite(r.distanceMm) || !isfinite(r.yawRad)) return false;
    float ds = r.distanceMm - a->previousDistanceMm;
    float heading = a->motionOrigin.yawRad + 0.5f * (a->previousYawRad + r.yawRad);
    a->pose.xMm += ds * cosf(heading);
    a->pose.yMm += ds * sinf(heading);
    a->pose.yawRad = a->motionOrigin.yawRad + r.yawRad;
    a->pathMm += fabsf(ds);
    a->previousDistanceMm = r.distanceMm;
    a->previousYawRad = r.yawRad;
    return true;
}
static const char *Launch(Task2App *a, const Task2RouteStep *s, uint32_t now) {
    if (a->io->busy(a->io->context)) return "T2_BUSY";
    a->primitive = false;
    a->previousDistanceMm = a->previousYawRad = 0.0f;
    a->motionOrigin = a->pose;
    if (!s->sensorTerminated && s->distanceMm == 0.0f) return NULL;
    const char *fault = s->sensorTerminated ? a->io->moveContinuous(a->io->context, s->speedCps)
        : s->radiusMm == 0.0f ? a->io->moveStraight(a->io->context, s->distanceMm, s->speedCps)
        : a->io->moveArc(a->io->context, s->distanceMm, s->radiusMm, s->speedCps);
    if (fault) return fault;
    a->primitive = true;
    if (s->sensorTerminated) {
        Task2ObstacleDetection_BeginIRTraverse(&a->detection, &a->pose);
        a->irStarted = true;
        a->irStartedMs = a->lastIRObservedMs = now;
    }
    if (a->phase == TASK2_APP_APPROACH1 || a->phase == TASK2_APP_RECOGNITION2 ||
        (a->phase == TASK2_APP_ROUTE && a->step == TASK2_ROUTE_O2_APPROACH_STEP))
        return a->io->requestSpeed(a->io->context, a->geometry.approachSpeedCps);
    return NULL;
}
static bool FreshUSS(Task2App *a, uint32_t now, UltrasonicReading *r, Task2RoutePose *p) {
    *r = a->io->ultrasonic(a->io->context);
    if (r->sequence == a->ultrasonicSequence) return false;
    a->ultrasonicSequence = r->sequence;
    SensorStatus status = SensorReading_Status(r, now);
    a->lastUSSStatus = (uint8_t)status;
    a->lastUSSSequence = r->sequence;
    a->lastUSSCapturedMs = r->capturedMs;
    if (status == SENSOR_NOT_READY || status == SENSOR_STALE || !CapturedPose(a, r->capturedMs, p)) return false;
    a->lastUSSObservedMs = now;
    return true;
}
static bool FaceWindow(Task2App *a, uint32_t now, float *face) {
    UltrasonicReading r; Task2RoutePose p;
    if (!FreshUSS(a, now, &r, &p) || SensorReading_Status(&r, now) != SENSOR_OK ||
        !isfinite(r.distanceMm) || r.distanceMm < 20.0f || r.distanceMm > 4000.0f) return false;
    Task2SensorMount m = a->sensors.ultrasonic;
    float value = p.xMm + m.aheadMm * cosf(p.yawRad) - m.leftMm * sinf(p.yawRad) +
        r.distanceMm * cosf(fmaxf(0.0f, fabsf(p.yawRad + m.yawDeg * PI / 180.0f) -
            a->sensors.ultrasonicBeamHalfAngleDeg * PI / 180.0f));
    a->faceWindow[a->faceCount++] = value;
    if (a->faceCount < a->sensors.ultrasonicApproachMedianSamples) return false;
    float sorted[TASK2_USS_HISTORY_CAPACITY];
    memcpy(sorted, a->faceWindow, a->faceCount * sizeof(float));
    for (unsigned i = 1; i < a->faceCount; ++i) {
        float x = sorted[i]; unsigned j = i;
        while (j && sorted[j - 1U] > x) { sorted[j] = sorted[j - 1U]; --j; }
        sorted[j] = x;
    }
    *face = sorted[a->faceCount / 2U];
    a->faceCount = 0U;
    return true;
}
float Task2App_FaceRange(const Task2App *a, const Task2RoutePose *p) {
    const Task2SensorMount *m = &a->sensors.ultrasonic;
    float yaw = p->yawRad + m->yawDeg * PI / 180.0f;
    float sx = p->xMm + m->aheadMm * cosf(p->yawRad) - m->leftMm * sinf(p->yawRad);
    float face = a->geometry.parkToObstacle1Mm + 100.0f + a->geometry.obstacleFaceGapMm;
    float near = fmaxf(0.0f, fabsf(yaw) - a->sensors.ultrasonicBeamHalfAngleDeg * PI / 180.0f);
    return face > sx && near < 0.5f * PI ? (face - sx) / cosf(near) : NAN;
}
bool Task2App_FrozenUSS(Task2App *a, bool valid, float rangeMm, float expectedMm) {
    if (a->ussFrozen)
        return true;
    if (!valid || !isfinite(rangeMm) || !isfinite(expectedMm)) {
        /* Nothing to compare; the next usable pair starts again. */
        a->frozenRangeMm = a->frozenExpectedMm = NAN;
    } else if (!isfinite(a->frozenExpectedMm) || !(fabsf(rangeMm - a->frozenRangeMm) <= USS_FROZEN_TOLERANCE_MM)) {
        a->frozenRangeMm = rangeMm;
        a->frozenExpectedMm = expectedMm;
    } else if (fabsf(expectedMm - a->frozenExpectedMm) >= USS_FROZEN_EXPECTED_CHANGE_MM) {
        a->ussFrozen = true;
    }
    return a->ussFrozen;
}
static float FaceFromRange(const Task2App *a, Task2RoutePose c, float range, bool nearest);
static void RecordNearFace(Task2App *a, const Task2RoutePose *p, const UltrasonicReading *r) {
    unsigned slot = a->nearFaceCount++ % TASK2_NEAR_FACE_SAMPLES;
    a->nearFaceHistoryMm[slot] = FaceFromRange(a, *p, r->distanceMm, false);
    a->nearFaceHistoryMs[slot] = r->capturedMs;
}
static bool ObserveUSS(Task2App *a, uint32_t now, bool departing) {
    UltrasonicReading r; Task2RoutePose p;
    if (FreshUSS(a, now, &r, &p)) {
        SensorStatus status = SensorReading_Status(&r, now);
        Task2ObstacleObservation o = {.valid = status == SENSOR_OK,
            .fault = status != SENSOR_OK && status != SENSOR_TIMEOUT && status != SENSOR_OUT_OF_RANGE,
            .value = r.distanceMm, .pose = p};
        Task2ObstacleDetection_Observe(&a->detection, &a->sensors, &a->route.config,
            TASK2_SENSOR_ULTRASONIC, &o, departing);
        (void)Task2App_FrozenUSS(a, status == SENSOR_OK, r.distanceMm, Task2App_FaceRange(a, &p));
        if (!departing && status == SENSOR_OK) RecordNearFace(a, &p, &r);
    }
    return !a->detection.fault && !a->ussFrozen;
}
static bool ObserveIR(Task2App *a, uint32_t now) {
    IRPairReading r = a->io->infrared(a->io->context);
    if (r.sequence == a->irSequence) return true;
    a->irSequence = r.sequence;
    bool right = a->geometry.secondArrow == TASK2_ARROW_LEFT;
    IRReading sample = right ? r.right : r.left;
    SensorStatus status = SensorReading_IRStatus(&sample, now);
    Task2RoutePose p;
    if (status == SENSOR_STALE || status == SENSOR_NOT_READY || !CapturedPose(a, sample.capturedMs, &p)) return true;
    a->lastIRObservedMs = now;
    a->irHistoryAdc[a->irHistoryCount++ % TASK2_DIAG_IR_HISTORY] = sample.adc;
    Task2ObstacleObservation o = {.valid = status == SENSOR_OK, .fault = status != SENSOR_OK,
        .value = sample.adc, .pose = p};
    Task2ObstacleDetection_Observe(&a->detection, &a->sensors, &a->route.config,
        right ? TASK2_SENSOR_RIGHT_IR : TASK2_SENSOR_LEFT_IR, &o, true);
    return !a->detection.fault;
}
static Task2RouteIRDeparture IRPlanParameters(const Task2App *a) {
    Task2RouteIRDeparture ir = {
        .mount = a->geometry.secondArrow == TASK2_ARROW_LEFT ? a->sensors.rightIR : a->sensors.leftIR,
        .edgeUncertaintyMm = a->sensors.edgeUncertaintyMm,
        .acquisitionMinYawDeg = a->sensors.irAcquisitionMinYawDeg,
    };
    float v = a->geometry.obstacle2DepartureSpeedCps * a->io->mmPerCount;
    ir.guardTravelMm = v * ((a->sensors.absentSamples + 1U) * a->sensors.samplePeriodMs + 10U) / 1000.0f +
        v * v / (2.0f * a->io->decelerationMmps2) + a->geometry.secondIRDepartureGuardExtraMm;
    ir.acquisitionTravelMm = fminf(a->geometry.obstacle2DepartureSpeedCps,
        a->geometry.obstacle2IRAcquisitionSpeedCps) * a->io->mmPerCount *
        (a->sensors.presentSamples * a->sensors.samplePeriodMs + 10U) / 1000.0f;
    return ir;
}
static bool PlanO2(Task2App *a, Task2Commands *t) {
    a->geometry.secondArrow = (Task2Arrow)Task2Commands_Arrow(t, 2U);
    uint16_t span;
    if (Task2Commands_Span(t, &span)) Task2Estimate_SetSpan(&a->estimates, span);
    if (!Task2Estimate_Apply(&a->estimates, &a->geometry)) return false;
    Task2RouteIRDeparture ir = IRPlanParameters(a);
    PlanMark m = Mark(a);
    bool ok = Task2Route_CreateFromPose(&a->geometry,
        a->geometry.firstArrow == a->geometry.secondArrow ? NULL : &ir, &a->pose, &a->route);
    Note(a, TASK2_DIAG_PLAN, "PLAN_O2", ok, NULL, &a->route, NULL, &m);
    return ok;
}
static bool PrepareDeparture(Task2App *a, Task2Commands *t) {
    uint16_t span;
    if (Task2Commands_Span(t, &span)) Task2Estimate_SetSpan(&a->estimates, span);
    if (!Task2Estimate_Apply(&a->estimates, &a->geometry)) return false;
    /* The same-side diagonal runs wide before O2's end is measured; plan it on proof alone. */
    a->sameSideDeparture = a->geometry.firstArrow == a->geometry.secondArrow;
    ApplyArena(a);
    return true;
}
static const char *StartDeparture(Task2App *a, Task2Commands *t, uint32_t now);
static const char *CommitDeparture(Task2App *a, Task2Commands *t, uint32_t now) {
    if (!PrepareDeparture(a, t)) return "T2_ESTIMATE";
    Task2RouteIRDeparture ir = IRPlanParameters(a);
    PlanMark m = Mark(a);
    bool ok = PlanDeparture(a, &a->route, &a->geometry, &a->pose, a->route.obstacle2IRDeparture ? &ir : NULL);
    if (a->planCancelled) return NULL;
    Note(a, TASK2_DIAG_PLAN, "DEPARTURE", ok, NULL, &a->route, NULL, &m);
    if (!ok) return a->route.report.reason;
    return StartDeparture(a, t, now);
}
static const char *StartDeparture(Task2App *a, Task2Commands *t, uint32_t now) {
    Enter(a, TASK2_APP_ROUTE, now);
    Task2Commands_SetStage(t, TASK2_STAGE_ROUND2);
    if (a->route.obstacle2LateralDeparture)
        Task2ObstacleDetection_BeginUSSDeparture(&a->detection, &a->sensors);
    return Launch(a, &a->route.steps[a->step], now);
}

/* Same-side departure: keep the obstacle buffer between the leading front corner and O2's face plane.
 * The plane comes from the latest valid USS range at its own capture pose (shortest ray of the cone);
 * the corner from the current pose. Allows one sample period plus a control tick of travel before the
 * brake acts, and the braking distance. */
static bool FrontGuardTripped(const Task2App *a) {
    const Task2ObstacleDetection *d = &a->detection;
    if (!d->ultrasonicValid || !isfinite(d->lastUltrasonicRangeMm)) return false;
    const Task2SensorMount *m = &a->sensors.ultrasonic;
    Task2RoutePose c = d->lastUltrasonicPose;
    float sx = c.xMm + m->aheadMm * cosf(c.yawRad) - m->leftMm * sinf(c.yawRad);
    float ray = fabsf(c.yawRad + m->yawDeg * PI / 180.0f) + a->sensors.ultrasonicBeamHalfAngleDeg * PI / 180.0f;
    if (ray >= 0.5f * PI) return false;
    float face = sx + d->lastUltrasonicRangeMm * cosf(ray);
    const Task2RouteConfig *g = &a->geometry;
    float f = g->bodyCenterAheadOfRearAxleMm + 0.5f * g->bodyLengthMm + 0.5f * g->frontProtrusionMm + 0.5f * g->frontProtrusionMm;
    float corner = a->pose.xMm + f * cosf(a->pose.yawRad) + 0.5f * g->bodyWidthMm * fabsf(sinf(a->pose.yawRad));
    float v = g->obstacle2DepartureSpeedCps * a->io->mmPerCount;
    float stop = v * (a->sensors.samplePeriodMs + 10U) / 1000.0f + v * v / (2.0f * a->io->decelerationMmps2);
    return face - corner < g->obstacleBufferMm + stop;
}
/* Face x from the latest fresh ultrasonic echo at its own capture pose; NAN if there is none. With
 * `nearest`, the cone's most oblique ray (the nearest the face can be, for braking); otherwise the ray
 * closest to square-on (the farthest it can be, so a correction only moves it in when proven). */
static float FaceFromRange(const Task2App *a, Task2RoutePose c, float range, bool nearest) {
    const Task2SensorMount *m = &a->sensors.ultrasonic;
    float sx = c.xMm + m->aheadMm * cosf(c.yawRad) - m->leftMm * sinf(c.yawRad);
    float yaw = fabsf(c.yawRad + m->yawDeg * PI / 180.0f), half = a->sensors.ultrasonicBeamHalfAngleDeg * PI / 180.0f;
    float ray = nearest ? yaw + half : fmaxf(0.0f, yaw - half);
    return ray < 0.5f * PI && isfinite(range) ? sx + range * cosf(ray) : NAN;
}
static float NearFaceFromUSS(const Task2App *a, uint32_t now, bool nearest) {
    const Task2ObstacleDetection *d = &a->detection;
    if (!d->ultrasonicValid || now - a->lastUSSCapturedMs > USS_FRESH_MS)
        return NAN;
    return FaceFromRange(a, d->lastUltrasonicPose, d->lastUltrasonicRangeMm, nearest);
}
/* Median face x of the echoes captured in the last second, if there are at least three. */
static float QualifiedNearFace(const Task2App *a, uint32_t now) {
    float values[TASK2_NEAR_FACE_SAMPLES];
    unsigned n = 0U, stored = a->nearFaceCount < TASK2_NEAR_FACE_SAMPLES ? a->nearFaceCount : TASK2_NEAR_FACE_SAMPLES;
    for (unsigned i = 0U; i < stored; ++i) {
        if (now - a->nearFaceHistoryMs[i] > 1000U || !isfinite(a->nearFaceHistoryMm[i])) continue;
        float v = a->nearFaceHistoryMm[i];
        unsigned j = n++;
        while (j && values[j - 1U] > v) { values[j] = values[j - 1U]; --j; }
        values[j] = v;
    }
    return n >= 3U ? values[n / 2U] : NAN;
}
/* O2 approach: brake when an echo proves the face is nearer than planned (even its farthest reading is
 * nearer by the tolerance) and puts it within the obstacle buffer plus the stopping distance of the
 * leading corner from the step's commanded speed. A run whose estimate is right never trips it. */
static bool ApproachBumper(const Task2App *a, uint32_t now, float speedCps) {
    float farthest = NearFaceFromUSS(a, now, false);
    float estimate = a->geometry.parkToObstacle1Mm + 100.0f + a->geometry.obstacleFaceGapMm;
    if (!isfinite(farthest) || farthest >= estimate - NEAR_FACE_TOLERANCE_MM) return false;
    float face = NearFaceFromUSS(a, now, true);
    if (!isfinite(face)) return false;
    const Task2RouteConfig *g = &a->geometry;
    float f = g->bodyCenterAheadOfRearAxleMm + 0.5f * g->bodyLengthMm + g->frontProtrusionMm;
    float corner = a->pose.xMm + f * cosf(a->pose.yawRad) + 0.5f * g->bodyWidthMm * fabsf(sinf(a->pose.yawRad));
    float v = speedCps * a->io->mmPerCount;
    float stop = v * (a->sensors.samplePeriodMs + 10U) / 1000.0f + v * v / (2.0f * a->io->decelerationMmps2);
    return face - corner < g->obstacleBufferMm + stop;
}
static float FaceEstimate(const Task2App *a) {
    return a->estimates.obstacle1Face.mm + 100.0f + a->estimates.faceGap.mm;
}
/* End of the O2 approach: when the median of at least three recent echoes puts the face nearer than the
 * estimate, it replaces it (bounded below by the rules' 600 mm gap). One echo never does: a stray near
 * reflection must not overwrite the camera's face. Returns true when the estimate moved. */
static bool CorrectFaceFromEcho(Task2App *a, uint32_t now) {
    a->nearFaceMm = NearFaceFromUSS(a, now, false);
    float qualified = QualifiedNearFace(a, now);
    if (a->faceCorrected || !isfinite(qualified) || !a->estimates.faceGap.valid ||
        qualified >= FaceEstimate(a) - NEAR_FACE_TOLERANCE_MM)
        return false;
    float gap = fmaxf(600.0f, qualified - a->estimates.obstacle1Face.mm - 100.0f);
    if (gap >= a->estimates.faceGap.mm - 1.0f) /* already at the rules' nearest: nothing to move */
        return false;
    if (!Task2Estimate_SetGap(&a->estimates, gap, 20.0f, TASK2_ESTIMATE_ULTRASONIC) ||
        !Task2Estimate_Apply(&a->estimates, &a->geometry))
        return false;
    /* The detector compares echoes with the route's face: move it too, and lock on again from scratch. */
    a->route.config.obstacleFaceGapMm = a->geometry.obstacleFaceGapMm;
    Task2ObstacleDetection *d = &a->detection;
    d->armed[TASK2_SENSOR_ULTRASONIC] = false;
    d->present[TASK2_SENSOR_ULTRASONIC] = 0U;
    d->ultrasonicHistoryCount = 0U;
    d->faceHitCount = 0U;
    a->frozenRangeMm = a->frozenExpectedMm = NAN; /* the expected range jumped, the sensor did not */
    a->nearFaceCount = 0U;
    a->faceCorrected = true;
    return true;
}
/* Depart on the ultrasonic lock only with a fresh echo that is not nearer than the face estimate. A
 * farther echo is accepted deliberately: at an angle the ultrasonic misses the face and times something
 * beyond, so it never moves the estimate out or stops the run. */
static bool DepartureEchoAgrees(const Task2App *a) {
    /* With the gap already at the rules' 600 mm minimum, O2 cannot be nearer: a short echo is the
     * ultrasonic's own error, not a reason to wait. */
    bool atMinimum = a->estimates.faceGap.mm <= 600.0f + 1.0f;
    return a->detection.armed[TASK2_SENSOR_ULTRASONIC] && isfinite(a->nearFaceMm) &&
        (atMinimum || a->nearFaceMm >= FaceEstimate(a) - NEAR_FACE_TOLERANCE_MM);
}
/* Departure after a corrected face: from the stop if it fits, otherwise reverse the shortest checked
 * distance from which it plans and plan again there. */
static const char *CommitCorrectedDeparture(Task2App *a, Task2Commands *t, uint32_t now) {
    if (!PrepareDeparture(a, t)) return "T2_ESTIMATE";
    Task2RouteIRDeparture ir = IRPlanParameters(a);
    planScratch = a->route;
    PlanMark m = Mark(a);
    bool fits = PlanDeparture(a, &planScratch, &a->geometry, &a->pose, a->route.obstacle2IRDeparture ? &ir : NULL);
    if (a->planCancelled) return NULL;
    Note(a, TASK2_DIAG_PLAN, "DEPARTURE", fits, NULL, &planScratch, NULL, &m);
    if (fits) {
        a->route = planScratch;
        return StartDeparture(a, t, now);
    }
    const char *reason = planScratch.report.reason;
    m = Mark(a);
    for (unsigned i = 0U; i < sizeof departureBackupMm / sizeof departureBackupMm[0]; ++i) {
        float back = departureBackupMm[i];
        Task2RoutePose stop = a->pose;
        stop.xMm -= back * cosf(stop.yawRad);
        stop.yMm -= back * sinf(stop.yawRad);
        if (!Task2Route_CheckStraight(&a->geometry, &a->pose, -back)) continue;
        planScratch = a->route;
        fits = PlanDeparture(a, &planScratch, &a->geometry, &stop, a->route.obstacle2IRDeparture ? &ir : NULL);
        if (a->planCancelled) return NULL;
        if (!fits) continue;
        a->departureBackingUp = true;
        Task2RouteStep reverse = {.distanceMm = -back, .speedCps = a->geometry.obstacle2DepartureSpeedCps};
        NoteNext(a, Note(a, TASK2_DIAG_PLAN, "DEPARTURE_BACKUP", true, NULL, &planScratch, &stop, &m), &reverse);
        Enter(a, TASK2_APP_ROUTE, now);
        return Launch(a, &reverse, now);
    }
    Note(a, TASK2_DIAG_PLAN, "DEPARTURE_BACKUP", false, "NO_BACKUP_FITS", &a->route, NULL, &m);
    return reason ? reason : "NO_DEPARTURE_FROM_STOP";
}
/* The probe lane ended (or the guard braked) with the face still seen: replan a wider lane from the
 * stop. Fails rather than guesses when no certified departure exists. */
/* Same-side, from a stop on the diagonal: run along the face with the side IR to measure the end. */
static const char *TraverseFromStop(Task2App *a, uint32_t now) {
    Task2RouteIRDeparture ir = IRPlanParameters(a);
    PlanMark m = Mark(a);
    bool ok = PlanTraverse(a, &a->route, &a->geometry, &a->pose, &ir);
    if (a->planCancelled) return NULL;
    Note(a, TASK2_DIAG_PLAN, "TRAVERSE", ok, NULL, &a->route, NULL, &m);
    if (!ok) {
        /* Too close to the face to turn along it: reverse the shortest clear distance from which
         * the traverse plans, then plan it again at the actual stop. Checked on a scratch route. */
        m = Mark(a);
        for (unsigned i = 0U; i < sizeof traverseBackupMm / sizeof traverseBackupMm[0]; ++i) {
            float back = traverseBackupMm[i];
            Task2RoutePose stop = a->pose;
            stop.xMm -= back * cosf(stop.yawRad);
            stop.yMm -= back * sinf(stop.yawRad);
            planScratch = a->route;
            if (!Task2Route_CheckStraight(&a->geometry, &a->pose, -back)) continue;
            bool fits = PlanTraverse(a, &planScratch, &a->geometry, &stop, &ir);
            if (a->planCancelled) return NULL;
            if (!fits) continue;
            a->backingUp = true;
            Task2RouteStep reverse = {.distanceMm = -back, .speedCps = a->geometry.obstacle2DepartureSpeedCps};
            NoteNext(a, Note(a, TASK2_DIAG_PLAN, "TRAVERSE_BACKUP", true, NULL, &planScratch, &stop, &m), &reverse);
            return Launch(a, &reverse, now);
        }
        Note(a, TASK2_DIAG_PLAN, "TRAVERSE_BACKUP", false, "NO_BACKUP_FITS", &a->route, NULL, &m);
        return "T2_NO_EDGE";
    }
    a->geometry.obstacle2WidthMm = a->route.config.obstacle2WidthMm;
    ApplyArena(a);
    a->step = TASK2_ROUTE_O2_DEPARTURE_STEP;
    return Launch(a, &a->route.steps[a->step], now);
}
static const char *ExtendDeparture(Task2App *a, uint32_t now) {
    if (++a->departureExtensions > MAX_DEPARTURE_EXTENSIONS) return "T2_NO_EDGE";
    float width = 2.0f * (a->provenHalfMm + DEPARTURE_EXTENSION_MM);
    if (width > a->geometry.obstacle2WidthMm) a->geometry.obstacle2WidthMm = width;
    ApplyArena(a);
    PlanMark m = Mark(a);
    bool ok = PlanDeparture(a, &a->route, &a->geometry, &a->pose, NULL);
    if (a->planCancelled) return NULL;
    Note(a, TASK2_DIAG_PLAN, "EXTEND", ok, NULL, &a->route, NULL, &m);
    /* No wider diagonal fits: measure the end along the face instead. */
    if (!ok) return TraverseFromStop(a, now);
    a->step = TASK2_ROUTE_O2_DEPARTURE_STEP;
    return Launch(a, &a->route.steps[a->step], now);
}

/* Lateral coordinate where the traverse IR's ray meets O2's face plane from pose p. */
static float TraverseHit(const Task2App *a, const Task2RoutePose *p, const Task2RouteIRDeparture *ir) {
    float co = cosf(p->yawRad), si = sinf(p->yawRad);
    float x = p->xMm + ir->mount.aheadMm * co - ir->mount.leftMm * si;
    float y = p->yMm + ir->mount.aheadMm * si + ir->mount.leftMm * co;
    float ray = p->yawRad + ir->mount.yawDeg * PI / 180.0f;
    float face = a->geometry.parkToObstacle1Mm + 100.0f + a->geometry.obstacleFaceGapMm;
    return a->geometry.secondArrow * (y + (face - x) * tanf(ray));
}
/* Reverse along the same-side traverse lane until the IR points at the proven face with room to
 * acquire it, then run the traverse again so it measures the end and proves the face up to it. */
static const char *RewindTraverse(Task2App *a, uint32_t now) {
    if (++a->traverseRewinds > MAX_TRAVERSE_REWINDS) return "T2_NO_EDGE";
    Task2RouteIRDeparture ir = IRPlanParameters(a);
    float target = a->provenHalfMm - ir.acquisitionTravelMm - REWIND_FACE_MARGIN_MM;
    float hit = TraverseHit(a, &a->pose, &ir);
    Task2RoutePose back = a->pose;
    back.xMm -= 100.0f * cosf(back.yawRad); back.yMm -= 100.0f * sinf(back.yawRad);
    float perMm = (hit - TraverseHit(a, &back, &ir)) / 100.0f;
    if (!(perMm > 0.1f) || !isfinite(hit) || hit <= target) return "T2_NO_EDGE";
    float distance = (hit - target) / perMm;
    /* The stop may sit inside the side buffer once the end measurement shrinks the limit. Reversing
     * here only moves inward, so the buffer may be relaxed to physical clearance for the check. */
    if (a->geometry.secondArrow * sinf(a->pose.yawRad) <= 0.0f) return "T2_NO_EDGE";
    Task2RouteConfig inward = a->geometry;
    inward.boundaryBufferMm = 1.0f;
    if (!Task2Route_CheckStraight(&inward, &a->pose, -distance)) return "T2_NO_EDGE";
    a->rewinding = true;
    a->rewindMm = distance;
    Task2RouteStep reverse = {.distanceMm = -distance, .speedCps = a->geometry.obstacle2DepartureSpeedCps};
    NoteNext(a, Note(a, TASK2_DIAG_PLAN, "REWIND", true, NULL, &a->route, NULL, NULL), &reverse);
    return Launch(a, &reverse, now);
}

#if TASK2_DIAGNOSTICS
static int16_t Clamp16(float v) {
    return (int16_t)lroundf(fmaxf(-32767.0f, fminf(32767.0f, isfinite(v) ? v : 0.0f)));
}
static uint16_t IRValue(const IRReading *r, uint32_t now) {
    return SensorReading_IRStatus(r, now) == SENSOR_OK ? r->adc : 65535U;
}
/* One run-trace sample: where the robot is, what it is doing and what its sensors read, so a floor run can
 * be followed between its decisions. */
/* Samples the sensor snapshot taken at the start of the update, judged at that update's time: the final sample
 * comes after the update's planning, which can run long, and a later reading has no time to be judged against. */
static void TraceSample(Task2App *a, uint32_t now, uint8_t state) {
    if (!a->io) return;
    const UltrasonicReading u = a->traceUss;
    const IRPairReading ir = a->traceIr;
    uint32_t ussNow = now, irNow = now;
    const Task2ObstacleDetection *d = &a->detection;
    unsigned endIR = a->geometry.secondArrow == TASK2_ARROW_LEFT ? TASK2_SENSOR_RIGHT_IR : TASK2_SENSOR_LEFT_IR;
    uint32_t t10 = (now - a->traceStartMs) / 10U;
    Task2TraceSample s = {
        .t10Ms = (uint16_t)(t10 > 65535U ? 65535U : t10),
        .xMm = Clamp16(a->pose.xMm), .yMm = Clamp16(a->pose.yMm),
        .yawCentiDeg = Clamp16(remainderf(a->pose.yawRad * 180.0f / PI, 360.0f) * 100.0f),
        .phase = (uint8_t)a->phase, .step = (uint8_t)(a->step > 255U ? 255U : a->step),
        .flags = (uint8_t)((d->armed[TASK2_SENSOR_ULTRASONIC] ? TASK2_TRACE_USS_ARMED : 0U) |
            (d->armed[endIR] ? TASK2_TRACE_IR_ARMED : 0U) | (d->detected ? TASK2_TRACE_DETECTED : 0U) |
            (d->reacquire ? TASK2_TRACE_REACQUIRE : 0U) | (a->edgeBraking ? TASK2_TRACE_EDGE_BRAKE : 0U) |
            (a->approachBraking ? TASK2_TRACE_APPROACH_BRAKE : 0U) |
            (a->faceCorrected ? TASK2_TRACE_FACE_CORRECTED : 0U) |
            (a->io->busy(a->io->context) ? TASK2_TRACE_MOVING : 0U)),
        .state = state,
        .ussMm = SensorReading_Status(&u, ussNow) == SENSOR_OK && u.distanceMm < 65535.0f
            ? (uint16_t)lroundf(u.distanceMm) : 65535U,
        .irLeft = IRValue(&ir.left, irNow), .irRight = IRValue(&ir.right, irNow), .faceLevel = d->irFaceLevelAdc,
    };
    Task2Trace_Add(&a->trace, &s);
}
static void Trace(Task2App *a, uint32_t now) {
    a->traceUss = a->io->ultrasonic(a->io->context);
    a->traceIr = a->io->infrared(a->io->context);
    if (now - a->traceLastMs < TASK2_TRACE_PERIOD_MS) return;
    a->traceLastMs = now;
    TraceSample(a, now, 0U);
}
/* The run's last sample, at its fault, finish or abort, whatever the period. */
static void TraceFinal(Task2App *a) {
    TraceSample(a, a->previousMs, TASK2_TRACE_STATE_FINAL);
}
#endif

const char *Task2App_Start(Task2App *a, const Task2AppIo *io, uint32_t now) {
    memset(a, 0, sizeof(*a));
    a->io = io; a->geometry = task2RouteConfig; a->sensors = task2AppSensorDefaults;
    a->provenHalfMm = 0.5f * RULE_O2_MIN_LENGTH_MM;
    a->frozenRangeMm = a->frozenExpectedMm = a->rejectedRangeMm = NAN;
    a->cameraFaceMm = a->ussFaceMm = a->nearFaceMm = NAN;
#if TASK2_DIAGNOSTICS
    Task2Diag_Init(&a->diag);
    Task2Trace_Init(&a->trace);
    a->traceStartMs = now;
    a->traceLastMs = now - TASK2_TRACE_PERIOD_MS;
#endif
    ApplyArena(a);
    a->pose.xMm = TASK2_ROUTE_START_X_MM;
    a->phaseStartedMs = a->previousMs = a->lastUSSObservedMs = a->lastIRObservedMs = now;
    if (!io || !io->moveStraight || !io->moveArc || !io->moveContinuous || !io->busy ||
        !io->brake || !io->requestSpeed || !io->motion || !io->ultrasonic || !io->infrared ||
        !isfinite(io->mmPerCount) || io->mmPerCount <= 0.0f ||
        !isfinite(io->decelerationMmps2) || io->decelerationMmps2 <= 0.0f)
        return "T2_IO";
    if (io->busy(io->context)) return "T2_BUSY";
    Task2Estimate_Init(&a->estimates, &a->geometry);
    /* Every run requires new measurements, even if the snapshot was valid at START. */
    a->ultrasonicSequence = io->ultrasonic(io->context).sequence;
    a->irSequence = io->infrared(io->context).sequence;
    RememberPose(a, now);
    return NULL;
}
void Task2App_NoteFault(Task2App *a, const char *reason) {
    if (a->io && !a->fault && a->phase != TASK2_APP_FINISHED && a->phase != TASK2_APP_ABORTED) {
        Note(a, TASK2_DIAG_FAULT, "EXTERNAL", false, reason, &a->route, NULL, NULL);
#if TASK2_DIAGNOSTICS
        TraceFinal(a);
#endif
        a->fault = reason;
    }
}

void Task2App_Abort(Task2App *a) {
    if (a->io && !a->fault && a->phase != TASK2_APP_FINISHED && a->phase != TASK2_APP_ABORTED) {
        Note(a, TASK2_DIAG_STOP, "ABORT", false, NULL, &a->route, NULL, NULL);
#if TASK2_DIAGNOSTICS
        TraceFinal(a);
#endif
    }
    a->phase = TASK2_APP_ABORTED;
    a->primitive = false;
    a->io = NULL;
}

Task2AppStatus Task2App_Update(Task2App *a, Task2Commands *t, uint32_t now) {
    if (!a->io || a->phase == TASK2_APP_ABORTED) return TASK2_APP_FAULT;
    if (a->planCancelled) return TASK2_APP_RUNNING;   /* stopped while planning: the stop aborts the run next */
    if (a->fault) return TASK2_APP_FAULT;
    if (a->phase == TASK2_APP_FINISHED) return TASK2_APP_DONE;
    if (a->phase != TASK2_APP_WAIT1 && a->phase != TASK2_APP_WAIT2)
        a->activeMs += now - a->previousMs;
    a->previousMs = now;
    if (a->activeMs >= ACTIVE_TIMEOUT_MS) return Fail(a, "T2_TIMEOUT");
    if (!UpdatePose(a)) return Fail(a, "ODOMETRY");
    RememberPose(a, now);
#if TASK2_DIAGNOSTICS
    Trace(a, now);
#endif
    ApplyArena(a);
    if (a->phase >= TASK2_APP_RANGE2 && a->phase <= TASK2_APP_ROUTE) {
        Task2RouteConfig bounds = a->route.returnWidthMm > 0.0f
            ? Task2Route_ReturnBounds(&a->route) : a->geometry;
        if (a->phase == TASK2_APP_ROUTE && a->route.obstacle2IRDeparture &&
            a->step >= TASK2_ROUTE_O2_DEPARTURE_STEP && t->stage != TASK2_STAGE_HOME)
            bounds.obstacleBufferMm = 1.0f; /* Existing certified IR bypass exception. */
        Task2RouteReport report = {.minimumObstacleGapMm = INFINITY, .minimumSideMarginMm = INFINITY,
            .minimumParkSideMarginMm = INFINITY, .minimumParkBackMarginMm = INFINITY};
        if (!Task2Route_CheckPose(&bounds, &a->pose, &report)) {
            /* Car park walls disqualify on contact, and a wall may stand past O2's ends: those still stop.
             * O1/O2 do not: a touch costs time, stopping fails the run, so the estimate is recorded once
             * and the route goes on. */
            if (report.minimumParkSideMarginMm < bounds.parkSideWallBufferMm ||
                report.minimumParkBackMarginMm < bounds.parkBackWallBufferMm)
                return FailMeasured(a, "T2_HARD_PARK_CLEARANCE", &report);
            bool sideShort = report.minimumSideMarginMm < 1.0f ||
                (report.minimumSideMarginMm < bounds.boundaryBufferMm &&
                 (a->phase != TASK2_APP_ROUTE || a->step < TASK2_ROUTE_O2_DEPARTURE_STEP));
            if (sideShort) return FailMeasured(a, "T2_CLEARANCE", &report);
            if (report.minimumObstacleGapMm < bounds.obstacleBufferMm && !a->obstacleShortfalls++)
                NoteRuntime(a, TASK2_DIAG_PLAN, "OBSTACLE_SHORTFALL", "T2_OBSTACLE_CLEARANCE_LOW", &report);
            a->route.report.runtimeReducedClearanceUsed = true;
        }
    }
    bool busy = a->io->busy(a->io->context);
    const char *fault = NULL;
    switch (a->phase) {
    case TASK2_APP_RANGE1: {
        float face;
        if (FaceWindow(a, now, &face)) {
            if (!Task2Estimate_SetFace(&a->estimates, 1U, face, 20.0f)) {
                a->rejectedRangeMm = face;
                return Fail(a, "T2_O1_RANGE");
            }
            /* The estimate holds the reading taken into the course's range; a raw 590 would fail the planner. */
            a->geometry.parkToObstacle1Mm = a->estimates.obstacle1Face.mm;
            PlanMark m = Mark(a);
            bool ok = Task2Route_CreateObstacle1(&a->geometry, &a->route);
            Note(a, TASK2_DIAG_PLAN, "PLAN_O1", ok, NULL, &a->route, NULL, &m);
            if (!ok) return Fail(a, a->route.report.reason);
            Enter(a, TASK2_APP_APPROACH1, now);
            fault = Launch(a, &a->route.steps[0], now);
        } else if (now - a->phaseStartedMs >= RANGE_TIMEOUT_MS) return Fail(a, "T2_O1_RANGE");
        break;
    }
    case TASK2_APP_APPROACH1:
        if (busy && !a->io->motion(a->io->context).braking && Task2Commands_Arrow(t, 1U)) fault = a->io->requestSpeed(a->io->context, a->geometry.cruiseSpeedCps);
        if (!busy) {
            a->primitive = false; Enter(a, TASK2_APP_WAIT1, now);
            if (!Task2Commands_Arrow(t, 1U)) Task2Commands_SetStage(t, TASK2_STAGE_WAIT1);
        }
        break;
    case TASK2_APP_WAIT1:
        if (Task2Commands_Arrow(t, 1U)) {
            a->geometry.firstArrow = (Task2Arrow)Task2Commands_Arrow(t, 1U);
            PlanMark m = Mark(a);
            bool ok = Task2Route_CreateObstacle1(&a->geometry, &a->route);
            Note(a, TASK2_DIAG_PLAN, "PLAN_O1_ARROW", ok, NULL, &a->route, NULL, &m);
            if (!ok) return Fail(a, a->route.report.reason);
            a->step = 1U; Enter(a, TASK2_APP_SWERVE1, now); Task2Commands_SetStage(t, TASK2_STAGE_SWERVE1);
            fault = Launch(a, &a->route.steps[a->step], now);
        }
        break;
    case TASK2_APP_SWERVE1:
        if (!busy) {
            a->primitive = false;
            if (++a->step == 3U) {
                /* Both O1 arcs have stopped; camera is past the O1 front face. */
                if (a->pose.xMm <= a->geometry.parkToObstacle1Mm) return Fail(a, "T2_O1_EXIT");
                a->faceCount = 0U; Enter(a, TASK2_APP_RANGE2, now);
                Task2Commands_SetStage(t, TASK2_STAGE_APPROACH2);
            } else fault = Launch(a, &a->route.steps[a->step], now);
        }
        break;
    case TASK2_APP_RANGE2: {
        /* O2's face from the camera: the ultrasonic, reaching the face at this angle, can miss its echo and
         * time something far beyond. The ultrasonic is used only when no camera depth comes in time. */
        float face;
        if (FaceWindow(a, now, &face)) a->ussFaceMm = face;
        uint16_t depth;
        if (!a->estimates.faceGap.valid && !isfinite(a->cameraFaceMm) && Task2Commands_Depth(t, &depth)) {
            a->cameraFaceMm = Task2Route_CameraFaceX(&a->geometry, &a->pose, (float)depth);
            /* Refused (outside the course): the ultrasonic decides. */
            if (isfinite(a->cameraFaceMm)) (void)Task2Estimate_SetFace(&a->estimates, 2U, a->cameraFaceMm, 20.0f);
        }
        bool waited = Task2Commands_Arrow(t, 2U) || now - a->phaseStartedMs >= CAMERA_DEPTH_WAIT_MS;
        if (a->estimates.faceGap.valid || (waited && isfinite(a->ussFaceMm))) {
            if (!a->estimates.faceGap.valid && !Task2Estimate_SetFace(&a->estimates, 2U, a->ussFaceMm, 20.0f)) {
                /* Outside the course: at this angle a long reading is usually a missed face. Plan for the
                 * nearest legal end of the range it points to; the approach bumper finds the real face. */
                a->rejectedRangeMm = isfinite(a->cameraFaceMm) ? a->cameraFaceMm : a->ussFaceMm;
                float gap = a->ussFaceMm - a->estimates.obstacle1Face.mm - 100.0f;
                if (!isfinite(gap) || !Task2Estimate_SetGap(&a->estimates, gap > 1500.0f ? 1500.0f : 600.0f, 20.0f,
                                                            TASK2_ESTIMATE_ULTRASONIC))
                    return Fail(a, "T2_GAP_RANGE");
                Note(a, TASK2_DIAG_PLAN, "GAP_CLAMPED", true, "T2_GAP_RANGE", &a->route, NULL, NULL);
            }
            if (!Task2Estimate_Apply(&a->estimates, &a->geometry)) return Fail(a, "T2_ESTIMATE");
            /* Known direction can depart immediately when its geometry allows it. */
            if (Task2Commands_Arrow(t, 2U)) {
                Enter(a, TASK2_APP_WAIT2, now);
            } else {
                float distance = Task2Route_RecognitionApproach(&a->geometry, &a->pose);
                if (!isfinite(distance) || !Task2Route_CheckStraight(&a->geometry, &a->pose, distance))
                    return Fail(a, "T2_CAMERA_APPROACH");
                Task2RouteStep straight = {.distanceMm = distance, .speedCps = a->geometry.cruiseSpeedCps};
                Enter(a, TASK2_APP_RECOGNITION2, now); fault = Launch(a, &straight, now);
            }
        } else if (now - a->phaseStartedMs >= RANGE_TIMEOUT_MS) return Fail(a, "T2_GAP_RANGE");
        break;
    }
    case TASK2_APP_RECOGNITION2: {
        /* Stop early for a real camera arrow. Further approach, if needed for an
         * opposite-arrow turn, is selected by the planner from that measured stop. */
        if (busy && Task2Commands_Arrow(t, 2U)) a->io->brake(a->io->context);
        /* This approach was sized from the RANGE2 face, which a lost echo can put far beyond O2. Echoes on
         * the way that prove the face nearer move it in and stop here; no turn is chosen before the arrow. */
        UltrasonicReading r; Task2RoutePose p;
        if (busy && !a->faceCorrected && FreshUSS(a, now, &r, &p) && SensorReading_Status(&r, now) == SENSOR_OK) {
            RecordNearFace(a, &p, &r);
            if (CorrectFaceFromEcho(a, now)) {
                Note(a, TASK2_DIAG_PLAN, "NEAR_FACE", true, "T2_FACE_NEARER_THAN_PLANNED", &a->route, NULL, NULL);
                a->io->brake(a->io->context);
            }
        }
        if (!busy) {
            a->primitive = false; Enter(a, TASK2_APP_WAIT2, now);
            if (!Task2Commands_Arrow(t, 2U)) Task2Commands_SetStage(t, TASK2_STAGE_WAIT2);
        }
        break;
    }
    case TASK2_APP_WAIT2:
        if (Task2Commands_Arrow(t, 2U)) {
            if (!PlanO2(a, t)) return Fail(a, a->route.report.reason ? a->route.report.reason : "T2_ESTIMATE");
            a->detection = (Task2ObstacleDetection){0};
            a->lastUSSObservedMs = now;
            a->step = TASK2_ROUTE_O2_APPROACH_STEP;
            Enter(a, TASK2_APP_ROUTE, now); Task2Commands_SetStage(t, TASK2_STAGE_APPROACH2);
            fault = Launch(a, &a->route.steps[a->step], now);
        }
        break;
    case TASK2_APP_ACQUIRE2:
        if (!ObserveUSS(a, now, false)) return Fail(a, "T2_USS_SENSOR");
        if (CorrectFaceFromEcho(a, now)) {
            Note(a, TASK2_DIAG_PLAN, "NEAR_FACE", true, "T2_FACE_NEARER_THAN_PLANNED", &a->route, NULL, NULL);
            a->phaseStartedMs = now; /* the lock starts again against the corrected face */
        } else if (DepartureEchoAgrees(a)) {
            fault = a->faceCorrected ? CommitCorrectedDeparture(a, t, now) : CommitDeparture(a, t, now);
        } else if (now - a->phaseStartedMs >= a->sensors.acquisitionTimeoutMs) {
            /* Opposite arrows find O2's end with the side IR, so the ultrasonic lock only confirms the
             * face: depart on the estimate rather than end the run. A same-side departure needs the
             * ultrasonic to see the end, so it cannot go without it. */
            if (a->geometry.firstArrow == a->geometry.secondArrow) return Fail(a, "T2_USS_ACQUIRE");
            Note(a, TASK2_DIAG_PLAN, "ACQUIRE_MISSED", true, "T2_USS_ACQUIRE", &a->route, NULL, NULL);
            fault = a->faceCorrected ? CommitCorrectedDeparture(a, t, now) : CommitDeparture(a, t, now);
        }
        break;
    case TASK2_APP_ROUTE: {
        bool observing = a->route.obstacle2ReturnPending;
        bool ir = observing && a->route.obstacle2IRDeparture && a->step == TASK2_ROUTE_O2_IR_STRAIGHT_STEP && !a->rewinding;
        bool uss = observing && (a->step == TASK2_ROUTE_O2_APPROACH_STEP || a->route.obstacle2LateralDeparture);
        /* A recovery reverse moves inward, away from the face: the forward detectors learn nothing from it, and
         * a missing echo then is no end. They resume with the next forward primitive. */
        bool reversing = a->backingUp || a->departureBackingUp || a->rewinding;
        if (!a->edgeBraking) {
            if (uss && !reversing && !ObserveUSS(a, now, a->step >= TASK2_ROUTE_O2_DEPARTURE_STEP)) return Fail(a, "T2_USS_SENSOR");
            if (ir && !ObserveIR(a, now)) return Fail(a, "T2_IR_SENSOR");
            ApplyArena(a);
            if ((uss && !reversing && now - a->lastUSSObservedMs >= SENSOR_ULTRASONIC_STALE_MS) ||
                (ir && now - a->lastIRObservedMs >= SENSOR_IR_STALE_MS)) return Fail(a, "T2_SENSOR_STALE");
            if (ir && busy) {
                unsigned sensor = a->geometry.secondArrow == TASK2_ARROW_LEFT ? TASK2_SENSOR_RIGHT_IR : TASK2_SENSOR_LEFT_IR;
                if (!a->detection.armed[sensor] && now - a->irStartedMs >= a->sensors.acquisitionTimeoutMs)
                    return Fail(a, "T2_IR_ACQUIRE");
                float speed = a->geometry.obstacle2DepartureSpeedCps;
                if (!a->detection.armed[sensor]) speed = fminf(speed, a->geometry.obstacle2IRAcquisitionSpeedCps);
                else if (a->detection.irTrendCandidateValid) speed = fminf(speed, a->geometry.obstacle2IRConfirmationSpeedCps);
                Task2MotionReading r = a->io->motion(a->io->context);
                if (!r.braking) fault = a->io->requestSpeed(a->io->context, speed);
            }
            if (a->detection.detected && observing && a->step >= TASK2_ROUTE_O2_DEPARTURE_STEP && !reversing) {
                a->edgeBraking = true; a->io->brake(a->io->context);
            } else if (ir && a->detection.reacquire && !a->reacquireBraking) {
                a->reacquireBraking = true; a->io->brake(a->io->context);
            } else if (busy && observing && a->route.obstacle2LateralDeparture &&
                       a->step >= TASK2_ROUTE_O2_DEPARTURE_STEP && !a->guardBraking && !reversing && FrontGuardTripped(a)) {
                /* The forward brakes watch the face ahead; a recovery reverse moves away from it and must run
                 * its checked length before the robot plans again. */
                a->guardBraking = true; a->io->brake(a->io->context);
            }
            if (busy && uss && a->step == TASK2_ROUTE_O2_APPROACH_STEP && !a->approachBraking &&
                ApproachBumper(a, now, a->route.steps[a->step].speedCps)) {
                a->approachBraking = true; a->io->brake(a->io->context);
            }
        }
        if (busy) break;
        a->primitive = false;
        if (a->backingUp || a->departureBackingUp || a->rewinding)
            a->lastUSSObservedMs = now;   /* the ultrasonic's freshness restarts with the forward primitive */
        if (a->departureBackingUp) {
            a->departureBackingUp = false;
            fault = CommitDeparture(a, t, now);
            break;
        }
        if (a->backingUp) {
            a->backingUp = false;
            Task2RouteIRDeparture ir = IRPlanParameters(a);
            PlanMark m = Mark(a);
            bool ok = PlanTraverse(a, &a->route, &a->geometry, &a->pose, &ir);
            if (a->planCancelled) return TASK2_APP_RUNNING;
            Note(a, TASK2_DIAG_PLAN, "TRAVERSE_AFTER_BACKUP", ok, NULL, &a->route, NULL, &m);
            if (!ok) return Fail(a, "T2_NO_EDGE");
            a->geometry.obstacle2WidthMm = a->route.config.obstacle2WidthMm;
            ApplyArena(a);
            a->step = TASK2_ROUTE_O2_DEPARTURE_STEP;
            fault = Launch(a, &a->route.steps[a->step], now);
            return fault ? Fail(a, fault) : TASK2_APP_RUNNING;
        }
        if (a->reacquireBraking) {
            /* The IR saw no face beyond the proven face: back onto it and run the traverse again. */
            a->reacquireBraking = false;
            a->detection.reacquire = false;
            Note(a, TASK2_DIAG_PLAN, "REACQUIRE", true, "T2_IR_PAST_PROOF", &a->route, NULL, NULL);
            fault = RewindTraverse(a, now);
            return fault ? Fail(a, fault) : TASK2_APP_RUNNING;
        }
        if (a->rewinding) {
            a->rewinding = false;
            a->detection.detected = false;
            a->detection.observedWidthMm = 0.0f;
            a->route.steps[a->step].distanceMm += a->rewindMm;
            fault = Launch(a, &a->route.steps[a->step], now);
            return fault ? Fail(a, fault) : TASK2_APP_RUNNING;
        }
        if (observing && a->step >= TASK2_ROUTE_O2_DEPARTURE_STEP) {
            if (a->edgeBraking) {
                /* A same-side traverse that started at or near O2's end proves too little face for the
                 * side limit (or, never seeing it, only bounds the end); if no return fits, run it
                 * again from further back. A diagonal whose USS edge leaves no return measures the
                 * end along the face instead. */
                bool sameSide = a->geometry.firstArrow == a->geometry.secondArrow;
                bool rewindable = sameSide && a->route.obstacle2IRDeparture && a->step == TASK2_ROUTE_O2_IR_STRAIGHT_STEP;
                bool traversable = sameSide && a->route.obstacle2LateralDeparture &&
                    ++a->departureExtensions <= MAX_DEPARTURE_EXTENSIONS;
                if (!Task2Estimate_SetEdge(&a->estimates, &a->detection)) return Fail(a, "T2_EDGE");
                /* The end cannot be shorter than the face already proven. If a reading says it is, plan
                 * around the larger of the two: stopping here fails the run, a longer O2 only widens the
                 * way round. */
                if (0.5f * a->detection.observedWidthMm + SENSOR_CONFLICT_TOLERANCE_MM < a->provenHalfMm) {
                    ++a->sensorConflicts;
                    a->detection.observedWidthMm = 2.0f * (a->provenHalfMm + a->sensors.edgeUncertaintyMm);
                    if (!Task2Estimate_SetEdge(&a->estimates, &a->detection)) return Fail(a, "T2_EDGE");
                    Note(a, TASK2_DIAG_PLAN, "SENSOR_CONFLICT", true, "T2_EDGE_WIDENED_TO_PROVEN", &a->route, NULL, NULL);
                }
                a->edgeMeasured = true;
                ApplyArena(a);
                a->route.steps[a->step].distanceMm = fabsf(a->previousDistanceMm);
                planScratch = a->route;
                PlanMark m = Mark(a);
                bool ok = PlanEdgeReturn(a, &planScratch, &a->pose, a->estimates.obstacle2Width.mm, a->step);
                if (a->planCancelled) return TASK2_APP_RUNNING;
                Note(a, TASK2_DIAG_PLAN, "EDGE_RETURN", ok, NULL, &planScratch, NULL, &m);
                float narrow = 2.0f * (a->provenHalfMm + a->sensors.edgeUncertaintyMm);
                if (!ok && !rewindable && !traversable && narrow < a->detection.observedWidthMm) {
                    /* No return fits the measured end. The IR can read "present" past the end when something
                     * else is in its range, so plan around the face it proved instead: a touch costs time,
                     * stopping fails the run. Walls and the car park keep their limits. */
                    a->detection.observedWidthMm = narrow;
                    if (!Task2Estimate_SetEdge(&a->estimates, &a->detection)) return Fail(a, "T2_EDGE");
                    ApplyArena(a);
                    planScratch = a->route;
                    m = Mark(a);
                    ok = PlanEdgeReturn(a, &planScratch, &a->pose, a->estimates.obstacle2Width.mm, a->step);
                    if (a->planCancelled) return TASK2_APP_RUNNING;
                    Note(a, TASK2_DIAG_PLAN, "EDGE_RETURN_PROVEN", ok, "T2_EDGE_NARROWED_TO_PROVEN", &planScratch, NULL, &m);
                }
                if (!ok) {
                    if (!rewindable && !traversable) return Fail(a, planScratch.report.reason);
                    a->edgeBraking = false;
                    if (traversable) {
                        a->detection.detected = false;
                        a->detection.observedWidthMm = 0.0f;
                    }
                    fault = rewindable ? RewindTraverse(a, now) : TraverseFromStop(a, now);
                    return fault ? Fail(a, fault) : TASK2_APP_RUNNING;
                }
                a->route = planScratch;
                a->edgeBraking = false;
            } else if (a->route.obstacle2LateralDeparture &&
                       (a->guardBraking || a->step == TASK2_ROUTE_O2_IR_STRAIGHT_STEP)) {
                /* Face still seen, or its end being confirmed: keep going on a wider lane. */
                const Task2ObstacleDetection *d = &a->detection;
                bool faceSeen = d->armed[TASK2_SENSOR_ULTRASONIC] &&
                    (d->present[0] > 0U || d->ultrasonicNoEchoSamples > 0U || d->ultrasonicJumpSamples > 0U);
                a->guardBraking = false;
                if (!faceSeen) return Fail(a, "T2_NO_EDGE");
                fault = ExtendDeparture(a, now);
                return fault ? Fail(a, fault) : TASK2_APP_RUNNING;
            } else if (a->step == TASK2_ROUTE_O2_IR_STRAIGHT_STEP) return Fail(a, "T2_NO_EDGE");
        }
        if (a->route.returnFallbackActive && a->route.fallbackHomeStep > 0U &&
            a->step + 1U == a->route.fallbackHomeStep) {
            Task2Commands_SetStage(t, TASK2_STAGE_HOME);
            PlanMark m = Mark(a);
            bool ok = PlanParkingReturn(a, &a->route, &a->pose, a->estimates.obstacle2Width.mm, a->step);
            if (a->planCancelled) return TASK2_APP_RUNNING;
            Note(a, TASK2_DIAG_PLAN, "FALLBACK_PARKING", ok, NULL, &a->route, NULL, &m);
            if (!ok) return Fail(a, a->route.report.reason);
        }
        if (a->step + 1U == a->route.stepCount) {
            float width = a->estimates.obstacle2Width.mm;
            PlanMark m = Mark(a);
            if (a->route.obstacle2ParallelReturnPending) {
                bool ok = PlanParallelReturn(a, &a->route, &a->pose, width, a->step);
                if (a->planCancelled) return TASK2_APP_RUNNING;
                Note(a, TASK2_DIAG_PLAN, "PARALLEL_RETURN", ok, NULL, &a->route, NULL, &m);
                if (!ok) return Fail(a, a->route.report.reason);
            } else if (a->route.obstacle2ParkReturnPending) {
                Task2Commands_SetStage(t, TASK2_STAGE_HOME);
                bool ok = PlanParkingReturn(a, &a->route, &a->pose, width, a->step);
                if (a->planCancelled) return TASK2_APP_RUNNING;
                Note(a, TASK2_DIAG_PLAN, "PARKING", ok, NULL, &a->route, NULL, &m);
                if (!ok) return Fail(a, a->route.report.reason);
            } else {
                if (!Task2Route_IsParked(&a->geometry, &a->pose)) {
                    Task2Commands_SetStage(t, TASK2_STAGE_HOME);
                    a->route.obstacle2ParkReturnPending = true;
                    bool ok = PlanParkingReturn(a, &a->route, &a->pose, a->estimates.obstacle2Width.mm, a->step);
                    if (a->planCancelled) return TASK2_APP_RUNNING;
                    Note(a, TASK2_DIAG_PLAN, "PARKING_CORRECTION", ok, NULL, &a->route, NULL, &m);
                    if (!ok) return Fail(a, a->route.report.reason);
                } else {
                    float measuredWidth = a->detection.edgeMethod == TASK2_EDGE_IR_INITIAL_ABSENCE_BOUND ? NAN : a->detection.observedWidthMm;
                    Task2Commands_Summary(t, a->estimates.obstacle1Face.mm,
                        a->estimates.obstacle1Face.mm + 100.0f + a->estimates.faceGap.mm, measuredWidth, a->pathMm);
                    Enter(a, TASK2_APP_FINISHED, now);
#if TASK2_DIAGNOSTICS
                    TraceFinal(a);
#endif
                    return TASK2_APP_DONE;
                }
            }
        }
        ++a->step;
        if (a->step == TASK2_ROUTE_O2_DEPARTURE_STEP) {
            a->lastUSSObservedMs = now;
            /* A near echo is direct evidence of the face: plan from it. A farther or missing echo never
             * overrides the estimate; an earlier ultrasonic lock is enough to depart. */
            if (CorrectFaceFromEcho(a, now)) {
                Note(a, TASK2_DIAG_PLAN, "NEAR_FACE", true, "T2_FACE_NEARER_THAN_PLANNED", &a->route, NULL, NULL);
                Enter(a, TASK2_APP_ACQUIRE2, now);
            } else if (!DepartureEchoAgrees(a))
                Enter(a, TASK2_APP_ACQUIRE2, now); /* no lock, no fresh echo, or a nearer echo to qualify */
            else fault = CommitDeparture(a, t, now);
        } else fault = Launch(a, &a->route.steps[a->step], now);
        break;
    }
    default: return Fail(a, "T2_STATE");
    }
    return fault ? Fail(a, fault) : TASK2_APP_RUNNING;
}
