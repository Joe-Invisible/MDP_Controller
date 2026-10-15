#include "Task2Wire.h"

#include <math.h>
#include <string.h>

typedef struct {
    uint8_t *data;
    size_t capacity, length;
    bool ok;
} Writer;

typedef struct {
    const uint8_t *data;
    size_t length, at;
    bool ok;
} Reader;

static void PutU8(Writer *w, uint8_t v) {
    if (w->length + 1U > w->capacity) { w->ok = false; return; }
    w->data[w->length++] = v;
}
static void PutU16(Writer *w, uint16_t v) { PutU8(w, (uint8_t)v); PutU8(w, (uint8_t)(v >> 8)); }
static void PutU32(Writer *w, uint32_t v) { PutU16(w, (uint16_t)v); PutU16(w, (uint16_t)(v >> 16)); }
static void PutF32(Writer *w, float v) { uint32_t bits; memcpy(&bits, &v, sizeof(bits)); PutU32(w, bits); }
static void PutBool(Writer *w, bool v) { PutU8(w, v ? 1U : 0U); }
static void PutText(Writer *w, const char *text) {
    size_t n = text ? strlen(text) : 0U;
    if (n >= TASK2_WIRE_REASON_BYTES) { w->ok = false; return; }
    PutU8(w, text ? (uint8_t)n : 0xFFU);
    for (size_t i = 0U; i < n; ++i) PutU8(w, (uint8_t)text[i]);
}

static uint8_t GetU8(Reader *r) {
    if (r->at + 1U > r->length) { r->ok = false; return 0U; }
    return r->data[r->at++];
}
static uint16_t GetU16(Reader *r) { uint16_t lo = GetU8(r); return (uint16_t)(lo | (uint16_t)GetU8(r) << 8); }
static uint32_t GetU32(Reader *r) { uint32_t lo = GetU16(r); return lo | (uint32_t)GetU16(r) << 16; }
static float GetF32(Reader *r) { uint32_t bits = GetU32(r); float v; memcpy(&v, &bits, sizeof(v)); return v; }
static bool GetBool(Reader *r) {
    uint8_t v = GetU8(r);
    if (v > 1U) r->ok = false;
    return v == 1U;
}

/* Decoded reasons must outlive the buffer: keep each distinct one once, never reusing a slot, so a
 * reason already stored elsewhere keeps its text. The planner has fewer reasons than slots; an
 * unknown one beyond them makes the message unreadable. */
#define TASK2_WIRE_REASONS 48U
static const char *Intern(const char *text, size_t n) {
    static char pool[TASK2_WIRE_REASONS][TASK2_WIRE_REASON_BYTES];
    static unsigned used;
    for (unsigned i = 0U; i < used; ++i)
        if (strlen(pool[i]) == n && memcmp(pool[i], text, n) == 0)
            return pool[i];
    if (used == TASK2_WIRE_REASONS || memchr(text, '\0', n)) return NULL;
    memcpy(pool[used], text, n);
    pool[used][n] = '\0';
    return pool[used++];
}
static const char *GetText(Reader *r) {
    uint8_t n = GetU8(r);
    if (n == 0xFFU) return NULL;
    if (n >= TASK2_WIRE_REASON_BYTES || r->at + n > r->length) { r->ok = false; return NULL; }
    const char *text = Intern((const char *)r->data + r->at, n);
    if (!text) r->ok = false;
    r->at += n;
    return text;
}

static void PutPose(Writer *w, const Task2RoutePose *p) { PutF32(w, p->xMm); PutF32(w, p->yMm); PutF32(w, p->yawRad); }
static void GetPose(Reader *r, Task2RoutePose *p) { p->xMm = GetF32(r); p->yMm = GetF32(r); p->yawRad = GetF32(r); }

static void PutArrow(Writer *w, Task2Arrow a) { PutU8(w, a == TASK2_ARROW_LEFT ? 1U : a == TASK2_ARROW_RIGHT ? 2U : 0U); }
static Task2Arrow GetArrow(Reader *r) {
    uint8_t v = GetU8(r);
    if (v > 2U) r->ok = false;
    return v == 1U ? TASK2_ARROW_LEFT : v == 2U ? TASK2_ARROW_RIGHT : (Task2Arrow)0;
}

/* Every Task2RouteConfig field, in declaration order. */
#define CONFIG_FLOATS(X) \
    X(obstacle2WidthMm) X(parkToObstacle1Mm) X(obstacleFaceGapMm) X(obstacleBufferMm) \
    X(parkBackWallBufferMm) X(parkSideWallBufferMm) X(boundaryBufferMm) X(obstacle2EndSpaceMm) \
    X(cruiseSpeedCps) X(approachSpeedCps) X(bodyLengthMm) X(bodyWidthMm) X(bodyCenterAheadOfRearAxleMm) \
    X(frontProtrusionMm) X(cameraBehindFrontMm) X(recognitionCameraRangeMm) X(cameraHorizontalFovDeg) \
    X(recognitionTargetWidthMm) X(firstArcCameraRangeMm) X(firstRejoinBeyondFaceMm) X(firstRadiusMm) \
    X(secondArcCameraRangeMm) X(obstacle1ExitYawDeg) X(obstacle2ApproachMaxMm) \
    X(obstacle2DepartureWidthReserveMm) X(obstacle2DepartureSpeedCps) X(obstacle2IRAcquisitionSpeedCps) \
    X(obstacle2IRConfirmationSpeedCps) X(secondDepartureRadiusMm) X(secondIRDepartureRadiusMm) \
    X(secondIRDepartureGuardExtraMm) X(secondBackExtraRadiusMm)

static void PutConfig(Writer *w, const Task2RouteConfig *c) {
    PutArrow(w, c->firstArrow);
    PutArrow(w, c->secondArrow);
#define PUT(f) PutF32(w, c->f);
    CONFIG_FLOATS(PUT)
#undef PUT
}
static void GetConfig(Reader *r, Task2RouteConfig *c) {
    c->firstArrow = GetArrow(r);
    c->secondArrow = GetArrow(r);
#define GET(f) c->f = GetF32(r);
    CONFIG_FLOATS(GET)
#undef GET
}

static void PutReport(Writer *w, const Task2RouteReport *q) {
    PutBool(w, q->valid); PutText(w, q->reason);
    PutF32(w, q->minimumObstacleGapMm); PutF32(w, q->minimumSideMarginMm); PutF32(w, q->minimumParkWallMarginMm);
    PutF32(w, q->minimumParkBackMarginMm); PutF32(w, q->minimumParkSideMarginMm);
    PutU32(w, q->plannerPoseChecks);
    PutBool(w, q->planningBudgetExceeded); PutBool(w, q->obstacleBufferExceptionUsed); PutBool(w, q->returnFallbackUsed);
    PutBool(w, q->runtimeReducedClearanceUsed); PutBool(w, q->poseAdjustmentUsed); PutBool(w, q->completeReturnCertified);
    PutF32(w, q->returnObstacleBufferMm); PutF32(w, q->returnBoundaryBufferMm);
    PutU32(w, q->clearanceFromStep);
    PutBool(w, q->recognitionPreviewValid);
    PutPose(w, &q->recognitionPose[0]); PutPose(w, &q->recognitionPose[1]);
    PutF32(w, q->recognitionProgressMm[0]); PutF32(w, q->recognitionProgressMm[1]);
    PutPose(w, &q->finishPose);
}
static void GetReport(Reader *r, Task2RouteReport *q) {
    q->valid = GetBool(r); q->reason = GetText(r);
    q->minimumObstacleGapMm = GetF32(r); q->minimumSideMarginMm = GetF32(r); q->minimumParkWallMarginMm = GetF32(r);
    q->minimumParkBackMarginMm = GetF32(r); q->minimumParkSideMarginMm = GetF32(r);
    q->plannerPoseChecks = GetU32(r);
    q->planningBudgetExceeded = GetBool(r); q->obstacleBufferExceptionUsed = GetBool(r); q->returnFallbackUsed = GetBool(r);
    q->runtimeReducedClearanceUsed = GetBool(r); q->poseAdjustmentUsed = GetBool(r); q->completeReturnCertified = GetBool(r);
    q->returnObstacleBufferMm = GetF32(r); q->returnBoundaryBufferMm = GetF32(r);
    q->clearanceFromStep = GetU32(r);
    q->recognitionPreviewValid = GetBool(r);
    GetPose(r, &q->recognitionPose[0]); GetPose(r, &q->recognitionPose[1]);
    q->recognitionProgressMm[0] = GetF32(r); q->recognitionProgressMm[1] = GetF32(r);
    GetPose(r, &q->finishPose);
}

static void PutRoute(Writer *w, const Task2Route *t) {
    PutConfig(w, &t->config);
    uint32_t count = t->stepCount <= TASK2_ROUTE_STEP_COUNT ? t->stepCount : TASK2_ROUTE_STEP_COUNT + 1U;
    PutU8(w, (uint8_t)count);
    if (count > TASK2_ROUTE_STEP_COUNT) { w->ok = false; return; }
    for (uint32_t i = 0U; i < count; ++i) {
        PutF32(w, t->steps[i].distanceMm); PutBool(w, t->steps[i].sensorTerminated);
        PutF32(w, t->steps[i].radiusMm); PutF32(w, t->steps[i].speedCps);
    }
    PutF32(w, t->totalTravelMm); PutF32(w, t->totalYawRad);
    PutBool(w, t->obstacle2IRDeparture); PutBool(w, t->obstacle2LateralDeparture); PutBool(w, t->obstacle2ReturnPending);
    PutBool(w, t->obstacle2TangentRearArc); PutBool(w, t->obstacle2StraightParkReturn); PutBool(w, t->obstacle2DirectRearArc);
    PutU32(w, t->parkingHeadingCorrections);
    PutBool(w, t->obstacle2ParkReturnPending); PutBool(w, t->obstacle2ParallelReturnPending);
    PutBool(w, t->returnFallbackActive);
    PutU32(w, (uint32_t)t->fallbackHomeStep);
    PutF32(w, t->returnObstacleBufferMm); PutF32(w, t->returnBoundaryBufferMm); PutF32(w, t->returnWidthMm);
    PutReport(w, &t->report);
}
static void GetRoute(Reader *r, Task2Route *t) {
    memset(t, 0, sizeof(*t));
    GetConfig(r, &t->config);
    uint8_t count = GetU8(r);
    if (count > TASK2_ROUTE_STEP_COUNT) { r->ok = false; return; }
    t->stepCount = count;
    for (uint32_t i = 0U; i < count; ++i) {
        t->steps[i].distanceMm = GetF32(r); t->steps[i].sensorTerminated = GetBool(r);
        t->steps[i].radiusMm = GetF32(r); t->steps[i].speedCps = GetF32(r);
    }
    t->totalTravelMm = GetF32(r); t->totalYawRad = GetF32(r);
    t->obstacle2IRDeparture = GetBool(r); t->obstacle2LateralDeparture = GetBool(r); t->obstacle2ReturnPending = GetBool(r);
    t->obstacle2TangentRearArc = GetBool(r); t->obstacle2StraightParkReturn = GetBool(r); t->obstacle2DirectRearArc = GetBool(r);
    t->parkingHeadingCorrections = GetU32(r);
    t->obstacle2ParkReturnPending = GetBool(r); t->obstacle2ParallelReturnPending = GetBool(r);
    t->returnFallbackActive = GetBool(r);
    t->fallbackHomeStep = GetU32(r);
    t->returnObstacleBufferMm = GetF32(r); t->returnBoundaryBufferMm = GetF32(r); t->returnWidthMm = GetF32(r);
    GetReport(r, &t->report);
}

static uint32_t Checksum(const uint8_t *data, size_t length) {
    uint32_t h = 2166136261U;
    for (size_t i = 0U; i < length; ++i) { h ^= data[i]; h *= 16777619U; }
    return h;
}
static size_t Seal(Writer *w) {
    uint32_t sum = Checksum(w->data, w->length);
    PutU32(w, sum);
    return w->ok ? w->length : 0U;
}
static bool Open(const uint8_t *in, size_t length, Reader *r) {
    if (length < 8U) return false;
    uint32_t sum = (uint32_t)in[length - 4U] | (uint32_t)in[length - 3U] << 8 |
                   (uint32_t)in[length - 2U] << 16 | (uint32_t)in[length - 1U] << 24;
    if (sum != Checksum(in, length - 4U)) return false;
    *r = (Reader){in, length - 4U, 0U, true};
    return GetU8(r) == TASK2_WIRE_FORMAT;
}

size_t Task2Wire_EncodeRequest(const Task2WireRequest *q, uint8_t *out, size_t capacity) {
    Writer w = {out, capacity, 0U, true};
    PutU8(&w, TASK2_WIRE_FORMAT); PutU8(&w, 'Q');
    PutU8(&w, q->call); PutU8(&w, q->run); PutU16(&w, q->generation);
    PutRoute(&w, &q->route);
    PutConfig(&w, &q->config);
    PutPose(&w, &q->pose);
    PutBool(&w, q->hasIr);
    PutF32(&w, q->ir.mount.aheadMm); PutF32(&w, q->ir.mount.leftMm); PutF32(&w, q->ir.mount.yawDeg);
    PutF32(&w, q->ir.guardTravelMm); PutF32(&w, q->ir.edgeUncertaintyMm);
    PutF32(&w, q->ir.acquisitionMinYawDeg); PutF32(&w, q->ir.acquisitionTravelMm);
    PutF32(&w, q->widthMm); PutU32(&w, q->step);
    return Seal(&w);
}

bool Task2Wire_DecodeRequest(const uint8_t *in, size_t length, Task2WireRequest *q) {
    Reader r;
    if (!Open(in, length, &r) || GetU8(&r) != 'Q') return false;
    memset(q, 0, sizeof(*q));
    q->call = GetU8(&r); q->run = GetU8(&r); q->generation = GetU16(&r);
    GetRoute(&r, &q->route);
    GetConfig(&r, &q->config);
    GetPose(&r, &q->pose);
    q->hasIr = GetBool(&r);
    q->ir.mount.aheadMm = GetF32(&r); q->ir.mount.leftMm = GetF32(&r); q->ir.mount.yawDeg = GetF32(&r);
    q->ir.guardTravelMm = GetF32(&r); q->ir.edgeUncertaintyMm = GetF32(&r);
    q->ir.acquisitionMinYawDeg = GetF32(&r); q->ir.acquisitionTravelMm = GetF32(&r);
    q->widthMm = GetF32(&r); q->step = GetU32(&r);
    return r.ok && r.at == r.length && q->call >= TASK2_WIRE_REPLAN_DEPARTURE && q->call < TASK2_WIRE_CALL_END;
}

size_t Task2Wire_EncodeReply(const Task2WireReply *p, uint8_t *out, size_t capacity) {
    Writer w = {out, capacity, 0U, true};
    PutU8(&w, TASK2_WIRE_FORMAT); PutU8(&w, 'R');
    PutU8(&w, p->run); PutU16(&w, p->generation);
    PutBool(&w, p->ok); PutU32(&w, p->checks); PutU32(&w, p->request);
    PutRoute(&w, &p->route);
    return Seal(&w);
}

bool Task2Wire_DecodeReply(const uint8_t *in, size_t length, Task2WireReply *p) {
    Reader r;
    if (!Open(in, length, &r) || GetU8(&r) != 'R') return false;
    memset(p, 0, sizeof(*p));
    p->run = GetU8(&r); p->generation = GetU16(&r);
    p->ok = GetBool(&r); p->checks = GetU32(&r); p->request = GetU32(&r);
    GetRoute(&r, &p->route);
    return r.ok && r.at == r.length;
}

uint32_t Task2Wire_RequestTag(const uint8_t *request, size_t length) {
    return Checksum(request, length);
}

size_t Task2Wire_Execute(const uint8_t *request, size_t length, uint8_t *reply, size_t capacity) {
    static Task2WireRequest q;
    static Task2WireReply p;
    if (!Task2Wire_DecodeRequest(request, length, &q))
        return 0U;
    p = (Task2WireReply){.run = q.run, .generation = q.generation, .request = Checksum(request, length)};
    p.route = q.route;
    uint32_t before = Task2Route_TotalPlanningChecks();
    const Task2RouteIRDeparture *ir = q.hasIr ? &q.ir : NULL;
    switch ((Task2WireCall)q.call) {
    case TASK2_WIRE_REPLAN_DEPARTURE:
        p.ok = Task2Route_ReplanDeparture(&p.route, &q.config, &q.pose, ir); break;
    case TASK2_WIRE_REPLAN_IR_TRAVERSE:
        p.ok = ir && Task2Route_ReplanIRTraverse(&p.route, &q.config, &q.pose, ir); break;
    case TASK2_WIRE_REPLAN_OBSTACLE2:
        p.ok = Task2Route_ReplanObstacle2(&p.route, &q.pose, q.widthMm, q.step); break;
    case TASK2_WIRE_REPLAN_PARALLEL_RETURN:
        p.ok = Task2Route_ReplanParallelReturn(&p.route, &q.pose, q.widthMm, q.step); break;
    case TASK2_WIRE_REPLAN_PARKING_RETURN:
        p.ok = Task2Route_ReplanParkingReturn(&p.route, &q.pose, q.widthMm, q.step); break;
    default:
        return 0U;
    }
    p.checks = Task2Route_TotalPlanningChecks() - before;
    return Task2Wire_EncodeReply(&p, reply, capacity);
}

size_t Task2Wire_ToHex(const uint8_t *in, size_t length, char *out, size_t capacity) {
    static const char digits[] = "0123456789abcdef";
    if (2U * length + 1U > capacity) return 0U;
    for (size_t i = 0U; i < length; ++i) {
        out[2U * i] = digits[in[i] >> 4];
        out[2U * i + 1U] = digits[in[i] & 15U];
    }
    out[2U * length] = '\0';
    return 2U * length;
}

static int Nibble(char c) {
    return c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
}
size_t Task2Wire_FromHex(const char *in, size_t length, uint8_t *out, size_t capacity) {
    if (length % 2U || length / 2U > capacity) return 0U;
    for (size_t i = 0U; i < length / 2U; ++i) {
        int hi = Nibble(in[2U * i]), lo = Nibble(in[2U * i + 1U]);
        if (hi < 0 || lo < 0) return 0U;
        out[i] = (uint8_t)(hi << 4 | lo);
    }
    return length / 2U;
}
