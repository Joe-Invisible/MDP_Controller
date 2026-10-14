#include "Task2App.h"
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846f
#define RANGE_TIMEOUT_MS 2500U
#define ACTIVE_TIMEOUT_MS 120000U

const Task2ObstacleSensorConfig task2AppSensorDefaults = {
    .ultrasonicEnabled = true, .sideIREnabled = true,
    .ultrasonic = {210.0f, 0.0f, 0.0f},
    .leftIR = {155.0f, 37.5f, 90.0f}, .rightIR = {135.0f, -33.5f, -90.0f},
    .ultrasonicBeamHalfAngleDeg = 7.5f, .ultrasonicRangeToleranceMm = 80.0f,
    .ultrasonicApproachMedianSamples = 11U, .ultrasonicDepartureMedianSamples = 5U,
    .ultrasonicJumpMm = 250.0f, .ultrasonicMaxDepartureYawDeg = 45.0f,
    .irPresentAdc = 700U, .irAbsentAdc = 400U, .irAcquisitionMinYawDeg = 80.0f,
    .irTrendMinDropAdc = 200U, .irTrendReboundAdc = 50U,
    .irTrendSamples = 3U, .irTrendMinTravelMm = 20.0f, .edgeUncertaintyMm = 20.0f,
    .samplePeriodMs = SENSOR_ULTRASONIC_PERIOD_MS, .acquisitionTimeoutMs = 1500U,
    .presentSamples = 3U, .absentSamples = 3U,
};

static void Enter(Task2App *a, Task2AppPhase phase, uint32_t now) {
    a->phase = phase;
    a->phaseStartedMs = now;
}
static Task2AppStatus Fail(Task2App *a, const char *reason) {
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
static bool ObserveUSS(Task2App *a, uint32_t now, bool departing) {
    UltrasonicReading r; Task2RoutePose p;
    if (FreshUSS(a, now, &r, &p)) {
        SensorStatus status = SensorReading_Status(&r, now);
        Task2ObstacleObservation o = {.valid = status == SENSOR_OK,
            .fault = status != SENSOR_OK && status != SENSOR_TIMEOUT && status != SENSOR_OUT_OF_RANGE,
            .value = r.distanceMm, .pose = p};
        Task2ObstacleDetection_Observe(&a->detection, &a->sensors, &a->route.config,
            TASK2_SENSOR_ULTRASONIC, &o, departing);
    }
    return !a->detection.fault;
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
    return Task2Route_CreateFromPose(&a->geometry,
        a->geometry.firstArrow == a->geometry.secondArrow ? NULL : &ir, &a->pose, &a->route);
}
static const char *CommitDeparture(Task2App *a, Task2Commands *t, uint32_t now) {
    uint16_t span;
    if (Task2Commands_Span(t, &span)) Task2Estimate_SetSpan(&a->estimates, span);
    if (!Task2Estimate_Apply(&a->estimates, &a->geometry)) return "T2_ESTIMATE";
    Task2RouteIRDeparture ir = IRPlanParameters(a);
    if (!Task2Route_ReplanDeparture(&a->route, &a->geometry, &a->pose,
        a->route.obstacle2IRDeparture ? &ir : NULL)) return a->route.report.reason;
    Enter(a, TASK2_APP_ROUTE, now);
    Task2Commands_SetStage(t, TASK2_STAGE_ROUND2);
    if (a->route.obstacle2LateralDeparture)
        Task2ObstacleDetection_BeginUSSDeparture(&a->detection, &a->sensors);
    return Launch(a, &a->route.steps[a->step], now);
}

const char *Task2App_Start(Task2App *a, const Task2AppIo *io, uint32_t now) {
    memset(a, 0, sizeof(*a));
    a->io = io; a->geometry = task2RouteConfig; a->sensors = task2AppSensorDefaults;
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
void Task2App_Abort(Task2App *a) {
    a->phase = TASK2_APP_ABORTED;
    a->primitive = false;
    a->io = NULL;
}

Task2AppStatus Task2App_Update(Task2App *a, Task2Commands *t, uint32_t now) {
    if (!a->io || a->phase == TASK2_APP_ABORTED) return TASK2_APP_FAULT;
    if (a->fault) return TASK2_APP_FAULT;
    if (a->phase == TASK2_APP_FINISHED) return TASK2_APP_DONE;
    if (a->phase != TASK2_APP_WAIT1 && a->phase != TASK2_APP_WAIT2)
        a->activeMs += now - a->previousMs;
    a->previousMs = now;
    if (a->activeMs >= ACTIVE_TIMEOUT_MS) return Fail(a, "T2_TIMEOUT");
    if (!UpdatePose(a)) return Fail(a, "ODOMETRY");
    RememberPose(a, now);
    if (a->phase >= TASK2_APP_RANGE2 && a->phase <= TASK2_APP_ROUTE) {
        Task2RouteConfig bounds = a->route.returnWidthMm > 0.0f
            ? Task2Route_ReturnBounds(&a->route) : a->geometry;
        if (a->phase == TASK2_APP_ROUTE && a->route.obstacle2IRDeparture &&
            a->step >= TASK2_ROUTE_O2_DEPARTURE_STEP && t->stage != TASK2_STAGE_HOME)
            bounds.obstacleBufferMm = 1.0f; /* Existing certified IR bypass exception. */
        Task2RouteReport report = {.minimumObstacleGapMm = INFINITY, .minimumSideMarginMm = INFINITY,
            .minimumParkSideMarginMm = INFINITY, .minimumParkBackMarginMm = INFINITY};
        if (!Task2Route_CheckPose(&bounds, &a->pose, &report)) {
            if (report.minimumParkSideMarginMm < bounds.parkSideWallBufferMm ||
                report.minimumParkBackMarginMm < bounds.parkBackWallBufferMm)
                return Fail(a, "T2_HARD_PARK_CLEARANCE");
            /* A measured allowance shortfall may invoke recovery, but an actual
             * obstacle/arena-wall intersection is never an approved fallback. */
            if (a->phase != TASK2_APP_ROUTE || a->step < TASK2_ROUTE_O2_DEPARTURE_STEP ||
                report.minimumObstacleGapMm < 1.0f || report.minimumSideMarginMm < 1.0f)
                return Fail(a, "T2_CLEARANCE");
            a->route.report.runtimeReducedClearanceUsed = true;
        }
    }
    bool busy = a->io->busy(a->io->context);
    const char *fault = NULL;
    switch (a->phase) {
    case TASK2_APP_RANGE1: {
        float face;
        if (FaceWindow(a, now, &face)) {
            if (!Task2Estimate_SetFace(&a->estimates, 1U, face, 20.0f)) return Fail(a, "T2_O1_RANGE");
            a->geometry.parkToObstacle1Mm = face;
            if (!Task2Route_CreateObstacle1(&a->geometry, &a->route)) return Fail(a, a->route.report.reason);
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
            if (!Task2Route_CreateObstacle1(&a->geometry, &a->route)) return Fail(a, a->route.report.reason);
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
        float face;
        if (a->estimates.faceGap.valid || FaceWindow(a, now, &face)) {
            if (!a->estimates.faceGap.valid && !Task2Estimate_SetFace(&a->estimates, 2U, face, 20.0f)) return Fail(a, "T2_GAP_RANGE");
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
    case TASK2_APP_RECOGNITION2:
        /* Stop early for a real camera arrow. Further approach, if needed for an
         * opposite-arrow turn, is selected by the planner from that measured stop. */
        if (busy && Task2Commands_Arrow(t, 2U)) a->io->brake(a->io->context);
        if (!busy) {
            a->primitive = false; Enter(a, TASK2_APP_WAIT2, now);
            if (!Task2Commands_Arrow(t, 2U)) Task2Commands_SetStage(t, TASK2_STAGE_WAIT2);
        }
        break;
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
        if (a->detection.armed[TASK2_SENSOR_ULTRASONIC]) {
            fault = CommitDeparture(a, t, now);
        } else if (now - a->phaseStartedMs >= a->sensors.acquisitionTimeoutMs) return Fail(a, "T2_USS_ACQUIRE");
        break;
    case TASK2_APP_ROUTE: {
        bool observing = a->route.obstacle2ReturnPending;
        bool ir = observing && a->route.obstacle2IRDeparture && a->step == TASK2_ROUTE_O2_IR_STRAIGHT_STEP;
        bool uss = observing && (a->step == TASK2_ROUTE_O2_APPROACH_STEP || a->route.obstacle2LateralDeparture);
        if (!a->edgeBraking) {
            if (uss && !ObserveUSS(a, now, a->step >= TASK2_ROUTE_O2_DEPARTURE_STEP)) return Fail(a, "T2_USS_SENSOR");
            if (ir && !ObserveIR(a, now)) return Fail(a, "T2_IR_SENSOR");
            if ((uss && now - a->lastUSSObservedMs >= SENSOR_ULTRASONIC_STALE_MS) ||
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
            if (a->detection.detected && observing && a->step >= TASK2_ROUTE_O2_DEPARTURE_STEP) {
                a->edgeBraking = true; a->io->brake(a->io->context);
            }
        }
        if (busy) break;
        a->primitive = false;
        if (observing && a->step >= TASK2_ROUTE_O2_DEPARTURE_STEP) {
            if (a->edgeBraking) {
                if (!Task2Estimate_SetEdge(&a->estimates, &a->detection)) return Fail(a, "T2_EDGE");
                a->route.steps[a->step].distanceMm = fabsf(a->previousDistanceMm);
                if (!Task2Route_ReplanObstacle2(&a->route, &a->pose, a->estimates.obstacle2Width.mm, a->step))
                    return Fail(a, a->route.report.reason);
                a->edgeBraking = false;
            } else if (a->step == TASK2_ROUTE_O2_IR_STRAIGHT_STEP) return Fail(a, "T2_NO_EDGE");
        }
        if (a->route.returnFallbackActive && a->route.fallbackHomeStep > 0U &&
            a->step + 1U == a->route.fallbackHomeStep) {
            Task2Commands_SetStage(t, TASK2_STAGE_HOME);
            if (!Task2Route_ReplanParkingReturn(&a->route, &a->pose,
                a->estimates.obstacle2Width.mm, a->step)) return Fail(a, a->route.report.reason);
        }
        if (a->step + 1U == a->route.stepCount) {
            float width = a->estimates.obstacle2Width.mm;
            if (a->route.obstacle2ParallelReturnPending) {
                if (!Task2Route_ReplanParallelReturn(&a->route, &a->pose, width, a->step)) return Fail(a, a->route.report.reason);
            } else if (a->route.obstacle2ParkReturnPending) {
                Task2Commands_SetStage(t, TASK2_STAGE_HOME);
                if (!Task2Route_ReplanParkingReturn(&a->route, &a->pose, width, a->step)) return Fail(a, a->route.report.reason);
            } else {
                if (!Task2Route_IsParked(&a->geometry, &a->pose)) {
                    Task2Commands_SetStage(t, TASK2_STAGE_HOME);
                    a->route.obstacle2ParkReturnPending = true;
                    if (!Task2Route_ReplanParkingReturn(&a->route, &a->pose,
                        a->estimates.obstacle2Width.mm, a->step)) return Fail(a, a->route.report.reason);
                } else {
                    float measuredWidth = a->detection.edgeMethod == TASK2_EDGE_IR_INITIAL_ABSENCE_BOUND ? NAN : a->detection.observedWidthMm;
                    Task2Commands_Summary(t, a->estimates.obstacle1Face.mm,
                        a->estimates.obstacle1Face.mm + 100.0f + a->estimates.faceGap.mm, measuredWidth, a->pathMm);
                    Enter(a, TASK2_APP_FINISHED, now); return TASK2_APP_DONE;
                }
            }
        }
        ++a->step;
        if (a->step == TASK2_ROUTE_O2_DEPARTURE_STEP) {
            a->lastUSSObservedMs = now;
            if (!a->detection.armed[TASK2_SENSOR_ULTRASONIC]) Enter(a, TASK2_APP_ACQUIRE2, now);
            else fault = CommitDeparture(a, t, now);
        } else fault = Launch(a, &a->route.steps[a->step], now);
        break;
    }
    default: return Fail(a, "T2_STATE");
    }
    return fault ? Fail(a, fault) : TASK2_APP_RUNNING;
}
