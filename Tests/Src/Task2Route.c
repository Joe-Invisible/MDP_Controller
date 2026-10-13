#include "Task2Route.h"

#include <math.h>
#include <stddef.h>

#define ROUTE_PI 3.14159265358979323846f
#define ROUTE_MIN_RADIUS_MM 275.0f
#define ROUTE_SAMPLE_SPACING_MM 1.0f
#define ROUTE_MAX_TRAVEL_MM 30000.0f

/* This standalone planner is called serially at startup and at an edge stop. */
static struct {
    uint32_t poseChecks;
    bool exhausted;
#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
    Task2RouteReport acceptedClearance; /* Reuse the last accepted candidate's certificate. */
#endif
} planningWork;

static bool PlanningPose(const Task2RouteConfig *c, const Task2RoutePose *p, Task2RouteReport *report)
{
#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
    if (planningWork.poseChecks >= TASK2_ROUTE_RUNTIME_POSE_CHECK_LIMIT)
    {
        planningWork.exhausted = true;
        return false;
    }
#endif
    ++planningWork.poseChecks;
    return Task2Route_CheckPose(c, p, report);
}

static bool FinishPlanning(Task2Route *route, bool accepted)
{
    if (!route)
        return false;
    route->report.plannerPoseChecks = planningWork.poseChecks;
    route->report.planningBudgetExceeded = planningWork.exhausted;
    if (planningWork.exhausted)
    {
        route->report.valid = false;
        route->report.reason = "PLANNING_BUDGET";
        return false;
    }
    return accepted;
}

/* Important experiment parameters. Positive y is LEFT of initial +x.
 * These are nominal simulator inputs, not measurements or uncertainty bounds.
 * Change the two arrows and width first; geometry knobs are below them. */
Task2RouteConfig task2RouteConfig = {
    /* Arena and selected arrows. No boundary is imposed behind obstacle 2. */
    .firstArrow = TASK2_ARROW_RIGHT,
    .secondArrow = TASK2_ARROW_LEFT,
    .obstacle2WidthMm = 500.0f,
    .parkToObstacle1Mm = 1050.0f,
    .obstacleFaceGapMm = 1050.0f,
    .obstacleBufferMm = 100.0f,
    .obstacle2EndSpaceMm = 500.0f,
    .boundaryBufferMm = 35.0f,
    .parkBackWallBufferMm = 75.0f,
    .parkSideWallBufferMm = 100.0f,

    /* Both approaches accelerate after simulated recognition.
     * Recognition is instantaneous; camera/RPi integration is deferred. */
    .cruiseSpeedCps = 7000.0f,
    .approachSpeedCps = 3000.0f,
    .bodyLengthMm = 227.0f,
    .bodyWidthMm = 200.0f,
    .bodyCenterAheadOfRearAxleMm = 76.5f, /* Front +190, tail -37 mm. */
    .frontProtrusionMm = 20.0f,          /* Ultrasonic face at +210 mm. */
    .cameraBehindFrontMm = 80.0f,
    .recognitionCameraRangeMm = 1000.0f,
    .cameraHorizontalFovDeg = 60.0f,
    .recognitionTargetWidthMm = 100.0f,

    /* Preserve the original O1 departure angle, but leave its bypass at
     * inward yaw 19 degrees. Camera ranges reference the lens. */
    /* Updated for the measured +110 mm camera offset; retain R275 sweeps. */
    .firstArcCameraRangeMm = 400.0f,
    .firstRejoinBeyondFaceMm = 535.0f,
    .firstRadiusMm = 275.0f,
    .secondArcCameraRangeMm = 500.0f,
    .obstacle1ExitYawDeg = 19.0f,
    .obstacle2ApproachMaxMm = 1000.0f,
    .obstacle2DepartureSpeedCps = 2000.0f,
    .obstacle2IRAcquisitionSpeedCps = 500.0f,
    .obstacle2IRConfirmationSpeedCps = 750.0f,
    .obstacle2DepartureWidthReserveMm = 100.0f,
    .secondDepartureRadiusMm = 275.0f,
    .secondIRDepartureRadiusMm = 325.0f,
    .secondIRDepartureGuardExtraMm = 30.0f,
    .secondBackExtraRadiusMm = 250.0f,


};

const char *const task2RouteStepNames[TASK2_ROUTE_STEP_COUNT] = {
    "Approach O1", "O1 depart", "O1 bypass exit", "Approach O2",
    "O2 sensor departure", "O2 clearance arc", "O2 back", "Return park",
    "Park align 1", "Park align 2", "Park straight", "Reserve",
    "Reserve", "Reserve", "Reserve", "Reserve", "Reserve", "Reserve",
};

static bool Reject(Task2Route *route, const char *reason)
{
    route->report.valid = false;
    route->report.reason = reason;
    return false;
}

/* Initial rear-axle origin and final body-centre target are separate anchors. */
static float ParkFinishX(const Task2RouteConfig *c)
{
    return TASK2_ROUTE_PARK_CENTER_X_MM + c->bodyCenterAheadOfRearAxleMm +
        0.5f * c->frontProtrusionMm;
}

static float CameraOffset(const Task2RouteConfig *config)
{
    return config->bodyCenterAheadOfRearAxleMm +
        0.5f * config->bodyLengthMm - config->cameraBehindFrontMm;
}

bool Task2Route_ObstacleCameraVisible(const Task2RouteConfig *c, const Task2RoutePose *pose, unsigned obstacle)
{
    float cosine = cosf(pose->yawRad), sine = sinf(pose->yawRad);
    float cameraX = pose->xMm + CameraOffset(c) * cosine;
    float cameraY = pose->yMm + CameraOffset(c) * sine;
    float face = c->parkToObstacle1Mm + (obstacle ? 100.0f + c->obstacleFaceGapMm : 0.0f);
    float dx = face - cameraX;
    if (dx <= 0.0f || hypotf(dx, cameraY) > c->recognitionCameraRangeMm)
        return false;
    for (int sign = -1; sign <= 1; sign += 2)
    {
        float dy = sign * 0.5f * c->recognitionTargetWidthMm - cameraY;
        float forward = dx * cosine + dy * sine;
        float lateral = -dx * sine + dy * cosine;
        if (forward <= 0.0f || fabsf(lateral) >=
            forward * tanf(c->cameraHorizontalFovDeg * ROUTE_PI / 360.0f))
            return false;
    }
    return true;
}

bool Task2Route_Obstacle1CameraVisible(const Task2RouteConfig *c, const Task2RoutePose *pose)
{
    return Task2Route_ObstacleCameraVisible(c, pose, 0U);
}

static bool ValidConfig(const Task2RouteConfig *config)
{
    const float positive[] = {
        config->obstacle2WidthMm, config->parkToObstacle1Mm,
        config->obstacleFaceGapMm, config->obstacleBufferMm,
        config->obstacle2EndSpaceMm, config->boundaryBufferMm, config->parkBackWallBufferMm, config->parkSideWallBufferMm,
        config->cruiseSpeedCps,
        config->approachSpeedCps, config->bodyLengthMm, config->bodyWidthMm,
        config->recognitionCameraRangeMm, config->cameraHorizontalFovDeg,
        config->recognitionTargetWidthMm, config->firstArcCameraRangeMm,
        config->firstRejoinBeyondFaceMm, config->firstRadiusMm,
        config->secondArcCameraRangeMm, config->secondDepartureRadiusMm,
        config->secondIRDepartureRadiusMm,
        config->secondIRDepartureGuardExtraMm,
        config->secondBackExtraRadiusMm,
        config->obstacle1ExitYawDeg, config->obstacle2ApproachMaxMm,
        config->obstacle2DepartureSpeedCps, config->obstacle2DepartureWidthReserveMm,
        config->obstacle2IRAcquisitionSpeedCps,
        config->obstacle2IRConfirmationSpeedCps,
    };
    for (size_t i = 0U; i < sizeof(positive) / sizeof(positive[0]); ++i)
        if (!isfinite(positive[i]) || positive[i] <= 0.0f)
            return false;
    return (config->firstArrow == TASK2_ARROW_LEFT || config->firstArrow == TASK2_ARROW_RIGHT) &&
        isfinite(config->bodyCenterAheadOfRearAxleMm) &&
        fabsf(config->bodyCenterAheadOfRearAxleMm) <= 0.5f * config->bodyLengthMm &&
        isfinite(config->frontProtrusionMm) && config->frontProtrusionMm >= 0.0f &&
        (config->secondArrow == TASK2_ARROW_LEFT || config->secondArrow == TASK2_ARROW_RIGHT) &&
        config->obstacle2WidthMm >= 300.0f && config->parkToObstacle1Mm >= 600.0f &&
        config->obstacleFaceGapMm >= 600.0f && config->obstacleFaceGapMm <= 1500.0f &&
        config->obstacle1ExitYawDeg < 60.0f &&
        config->obstacle2DepartureSpeedCps <= config->cruiseSpeedCps &&
        config->secondArcCameraRangeMm >= 500.0f &&
        config->secondArcCameraRangeMm <= config->recognitionCameraRangeMm &&
        config->cameraHorizontalFovDeg < 180.0f &&
        isfinite(config->cameraBehindFrontMm) && config->cameraBehindFrontMm >= 0.0f &&
        config->cameraBehindFrontMm <= config->bodyLengthMm &&
        config->firstRadiusMm >= ROUTE_MIN_RADIUS_MM &&
        config->secondDepartureRadiusMm >= ROUTE_MIN_RADIUS_MM &&
        config->secondIRDepartureRadiusMm >= ROUTE_MIN_RADIUS_MM &&
        config->approachSpeedCps <= config->cruiseSpeedCps;
}

static void SetStep(Task2Route *route, unsigned index, float length, float radius)
{
    route->steps[index] = (Task2RouteStep){
        .distanceMm = length,
        .radiusMm = radius,
        /* Both approaches launch with a cruise ceiling and a soft approach
         * request; recognition raises it without changing the primitive. */
        .speedCps = route->config.cruiseSpeedCps,
    };
}

static Task2RoutePose Advance(Task2RoutePose p, float distance, float radius)
{
    if (radius == 0.0f)
    {
        p.xMm += distance * cosf(p.yawRad);
        p.yMm += distance * sinf(p.yawRad);
    }
    else
    {
        float next = p.yawRad + distance / radius;
        p.xMm += radius * (sinf(next) - sinf(p.yawRad));
        p.yMm += radius * (cosf(p.yawRad) - cosf(next));
        p.yawRad = next;
    }
    return p;
}

static Task2RouteReport EmptyReport(void)
{
    return (Task2RouteReport){.minimumObstacleGapMm = INFINITY,
        .minimumSideMarginMm = INFINITY, .minimumParkWallMarginMm = INFINITY,
        .minimumParkBackMarginMm = INFINITY, .minimumParkSideMarginMm = INFINITY};
}

static bool Parked(const Task2RouteConfig *c, const Task2RoutePose *p)
{
    float centre = c->bodyCenterAheadOfRearAxleMm + 0.5f * c->frontProtrusionMm;
    float length = c->bodyLengthMm + c->frontProtrusionMm;
    float cx = p->xMm + centre * cosf(p->yawRad);
    float cy = p->yMm + centre * sinf(p->yawRad);
    float ex = 0.5f * (length * fabsf(cosf(p->yawRad)) + c->bodyWidthMm * fabsf(sinf(p->yawRad)));
    float ey = 0.5f * (length * fabsf(sinf(p->yawRad)) + c->bodyWidthMm * fabsf(cosf(p->yawRad)));
    return cx - ex >= -500.0f + c->parkBackWallBufferMm && cx + ex < 0.0f &&
        fabsf(cy) + ey <= 300.0f - c->parkSideWallBufferMm;
}

#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
static void MergeMargins(Task2RouteReport *out, const Task2RouteReport *point)
{
    if (!out)
        return;
    out->minimumObstacleGapMm = fminf(out->minimumObstacleGapMm, point->minimumObstacleGapMm);
    out->minimumSideMarginMm = fminf(out->minimumSideMarginMm, point->minimumSideMarginMm);
    out->minimumParkSideMarginMm = fminf(out->minimumParkSideMarginMm, point->minimumParkSideMarginMm);
    out->minimumParkBackMarginMm = fminf(out->minimumParkBackMarginMm, point->minimumParkBackMarginMm);
    out->minimumParkWallMarginMm = fminf(out->minimumParkSideMarginMm, out->minimumParkBackMarginMm);
    out->obstacleBufferExceptionUsed |= point->obstacleBufferExceptionUsed;
}

/* Along an arc every body point moves at most ds*(1+cornerRadius/|R|).
 * Expand the midpoint rectangle by that displacement in both local axes:
 * it contains every footprint in the interval. Checking this superset certifies
 * the whole interval, including park-mouth crossings. Ambiguous intervals
 * subdivide; uncertainty or budget exhaustion rejects the candidate. */
static bool CheckPrimitive(const Task2RouteConfig *c, Task2RoutePose start,
    const Task2RouteStep *step, float orbitRadius, Task2RouteReport *out)
{
    struct { float lo, hi; unsigned depth; } pending[17];
    unsigned count = 1U;
    pending[0].lo = 0.0f; pending[0].hi = step->distanceMm; pending[0].depth = 0U;
    float cosine = cosf(start.yawRad), sine = sinf(start.yawRad);
    float cx = start.xMm - step->radiusMm * sine;
    float cy = start.yMm + step->radiusMm * cosine;
    while (count)
    {
        --count;
        float lo = pending[count].lo, hi = pending[count].hi;
        unsigned depth = pending[count].depth;
        float mid = 0.5f * (lo + hi);
        Task2RoutePose p = start;
        float displacement = 0.5f * (hi - lo);
        if (step->radiusMm == 0.0f)
        {
            p.xMm += mid * cosine; p.yMm += mid * sine;
        }
        else
        {
            p.yawRad += mid / step->radiusMm;
            p.xMm = cx + step->radiusMm * sinf(p.yawRad);
            p.yMm = cy - step->radiusMm * cosf(p.yawRad);
            float angle = fminf(ROUTE_PI, displacement / fabsf(step->radiusMm));
            displacement = 2.0f * orbitRadius * sinf(0.5f * angle);
        }
        Task2RouteConfig expanded = *c;
        expanded.bodyLengthMm += 2.0f * (displacement + 0.001f);
        expanded.bodyWidthMm += 2.0f * (displacement + 0.001f);
        Task2RouteReport bound = EmptyReport();
        if (PlanningPose(&expanded, &p, &bound))
        {
            MergeMargins(out, &bound);
            continue;
        }
        /* A certified expanded footprint already implies this point is clear.
         * Check the actual midpoint only after the interval certificate fails. */
        Task2RouteReport point = EmptyReport();
        if (!PlanningPose(c, &p, &point))
            return false;
        if (depth >= 16U || hi - lo <= 0.05f || count + 2U > 17U)
            return false;
        pending[count].lo = mid; pending[count].hi = hi; pending[count++].depth = depth + 1U;
        pending[count].lo = lo; pending[count].hi = mid; pending[count++].depth = depth + 1U;
    }
    return true;
}

static bool BoundedSweep(const Task2RouteConfig *c, Task2RoutePose start,
    const Task2RouteStep *steps, unsigned count, bool requirePark, Task2RouteReport *out)
{
    float front = c->bodyCenterAheadOfRearAxleMm + 0.5f * c->bodyLengthMm + c->frontProtrusionMm;
    float rear = c->bodyCenterAheadOfRearAxleMm - 0.5f * c->bodyLengthMm;
    float far = fmaxf(fabsf(front), fabsf(rear));
    for (unsigned i = 0U; i < count; ++i)
    {
        if (!isfinite(steps[i].distanceMm) || steps[i].distanceMm < 0.0f || !isfinite(steps[i].radiusMm))
            return false;
        if (steps[i].distanceMm == 0.0f)
        {
            if (steps[i].radiusMm != 0.0f || !PlanningPose(c, &start, out))
                return false;
            continue;
        }
        float orbit = hypotf(far, fabsf(steps[i].radiusMm) + 0.5f * c->bodyWidthMm);
        if (!CheckPrimitive(c, start, &steps[i], orbit, out))
            return false;
        start = Advance(start, steps[i].distanceMm, steps[i].radiusMm);
    }
    return !requirePark || Parked(c, &start);
}
#endif

#if TASK2_ROUTE_EXHAUSTIVE_CHECKS
static bool Sweep(const Task2RouteConfig *c, Task2RoutePose start,
    const Task2RouteStep *steps, unsigned count, float spacing, bool requirePark)
{
    Task2RouteReport report = EmptyReport();
    if (!PlanningPose(c, &start, &report))
        return false;
    for (unsigned i = 0U; i < count; ++i)
    {
        if (!isfinite(steps[i].distanceMm) || steps[i].distanceMm < 0.0f ||
            !isfinite(steps[i].radiusMm) || (steps[i].distanceMm == 0.0f && steps[i].radiusMm != 0.0f))
            return false;
        for (float d = spacing; d < steps[i].distanceMm; d += spacing)
        {
            Task2RoutePose p = Advance(start, d, steps[i].radiusMm);
            if (!PlanningPose(c, &p, &report))
                return false;
        }
        start = Advance(start, steps[i].distanceMm, steps[i].radiusMm);
        if (!PlanningPose(c, &start, &report))
            return false;
    }
    return !requirePark || Parked(c, &start);
}
#endif

static bool RecordCrossing(const Task2RouteConfig *c, Task2RoutePose pose,
    unsigned obstacle, bool crossed[2][2])
{
    if (fabsf(cosf(pose.yawRad)) < 0.00001f)
        return true; /* Tangency does not cross the obstacle centre plane. */
    unsigned returning = cosf(pose.yawRad) < 0.0f;
    int side = returning ? -c->secondArrow : obstacle == 0U ? c->firstArrow : c->secondArrow;
    if (side * pose.yMm <= 0.0f)
        return false;
    crossed[obstacle][returning] = true;
    return true;
}

static bool Crossings(const Task2RouteConfig *c, Task2RoutePose start,
    const Task2RouteStep *step, bool crossed[2][2])
{
    for (unsigned obstacle = 0U; obstacle < 2U; ++obstacle)
    {
        float centre = c->parkToObstacle1Mm + 50.0f +
            (obstacle ? 100.0f + c->obstacleFaceGapMm : 0.0f);
        if (step->radiusMm == 0.0f)
        {
            float cosine = cosf(start.yawRad);
            if (fabsf(cosine) < 0.00001f)
                continue;
            float d = (centre - start.xMm) / cosine;
            if (d > 0.0f && d <= step->distanceMm && !RecordCrossing(c, Advance(start, d, 0.0f), obstacle, crossed))
                return false;
        }
        else
        {
            float end = start.yawRad + step->distanceMm / step->radiusMm;
            if (fabsf(end - start.yawRad) > 2.0f * ROUTE_PI)
                return false;
            float cx = start.xMm - step->radiusMm * sinf(start.yawRad);
            float sine = (centre - cx) / step->radiusMm;
            if (fabsf(sine) > 1.0f)
                continue;
            float angle = asinf(sine), roots[] = {angle, ROUTE_PI - angle};
            for (unsigned root = 0U; root < 2U; ++root)
            {
                int first = (int)floorf((fminf(start.yawRad, end) - roots[root]) / (2.0f * ROUTE_PI));
                for (int k = first; k <= first + 2; ++k)
                {
                    float d = (roots[root] + k * 2.0f * ROUTE_PI - start.yawRad) * step->radiusMm;
                    if (d > 0.0f && d <= step->distanceMm &&
                        !RecordCrossing(c, Advance(start, d, step->radiusMm), obstacle, crossed))
                        return false;
                }
            }
        }
    }
    return true;
}

static bool CandidateSweepFrom(const Task2RouteConfig *c, Task2RoutePose start,
    const Task2RouteStep *steps, unsigned count, unsigned checkedPrefix, bool forwardAlreadyPassed)
{
    bool crossed[2][2] = {{false, false}, {false, false}};
    crossed[1][0] = forwardAlreadyPassed;
    Task2RoutePose pose = start;
    for (unsigned i = 0U; i < count; ++i)
    {
        if (!Crossings(c, pose, &steps[i], crossed))
            return false;
        pose = Advance(pose, steps[i].distanceMm, steps[i].radiusMm);
    }
    if (!crossed[1][0] || !crossed[1][1])
        return false; /* A clear loop entirely in front of O2 is not a bypass. */
    Task2RoutePose sweepStart = start;
    for (unsigned i = 0U; i < checkedPrefix; ++i)
        sweepStart = Advance(sweepStart, steps[i].distanceMm, steps[i].radiusMm);
#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
    /* Cheap rejection samples precede the one accepting interval sweep. */
    pose = sweepStart;
    for (unsigned i = checkedPrefix; i < count; ++i)
    {
        for (unsigned j = 1U; j <= 4U; ++j)
        {
            Task2RoutePose sample = Advance(pose, steps[i].distanceMm * j / 4.0f, steps[i].radiusMm);
            Task2RouteReport point = EmptyReport();
            if (!PlanningPose(c, &sample, &point))
                return false;
        }
        pose = Advance(pose, steps[i].distanceMm, steps[i].radiusMm);
    }
#endif
#if TASK2_ROUTE_EXHAUSTIVE_CHECKS
    if (!Sweep(c, sweepStart, steps + checkedPrefix, count - checkedPrefix, 5.0f, true))
        return false;
    return Sweep(c, sweepStart, steps + checkedPrefix, count - checkedPrefix, ROUTE_SAMPLE_SPACING_MM, true);
#else
    Task2RouteReport accepted = EmptyReport();
    if (!BoundedSweep(c, sweepStart, steps + checkedPrefix, count - checkedPrefix, true, &accepted))
        return false;
    planningWork.acceptedClearance = accepted;
    return true;
#endif
}

static bool CandidateSweep(const Task2RouteConfig *c, Task2RoutePose start,
    const Task2RouteStep *steps, unsigned count)
{
    return CandidateSweepFrom(c, start, steps, count, 0U, false);
}

/* Front/back traverses with R275 end caps provide a bounded fallback for
 * opposite-side approaches whose direct circle join cuts the front corner. */
static bool BuildTraverseTail(const Task2RouteConfig *c, const Task2RouteConfig *bounds,
    Task2RoutePose p, float width, Task2RouteStep out[9], unsigned *count)
{
    float side = c->secondArrow, h = side * p.yawRad;
    if (h >= 0.5f * ROUTE_PI)
        return false;
    for (unsigned d = 0U; d < 4U; ++d)
    for (unsigned offset = 0U; offset < 4U; ++offset)
    for (unsigned park = 0U; park < 4U; ++park)
    {
        if (planningWork.exhausted)
            return false;
        float rd = c->secondDepartureRadiusMm + 25.0f * d;
        out[0] = (Task2RouteStep){.distanceMm = rd * (0.5f * ROUTE_PI - h), .radiusMm = side * rd, .speedCps = c->obstacle2DepartureSpeedCps};
        Task2RoutePose front = Advance(p, out[0].distanceMm, out[0].radiusMm);
        float edgeY = 0.5f * width - 25.0f + 10.0f * offset;
        out[1] = (Task2RouteStep){.distanceMm = edgeY - side * front.yMm, .radiusMm = 0.0f, .speedCps = c->obstacle2DepartureSpeedCps};
        float radius = ROUTE_MIN_RADIUS_MM;
        out[2] = (Task2RouteStep){.distanceMm = ROUTE_PI * radius, .radiusMm = -side * radius, .speedCps = c->cruiseSpeedCps};
        out[3] = (Task2RouteStep){.distanceMm = 2.0f * edgeY, .radiusMm = 0.0f, .speedCps = c->cruiseSpeedCps};
        out[4] = (Task2RouteStep){.distanceMm = 0.5f * ROUTE_PI * radius, .radiusMm = -side * radius, .speedCps = c->cruiseSpeedCps};
        Task2RoutePose returnStart = front;
        for (unsigned i = 1U; i < 5U; ++i)
            returnStart = Advance(returnStart, out[i].distanceMm, out[i].radiusMm);
        float lateral = -side * returnStart.yMm;
        float rejoinRadius = fmaxf(ROUTE_MIN_RADIUS_MM, 0.5f * lateral + 50.0f);
        float angle = acosf(1.0f - lateral / (2.0f * rejoinRadius));
        float parkStraight = 100.0f + 100.0f * park;
        float sx = ParkFinishX(c) + parkStraight + 2.0f * rejoinRadius * sinf(angle);
        out[5] = (Task2RouteStep){.distanceMm = returnStart.xMm - sx, .radiusMm = 0.0f, .speedCps = c->cruiseSpeedCps};
        out[6] = (Task2RouteStep){.distanceMm = rejoinRadius * angle, .radiusMm = -side * rejoinRadius, .speedCps = c->cruiseSpeedCps};
        out[7] = (Task2RouteStep){.distanceMm = rejoinRadius * angle, .radiusMm = side * rejoinRadius, .speedCps = c->cruiseSpeedCps};
        out[8] = (Task2RouteStep){.distanceMm = parkStraight, .radiusMm = 0.0f, .speedCps = c->approachSpeedCps};
        if (CandidateSweep(bounds, p, out, 9U))
        {
            *count = 9U;
            return true;
        }
    }
    return false;
}

/* Opposite-turn circles meet tangentially. Search only four departure radii,
 * four back offsets, and a bounded set of return headings/park joins. Prefer a tangent
 * straight return; use one return arc only when the straight fails clearance. */
static bool BuildTail(const Task2RouteConfig *c, Task2RoutePose p,
    float width, Task2RouteStep out[9], unsigned *count)
{
    Task2RouteConfig bounds = *c;
    /* Leave room for the stopped controller's arc-exit pose error before
     * rebuilding parking. This reserve applies to the nominal USS return. */
    if (c->firstArrow == c->secondArrow)
        bounds.boundaryBufferMm += 20.0f;
    float sideLimit = 0.5f * c->obstacle2WidthMm + c->obstacle2EndSpaceMm;
    bounds.obstacle2WidthMm = fmaxf(c->obstacle2WidthMm, width);
    bounds.obstacle2EndSpaceMm = sideLimit - 0.5f * bounds.obstacle2WidthMm;
    if (bounds.obstacle2EndSpaceMm <= 0.0f)
        return false;
    float side = c->secondArrow;
    float h = side * p.yawRad, y = side * p.yMm;
    for (unsigned curvedReturn = 0U; curvedReturn < 3U; ++curvedReturn)
    for (unsigned dr = 0U; dr < 4U; ++dr)
    for (unsigned br = 0U; br < 4U; ++br)
    {
        if (planningWork.exhausted)
            return false;
        float rd = c->secondDepartureRadiusMm + 50.0f * dr;
        float rb = 0.5f * width + c->secondBackExtraRadiusMm + 25.0f * br;
        float cx = p.xMm - rd * sinf(h), cy = y + rd * cosf(h);
        if (fabsf(cy) >= rd + rb)
            continue;
        float dx = sqrtf((rd + rb) * (rd + rb) - cy * cy);
        float bx = cx + dx, join = atan2f(dx, cy);
        if (join <= h || join - h >= ROUTE_PI || bx + 250.0f <= rb)
            continue;
        unsigned headings = curvedReturn == 1U ? 21U : curvedReturn == 2U ? 4U : 1U;
        for (unsigned k = 0U; k < headings; ++k)
        {
            float rh = curvedReturn == 2U ? -ROUTE_PI : curvedReturn == 1U
                ? -(160.0f + 3.0f * k) * ROUTE_PI / 180.0f
                : -ROUTE_PI - asinf(rb / (bx + 250.0f));
            float rx = bx - rb * sinf(rh), ry = rb * cosf(rh);
            float finishX = curvedReturn == 0U ? TASK2_ROUTE_PARK_CENTER_X_MM : ParkFinishX(c);
            float gx = finishX - rx, gy = -ry;
            float returnRadius = 0.0f, returnLength = hypotf(gx, gy);
            if (curvedReturn == 0U)
                returnLength -= c->bodyCenterAheadOfRearAxleMm + 0.5f * c->frontProtrusionMm;
            if (curvedReturn == 1U)
            {
                float den = 2.0f * (-gx * sinf(rh) + gy * cosf(rh));
                if (fabsf(den) < 0.001f)
                    continue;
                returnRadius = (gx * gx + gy * gy) / den;
                if (fabsf(returnRadius) < ROUTE_MIN_RADIUS_MM || fabsf(returnRadius) > 1000000.0f)
                    continue;
                float rcx = rx - returnRadius * sinf(rh);
                float rcy = ry + returnRadius * cosf(rh);
                float finish = atan2f((ParkFinishX(c) - rcx) / returnRadius,
                                     rcy / returnRadius);
                float delta = finish - rh;
                while (delta * returnRadius < 0.0f)
                    delta += copysignf(2.0f * ROUTE_PI, returnRadius);
                while (fabsf(delta) >= 2.0f * ROUTE_PI)
                    delta -= copysignf(2.0f * ROUTE_PI, delta);
                if (fabsf(delta) >= ROUTE_PI)
                    continue;
                returnLength = delta * returnRadius;
            }
            out[0] = (Task2RouteStep){.distanceMm = rd * (join - h), .radiusMm = side * rd, .speedCps = c->obstacle2DepartureSpeedCps};
            out[1] = (Task2RouteStep){.distanceMm = rb * (join - rh), .radiusMm = -side * rb, .speedCps = c->cruiseSpeedCps};
            out[2] = (Task2RouteStep){.distanceMm = returnLength, .radiusMm = side * returnRadius, .speedCps = c->cruiseSpeedCps};
            unsigned n = 3U;
            if (curvedReturn == 2U)
            {
                /* Tangent return at yaw 180, then two stopped arcs restore
                 * centreline and yaw before parking. This handles cases where
                 * one direct return primitive would clip a park wall or O1. */
                float radius = fmaxf(ROUTE_MIN_RADIUS_MM, 0.5f * rb + 50.0f);
                float angle = acosf(1.0f - rb / (2.0f * radius));
                float parkStraight = 100.0f + 100.0f * k;
                float sx = ParkFinishX(c) + parkStraight + 2.0f * radius * sinf(angle);
                out[2] = (Task2RouteStep){.distanceMm = bx - sx, .radiusMm = 0.0f, .speedCps = c->cruiseSpeedCps};
                out[3] = (Task2RouteStep){.distanceMm = radius * angle, .radiusMm = -side * radius, .speedCps = c->cruiseSpeedCps};
                out[4] = (Task2RouteStep){.distanceMm = radius * angle, .radiusMm = side * radius, .speedCps = c->cruiseSpeedCps};
                out[5] = (Task2RouteStep){.distanceMm = parkStraight, .radiusMm = 0.0f, .speedCps = c->approachSpeedCps};
                n = 6U;
            }
            if (CandidateSweep(&bounds, p, out, n))
            {
                *count = n;
                return true;
            }
        }
    }
    return BuildTraverseTail(c, &bounds, p, width, out, count);
}

/* The user-authorized IR quarter-turn and rear bypass may use less than the
 * nominal obstacle buffer after a confirmed IR edge. One millimetre enforces
 * positive model separation; body overlap and boundary checks still reject. */
static bool CheckIROutbound(const Task2RouteConfig *c, Task2RoutePose p,
    const Task2RouteStep *steps, unsigned count)
{
    Task2RouteConfig exception = *c;
    if (c->firstArrow != c->secondArrow)
        exception.obstacleBufferMm = 1.0f;
#if TASK2_ROUTE_EXHAUSTIVE_CHECKS
    return Sweep(&exception, p, steps, count, 1.0f, false);
#else
    Task2RouteReport accepted = EmptyReport();
    if (!BoundedSweep(&exception, p, steps, count, false, &accepted))
        return false;
    accepted.obstacleBufferExceptionUsed = c->firstArrow != c->secondArrow;
    planningWork.acceptedClearance = accepted;
    return true;
#endif
}

static Task2RouteConfig ObservedBounds(const Task2RouteConfig *c, float width)
{
    Task2RouteConfig bounds = *c;
    float sideLimit = 0.5f * c->obstacle2WidthMm + c->obstacle2EndSpaceMm;
    bounds.obstacle2WidthMm = fmaxf(c->obstacle2WidthMm, width);
    bounds.obstacle2EndSpaceMm = sideLimit - 0.5f * bounds.obstacle2WidthMm;
    return bounds;
}

static bool BuildIRQuarterTurn(const Task2RouteConfig *c, Task2RoutePose p,
    float width, Task2RouteStep *turn)
{
    Task2RouteConfig bounds = ObservedBounds(c, width);
    float h = c->secondArrow * p.yawRad;
    if (bounds.obstacle2EndSpaceMm <= 0.0f || h <= 0.0f || h >= ROUTE_PI)
        return false;
    *turn = (Task2RouteStep){.distanceMm = ROUTE_MIN_RADIUS_MM * h, .radiusMm = -c->secondArrow * ROUTE_MIN_RADIUS_MM, .speedCps = c->cruiseSpeedCps};
    return CheckIROutbound(&bounds, p, turn, 1U);
}

/* Calculate the return lane from obstacle clearance and the swept-body side
 * boundary. A corner rotating around the arc centre is bounded by its radius;
 * this gives a closed-form maximum back-turn radius, without a radius grid. */
static bool MakeRearArc(const Task2RouteConfig *c, Task2RoutePose start, Task2RouteStep *arc)
{
    float side = c->secondArrow, h = side * start.yawRad, lateral = side * start.yMm;
    float half = 0.5f * c->bodyWidthMm;
    float front = c->bodyCenterAheadOfRearAxleMm + 0.5f * c->bodyLengthMm + c->frontProtrusionMm;
    float limit = 0.5f * c->obstacle2WidthMm + c->obstacle2EndSpaceMm - c->boundaryBufferMm;
    float k = limit + lateral;
    float b = k * cosf(h) + half;
    float d = half * half + front * front - k * k;
    float discriminant = b * b - sinf(h) * sinf(h) * d;
    if (k <= 0.0f || discriminant <= 0.0f || b + sqrtf(discriminant) <= 0.0f)
        return false;
    float maxRadius = -d / (b + sqrtf(discriminant));
    float minLane = 0.5f * c->obstacle2WidthMm + half + c->obstacleBufferMm;
    float maxLane = maxRadius * (1.0f + cosf(h)) - lateral;
    float returnLane = fminf(minLane + 20.0f, maxLane - 2.0f);
    if (returnLane < minLane + 1.0f)
        return false;
    float radius = (lateral + returnLane) / (1.0f + cosf(h));
    if (!isfinite(radius) || radius < ROUTE_MIN_RADIUS_MM)
        return false;
    *arc = (Task2RouteStep){.distanceMm = radius * (ROUTE_PI + h),
        .radiusMm = -side * radius, .speedCps = c->cruiseSpeedCps};
    return true;
}

static bool CheckParkingStraight(const Task2RouteConfig *c, Task2RoutePose p,
    Task2RouteStep *straight)
{
    float cosine = cosf(p.yawRad), sine = sinf(p.yawRad);
    float centre = c->bodyCenterAheadOfRearAxleMm + 0.5f * c->frontProtrusionMm;
    float travel = (TASK2_ROUTE_PARK_CENTER_X_MM - p.xMm) * cosine - p.yMm * sine - centre;
    if (cosine >= -0.5f || travel <= 0.0f)
        return false;
    *straight = (Task2RouteStep){.distanceMm = travel, .speedCps = c->cruiseSpeedCps};
#if TASK2_ROUTE_EXHAUSTIVE_CHECKS
    return Sweep(c, p, straight, 1U, 1.0f, true);
#else
    Task2RouteReport accepted = EmptyReport();
    if (!BoundedSweep(c, p, straight, 1U, true, &accepted))
        return false;
    planningWork.acceptedClearance = accepted;
    return true;
#endif
}

/* The body centre follows a circle of radius hypot(axle radius, centre offset).
 * Solve that circle through the body-centre parking goal, then recover the
 * axle's exit heading. A small lateral goal offset can preserve O1 clearance
 * after a measured rear-stop error; every candidate checks full containment. */
static bool CheckParkingArc(const Task2RouteConfig *c, Task2RoutePose p,
    float goalY, Task2RouteStep *arc)
{
    float gx = TASK2_ROUTE_PARK_CENTER_X_MM - p.xMm, gy = goalY - p.yMm;
    float centre = c->bodyCenterAheadOfRearAxleMm + 0.5f * c->frontProtrusionMm;
    float den = 2.0f * (-gx * sinf(p.yawRad) + gy * cosf(p.yawRad));
    if (fabsf(den) < 0.001f || cosf(p.yawRad) >= -0.5f)
        return false;
    float radius = (gx * gx + gy * gy - centre * centre) / den;
    if (!isfinite(radius) || fabsf(radius) < ROUTE_MIN_RADIUS_MM || fabsf(radius) > 1000000.0f)
        return false;
    float cx = p.xMm - radius * sinf(p.yawRad), cy = p.yMm + radius * cosf(p.yawRad);
    float heading = atan2f(goalY - cy, TASK2_ROUTE_PARK_CENTER_X_MM - cx) + atan2f(radius, centre);
    float delta = atan2f(sinf(heading - p.yawRad), cosf(heading - p.yawRad));
    if (delta * radius <= 0.0f || fabsf(delta) >= 0.5f * ROUTE_PI)
        return false;
    *arc = (Task2RouteStep){.distanceMm = delta * radius,
        .radiusMm = radius, .speedCps = c->cruiseSpeedCps};
#if TASK2_ROUTE_EXHAUSTIVE_CHECKS
    return Sweep(c, p, arc, 1U, 1.0f, true);
#else
    Task2RouteReport accepted = EmptyReport();
    if (!BoundedSweep(c, p, arc, 1U, true, &accepted))
        return false;
    planningWork.acceptedClearance = accepted;
    return true;
#endif
}

/* The tangent passes through the park's body-centre target. Stopping before
 * that point by the axle-to-body-centre offset parks the whole footprint. */
static bool AimRearArcAtPark(Task2RoutePose start, Task2RouteStep *arc)
{
    float r = arc->radiusMm;
    float cx = start.xMm - r * sinf(start.yawRad), cy = start.yMm + r * cosf(start.yawRad);
    float dx = TASK2_ROUTE_PARK_CENTER_X_MM - cx, dy = -cy, distance = hypotf(dx, dy);
    if (distance <= fabsf(r))
        return false;
    float bearing = atan2f(dy, dx), offset = acosf(fabsf(r) / distance);
    for (int sign = -1; sign <= 1; sign += 2)
    {
        float angle = bearing + sign * offset;
        float ex = cx + fabsf(r) * cosf(angle), ey = cy + fabsf(r) * sinf(angle);
        float heading = atan2f((ex - cx) / r, (cy - ey) / r);
        float goalHeading = atan2f(-ey, TASK2_ROUTE_PARK_CENTER_X_MM - ex);
        if (cosf(heading - goalHeading) < 0.9999f)
            continue;
        float delta = atan2f(sinf(heading - start.yawRad), cosf(heading - start.yawRad));
        if (delta * r < 0.0f)
            delta += copysignf(2.0f * ROUTE_PI, r);
        if (fabsf(delta) > 1.5f * ROUTE_PI)
            continue;
        arc->distanceMm = delta * r;
        return true;
    }
    return false;
}

static bool RearBypassAt(const Task2RouteConfig *bounds, Task2RoutePose p,
    float positioningDistance, Task2RouteStep out[2], unsigned *count, bool tangentReturn)
{
    unsigned arcIndex = positioningDistance > 0.0f ? 1U : 0U;
    Task2RoutePose arcStart = p;
    if (arcIndex)
    {
        out[0] = (Task2RouteStep){.distanceMm = positioningDistance, .speedCps = bounds->cruiseSpeedCps};
        arcStart = Advance(p, positioningDistance, 0.0f);
    }
    if (!MakeRearArc(bounds, arcStart, &out[arcIndex]) ||
        (tangentReturn && !AimRearArcAtPark(arcStart, &out[arcIndex])))
        return false;
    bool crossed[2][2] = {{false, false}, {false, false}};
    crossed[1][0] = p.xMm >= bounds->parkToObstacle1Mm + 150.0f + bounds->obstacleFaceGapMm;
    Task2RoutePose crossingStart = p;
    for (unsigned i = 0U; i <= arcIndex; ++i)
    {
        if (!Crossings(bounds, crossingStart, &out[i], crossed))
            return false;
        crossingStart = Advance(crossingStart, out[i].distanceMm, out[i].radiusMm);
    }
    if (!crossed[1][0] || !CheckIROutbound(bounds, p, out, arcIndex + 1U))
        return false;
    *count = arcIndex + 1U;
    return true;
}

/* Plan only the rear bypass. Parking is built after its measured brake stop.
 * Prefer a direct arc; otherwise bracket a positioning straight geometrically
 * and refine it with at most ten clearance checks. The accepting check is reused. */
static bool BuildParallelReturn(const Task2RouteConfig *c, Task2RoutePose p,
    float width, Task2RouteStep out[6], unsigned *count, bool *tangentReturn)
{
    Task2RouteConfig bounds = ObservedBounds(c, width);
    float h = c->secondArrow * p.yawRad;
    if (bounds.obstacle2EndSpaceMm <= 0.0f || fabsf(h) > 15.0f * ROUTE_PI / 180.0f)
        return false;
    float rear = c->parkToObstacle1Mm + 200.0f + c->obstacleFaceGapMm;
    float front = c->bodyCenterAheadOfRearAxleMm + 0.5f * c->bodyLengthMm + c->frontProtrusionMm;
    float clearance = c->firstArrow == c->secondArrow ? c->obstacleBufferMm + 20.0f : 21.0f;
    float upperPosition = (rear + front + clearance - p.xMm) / cosf(h);
    for (unsigned attempt = 0U; attempt < 2U; ++attempt)
    {
        bool tangent = attempt == 0U;
        if (!RearBypassAt(&bounds, p, 0.0f, out, count, tangent))
        {
            float hi = upperPosition, lo = 0.0f;
            if (hi <= 0.0f || !RearBypassAt(&bounds, p, hi, out, count, tangent))
                continue;
            for (unsigned i = 0U; i < 10U; ++i)
            {
                if (planningWork.exhausted)
                    return false;
                float mid = 0.5f * (lo + hi);
                if (RearBypassAt(&bounds, p, mid, out, count, tangent))
                    hi = mid;
                else
                    lo = mid;
            }
            if (!RearBypassAt(&bounds, p, hi + 10.0f, out, count, tangent))
                continue;
        }
        if (tangent)
        {
#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
            Task2RouteReport arcClearance = planningWork.acceptedClearance;
#endif
            Task2RoutePose finish = p;
            for (unsigned i = 0U; i < *count; ++i)
                finish = Advance(finish, out[i].distanceMm, out[i].radiusMm);
            Task2RouteStep straight;
            if (!CheckParkingStraight(&bounds, finish, &straight))
                continue;
#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
            MergeMargins(&planningWork.acceptedClearance, &arcClearance);
#endif
        }
        *tangentReturn = tangent;
        return true;
    }
    return false;
}

static float WrapYaw(float yaw)
{
    return atan2f(sinf(yaw), cosf(yaw));
}

/* Construct a measured-pose parking join ending at the exact park heading.
 * The unequal arc angles remove the measured rear-arc yaw error. */
static bool ParkingJoinAt(const Task2RouteConfig *c, Task2RoutePose p, float travel,
    float parkStraight, Task2RouteStep out[4], float *finishX)
{
    Task2RoutePose start = Advance(p, travel, 0.0f);
    float side = c->secondArrow, lateral = -side * start.yMm;
    float error = side * WrapYaw(start.yawRad + side * ROUTE_PI);
    float radius = fmaxf(ROUTE_MIN_RADIUS_MM, 0.5f * lateral + 50.0f);
    float cosine = 0.5f * (1.0f + cosf(error) - lateral / radius);
    if (lateral <= 0.0f || cosine <= -1.0f || cosine >= 1.0f)
        return false;
    float secondAngle = acosf(cosine), firstAngle = secondAngle + error;
    if (firstAngle <= 0.0f)
        return false;
    out[0] = (Task2RouteStep){.distanceMm = travel, .speedCps = c->cruiseSpeedCps};
    out[1] = (Task2RouteStep){.distanceMm = radius * firstAngle,
        .radiusMm = -side * radius, .speedCps = c->cruiseSpeedCps};
    out[2] = (Task2RouteStep){.distanceMm = radius * secondAngle,
        .radiusMm = side * radius, .speedCps = c->cruiseSpeedCps};
    out[3] = (Task2RouteStep){.distanceMm = parkStraight, .speedCps = c->approachSpeedCps};
    Task2RoutePose finish = Advance(start, out[1].distanceMm, out[1].radiusMm);
    finish = Advance(finish, out[2].distanceMm, out[2].radiusMm);
    finish = Advance(finish, parkStraight, 0.0f);
    *finishX = finish.xMm;
    return true;
}

static bool BuildParkingReturn(const Task2RouteConfig *c, Task2RoutePose p,
    float width, Task2RouteStep out[4], unsigned *count, bool allowHeadingCorrection, bool *headingOnly)
{
    *headingOnly = false;
    Task2RouteConfig bounds = ObservedBounds(c, width);
    if (CheckParkingStraight(&bounds, p, &out[0]))
    {
        *count = 1U;
        return true;
    }
    /* Prefer the centreline, then two small offsets toward the return side.
     * This bounded choice applies to both USS and IR routes. */
    for (unsigned goal = 0U; goal < 3U; ++goal)
    {
        if (CheckParkingArc(&bounds, p, -c->secondArrow * 25.0f * goal, &out[0]))
        {
            *count = 1U;
            return true;
        }
    }
    if (fabsf(WrapYaw(p.yawRad + c->secondArrow * ROUTE_PI)) > 45.0f * ROUTE_PI / 180.0f)
        return false;
    /* Correct an outward exit heading before a long return straight can
     * consume the remaining arena side clearance. Preflight the parking join
     * from the corrected pose, then execute only the correction and replan
     * from its measured stop. The preflight call disables further correction. */
    float correction = WrapYaw(-c->secondArrow * ROUTE_PI - p.yawRad);
    if (allowHeadingCorrection && c->firstArrow == c->secondArrow && fabsf(correction) > 0.5f * ROUTE_PI / 180.0f)
    {
        Task2RouteStep heading = {.distanceMm = ROUTE_MIN_RADIUS_MM * fabsf(correction),
            .radiusMm = copysignf(ROUTE_MIN_RADIUS_MM, correction), .speedCps = c->approachSpeedCps};
        Task2RoutePose corrected = Advance(p, heading.distanceMm, heading.radiusMm);
        bool headingClear;
#if TASK2_ROUTE_EXHAUSTIVE_CHECKS
        headingClear = Sweep(&bounds, p, &heading, 1U, 1.0f, false);
#else
        Task2RouteReport headingReport = EmptyReport();
        headingClear = BoundedSweep(&bounds, p, &heading, 1U, false, &headingReport);
#endif
        Task2RouteStep parking[4];
        bool unusedHeadingOnly;
        unsigned parkingCount = 0U;
        if (headingClear && BuildParkingReturn(c, corrected, width, parking, &parkingCount, false, &unusedHeadingOnly) && parkingCount <= 4U)
        {
            out[0] = heading;
            *count = 1U;
            *headingOnly = true; /* Replan the remainder at its measured stop. */
#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
            MergeMargins(&planningWork.acceptedClearance, &headingReport);
#endif
            return true;
        }
    }
    for (unsigned park = 0U; park < 4U; ++park)
    {
        float parkStraight = 100.0f + 100.0f * park;
        float lo = 0.0f, hi = p.xMm - ParkFinishX(c), x;
        if (hi <= 0.0f || !ParkingJoinAt(c, p, lo, parkStraight, out, &x) || x < ParkFinishX(c) ||
            !ParkingJoinAt(c, p, hi, parkStraight, out, &x) || x > ParkFinishX(c))
            continue;
        for (unsigned i = 0U; i < 14U; ++i)
        {
            float mid = 0.5f * (lo + hi);
            if (!ParkingJoinAt(c, p, mid, parkStraight, out, &x))
                return false;
            if (x > ParkFinishX(c))
                lo = mid;
            else
                hi = mid;
        }
        if (!ParkingJoinAt(c, p, 0.5f * (lo + hi), parkStraight, out, &x))
            continue;
#if TASK2_ROUTE_EXHAUSTIVE_CHECKS
        if (Sweep(&bounds, p, out, 4U, 1.0f, true))
        {
            *count = 4U;
            return true;
        }
#else
        Task2RouteReport accepted = EmptyReport();
        if (BoundedSweep(&bounds, p, out, 4U, true, &accepted))
        {
            planningWork.acceptedClearance = accepted;
            *count = 4U;
            return true;
        }
#endif
    }
    return false;
}

/* Geometry only: obtain an outside parallel lane through a capped outward
 * turn, optional diagonal straight and counterturn. The caller owns sensing.
 * Preserve 40 mm for measured stop-pose error; rear planning follows the actual stop. */
static bool BuildLateralClearance(const Task2RouteConfig *c, Task2RoutePose start,
    float width, Task2RouteStep out[3], unsigned *count)
{
    Task2RouteConfig bounds = ObservedBounds(c, width);
    float side = c->secondArrow, h = side * start.yawRad;
    float lane = 0.5f * bounds.obstacle2WidthMm + 0.5f * c->bodyWidthMm + c->obstacleBufferMm + 40.0f;
    float half = 0.5f * c->bodyWidthMm;
    float front = c->bodyCenterAheadOfRearAxleMm + 0.5f * c->bodyLengthMm + c->frontProtrusionMm;
    float limit = 0.5f * c->obstacle2WidthMm + c->obstacle2EndSpaceMm - c->boundaryBufferMm - 10.0f;
    float room = limit - lane;
    if (room <= half)
        return false;
    /* Bound the front corner during the counterturn analytically. A looser
     * counterturn can obtain the lane without an excessive outer swing. */
    float counterRadius = fmaxf(ROUTE_MIN_RADIUS_MM,
        (half * half + front * front - room * room) / (2.0f * (room - half)));
    if (bounds.obstacle2EndSpaceMm <= 0.0f || !isfinite(counterRadius) || counterRadius > 1000000.0f)
        return false;
    for (unsigned angle = 20U; angle <= 40U; angle += 5U)
    {
        float heading = angle * ROUTE_PI / 180.0f;
        heading = fmaxf(heading, h);
        if (heading > 45.0f * ROUTE_PI / 180.0f)
            return false;
        unsigned n = 0U;
        Task2RoutePose p = start;
        if (heading > h + 0.001f)
        {
            out[n] = (Task2RouteStep){.distanceMm = ROUTE_MIN_RADIUS_MM * (heading - h),
                .radiusMm = side * ROUTE_MIN_RADIUS_MM, .speedCps = c->obstacle2DepartureSpeedCps};
            p = Advance(p, out[n].distanceMm, out[n].radiusMm);
            ++n;
        }
        float counterShift = counterRadius * (1.0f - cosf(heading));
        float straight = (lane - side * p.yMm - counterShift) / sinf(heading);
        if (straight > 0.0f)
        {
            out[n] = (Task2RouteStep){.distanceMm = straight, .speedCps = c->obstacle2DepartureSpeedCps};
            p = Advance(p, straight, 0.0f);
            ++n;
        }
        out[n] = (Task2RouteStep){.distanceMm = counterRadius * heading,
            .radiusMm = -side * counterRadius, .speedCps = c->obstacle2DepartureSpeedCps};
        p = Advance(p, out[n].distanceMm, out[n].radiusMm);
        ++n;
#if TASK2_ROUTE_EXHAUSTIVE_CHECKS
        if (!Sweep(&bounds, start, out, n, 1.0f, false))
            continue;
#else
        Task2RouteReport lateralReport = EmptyReport();
        if (!BoundedSweep(&bounds, start, out, n, false, &lateralReport))
            continue;
#endif
#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
        planningWork.acceptedClearance = lateralReport;
#endif
        *count = n;
        return true;
    }
    return false;
}

/* Startup screens quarter-turn room only. Both host and MCU build the return
 * from actual stopped poses; the reference edge is not a brake-stop pose. */
static bool IRReturnHasRoom(const Task2RouteConfig *c, Task2RoutePose p)
{
    float h = c->secondArrow * p.yawRad;
    float front = c->bodyCenterAheadOfRearAxleMm + 0.5f * c->bodyLengthMm + c->frontProtrusionMm;
    float outer = c->secondArrow * p.yMm - ROUTE_MIN_RADIUS_MM * cosf(h) +
        hypotf(ROUTE_MIN_RADIUS_MM + 0.5f * c->bodyWidthMm, front);
    return outer <= 0.5f * c->obstacle2WidthMm + c->obstacle2EndSpaceMm - c->boundaryBufferMm;
}

static bool BuildIRDeparture(const Task2RouteConfig *c, Task2RoutePose p,
    const Task2RouteIRDeparture *ir, float turnRadius, Task2RouteStep *step)
{
    float side = c->secondArrow, radius = side * turnRadius;
    float face = c->parkToObstacle1Mm + 100.0f + c->obstacleFaceGapMm;
    step[0] = (Task2RouteStep){.distanceMm = turnRadius * (0.5f * ROUTE_PI - side * p.yawRad), .radiusMm = radius, .speedCps = c->obstacle2DepartureSpeedCps};
    Task2RoutePose front = Advance(p, step[0].distanceMm, radius);
    float x = front.xMm + ir->mount.aheadMm * cosf(front.yawRad) - ir->mount.leftMm * sinf(front.yawRad);
    float y = front.yMm + ir->mount.aheadMm * sinf(front.yawRad) + ir->mount.leftMm * cosf(front.yawRad);
    float ray = front.yawRad + ir->mount.yawDeg * ROUTE_PI / 180.0f;
    if (face <= x || cosf(ray) <= 0.05f)
        return false;
    float hit = side * (y + (face - x) * tanf(ray));
    /* Keep the straight-start ray within the nominal outgoing face plus its
     * stated uncertainty, so initial absence does not create an excessive bound. */
    if (!isfinite(hit) || hit > 0.5f * c->obstacle2WidthMm + ir->edgeUncertaintyMm)
        return false;
    /* Nominal preview for offline geometry, never an execution distance cap. */
    step[1] = (Task2RouteStep){.distanceMm = fmaxf(0.0f, 0.5f * c->obstacle2WidthMm - hit) + ir->guardTravelMm,
        .sensorTerminated = true, .radiusMm = 0.0f, .speedCps = c->obstacle2DepartureSpeedCps};
    float edgeTravel = step[0].distanceMm + fmaxf(0.0f, 0.5f * c->obstacle2WidthMm - hit);
    /* Screen return room at the nominal reference edge. Preview travel is
     * not an execution limit or a prediction of the actual stopped pose. The actual
     * brake stop must pass suffix preflight before any return motion starts. */
    Task2RoutePose predictedStop = Advance(front, fmaxf(0.0f, edgeTravel - step[0].distanceMm), 0.0f);
    if (!IRReturnHasRoom(c, predictedStop))
        return false;
#if TASK2_ROUTE_EXHAUSTIVE_CHECKS
    return Sweep(c, p, step, 2U, 1.0f, false);
#else
    Task2RouteReport accepted = EmptyReport();
    if (!BoundedSweep(c, p, step, 2U, false, &accepted))
        return false;
    planningWork.acceptedClearance = accepted;
    return true;
#endif
}

static bool BuildSteps(Task2Route *route, const Task2RouteIRDeparture *ir, bool lateral)
{
    const Task2RouteConfig *c = &route->config;
    float firstStart = c->parkToObstacle1Mm - CameraOffset(c) - c->firstArcCameraRangeMm;
    /* The offline-validated O1 arcs are invariant under O1 translation. */
    float sine = (CameraOffset(c) + c->firstArcCameraRangeMm + c->firstRejoinBeyondFaceMm) /
        (4.0f * c->firstRadiusMm);
    if (sine <= 0.0f || sine >= 1.0f || firstStart <= TASK2_ROUTE_START_X_MM)
        return Reject(route, "NO_O1_BYPASS");
    float angle = asinf(sine), exit = c->obstacle1ExitYawDeg * ROUTE_PI / 180.0f;
    if (exit >= angle)
        return Reject(route, "INVALID_O1_EXIT");
    SetStep(route, 0U, firstStart - TASK2_ROUTE_START_X_MM, 0.0f);
    SetStep(route, 1U, c->firstRadiusMm * angle, c->firstArrow * c->firstRadiusMm);
    SetStep(route, 2U, c->firstRadiusMm * (angle + exit), -c->firstArrow * c->firstRadiusMm);
    Task2RoutePose start = {TASK2_ROUTE_START_X_MM, 0.0f, 0.0f};
    for (unsigned i = 0U; i < 3U; ++i)
        start = Advance(start, route->steps[i].distanceMm, route->steps[i].radiusMm);
    float face = c->parkToObstacle1Mm + 100.0f + c->obstacleFaceGapMm;
    float dx = face - start.xMm;
    float forward = dx * cosf(start.yawRad) - start.yMm * sinf(start.yawRad) - CameraOffset(c);
    float cameraLateral = -dx * sinf(start.yawRad) - start.yMm * cosf(start.yawRad);
    /* 500 mm is the recognition baseline. If a departure cannot fit there,
     * try an earlier endpoint within camera range, retaining the same straight. */
#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
    float checkedApproachTravel = -1.0f;
    Task2RouteReport approachClearance = EmptyReport();
#endif
    for (unsigned radiusTry = 0U; radiusTry < (ir ? 3U : 1U); ++radiusTry)
    {
        float irRadius = radiusTry == 2U ? ROUTE_MIN_RADIUS_MM
            : fmaxf(ROUTE_MIN_RADIUS_MM, c->secondIRDepartureRadiusMm - 25.0f * radiusTry);
        for (float range = lateral ? c->recognitionCameraRangeMm - 25.0f : c->secondArcCameraRangeMm;
             range <= c->recognitionCameraRangeMm; range += 25.0f)
        {
            if (planningWork.exhausted)
                return Reject(route, "PLANNING_BUDGET");
            if (fabsf(cameraLateral) >= range)
                continue;
            float travel = forward - sqrtf(range * range - cameraLateral * cameraLateral);
            if (lateral && travel < 0.0f)
                travel = 0.0f;
            if (travel < 0.0f || travel > c->obstacle2ApproachMaxMm)
                continue;
            Task2RoutePose departure = Advance(start, travel, 0.0f);
            if (!Task2Route_ObstacleCameraVisible(c, &departure, 1U))
                continue;
            if (ir)
            {
                Task2RouteConfig camera = *c;
                camera.recognitionCameraRangeMm -= 25.0f;
                camera.cameraHorizontalFovDeg -= 4.0f; /* Two-degree endpoint recognition reserve. */
                if (!Task2Route_ObstacleCameraVisible(&camera, &departure, 1U))
                    continue;
            }
            SetStep(route, 3U, travel, 0.0f);
#if TASK2_ROUTE_EXHAUSTIVE_CHECKS
            if (!Sweep(c, (Task2RoutePose){TASK2_ROUTE_START_X_MM, 0.0f, 0.0f},
                route->steps, 4U, 1.0f, false))
                continue;
#else
            /* Steps 0-2 are the trusted O1 template. Certify only the variable
             * O2 approach; a previously checked longer straight covers shorter
             * candidates on this same ray, including other departure radii. */
            if (travel > checkedApproachTravel)
            {
                Task2RouteReport accepted = EmptyReport();
                if (!BoundedSweep(c, start, &route->steps[3], 1U, false, &accepted))
                    continue;
                checkedApproachTravel = travel;
                approachClearance = accepted;
            }
#endif
            unsigned count = 0U;
            if (lateral)
            {
                Task2RouteStep clearance[3];
                if (!BuildLateralClearance(c, departure, c->obstacle2WidthMm + c->obstacle2DepartureWidthReserveMm,
                        clearance, &count))
                    continue;
                route->steps[4] = clearance[0];
                /* Both sensor legs have finite preflighted limits. If loss
                 * occurs during the turn, brake there; otherwise continue
                 * along the selected heading until the straight limit. */
                route->steps[5] = count == 3U ? clearance[1]
                    : (Task2RouteStep){.distanceMm = 1.0f, .speedCps = c->obstacle2DepartureSpeedCps};
                route->stepCount = 6U;
                route->obstacle2LateralDeparture = route->obstacle2ReturnPending = true;
#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
                route->report = approachClearance;
                MergeMargins(&route->report, &planningWork.acceptedClearance);
                route->report.clearanceFromStep = TASK2_ROUTE_O2_APPROACH_STEP;
#endif
                return true;
            }
            if (ir)
            {
                if (BuildIRDeparture(c, departure, ir, irRadius, &route->steps[4]))
                {
                    route->stepCount = 6U;
                    route->obstacle2IRDeparture = route->obstacle2ReturnPending = true;
#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
                    route->report = approachClearance;
                    MergeMargins(&route->report, &planningWork.acceptedClearance);
                    route->report.clearanceFromStep = TASK2_ROUTE_O2_APPROACH_STEP;
#endif
                    return true;
                }
                continue;
            }
            if (BuildTail(c, departure, c->obstacle2WidthMm + c->obstacle2DepartureWidthReserveMm,
                    &route->steps[4], &count))
            {
                route->stepCount = 4U + count;
#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
                route->report = approachClearance;
                MergeMargins(&route->report, &planningWork.acceptedClearance);
                route->report.clearanceFromStep = TASK2_ROUTE_O2_APPROACH_STEP;
#endif
                return true;
            }
        }
    }
    return Reject(route, "NO_O2_APPROACH_OR_RETURN");
}

static bool InstallSuffix(Task2Route *route, const Task2RoutePose *pose, float width,
    const Task2RouteStep *tail, unsigned count, unsigned completedStep, unsigned exceptionPrefixCount, const char *reason)
{
#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
    (void)width;
#endif
    if (completedStep + 1U + count > TASK2_ROUTE_STEP_COUNT)
        return FinishPlanning(route, Reject(route, "RETURN_CAPACITY"));
    for (unsigned i = 0U; i < count; ++i)
        route->steps[completedStep + 1U + i] = tail[i];
    route->stepCount = completedStep + 1U + count;
    route->totalTravelMm = route->totalYawRad = 0.0f;
    for (unsigned i = 0U; i < route->stepCount; ++i)
    {
        route->totalTravelMm += route->steps[i].distanceMm;
        if (route->steps[i].radiusMm != 0.0f)
            route->totalYawRad += route->steps[i].distanceMm / route->steps[i].radiusMm;
    }
    Task2RoutePose finish = *pose;
#if TASK2_ROUTE_EXHAUSTIVE_CHECKS
    Task2RouteConfig bounds = route->config;
    float sideLimit = 0.5f * bounds.obstacle2WidthMm + bounds.obstacle2EndSpaceMm;
    bounds.obstacle2WidthMm = fmaxf(bounds.obstacle2WidthMm, width);
    bounds.obstacle2EndSpaceMm = sideLimit - 0.5f * bounds.obstacle2WidthMm;
#else
    if (!route->report.valid)
    {
        route->report = EmptyReport();
        route->report.clearanceFromStep = completedStep + 1U;
    }
    MergeMargins(&route->report, &planningWork.acceptedClearance);
#endif
    for (unsigned i = 0U; i < count; ++i)
    {
#if TASK2_ROUTE_EXHAUSTIVE_CHECKS
        Task2RouteConfig section = bounds;
        if (i < exceptionPrefixCount)
            section.obstacleBufferMm = 1.0f;
        for (float d = 0.0f; d < tail[i].distanceMm; d += 1.0f)
        {
            Task2RoutePose sample = Advance(finish, d, tail[i].radiusMm);
            (void)PlanningPose(&section, &sample, &route->report);
        }
#endif
        finish = Advance(finish, tail[i].distanceMm, tail[i].radiusMm);
    }
    route->report.finishPose = finish;
    route->report.valid = true;
    route->report.obstacleBufferExceptionUsed |= exceptionPrefixCount > 0U;
    route->report.reason = reason;
    return FinishPlanning(route, true);
}

bool Task2Route_ReplanObstacle2(Task2Route *route, const Task2RoutePose *pose, float width,
    unsigned completedDepartureStep)
{
    planningWork.poseChecks = 0U;
    planningWork.exhausted = false;
    Task2RouteStep tail[9];
    unsigned count = 0U;
    if (!route || !pose || !isfinite(width) || width < 300.0f ||
        (completedDepartureStep != TASK2_ROUTE_O2_DEPARTURE_STEP &&
         !((route->obstacle2IRDeparture || route->obstacle2LateralDeparture) && completedDepartureStep == TASK2_ROUTE_O2_IR_STRAIGHT_STEP)) ||
        !isfinite(pose->xMm) || !isfinite(pose->yMm) || !isfinite(pose->yawRad))
        return route ? FinishPlanning(route, Reject(route, "NO_OBSERVED_EDGE_RETURN")) : false;
    if (route->obstacle2LateralDeparture)
    {
        if (!BuildLateralClearance(&route->config, *pose, width, tail, &count))
            return FinishPlanning(route, Reject(route, "NO_LATERAL_CLEARANCE_RETURN"));
        route->obstacle2ParallelReturnPending = true;
    }
    else if (route->obstacle2IRDeparture)
    {
        if (!BuildIRQuarterTurn(&route->config, *pose, width, tail))
            return FinishPlanning(route, Reject(route, "NO_IR_QUARTER_TURN"));
        count = 1U;
        route->obstacle2ParallelReturnPending = true;
    }
    else
    {
        if (!BuildTail(&route->config, *pose, width, tail, &count))
            return FinishPlanning(route, Reject(route, "NO_OBSERVED_EDGE_RETURN"));
        /* The complete nominal tail is certified above. Execute through the
         * rear bypass only, then rebuild parking from that measured stop,
         * just as for IR routes. Do not carry arc exit error into the park. */
        if (route->config.firstArrow == route->config.secondArrow)
        {
            route->obstacle2TangentRearArc = count == 3U && tail[2].radiusMm == 0.0f;
            count = count == 9U ? 5U : 2U;
            route->obstacle2ParkReturnPending = true;
        }
    }
    route->obstacle2ReturnPending = false;
    return InstallSuffix(route, pose, width, tail, count, completedDepartureStep,
        route->obstacle2IRDeparture, route->obstacle2IRDeparture ? "OK_IR_QUARTER" : "OK_OBSERVED_EDGE");
}

bool Task2Route_ReplanParallelReturn(Task2Route *route, const Task2RoutePose *pose, float width,
    unsigned completedQuarterStep)
{
    planningWork.poseChecks = 0U;
    planningWork.exhausted = false;
    Task2RouteStep tail[9];
    unsigned count = 0U;
    bool tangentReturn = false;
    if (!route || !pose || !route->obstacle2ParallelReturnPending ||
        completedQuarterStep + 1U != route->stepCount || !isfinite(width) || width < 300.0f ||
        !isfinite(pose->xMm) || !isfinite(pose->yMm) || !isfinite(pose->yawRad) ||
        !BuildParallelReturn(&route->config, *pose, width, tail, &count, &tangentReturn))
        return route ? FinishPlanning(route, Reject(route, "NO_PARALLEL_RETURN")) : false;
    route->obstacle2ParallelReturnPending = false;
    route->obstacle2TangentRearArc = tangentReturn;
    route->obstacle2ParkReturnPending = true;
    route->obstacle2DirectRearArc = tail[0].radiusMm != 0.0f;
    return InstallSuffix(route, pose, width, tail, count, completedQuarterStep,
        route->obstacle2IRDeparture ? (route->obstacle2DirectRearArc ? 1U : 2U) : 0U, "OK_PARALLEL_RETURN");
}

bool Task2Route_ReplanParkingReturn(Task2Route *route, const Task2RoutePose *pose,
    float width, unsigned completedRearStep)
{
    planningWork.poseChecks = 0U;
    planningWork.exhausted = false;
    Task2RouteStep tail[4];
    unsigned count = 0U;
    bool headingOnly = false;
    if (!route || !pose || !route->obstacle2ParkReturnPending ||
        completedRearStep + 1U != route->stepCount || !isfinite(width) || width < 300.0f ||
        !isfinite(pose->xMm) || !isfinite(pose->yMm) || !isfinite(pose->yawRad) ||
        !BuildParkingReturn(&route->config, *pose, width, tail, &count, route->parkingHeadingCorrections < 2U, &headingOnly))
        return route ? FinishPlanning(route, Reject(route, "NO_PARKING_RETURN")) : false;
    route->obstacle2ParkReturnPending = headingOnly;
    if (headingOnly)
        ++route->parkingHeadingCorrections;
    route->obstacle2StraightParkReturn = count == 1U && tail[0].radiusMm == 0.0f;
    return InstallSuffix(route, pose, width, tail, count, completedRearStep, 0U,
        headingOnly ? "OK_PARK_HEADING_CORRECTION" : route->obstacle2StraightParkReturn
            ? "OK_STRAIGHT_PARK_RETURN" : "OK_PARKING_RETURN");
}

/* Exact rectangle separation axes plus vertex-to-rectangle distances. */
static float ObstacleGap(const Task2RouteConfig *c, const Task2RoutePose *p,
                         const float *cx, const float *cy, float ox, float halfWidth)
{
    float gap = INFINITY;
    float minX = INFINITY, maxX = -INFINITY, minY = INFINITY, maxY = -INFINITY;
    float minF = INFINITY, maxF = -INFINITY, minS = INFINITY, maxS = -INFINITY;
    float hf = (c->bodyLengthMm + c->frontProtrusionMm) * 0.5f, hs = c->bodyWidthMm * 0.5f;
    float centre = c->bodyCenterAheadOfRearAxleMm + 0.5f * c->frontProtrusionMm;
    float cosine = cosf(p->yawRad), sine = sinf(p->yawRad);
    for (unsigned j = 0U; j < 4U; ++j)
    {
        minX = fminf(minX, cx[j]); maxX = fmaxf(maxX, cx[j]);
        minY = fminf(minY, cy[j]); maxY = fmaxf(maxY, cy[j]);
        gap = fminf(gap, hypotf(fmaxf(fabsf(cx[j] - ox) - 50.0f, 0.0f),
                                fmaxf(fabsf(cy[j]) - halfWidth, 0.0f)));
        float dx = ox + (j < 2U ? -50.0f : 50.0f) - p->xMm;
        float dy = (j % 2U ? halfWidth : -halfWidth) - p->yMm;
        float forward = dx * cosine + dy * sine - centre;
        float side = -dx * sine + dy * cosine;
        minF = fminf(minF, forward); maxF = fmaxf(maxF, forward);
        minS = fminf(minS, side); maxS = fmaxf(maxS, side);
        gap = fminf(gap, hypotf(fmaxf(fabsf(forward) - hf, 0.0f),
                                fmaxf(fabsf(side) - hs, 0.0f)));
    }
    bool overlap = minX <= ox + 50.0f && maxX >= ox - 50.0f &&
        minY <= halfWidth && maxY >= -halfWidth &&
        minF <= hf && maxF >= -hf && minS <= hs && maxS >= -hs;
    return overlap ? 0.0f : gap;
}

static float ParkWallGap(const Task2RouteConfig *c, const Task2RoutePose *p,
    const float cx[4], const float cy[4], float ax, float ay, float bx, float by,
    float cosine, float sine)
{
    float gap = INFINITY, vx = bx - ax, vy = by - ay, length2 = vx * vx + vy * vy;
    for (unsigned i = 0U; i < 4U; ++i)
    {
        float t = fminf(1.0f, fmaxf(0.0f, ((cx[i] - ax) * vx + (cy[i] - ay) * vy) / length2));
        gap = fminf(gap, hypotf(cx[i] - ax - t * vx, cy[i] - ay - t * vy));
        unsigned next = (i + 1U) % 4U;
        float ex = cx[next] - cx[i], ey = cy[next] - cy[i];
        float den = ex * vy - ey * vx;
        if (fabsf(den) > 0.00001f)
        {
            float dx = ax - cx[i], dy = ay - cy[i];
            float edgeT = (dx * vy - dy * vx) / den;
            float wallT = (dx * ey - dy * ex) / den;
            if (edgeT >= 0.0f && edgeT <= 1.0f && wallT >= 0.0f && wallT <= 1.0f)
                return 0.0f;
        }
    }
    float centre = c->bodyCenterAheadOfRearAxleMm + 0.5f * c->frontProtrusionMm;
    float halfLength = 0.5f * (c->bodyLengthMm + c->frontProtrusionMm);
    for (unsigned end = 0U; end < 2U; ++end)
    {
        float dx = (end ? bx : ax) - p->xMm, dy = (end ? by : ay) - p->yMm;
        float forward = dx * cosine + dy * sine - centre;
        float lateral = -dx * sine + dy * cosine;
        gap = fminf(gap, hypotf(fmaxf(0.0f, fabsf(forward) - halfLength),
            fmaxf(0.0f, fabsf(lateral) - 0.5f * c->bodyWidthMm)));
    }
    return gap;
}

static bool CheckFootprint(const Task2RouteConfig *c,
    const Task2RoutePose *p, float sideLimit, bool includeObstacle2, Task2RouteReport *report)
{
    float cx[4], cy[4];
    float cosine = cosf(p->yawRad), sine = sinf(p->yawRad);
    float hf = 0.5f * (c->bodyLengthMm + c->frontProtrusionMm), hs = 0.5f * c->bodyWidthMm;
    float centre = c->bodyCenterAheadOfRearAxleMm + 0.5f * c->frontProtrusionMm;
    for (unsigned j = 0U; j < 4U; ++j)
    {
        float f = centre + (j < 2U ? hf : -hf);
        float side = (j == 0U || j == 3U) ? hs : -hs;
        cx[j] = p->xMm + f * cosine - side * sine;
        cy[j] = p->yMm + f * sine + side * cosine;
        report->minimumSideMarginMm = fminf(report->minimumSideMarginMm,
                                            sideLimit - fabsf(cy[j]));
        if (cx[j] < 0.0f)
        {
            report->minimumParkBackMarginMm = fminf(report->minimumParkBackMarginMm, cx[j] + 500.0f);
            report->minimumParkSideMarginMm = fminf(report->minimumParkSideMarginMm, 300.0f - fabsf(cy[j]));
        }
    }
    /* Check body edges through the park mouth, including wall-tip crossings. */
    for (unsigned j = 0U; j < 4U; ++j)
    {
        unsigned next = (j + 1U) % 4U;
        if ((cx[j] < 0.0f) != (cx[next] < 0.0f))
        {
            float fraction = -cx[j] / (cx[next] - cx[j]);
            float y = cy[j] + fraction * (cy[next] - cy[j]);
            report->minimumParkSideMarginMm = fminf(report->minimumParkSideMarginMm,
                                                    300.0f - fabsf(y));
        }
    }
    report->minimumParkSideMarginMm = fminf(report->minimumParkSideMarginMm,
        fminf(ParkWallGap(c, p, cx, cy, -500.0f, -300.0f, 0.0f, -300.0f, cosine, sine),
              ParkWallGap(c, p, cx, cy, -500.0f, +300.0f, 0.0f, +300.0f, cosine, sine)));
    report->minimumParkBackMarginMm = fminf(report->minimumParkBackMarginMm,
        ParkWallGap(c, p, cx, cy, -500.0f, -300.0f, -500.0f, +300.0f, cosine, sine));
    report->minimumParkWallMarginMm = fminf(report->minimumParkBackMarginMm, report->minimumParkSideMarginMm);
    float firstX = c->parkToObstacle1Mm + 50.0f;
    float secondX = c->parkToObstacle1Mm + 150.0f + c->obstacleFaceGapMm;
    report->minimumObstacleGapMm = fminf(report->minimumObstacleGapMm,
        fminf(ObstacleGap(c, p, cx, cy, firstX, 50.0f),
              includeObstacle2 ? ObstacleGap(c, p, cx, cy, secondX, 0.5f * c->obstacle2WidthMm) : INFINITY));
    return report->minimumObstacleGapMm >= c->obstacleBufferMm &&
        report->minimumSideMarginMm >= c->boundaryBufferMm &&
        report->minimumParkSideMarginMm >= c->parkSideWallBufferMm &&
        report->minimumParkBackMarginMm >= c->parkBackWallBufferMm;
}

bool Task2Route_CheckPose(const Task2RouteConfig *config,
    const Task2RoutePose *pose, Task2RouteReport *report)
{
    return CheckFootprint(config, pose,
        0.5f * config->obstacle2WidthMm + config->obstacle2EndSpaceMm, true, report);
}

bool Task2Route_CheckObstacle1Pose(const Task2RouteConfig *config,
    float arenaHalfWidthMm, const Task2RoutePose *pose, Task2RouteReport *report)
{
    return CheckFootprint(config, pose, arenaHalfWidthMm, false, report);
}

#if TASK2_ROUTE_EXHAUSTIVE_CHECKS
static bool ValidatePath(Task2Route *route)
{
    Task2RouteReport *report = &route->report;
    report->recognitionPreviewValid = true;
    report->minimumObstacleGapMm = INFINITY;
    report->minimumSideMarginMm = INFINITY;
    report->minimumParkWallMarginMm = INFINITY;
    report->minimumParkBackMarginMm = report->minimumParkSideMarginMm = INFINITY;
    Task2RoutePose pose = {TASK2_ROUTE_START_X_MM, 0.0f, 0.0f};
    bool seen[2] = {false, false};
    bool crossed[2][2] = {{false, false}, {false, false}};
    float progress = 0.0f;
    float faces[] = {route->config.parkToObstacle1Mm,
        route->config.parkToObstacle1Mm + 100.0f + route->config.obstacleFaceGapMm};
    for (unsigned index = 0U; index < route->stepCount; ++index)
    {
        const Task2RouteStep *primitive = &route->steps[index];
        Task2RoutePose stepStart = pose;
        if (index == TASK2_ROUTE_O2_APPROACH_STEP && primitive->distanceMm == 0.0f &&
            Task2Route_ObstacleCameraVisible(&route->config, &pose, 1U))
        {
            seen[1] = true;
            report->recognitionPose[1] = pose;
            report->recognitionProgressMm[1] = progress;
        }
        for (float distance = 0.0f; distance < primitive->distanceMm;)
        {
            float ds = fminf(ROUTE_SAMPLE_SPACING_MM, primitive->distanceMm - distance);
            float previousX = pose.xMm;
            pose = Advance(stepStart, distance + ds, primitive->radiusMm);
            if (!PlanningPose(&route->config, &pose, report))
                return Reject(route, "CLEARANCE_OR_BOUNDARY");
            for (unsigned obstacle = 0U; obstacle < 2U; ++obstacle)
            {
                float centre = faces[obstacle] + 50.0f;
                if ((previousX < centre && pose.xMm >= centre) ||
                    (previousX > centre && pose.xMm <= centre))
                {
                    unsigned returning = cosf(pose.yawRad) < 0.0f;
                    int side = returning ? -route->config.secondArrow :
                        obstacle == 0U ? route->config.firstArrow : route->config.secondArrow;
                    if (side * pose.yMm <= 0.0f)
                        return Reject(route, "WRONG_BYPASS_SIDE");
                    crossed[obstacle][returning] = true;
                }
            }
            if (index == 0U || index == TASK2_ROUTE_O2_APPROACH_STEP)
            {
                unsigned obstacle = index == 0U ? 0U : 1U;
                if (!seen[obstacle] && Task2Route_ObstacleCameraVisible(&route->config, &pose, obstacle))
                {
                    seen[obstacle] = true;
                    report->recognitionPose[obstacle] = pose;
                    report->recognitionProgressMm[obstacle] = progress + ds;
                }
            }
            distance += ds;
            progress += ds;
        }
    }
    if (!seen[0] || !seen[1])
        return Reject(route, "NO_CAMERA_HANDOFF");
    if (route->obstacle2ReturnPending)
    {
        if (!crossed[0][0])
            return Reject(route, "INCOMPLETE_O1_BYPASS");
        report->finishPose = pose; /* Nominal preview pose; parking suffix is still pending. */
        report->valid = true;
        report->reason = route->obstacle2LateralDeparture ? "OK_LATERAL_PREFIX" : "OK_IR_PREFIX";
        return true;
    }
    for (unsigned i = 0U; i < 2U; ++i)
        if (!crossed[i][0] || !crossed[i][1])
            return Reject(route, "INCOMPLETE_BYPASS");
    float length = route->config.bodyLengthMm + route->config.frontProtrusionMm;
    float centre = route->config.bodyCenterAheadOfRearAxleMm + 0.5f * route->config.frontProtrusionMm;
    float cx = pose.xMm + centre * cosf(pose.yawRad);
    float cy = pose.yMm + centre * sinf(pose.yawRad);
    float ex = 0.5f * (length * fabsf(cosf(pose.yawRad)) + route->config.bodyWidthMm * fabsf(sinf(pose.yawRad)));
    float ey = 0.5f * (length * fabsf(sinf(pose.yawRad)) + route->config.bodyWidthMm * fabsf(cosf(pose.yawRad)));
    if (cx - ex <= -500.0f || cx + ex >= 0.0f || fabsf(cy) + ey >= 300.0f)
        return Reject(route, "PARKING_FOOTPRINT");
    report->finishPose = pose;
    report->valid = true;
    report->reason = "OK";
    return true;
}
#else
static void FinalizeReport(Task2Route *route)
{
    /* Candidate selection already certified the variable section. The fixed
     * O1 template is validated offline; do not sweep either section again. */
    Task2RoutePose pose = {TASK2_ROUTE_START_X_MM, 0.0f, 0.0f};
    for (unsigned i = 0U; i < route->stepCount; ++i)
        pose = Advance(pose, route->steps[i].distanceMm, route->steps[i].radiusMm);
    route->report.finishPose = pose;
    route->report.valid = true;
    route->report.reason = route->obstacle2ReturnPending
        ? (route->obstacle2LateralDeparture ? "OK_LATERAL_PREFIX" : "OK_IR_PREFIX") : "OK";
}
#endif

static bool Create(const Task2RouteConfig *config, const Task2RouteIRDeparture *ir, bool lateral, Task2Route *route)
{
    if (route == NULL)
        return false;
    planningWork.poseChecks = 0U;
    planningWork.exhausted = false;
    *route = (Task2Route){0};
    if (config != NULL)
        route->config = *config;
    if (config == NULL || !ValidConfig(config))
        return Reject(route, "INVALID_PARAMETERS");
    if (ir && (config->firstArrow == config->secondArrow ||
        !isfinite(ir->mount.aheadMm) || !isfinite(ir->mount.leftMm) ||
        !isfinite(ir->mount.yawDeg) || fabsf(ir->mount.yawDeg) > 180.0f ||
        !isfinite(ir->guardTravelMm) || ir->guardTravelMm <= 0.0f ||
        !isfinite(ir->acquisitionMinYawDeg) || ir->acquisitionMinYawDeg < 0.0f || ir->acquisitionMinYawDeg > 90.0f ||
        !isfinite(ir->acquisitionTravelMm) || ir->acquisitionTravelMm < 0.0f ||
        !isfinite(ir->edgeUncertaintyMm) || ir->edgeUncertaintyMm < 0.0f))
        return Reject(route, "INVALID_IR_DEPARTURE");
    if (!BuildSteps(route, ir, lateral))
        return FinishPlanning(route, false);
    for (unsigned i = 0U; i < route->stepCount; ++i)
    {
        route->totalTravelMm += route->steps[i].distanceMm;
        if (route->steps[i].radiusMm != 0.0f)
            route->totalYawRad += route->steps[i].distanceMm / route->steps[i].radiusMm;
    }
    if (route->totalTravelMm > ROUTE_MAX_TRAVEL_MM)
        return Reject(route, "TRAVEL_LIMIT");
#if TASK2_ROUTE_EXHAUSTIVE_CHECKS
    return FinishPlanning(route, ValidatePath(route));
#else
    FinalizeReport(route);
    return FinishPlanning(route, true);
#endif
}

bool Task2Route_Create(const Task2RouteConfig *config, Task2Route *route)
{
    return Create(config, NULL, false, route);
}

bool Task2Route_CreateIR(const Task2RouteConfig *config,
    const Task2RouteIRDeparture *ir, Task2Route *route)
{
    if (!ir)
        return route ? Reject(route, "MISSING_IR_MOUNT") : false;
    return Create(config, ir, false, route);
}

bool Task2Route_CreateLateral(const Task2RouteConfig *config, Task2Route *route)
{
    if (!config || config->firstArrow != config->secondArrow)
        return route ? Reject(route, "INVALID_LATERAL_DEPARTURE") : false;
    return Create(config, NULL, true, route);
}
