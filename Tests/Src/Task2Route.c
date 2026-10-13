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
    .parkBackWallBufferMm = 15.0f,

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
    "Reserve", "Reserve", "Reserve", "Reserve",
};

static bool Reject(Task2Route *route, const char *reason)
{
    route->report.valid = false;
    route->report.reason = reason;
    return false;
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
        config->obstacle2EndSpaceMm, config->boundaryBufferMm, config->parkBackWallBufferMm, config->cruiseSpeedCps,
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
    return cx - ex > -500.0f && cx + ex < 0.0f && fabsf(cy) + ey < 300.0f;
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
        if (!isfinite(steps[i].distanceMm) || steps[i].distanceMm <= 0.0f || !isfinite(steps[i].radiusMm))
            return false;
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
        if (!isfinite(steps[i].distanceMm) || steps[i].distanceMm <= 0.0f ||
            !isfinite(steps[i].radiusMm))
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
        float sx = TASK2_ROUTE_START_X_MM + parkStraight + 2.0f * rejoinRadius * sinf(angle);
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
            float gx = TASK2_ROUTE_START_X_MM - rx, gy = -ry;
            float returnRadius = 0.0f, returnLength = hypotf(gx, gy);
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
                float finish = atan2f((TASK2_ROUTE_START_X_MM - rcx) / returnRadius,
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
                float sx = TASK2_ROUTE_START_X_MM + parkStraight + 2.0f * radius * sinf(angle);
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
    exception.obstacleBufferMm = 1.0f;
#if TASK2_ROUTE_EXHAUSTIVE_CHECKS
    return Sweep(&exception, p, steps, count, 1.0f, false);
#else
    Task2RouteReport accepted = EmptyReport();
    if (!BoundedSweep(&exception, p, steps, count, false, &accepted))
        return false;
    accepted.obstacleBufferExceptionUsed = true;
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

/* From the measured parallel stop, arc directly around O2's rear to the
 * opposite return lane. The radius follows from the lateral displacement;
 * only four parking joins are considered, with no back-radius grid. */
static bool BuildParallelReturn(const Task2RouteConfig *c, Task2RoutePose p,
    float width, Task2RouteStep out[6], unsigned *count)
{
    Task2RouteConfig bounds = ObservedBounds(c, width);
    float side = c->secondArrow, h = side * p.yawRad;
    if (bounds.obstacle2EndSpaceMm <= 0.0f || fabsf(h) > 15.0f * ROUTE_PI / 180.0f)
        return false;
    float returnY = 0.5f * bounds.obstacle2WidthMm + 0.5f * c->bodyWidthMm + c->obstacleBufferMm + 20.0f;
    float rejoinRadius = fmaxf(ROUTE_MIN_RADIUS_MM, 0.5f * returnY + 50.0f);
    float angle = acosf(1.0f - returnY / (2.0f * rejoinRadius));
    float centre = c->parkToObstacle1Mm + 150.0f + c->obstacleFaceGapMm;
    /* Prefer a direct back arc. Only if that collides, try one forward
     * positioning straight derived from rear-face/body clearance. */
    for (unsigned positioning = 0U; positioning < 2U; ++positioning)
    {
        unsigned arcIndex = positioning;
        Task2RoutePose arcStart = p;
        if (positioning)
        {
            float front = c->bodyCenterAheadOfRearAxleMm + 0.5f * c->bodyLengthMm + c->frontProtrusionMm;
            float clearX = centre + 50.0f + front + 21.0f;
            out[0] = (Task2RouteStep){.distanceMm = (clearX - p.xMm) / cosf(h), .radiusMm = 0.0f, .speedCps = c->cruiseSpeedCps};
            if (out[0].distanceMm <= 0.0f)
                continue;
            arcStart = Advance(p, out[0].distanceMm, 0.0f);
        }
        float radius = (side * arcStart.yMm + returnY) / (1.0f + cosf(h));
        if (!isfinite(radius) || radius < ROUTE_MIN_RADIUS_MM)
            continue;
        out[arcIndex] = (Task2RouteStep){.distanceMm = radius * (ROUTE_PI + h), .radiusMm = -side * radius, .speedCps = c->cruiseSpeedCps};
        unsigned outboundCount = arcIndex + 1U;
        if (!CheckIROutbound(&bounds, p, out, outboundCount))
            continue;
#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
        Task2RouteReport outboundClearance = planningWork.acceptedClearance;
#endif
        Task2RoutePose returnStart = Advance(arcStart, out[arcIndex].distanceMm, out[arcIndex].radiusMm);
        for (unsigned park = 0U; park < 4U; ++park)
        {
            if (planningWork.exhausted)
                return false;
            float parkStraight = 100.0f + 100.0f * park;
            float sx = TASK2_ROUTE_START_X_MM + parkStraight + 2.0f * rejoinRadius * sinf(angle);
            out[outboundCount] = (Task2RouteStep){.distanceMm = returnStart.xMm - sx, .radiusMm = 0.0f, .speedCps = c->cruiseSpeedCps};
            out[outboundCount + 1U] = (Task2RouteStep){.distanceMm = rejoinRadius * angle, .radiusMm = -side * rejoinRadius, .speedCps = c->cruiseSpeedCps};
            out[outboundCount + 2U] = (Task2RouteStep){.distanceMm = rejoinRadius * angle, .radiusMm = side * rejoinRadius, .speedCps = c->cruiseSpeedCps};
            out[outboundCount + 3U] = (Task2RouteStep){.distanceMm = parkStraight, .radiusMm = 0.0f, .speedCps = c->approachSpeedCps};
            unsigned n = outboundCount + 4U;
            if (CandidateSweepFrom(&bounds, p, out, n, outboundCount, p.xMm >= centre))
            {
#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
                MergeMargins(&planningWork.acceptedClearance, &outboundClearance);
#endif
                *count = n;
                return true;
            }
        }
    }
    return false;
}

#if !TASK2_ROUTE_EXHAUSTIVE_CHECKS
static float IRHit(float h, float beta, float cxDistance, float cy, float intercept)
{
    float beam = h + beta;
    return cy + cxDistance * tanf(beam) + intercept / cosf(beam);
}

static float IRBoundaryTravel(float boundary, float gate, float endHit, float beta,
    float dx, float cy, float intercept, float radius, float entry, float arcLength)
{
    if (IRHit(gate, beta, dx, cy, intercept) >= boundary)
        return radius * (gate - entry);
    if (endHit < boundary)
        return arcLength + boundary - endHit;
    float lo = gate, hi = 0.5f * ROUTE_PI;
    for (unsigned i = 0U; i < 16U; ++i)
    {
        float mid = 0.5f * (lo + hi);
        if (IRHit(mid, beta, dx, cy, intercept) < boundary)
            lo = mid;
        else
            hi = mid;
    }
    return radius * (hi - entry);
}

/* Fixed-cost face intersections. The derivative is
 * (dx+intercept*sin(beam))/cos(beam)^2, so dx>|intercept| proves monotonicity.
 * Unsupported mount geometry is rejected rather than sampled exhaustively. */
static bool IRWindow(const Task2RouteConfig *c, Task2RoutePose p,
    const Task2RouteIRDeparture *ir, float radius, float arcLength,
    float *acquireTravel, float *acquisitionEnd, float *edgeTravel)
{
    float side = c->secondArrow, entry = side * p.yawRad;
    float gate = fmaxf(entry, ir->acquisitionMinYawDeg * ROUTE_PI / 180.0f);
    float beta = side * ir->mount.yawDeg * ROUTE_PI / 180.0f;
    float dx = c->parkToObstacle1Mm + 100.0f + c->obstacleFaceGapMm -
        (p.xMm - radius * sinf(entry));
    float cy = side * p.yMm + radius * cosf(entry);
    float n = side * ir->mount.leftMm - radius;
    float intercept = n * cosf(beta) - ir->mount.aheadMm * sinf(beta);
    if (gate > 0.5f * ROUTE_PI || cosf(gate + beta) <= 0.05f ||
        cosf(0.5f * ROUTE_PI + beta) <= 0.05f || dx <= fabsf(intercept) ||
        dx <= hypotf(ir->mount.aheadMm, n))
        return false;
    float inner = fmaxf(0.0f, 0.5f * c->obstacle2WidthMm - ir->edgeUncertaintyMm);
    float endHit = IRHit(0.5f * ROUTE_PI, beta, dx, cy, intercept);
    if (IRHit(gate, beta, dx, cy, intercept) > inner)
        return false;
    *acquireTravel = IRBoundaryTravel(-inner, gate, endHit, beta, dx, cy, intercept, radius, entry, arcLength);
    *acquisitionEnd = IRBoundaryTravel(inner, gate, endHit, beta, dx, cy, intercept, radius, entry, arcLength);
    *edgeTravel = IRBoundaryTravel(0.5f * c->obstacle2WidthMm, gate, endHit,
        beta, dx, cy, intercept, radius, entry, arcLength);
    return true;
}

#endif

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
    if (!isfinite(hit))
        return false;
    /* Nominal preview for offline geometry, never an execution distance cap. */
    step[1] = (Task2RouteStep){.distanceMm = fmaxf(0.0f, 0.5f * c->obstacle2WidthMm - hit) + ir->guardTravelMm,
        .sensorTerminated = true, .radiusMm = 0.0f, .speedCps = c->obstacle2DepartureSpeedCps};
    float edgeTravel = step[0].distanceMm + fmaxf(0.0f, 0.5f * c->obstacle2WidthMm - hit);
#if TASK2_ROUTE_EXHAUSTIVE_CHECKS
    bool acquired = false;
    float acquireTravel = INFINITY;
    float innerHalf = fmaxf(0.0f, 0.5f * c->obstacle2WidthMm - ir->edgeUncertaintyMm);
    float acquisitionEnd = step[0].distanceMm + fmaxf(0.0f, innerHalf - hit);
    bool acquisitionEnded = false;
    for (float d = 0.0f; d < step[0].distanceMm; d += 1.0f)
    {
        Task2RoutePose pose = Advance(p, d, radius);
        if (side * pose.yawRad * 180.0f / ROUTE_PI < ir->acquisitionMinYawDeg)
            continue;
        float sx = pose.xMm + ir->mount.aheadMm * cosf(pose.yawRad) - ir->mount.leftMm * sinf(pose.yawRad);
        float sy = pose.yMm + ir->mount.aheadMm * sinf(pose.yawRad) + ir->mount.leftMm * cosf(pose.yawRad);
        float a = pose.yawRad + ir->mount.yawDeg * ROUTE_PI / 180.0f;
        if (face <= sx || cosf(a) <= 0.05f)
            continue;
        float fy = sy + (face - sx) * tanf(a);
        if (!acquired && fabsf(fy) <= innerHalf)
        {
            acquired = true;
            acquireTravel = d;
        }
        if (acquired && !acquisitionEnded && side * fy > innerHalf)
        {
            acquisitionEnd = d;
            acquisitionEnded = true;
        }
        if (acquired && side * fy > 0.5f * c->obstacle2WidthMm)
        {
            edgeTravel = d;
            break;
        }
    }
    if (!acquired)
        acquireTravel = step[0].distanceMm + fmaxf(0.0f, -innerHalf - hit);
#else
    float acquireTravel, acquisitionEnd;
    if (!IRWindow(c, p, ir, turnRadius, step[0].distanceMm, &acquireTravel, &acquisitionEnd, &edgeTravel))
        return false;
#endif
    if (acquisitionEnd - acquireTravel < ir->acquisitionTravelMm)
        return false;
    /* Screen return room at the nominal reference edge. Preview travel is
     * not an execution limit or a prediction of the actual stopped pose. The actual
     * brake stop must pass suffix preflight before any return motion starts. */
    Task2RoutePose predictedStop = edgeTravel < step[0].distanceMm
        ? Advance(p, edgeTravel, radius) : Advance(front, edgeTravel - step[0].distanceMm, 0.0f);
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

static bool BuildSteps(Task2Route *route, const Task2RouteIRDeparture *ir)
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
    float lateral = -dx * sinf(start.yawRad) - start.yMm * cosf(start.yawRad);
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
        for (float range = c->secondArcCameraRangeMm;
             range <= c->recognitionCameraRangeMm; range += 25.0f)
        {
            if (planningWork.exhausted)
                return Reject(route, "PLANNING_BUDGET");
            if (fabsf(lateral) >= range)
                continue;
            float travel = forward - sqrtf(range * range - lateral * lateral);
            if (travel < 0.0f || travel > c->obstacle2ApproachMaxMm)
                continue;
            Task2RoutePose departure = Advance(start, travel, 0.0f);
            if (!Task2Route_ObstacleCameraVisible(c, &departure, 1U))
                continue;
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
         !(route->obstacle2IRDeparture && completedDepartureStep == TASK2_ROUTE_O2_IR_STRAIGHT_STEP)) ||
        !isfinite(pose->xMm) || !isfinite(pose->yMm) || !isfinite(pose->yawRad))
        return route ? FinishPlanning(route, Reject(route, "NO_OBSERVED_EDGE_RETURN")) : false;
    if (route->obstacle2IRDeparture)
    {
        if (!BuildIRQuarterTurn(&route->config, *pose, width, tail))
            return FinishPlanning(route, Reject(route, "NO_IR_QUARTER_TURN"));
        count = 1U;
        route->obstacle2ParallelReturnPending = true;
    }
    else if (!BuildTail(&route->config, *pose, width, tail, &count))
        return FinishPlanning(route, Reject(route, "NO_OBSERVED_EDGE_RETURN"));
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
    if (!route || !pose || !route->obstacle2ParallelReturnPending ||
        completedQuarterStep + 1U != route->stepCount || !isfinite(width) || width < 300.0f ||
        !isfinite(pose->xMm) || !isfinite(pose->yMm) || !isfinite(pose->yawRad) ||
        !BuildParallelReturn(&route->config, *pose, width, tail, &count))
        return route ? FinishPlanning(route, Reject(route, "NO_PARALLEL_RETURN")) : false;
    route->obstacle2ParallelReturnPending = false;
    route->obstacle2DirectRearArc = tail[0].radiusMm != 0.0f;
    return InstallSuffix(route, pose, width, tail, count, completedQuarterStep,
        route->obstacle2DirectRearArc ? 1U : 2U, "OK_PARALLEL_RETURN");
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
    report->minimumParkWallMarginMm = fminf(report->minimumParkBackMarginMm, report->minimumParkSideMarginMm);
    float firstX = c->parkToObstacle1Mm + 50.0f;
    float secondX = c->parkToObstacle1Mm + 150.0f + c->obstacleFaceGapMm;
    report->minimumObstacleGapMm = fminf(report->minimumObstacleGapMm,
        fminf(ObstacleGap(c, p, cx, cy, firstX, 50.0f),
              includeObstacle2 ? ObstacleGap(c, p, cx, cy, secondX, 0.5f * c->obstacle2WidthMm) : INFINITY));
    return report->minimumObstacleGapMm >= c->obstacleBufferMm &&
        report->minimumSideMarginMm >= c->boundaryBufferMm &&
        report->minimumParkSideMarginMm >= c->boundaryBufferMm &&
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
        report->reason = "OK_IR_PREFIX";
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
    route->report.reason = route->obstacle2ReturnPending ? "OK_IR_PREFIX" : "OK";
}
#endif

static bool Create(const Task2RouteConfig *config, const Task2RouteIRDeparture *ir, Task2Route *route)
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
    if (!BuildSteps(route, ir))
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
    return Create(config, NULL, route);
}

bool Task2Route_CreateIR(const Task2RouteConfig *config,
    const Task2RouteIRDeparture *ir, Task2Route *route)
{
    if (!ir)
        return route ? Reject(route, "MISSING_IR_MOUNT") : false;
    return Create(config, ir, route);
}
