#include "Task2Route.h"

#include <math.h>
#include <stddef.h>

#define ROUTE_PI 3.14159265358979323846f
#define ROUTE_MIN_RADIUS_MM 275.0f
#define ROUTE_SAMPLE_SPACING_MM 1.0f
#define ROUTE_MAX_TRAVEL_MM 30000.0f

/* Important experiment parameters. Positive y is LEFT of initial +x.
 * These are nominal simulator inputs, not measurements or uncertainty bounds.
 * Change the two arrows and width first; geometry knobs are below them. */
Task2RouteConfig task2RouteConfig = {
    /* Arena and selected arrows. No boundary is imposed behind obstacle 2. */
    .firstArrow = TASK2_ARROW_LEFT,
    .secondArrow = TASK2_ARROW_RIGHT,
    .obstacle2WidthMm = 500.0f,
    .parkToObstacle1Mm = 1050.0f,
    .obstacleFaceGapMm = 1050.0f,
    .obstacleBufferMm = 100.0f,
    .obstacle2EndSpaceMm = 500.0f,
    .boundaryBufferMm = 35.0f,
    .parkBackWallBufferMm = 15.0f,

    /* O1 accelerates after simulated recognition. O2 keeps approach speed.
     * Recognition is instantaneous; camera/RPi integration is deferred. */
    .cruiseSpeedCps = 5000.0f,
    .approachSpeedCps = 2000.0f,
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
    .obstacle2DepartureSpeedCps = 1000.0f,
    .obstacle2DepartureWidthReserveMm = 100.0f,
    .secondDepartureRadiusMm = 275.0f,
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
        config->secondBackExtraRadiusMm,
        config->obstacle1ExitYawDeg, config->obstacle2ApproachMaxMm,
        config->obstacle2DepartureSpeedCps, config->obstacle2DepartureWidthReserveMm,
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
        config->approachSpeedCps <= config->cruiseSpeedCps;
}

static void SetStep(Task2Route *route, unsigned index, float length, float radius)
{
    route->steps[index] = (Task2RouteStep){
        .distanceMm = length,
        .radiusMm = radius,
        /* O1 launches with a cruise ceiling and a soft approach request;
         * recognition raises that request without changing the primitive.
         * O2 retains the original constant approach-speed policy. */
        .speedCps = radius == 0.0f && index != 0U ? route->config.approachSpeedCps
                                                : route->config.cruiseSpeedCps,
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

static bool Sweep(const Task2RouteConfig *c, Task2RoutePose start,
    const Task2RouteStep *steps, unsigned count, float spacing, bool requirePark)
{
    Task2RouteReport report = EmptyReport();
    if (!Task2Route_CheckPose(c, &start, &report))
        return false;
    for (unsigned i = 0U; i < count; ++i)
    {
        if (!isfinite(steps[i].distanceMm) || steps[i].distanceMm <= 0.0f ||
            !isfinite(steps[i].radiusMm))
            return false;
        for (float d = spacing; d < steps[i].distanceMm; d += spacing)
        {
            Task2RoutePose p = Advance(start, d, steps[i].radiusMm);
            if (!Task2Route_CheckPose(c, &p, &report))
                return false;
        }
        start = Advance(start, steps[i].distanceMm, steps[i].radiusMm);
        if (!Task2Route_CheckPose(c, &start, &report))
            return false;
    }
    return !requirePark || Parked(c, &start);
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
        float rd = c->secondDepartureRadiusMm + 25.0f * d;
        out[0] = (Task2RouteStep){rd * (0.5f * ROUTE_PI - h), side * rd, c->obstacle2DepartureSpeedCps};
        Task2RoutePose front = Advance(p, out[0].distanceMm, out[0].radiusMm);
        float edgeY = 0.5f * width - 25.0f + 10.0f * offset;
        out[1] = (Task2RouteStep){edgeY - side * front.yMm, 0.0f, c->obstacle2DepartureSpeedCps};
        float radius = ROUTE_MIN_RADIUS_MM;
        out[2] = (Task2RouteStep){ROUTE_PI * radius, -side * radius, c->cruiseSpeedCps};
        out[3] = (Task2RouteStep){2.0f * edgeY, 0.0f, c->cruiseSpeedCps};
        out[4] = (Task2RouteStep){0.5f * ROUTE_PI * radius, -side * radius, c->cruiseSpeedCps};
        Task2RoutePose returnStart = front;
        for (unsigned i = 1U; i < 5U; ++i)
            returnStart = Advance(returnStart, out[i].distanceMm, out[i].radiusMm);
        float lateral = -side * returnStart.yMm;
        float rejoinRadius = fmaxf(ROUTE_MIN_RADIUS_MM, 0.5f * lateral + 50.0f);
        float angle = acosf(1.0f - lateral / (2.0f * rejoinRadius));
        float parkStraight = 100.0f + 100.0f * park;
        float sx = TASK2_ROUTE_START_X_MM + parkStraight + 2.0f * rejoinRadius * sinf(angle);
        out[5] = (Task2RouteStep){returnStart.xMm - sx, 0.0f, c->cruiseSpeedCps};
        out[6] = (Task2RouteStep){rejoinRadius * angle, -side * rejoinRadius, c->cruiseSpeedCps};
        out[7] = (Task2RouteStep){rejoinRadius * angle, side * rejoinRadius, c->cruiseSpeedCps};
        out[8] = (Task2RouteStep){parkStraight, 0.0f, c->approachSpeedCps};
        if (Sweep(bounds, p, out, 9U, 5.0f, true) && Sweep(bounds, p, out, 9U, 1.0f, true))
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
            out[0] = (Task2RouteStep){rd * (join - h), side * rd, c->obstacle2DepartureSpeedCps};
            out[1] = (Task2RouteStep){rb * (join - rh), -side * rb, c->cruiseSpeedCps};
            out[2] = (Task2RouteStep){returnLength, side * returnRadius, c->cruiseSpeedCps};
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
                out[2] = (Task2RouteStep){bx - sx, 0.0f, c->cruiseSpeedCps};
                out[3] = (Task2RouteStep){radius * angle, -side * radius, c->cruiseSpeedCps};
                out[4] = (Task2RouteStep){radius * angle, side * radius, c->cruiseSpeedCps};
                out[5] = (Task2RouteStep){parkStraight, 0.0f, c->approachSpeedCps};
                n = 6U;
            }
            if (Sweep(&bounds, p, out, n, 5.0f, true) &&
                Sweep(&bounds, p, out, n, ROUTE_SAMPLE_SPACING_MM, true))
            {
                *count = n;
                return true;
            }
        }
    }
    return BuildTraverseTail(c, &bounds, p, width, out, count);
}

static bool BuildSteps(Task2Route *route)
{
    const Task2RouteConfig *c = &route->config;
    float firstStart = c->parkToObstacle1Mm - CameraOffset(c) - c->firstArcCameraRangeMm;
    float firstFinish = c->parkToObstacle1Mm + c->firstRejoinBeyondFaceMm;
    float sine = (firstFinish - firstStart) / (4.0f * c->firstRadiusMm);
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
    for (float range = c->secondArcCameraRangeMm;
         range <= c->recognitionCameraRangeMm; range += 25.0f)
    {
        if (fabsf(lateral) >= range)
            continue;
        float travel = forward - sqrtf(range * range - lateral * lateral);
        if (travel < 0.0f || travel > c->obstacle2ApproachMaxMm)
            continue;
        Task2RoutePose departure = Advance(start, travel, 0.0f);
        if (!Task2Route_ObstacleCameraVisible(c, &departure, 1U))
            continue;
        SetStep(route, 3U, travel, 0.0f);
        unsigned count = 0U;
        if (Sweep(c, (Task2RoutePose){TASK2_ROUTE_START_X_MM, 0.0f, 0.0f}, route->steps, 4U, 1.0f, false) &&
            BuildTail(c, departure, c->obstacle2WidthMm + c->obstacle2DepartureWidthReserveMm,
                &route->steps[4], &count))
        {
            route->stepCount = 4U + count;
            return true;
        }
    }
    return Reject(route, "NO_O2_APPROACH_OR_RETURN");
}

bool Task2Route_ReplanObstacle2(Task2Route *route, const Task2RoutePose *pose, float width)
{
    Task2RouteStep tail[9];
    unsigned count = 0U;
    if (!route || !pose || !isfinite(width) || width < 300.0f ||
        !isfinite(pose->xMm) || !isfinite(pose->yMm) || !isfinite(pose->yawRad) ||
        !BuildTail(&route->config, *pose, width, tail, &count))
        return route ? Reject(route, "NO_OBSERVED_EDGE_RETURN") : false;
    for (unsigned i = 0U; i < count; ++i)
        route->steps[5U + i] = tail[i];
    route->stepCount = 5U + count;
    route->totalTravelMm = route->totalYawRad = 0.0f;
    for (unsigned i = 0U; i < route->stepCount; ++i)
    {
        route->totalTravelMm += route->steps[i].distanceMm;
        if (route->steps[i].radiusMm != 0.0f)
            route->totalYawRad += route->steps[i].distanceMm / route->steps[i].radiusMm;
    }
    Task2RoutePose finish = *pose;
    Task2RouteConfig bounds = route->config;
    float sideLimit = 0.5f * bounds.obstacle2WidthMm + bounds.obstacle2EndSpaceMm;
    bounds.obstacle2WidthMm = fmaxf(bounds.obstacle2WidthMm, width);
    bounds.obstacle2EndSpaceMm = sideLimit - 0.5f * bounds.obstacle2WidthMm;
    for (unsigned i = 0U; i < count; ++i)
    {
        for (float d = 0.0f; d < tail[i].distanceMm; d += 1.0f)
        {
            Task2RoutePose sample = Advance(finish, d, tail[i].radiusMm);
            (void)Task2Route_CheckPose(&bounds, &sample, &route->report);
        }
        finish = Advance(finish, tail[i].distanceMm, tail[i].radiusMm);
    }
    route->report.finishPose = finish;
    route->report.valid = true;
    route->report.reason = "OK_OBSERVED_EDGE";
    return true;
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

static bool ValidatePath(Task2Route *route)
{
    Task2RouteReport *report = &route->report;
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
            if (!Task2Route_CheckPose(&route->config, &pose, report))
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

bool Task2Route_Create(const Task2RouteConfig *config, Task2Route *route)
{
    if (route == NULL)
        return false;
    *route = (Task2Route){0};
    if (config != NULL)
        route->config = *config;
    if (config == NULL || !ValidConfig(config))
        return Reject(route, "INVALID_PARAMETERS");
    if (!BuildSteps(route))
        return false;
    for (unsigned i = 0U; i < route->stepCount; ++i)
    {
        route->totalTravelMm += route->steps[i].distanceMm;
        if (route->steps[i].radiusMm != 0.0f)
            route->totalYawRad += route->steps[i].distanceMm / route->steps[i].radiusMm;
    }
    if (route->totalTravelMm > ROUTE_MAX_TRAVEL_MM)
        return Reject(route, "TRAVEL_LIMIT");
    return ValidatePath(route);
}
