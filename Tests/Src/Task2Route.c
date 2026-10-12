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

    /* Anchored eight-primitive family. Camera range references the lens,
     * not the ultrasonic transducer. O1 always rejoins at yaw=0, y=0. */
    /* Updated for the measured +110 mm camera offset; retain R275 sweeps. */
    .firstArcCameraRangeMm = 400.0f,
    .firstRejoinBeyondFaceMm = 535.0f,
    .firstRadiusMm = 275.0f,
    .secondArcCameraRangeMm = 485.0f,
    .secondDepartureRadiusMm = 275.0f,
    .secondBackExtraRadiusMm = 300.0f,
    .returnJoinHeadingDeg = 180.4817f,


};

const char *const task2RouteStepNames[TASK2_ROUTE_STEP_COUNT] = {
    "Approach O1", "O1 depart", "O1 bypass", "O1 centre",
    "Approach O2", "O2 depart", "O2 back", "Return park",
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

bool Task2Route_Obstacle1CameraVisible(const Task2RouteConfig *c, const Task2RoutePose *pose)
{
    float cosine = cosf(pose->yawRad), sine = sinf(pose->yawRad);
    float cameraX = pose->xMm + CameraOffset(c) * cosine;
    float cameraY = pose->yMm + CameraOffset(c) * sine;
    float dx = c->parkToObstacle1Mm - cameraX;
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

static bool ValidConfig(const Task2RouteConfig *config)
{
    const float positive[] = {
        config->obstacle2WidthMm, config->parkToObstacle1Mm,
        config->obstacleFaceGapMm, config->obstacleBufferMm,
        config->obstacle2EndSpaceMm, config->cruiseSpeedCps,
        config->approachSpeedCps, config->bodyLengthMm, config->bodyWidthMm,
        config->recognitionCameraRangeMm, config->cameraHorizontalFovDeg,
        config->recognitionTargetWidthMm, config->firstArcCameraRangeMm,
        config->firstRejoinBeyondFaceMm, config->firstRadiusMm,
        config->secondArcCameraRangeMm, config->secondDepartureRadiusMm,
        config->secondBackExtraRadiusMm, config->returnJoinHeadingDeg,
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
        config->cameraHorizontalFovDeg < 180.0f &&
        isfinite(config->cameraBehindFrontMm) && config->cameraBehindFrontMm >= 0.0f &&
        config->cameraBehindFrontMm <= config->bodyLengthMm &&
        config->firstRadiusMm >= ROUTE_MIN_RADIUS_MM &&
        config->secondDepartureRadiusMm >= ROUTE_MIN_RADIUS_MM &&
        config->approachSpeedCps <= config->cruiseSpeedCps &&
        config->returnJoinHeadingDeg >= 160.0f && config->returnJoinHeadingDeg <= 220.0f;
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

static bool BuildSteps(Task2Route *route)
{
    const Task2RouteConfig *c = &route->config;
    float firstFace = c->parkToObstacle1Mm;
    float secondFace = firstFace + 100.0f + c->obstacleFaceGapMm;
    float firstStart = firstFace - CameraOffset(c) - c->firstArcCameraRangeMm;
    float firstFinish = firstFace + c->firstRejoinBeyondFaceMm;
    float secondStart = secondFace - CameraOffset(c) - c->secondArcCameraRangeMm;

    /* Three symmetric O1 arcs: +a, -2a, +a, independently mirrored.
     * Four R*sin(a) of forward travel, zero net heading/lateral displacement. */
    float sine = (firstFinish - firstStart) / (4.0f * c->firstRadiusMm);
    if (sine <= 0.0f || sine >= 1.0f || firstStart <= TASK2_ROUTE_START_X_MM ||
        secondStart <= firstFinish)
        return Reject(route, "NO_CENTRED_APPROACH");
    float firstAngle = asinf(sine);
    float firstRadius = c->firstArrow * c->firstRadiusMm;
    SetStep(route, 0U, firstStart - TASK2_ROUTE_START_X_MM, 0.0f);
    SetStep(route, 1U, c->firstRadiusMm * firstAngle, firstRadius);
    SetStep(route, 2U, 2.0f * c->firstRadiusMm * firstAngle, -firstRadius);
    SetStep(route, 3U, c->firstRadiusMm * firstAngle, firstRadius);
    SetStep(route, 4U, secondStart - firstFinish, 0.0f);

    /* O2 departure circle touches the opposite-turn back circle externally.
     * Its centre is on y=0; changing width changes the back radius and tangent.
     * Work on the left branch, then mirror the entire O2/return section. */
    float departureRadius = c->secondDepartureRadiusMm;
    float backRadius = 0.5f * c->obstacle2WidthMm + c->secondBackExtraRadiusMm;
    float centreDx = sqrtf(backRadius * backRadius +
                          2.0f * departureRadius * backRadius);
    float backX = secondStart + centreDx;
    float joinAngle = atan2f(centreDx, departureRadius);
    float returnHeading = -c->returnJoinHeadingDeg * ROUTE_PI / 180.0f;
    float returnX = backX - backRadius * sinf(returnHeading);
    float returnY = backRadius * cosf(returnHeading);
    /* The back radius is negative: p = centre + Rback * normal(yaw). */
    float deltaX = TASK2_ROUTE_START_X_MM - returnX;
    float deltaY = -returnY;
    float denominator = 2.0f * (-deltaX * sinf(returnHeading) +
                               deltaY * cosf(returnHeading));
    if (denominator >= 0.0f || backRadius < ROUTE_MIN_RADIUS_MM)
        return Reject(route, "NO_RETURN_TANGENT");
    float returnRadius = (deltaX * deltaX + deltaY * deltaY) / denominator;
    float returnCentreX = returnX - returnRadius * sinf(returnHeading);
    float returnCentreY = returnY + returnRadius * cosf(returnHeading);
    float finishHeading = atan2f((TASK2_ROUTE_START_X_MM - returnCentreX) / returnRadius,
                                returnCentreY / returnRadius);
    while (finishHeading > returnHeading)
        finishHeading -= 2.0f * ROUTE_PI;
    if (returnRadius > -ROUTE_MIN_RADIUS_MM ||
        returnHeading - finishHeading <= 0.0f ||
        returnHeading - finishHeading >= ROUTE_PI)
        return Reject(route, "NO_RETURN_TANGENT");
    SetStep(route, 5U, departureRadius * joinAngle, c->secondArrow * departureRadius);
    SetStep(route, 6U, backRadius * (joinAngle - returnHeading), -c->secondArrow * backRadius);
    SetStep(route, 7U, -returnRadius * (returnHeading - finishHeading),
            c->secondArrow * returnRadius);

    for (unsigned i = 0U; i < TASK2_ROUTE_STEP_COUNT; ++i)
        if (!isfinite(route->steps[i].distanceMm) ||
            route->steps[i].distanceMm <= 0.0f ||
            !isfinite(route->steps[i].radiusMm))
            return Reject(route, "INVALID_GEOMETRY");
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
            report->minimumParkWallMarginMm = fminf(report->minimumParkWallMarginMm,
                fminf(cx[j] + 500.0f, 300.0f - fabsf(cy[j])));
    }
    /* Check body edges through the park mouth, including wall-tip crossings. */
    for (unsigned j = 0U; j < 4U; ++j)
    {
        unsigned next = (j + 1U) % 4U;
        if ((cx[j] < 0.0f) != (cx[next] < 0.0f))
        {
            float fraction = -cx[j] / (cx[next] - cx[j]);
            float y = cy[j] + fraction * (cy[next] - cy[j]);
            report->minimumParkWallMarginMm = fminf(report->minimumParkWallMarginMm,
                                                    300.0f - fabsf(y));
        }
    }
    float firstX = c->parkToObstacle1Mm + 50.0f;
    float secondX = c->parkToObstacle1Mm + 150.0f + c->obstacleFaceGapMm;
    report->minimumObstacleGapMm = fminf(report->minimumObstacleGapMm,
        fminf(ObstacleGap(c, p, cx, cy, firstX, 50.0f),
              includeObstacle2 ? ObstacleGap(c, p, cx, cy, secondX, 0.5f * c->obstacle2WidthMm) : INFINITY));
    return report->minimumObstacleGapMm >= c->obstacleBufferMm &&
        report->minimumSideMarginMm > 0.0f && report->minimumParkWallMarginMm > 0.0f;
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
    Task2RoutePose pose = {TASK2_ROUTE_START_X_MM, 0.0f, 0.0f};
    bool seen[2] = {false, false};
    bool crossed[2][2] = {{false, false}, {false, false}};
    float progress = 0.0f;
    float faces[] = {route->config.parkToObstacle1Mm,
        route->config.parkToObstacle1Mm + 100.0f + route->config.obstacleFaceGapMm};
    for (unsigned index = 0U; index < TASK2_ROUTE_STEP_COUNT; ++index)
    {
        const Task2RouteStep *primitive = &route->steps[index];
        for (float distance = 0.0f; distance < primitive->distanceMm;)
        {
            float ds = fminf(ROUTE_SAMPLE_SPACING_MM, primitive->distanceMm - distance);
            float previousX = pose.xMm;
            if (primitive->radiusMm == 0.0f)
            {
                pose.xMm += ds * cosf(pose.yawRad);
                pose.yMm += ds * sinf(pose.yawRad);
            }
            else
            {
                float nextYaw = pose.yawRad + ds / primitive->radiusMm;
                pose.xMm += primitive->radiusMm * (sinf(nextYaw) - sinf(pose.yawRad));
                pose.yMm += primitive->radiusMm * (cosf(pose.yawRad) - cosf(nextYaw));
                pose.yawRad = nextYaw;
            }
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
            if (index == 0U || index == 4U)
            {
                unsigned obstacle = index == 0U ? 0U : 1U;
                float range = faces[obstacle] - pose.xMm - CameraOffset(&route->config);
                if (!seen[obstacle] && range > 0.0f && range <= route->config.recognitionCameraRangeMm &&
                    fabsf(pose.yMm) < 0.1f && fabsf(pose.yawRad) < 0.0001f &&
                    0.5f * route->config.recognitionTargetWidthMm <
                        range * tanf(route->config.cameraHorizontalFovDeg * ROUTE_PI / 360.0f))
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
    for (unsigned i = 0U; i < TASK2_ROUTE_STEP_COUNT; ++i)
    {
        route->totalTravelMm += route->steps[i].distanceMm;
        if (route->steps[i].radiusMm != 0.0f)
            route->totalYawRad += route->steps[i].distanceMm / route->steps[i].radiusMm;
    }
    if (route->totalTravelMm > ROUTE_MAX_TRAVEL_MM)
        return Reject(route, "TRAVEL_LIMIT");
    return ValidatePath(route);
}
