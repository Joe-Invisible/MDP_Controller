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

    /* Speeds and assumed body/camera geometry. Recognition is instantaneous.
     * Both whole approaches use the reduced speed; live speed zones are deferred. */
    .cruiseSpeedCps = 5000.0f,
    .approachSpeedCps = 2000.0f,
    .bodyLengthMm = 227.0f,
    .bodyWidthMm = 200.0f,
    .bodyCenterAheadOfRearAxleMm = 76.5f, /* Front +190, tail -37 mm. */
    .frontProtrusionMm = 20.0f,          /* Ultrasonic face at +210 mm. */
    .cameraBehindFrontMm = 80.0f,
    .recognitionCameraRangeMm = 500.0f,
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

    /* Full blend lengths. Camera entries/rejoin must preserve a centred view.
     * The unchanged 500 mm request still applies to the broad return junction.
     * Larger values may be rejected by the geometry/camera preflight. */
    .cameraJunctionBlendMm = 10.0f,
    .firstBypassBlendMm = 100.0f,
    .secondBypassBlendMm = 100.0f,
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
        config->cameraJunctionBlendMm, config->firstBypassBlendMm,
        config->secondBypassBlendMm,
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
        config->recognitionCameraRangeMm <= 500.0f && config->cameraHorizontalFovDeg < 180.0f &&
        isfinite(config->cameraBehindFrontMm) && config->cameraBehindFrontMm >= 0.0f &&
        config->cameraBehindFrontMm <= config->bodyLengthMm &&
        config->firstRadiusMm >= ROUTE_MIN_RADIUS_MM &&
        config->secondDepartureRadiusMm >= ROUTE_MIN_RADIUS_MM &&
        config->approachSpeedCps <= config->cruiseSpeedCps &&
        config->returnJoinHeadingDeg >= 160.0f && config->returnJoinHeadingDeg <= 220.0f;
}

static void SetStep(Task2Route *route, unsigned index, float length, float radius)
{
    route->steps[index] = (MotionSequenceSegment){
        .signedDistanceMm = length,
        .curvaturePerMm = radius == 0.0f ? 0.0f : 1.0f / radius,
        .speedCps = radius == 0.0f ? route->config.approachSpeedCps
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
        if (!isfinite(route->steps[i].signedDistanceMm) ||
            route->steps[i].signedDistanceMm <= 0.0f ||
            !isfinite(route->steps[i].curvaturePerMm))
            return Reject(route, "INVALID_GEOMETRY");
    route->blendLimitMm[0] = c->cameraJunctionBlendMm;
    route->blendLimitMm[1] = c->firstBypassBlendMm;
    route->blendLimitMm[2] = c->firstBypassBlendMm;
    route->blendLimitMm[3] = c->cameraJunctionBlendMm;
    route->blendLimitMm[4] = c->cameraJunctionBlendMm;
    route->blendLimitMm[5] = c->secondBypassBlendMm;
    return true;
}

static bool Prepare(const Task2Route *route, const MotionSequenceConfig *config,
                    float mmPerCount, bool fused, MotionSequencePlan *plan)
{
    *plan = (MotionSequencePlan){0};
    plan->count = TASK2_ROUTE_STEP_COUNT;
    for (unsigned i = 0U; i < plan->count; ++i)
    {
        plan->segments[i] = route->steps[i];
        plan->segments[i].stopAfter = !fused;
        plan->junctionBlendLimitMm[i] = route->blendLimitMm[i];
    }
    return MotionSequencePlan_Prepare(plan, config, mmPerCount,
        3000.0f, 3000.0f, 2000.0f, 2000.0f, 0.0f, 480.0f);
}

static bool Integrate(const MotionSequencePlan *plan, float finishProgress,
                      Task2RoutePose *pose)
{
    *pose = (Task2RoutePose){TASK2_ROUTE_START_X_MM, 0.0f, 0.0f};
    MotionSequenceRun run = {plan, 0U, TASK2_ROUTE_STEP_COUNT - 1U};
    for (float d = 0.0f; d < finishProgress;)
    {
        float step = fminf(ROUTE_SAMPLE_SPACING_MM, finishProgress - d);
        MotionPathSample middle, end;
        if (!MotionSequencePlan_Evaluate(&run, d + 0.5f * step, &middle) ||
            !MotionSequencePlan_Evaluate(&run, d + step, &end))
            return false;
        pose->xMm += step * cosf(middle.desiredYawRad);
        pose->yMm += step * sinf(middle.desiredYawRad);
        pose->yawRad = end.desiredYawRad;
        d += step;
    }
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

static bool ValidatePath(Task2Route *route, const MotionSequencePlan *plan, bool fused)
{
    Task2RouteReport *report = &route->report;
    report->minimumObstacleGapMm = INFINITY;
    report->minimumSideMarginMm = INFINITY;
    report->minimumParkWallMarginMm = INFINITY;
    Task2RoutePose pose = {TASK2_ROUTE_START_X_MM, 0.0f, 0.0f};
    bool seen[2] = {false, false};
    bool crossed[2][2] = {{false, false}, {false, false}};
    float progress = 0.0f;
    float firstFace = route->config.parkToObstacle1Mm;
    float secondFace = firstFace + 100.0f + route->config.obstacleFaceGapMm;

    for (unsigned first = 0U; first < plan->count;)
    {
        MotionSequenceRun run = {plan, first, MotionSequencePlan_RunEnd(plan, first)};
        float length = 0.0f;
        for (unsigned i = first; i <= run.last; ++i)
            length += plan->segments[i].signedDistanceMm;
        float initialYaw = pose.yawRad;
        for (float d = 0.0f; d < length;)
        {
            float step = fminf(ROUTE_SAMPLE_SPACING_MM, length - d);
            MotionPathSample middle, end;
            if (!MotionSequencePlan_Evaluate(&run, d + 0.5f * step, &middle) ||
                !MotionSequencePlan_Evaluate(&run, d + step, &end))
                return Reject(route, "PROFILE_EVALUATION");
            float previousX = pose.xMm;
            float heading = initialYaw + middle.desiredYawRad;
            pose.xMm += step * cosf(heading);
            pose.yMm += step * sinf(heading);
            pose.yawRad = initialYaw + end.desiredYawRad;
            if (!Task2Route_CheckPose(&route->config, &pose, report))
                return Reject(route, "CLEARANCE_OR_BOUNDARY");

            for (unsigned obstacle = 0U; obstacle < 2U; ++obstacle)
            {
                float ox = (obstacle == 0U ? firstFace : secondFace) + 50.0f;
                if ((previousX < ox && pose.xMm >= ox) ||
                    (previousX > ox && pose.xMm <= ox))
                {
                    unsigned returning = cosf(pose.yawRad) < 0.0f;
                    int side = obstacle == 0U ? route->config.firstArrow : route->config.secondArrow;
                    if (returning)
                        side = -route->config.secondArrow;
                    if (crossed[obstacle][returning] || pose.yMm * side <= 0.0f)
                        return Reject(route, "WRONG_OBSTACLE_SIDE");
                    crossed[obstacle][returning] = true;
                }
            }

            /* Only a truly centred, forward, zero-curvature approach qualifies.
             * Direction is assumed available immediately at this first pose. */
            unsigned approach = 0U;
            float firstEnd = plan->segments[0].signedDistanceMm;
            float secondBegin = firstEnd;
            for (unsigned i = 1U; i < 4U; ++i)
                secondBegin += plan->segments[i].signedDistanceMm;
            float secondEnd = secondBegin + plan->segments[4].signedDistanceMm;
            bool inApproach = progress + step < firstEnd ||
                (progress + step >= secondBegin && progress + step < secondEnd);
            if (progress >= secondBegin)
                approach = 1U;
            float face = approach == 0U ? firstFace : secondFace;
            float range = face - pose.xMm - CameraOffset(&route->config);
            float steeringBegin = approach == 0U ? firstEnd : secondEnd;
            if (fused)
                steeringBegin -= plan->junctionHalfLengthMm[approach == 0U ? 0U : 4U];
            if (inApproach && !seen[approach] && fabsf(pose.yMm) < 0.1f &&
                fabsf(pose.yawRad) < 0.0001f && fabsf(end.curvaturePerMm) < 1e-8f &&
                range <= route->config.recognitionCameraRangeMm && range > 0.0f &&
                0.5f * route->config.recognitionTargetWidthMm <
                    range * tanf(route->config.cameraHorizontalFovDeg * ROUTE_PI / 360.0f) &&
                progress + step < steeringBegin - 0.5f)
            {
                seen[approach] = true;
                report->recognitionPose[approach] = pose;
                report->recognitionProgressMm[approach] = progress + step;
            }
            progress += step;
            d += step;
        }
        first = run.last + 1U;
    }
    if (!seen[0] || !seen[1])
        return Reject(route, "NO_CAMERA_HANDOFF");
    for (unsigned i = 0U; i < 2U; ++i)
        if (!crossed[i][0] || !crossed[i][1])
            return Reject(route, "INCOMPLETE_BYPASS");
    /* Entire final body must fit in the park, including the open-mouth side. */
    float footprintLength = route->config.bodyLengthMm + route->config.frontProtrusionMm;
    float centre = route->config.bodyCenterAheadOfRearAxleMm + 0.5f * route->config.frontProtrusionMm;
    float centreX = pose.xMm + centre * cosf(pose.yawRad);
    float centreY = pose.yMm + centre * sinf(pose.yawRad);
    float extentX = 0.5f * (footprintLength * fabsf(cosf(pose.yawRad)) +
                           route->config.bodyWidthMm * fabsf(sinf(pose.yawRad)));
    float extentY = 0.5f * (footprintLength * fabsf(sinf(pose.yawRad)) +
                           route->config.bodyWidthMm * fabsf(cosf(pose.yawRad)));
    if (centreX - extentX <= -500.0f || centreX + extentX >= 0.0f ||
        fabsf(centreY) + extentY >= 300.0f)
        return Reject(route, "PARKING_FOOTPRINT");
    report->finishPose = pose;
    report->valid = true;
    report->reason = "OK";
    return true;
}

bool Task2Route_Create(const Task2RouteConfig *config,
    const MotionSequenceConfig *sequenceConfig, float mmPerCount,
    bool fused, Task2Route *route)
{
    if (route == NULL)
        return false;
    *route = (Task2Route){0};
    if (config != NULL)
        route->config = *config;
    if (config == NULL || sequenceConfig == NULL || !ValidConfig(config))
        return Reject(route, "INVALID_PARAMETERS");
    if (!BuildSteps(route))
        return false;
    for (unsigned i = 0U; i < TASK2_ROUTE_STEP_COUNT; ++i)
        route->steps[i].stopAfter = !fused;
    MotionSequencePlan plan;
    if (!Prepare(route, sequenceConfig, mmPerCount, fused, &plan) ||
        plan.totalTravelMm > ROUTE_MAX_TRAVEL_MM)
        return Reject(route, "PROFILE_LIMITS");
    if (fused)
    {
        /* Symmetric O1 ramps preserve y=0/yaw=0 but change forward displacement.
         * Compensate only straight 5, keeping the O2 entry anchored physically.
         * This is nominal blend geometry, not a correction for measured drift. */
        float afterFirst = plan.junctionHalfLengthMm[3];
        for (unsigned i = 0U; i < 4U; ++i)
            afterFirst += plan.segments[i].signedDistanceMm;
        Task2RoutePose pose;
        if (!Integrate(&plan, afterFirst, &pose) || fabsf(pose.yMm) > 0.1f ||
            fabsf(pose.yawRad) > 0.0001f)
            return Reject(route, "O1_REJOIN_NOT_CENTRED");
        float expectedX = config->parkToObstacle1Mm + config->firstRejoinBeyondFaceMm +
                          plan.junctionHalfLengthMm[3];
        float correction = pose.xMm - expectedX;
        route->report.secondApproachCorrectionMm = correction;
        route->steps[4].signedDistanceMm -= correction;
        if (route->steps[4].signedDistanceMm <= 0.0f ||
            !Prepare(route, sequenceConfig, mmPerCount, true, &plan))
            return Reject(route, "NO_CENTRED_APPROACH");
    }
    return ValidatePath(route, &plan, fused);
}
