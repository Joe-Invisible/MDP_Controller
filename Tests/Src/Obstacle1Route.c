#include "Obstacle1Route.h"

#include <math.h>
#include <stddef.h>

#define O1_PI 3.14159265358979323846f
#define O1_MIN_RADIUS_MM 350.0f
#define O1_MAX_TRAVEL_MM 12000.0f
#define O1_SAMPLE_MM 1.0f

/* Important experiment settings. O2 is not present in this isolated test.
 * The original Task2Route.c configuration and full-route mode remain intact. */
Obstacle1RouteConfig obstacle1RouteConfig = {
    /* Arena and nominal body buffer. Positive y is left of the initial +x. */
    .arrow = TASK2_ARROW_LEFT,
    .parkToObstacleMm = 1050.0f,
    .obstacleBufferMm = 100.0f,
    .arenaHalfWidthMm = 750.0f,

    /* Instant recognition; accelerate as soon as the simulated reply arrives. */
    .approachSpeedCps = 2000.0f,
    .cruiseSpeedCps = 5000.0f,
    .recognitionCameraRangeMm = 1000.0f,
    .minimumApproachCameraRangeMm = 500.0f,
    .recognitionLeadMm = 5.0f,
    .bodyLengthMm = 227.0f,
    .bodyWidthMm = 200.0f,
    .bodyCenterAheadOfRearAxleMm = 76.5f,
    .frontProtrusionMm = 20.0f,
    .cameraBehindFrontMm = 80.0f,
    .cameraHorizontalFovDeg = 60.0f,
    .recognitionTargetWidthMm = 100.0f,

    /* Gentle O1 sweep, followed by a loose exit back to y=0, yaw=0.
     * No minimum primitive length is imposed just to obtain a long blend. */
    .departureRadiusMm = 500.0f,
    .bypassRadiusMm = 500.0f,
    .departureAngleDeg = 55.0f,
    .exitRadiusMm = 1500.0f,
    .entryBlendMm = 500.0f,
    .bypassBlendMm = 500.0f,
};

const char *const obstacle1RouteStepNames[OBSTACLE1_ROUTE_STEP_COUNT] = {
    "Approach O1", "O1 depart", "O1 bypass", "Loose O1 exit",
};

static bool Reject(Obstacle1Route *route, const char *reason)
{
    route->report.valid = false;
    route->report.reason = reason;
    return false;
}

static float CameraOffset(const Obstacle1RouteConfig *config)
{
    return config->bodyCenterAheadOfRearAxleMm +
        0.5f * config->bodyLengthMm - config->cameraBehindFrontMm;
}

float Obstacle1Route_CameraRangeMm(const Obstacle1RouteConfig *config,
    const Task2RoutePose *pose)
{
    float offset = CameraOffset(config);
    return hypotf(config->parkToObstacleMm - pose->xMm - offset * cosf(pose->yawRad),
                  pose->yMm + offset * sinf(pose->yawRad));
}

bool Obstacle1Route_CameraVisible(const Obstacle1RouteConfig *c, const Task2RoutePose *pose)
{
    float cosine = cosf(pose->yawRad), sine = sinf(pose->yawRad);
    float cameraX = pose->xMm + CameraOffset(c) * cosine;
    float cameraY = pose->yMm + CameraOffset(c) * sine;
    float dx = c->parkToObstacleMm - cameraX;
    if (dx <= 0.0f || hypotf(dx, cameraY) > c->recognitionCameraRangeMm)
        return false;
    for (int side = -1; side <= 1; side += 2)
    {
        float dy = side * 0.5f * c->recognitionTargetWidthMm - cameraY;
        float forward = dx * cosine + dy * sine;
        float lateral = -dx * sine + dy * cosine;
        if (forward <= 0.0f || fabsf(lateral) >=
            forward * tanf(c->cameraHorizontalFovDeg * O1_PI / 360.0f))
            return false;
    }
    return true;
}

bool Obstacle1Route_CheckPose(const Obstacle1RouteConfig *c,
    const Task2RoutePose *pose, Obstacle1RouteReport *report)
{
    const Task2RouteConfig bodyConfig = {
        .parkToObstacle1Mm = c->parkToObstacleMm,
        .bodyLengthMm = c->bodyLengthMm, .bodyWidthMm = c->bodyWidthMm,
        .bodyCenterAheadOfRearAxleMm = c->bodyCenterAheadOfRearAxleMm,
        .frontProtrusionMm = c->frontProtrusionMm,
        .obstacleBufferMm = c->obstacleBufferMm,
    };
    Task2RouteReport footprint = {
        .minimumObstacleGapMm = report->minimumObstacleGapMm,
        .minimumSideMarginMm = report->minimumSideMarginMm,
        .minimumParkWallMarginMm = report->minimumParkWallMarginMm,
    };
    bool clear = Task2Route_CheckObstacle1Pose(&bodyConfig, c->arenaHalfWidthMm, pose, &footprint);
    report->minimumObstacleGapMm = footprint.minimumObstacleGapMm;
    report->minimumSideMarginMm = footprint.minimumSideMarginMm;
    report->minimumParkWallMarginMm = footprint.minimumParkWallMarginMm;
    return clear;
}

static bool ValidConfig(const Obstacle1RouteConfig *c)
{
    const float positive[] = {
        c->parkToObstacleMm, c->obstacleBufferMm, c->arenaHalfWidthMm,
        c->approachSpeedCps, c->cruiseSpeedCps, c->recognitionCameraRangeMm,
        c->minimumApproachCameraRangeMm, c->recognitionLeadMm,
        c->bodyLengthMm, c->bodyWidthMm, c->cameraHorizontalFovDeg,
        c->recognitionTargetWidthMm, c->departureRadiusMm, c->bypassRadiusMm,
        c->departureAngleDeg, c->exitRadiusMm, c->entryBlendMm, c->bypassBlendMm,
    };
    for (unsigned i = 0; i < sizeof(positive) / sizeof(positive[0]); ++i)
        if (!isfinite(positive[i]) || positive[i] <= 0.0f)
            return false;
    return (c->arrow == TASK2_ARROW_LEFT || c->arrow == TASK2_ARROW_RIGHT) &&
        isfinite(c->bodyCenterAheadOfRearAxleMm) &&
        fabsf(c->bodyCenterAheadOfRearAxleMm) <= 0.5f * c->bodyLengthMm &&
        isfinite(c->frontProtrusionMm) && c->frontProtrusionMm >= 0.0f &&
        c->parkToObstacleMm >= 600.0f && c->approachSpeedCps <= c->cruiseSpeedCps &&
        c->recognitionCameraRangeMm <= 1000.0f &&
        c->minimumApproachCameraRangeMm < c->recognitionCameraRangeMm &&
        c->cameraHorizontalFovDeg < 180.0f && c->departureAngleDeg < 85.0f &&
        c->departureRadiusMm >= O1_MIN_RADIUS_MM && c->bypassRadiusMm >= O1_MIN_RADIUS_MM &&
        c->exitRadiusMm >= fmaxf(c->departureRadiusMm, c->bypassRadiusMm) &&
        isfinite(c->cameraBehindFrontMm) && c->cameraBehindFrontMm >= 0.0f &&
        c->cameraBehindFrontMm <= c->bodyLengthMm;
}

static bool Prepare(const Obstacle1Route *route, const MotionSequenceConfig *config,
                    float mmPerCount, bool fused, MotionSequencePlan *plan)
{
    *plan = (MotionSequencePlan){0};
    plan->count = OBSTACLE1_ROUTE_STEP_COUNT;
    for (unsigned i = 0; i < plan->count; ++i)
    {
        plan->segments[i] = route->steps[i];
        plan->segments[i].stopAfter = !fused;
        plan->junctionBlendLimitMm[i] = route->blendLimitMm[i];
    }
    /* Geometry only: the same curvature planner, without speed calibration. */
    return MotionSequencePlan_Prepare(plan, config, mmPerCount,
        3000, 3000, 2000, 2000, 0, 480);
}

static bool Integrate(const MotionSequencePlan *plan, float finish, Task2RoutePose *pose)
{
    *pose = (Task2RoutePose){TASK2_ROUTE_START_X_MM, 0, 0};
    MotionSequenceRun run = {plan, 0, OBSTACLE1_ROUTE_STEP_COUNT - 1U};
    for (float s = 0; s < finish;)
    {
        float step = fminf(O1_SAMPLE_MM, finish - s);
        MotionPathSample middle, end;
        if (!MotionSequencePlan_Evaluate(&run, s + 0.5f * step, &middle) ||
            !MotionSequencePlan_Evaluate(&run, s + step, &end))
            return false;
        pose->xMm += step * cosf(middle.desiredYawRad);
        pose->yMm += step * sinf(middle.desiredYawRad);
        pose->yawRad = end.desiredYawRad;
        s += step;
    }
    return true;
}

/* Long ramps move the curve's endpoint even though net yaw is unchanged.
 * Fit the first two arc lengths; derive the loose exit length for zero net yaw.
 * This nominal geometry correction uses no measured servo-response model. */
static bool AlignFusedEndpoint(Obstacle1Route *route, const MotionSequenceConfig *config,
                              float mmPerCount, float targetX)
{
    const float r1 = route->config.departureRadiusMm;
    const float r2 = route->config.bypassRadiusMm;
    const float r3 = route->config.exitRadiusMm;
    MotionSequencePlan plan;
    for (unsigned iteration = 0; iteration < 16; ++iteration)
    {
        route->steps[3].signedDistanceMm = r3 *
            (route->steps[2].signedDistanceMm / r2 - route->steps[1].signedDistanceMm / r1);
        if (route->steps[3].signedDistanceMm <= 0 ||
            !Prepare(route, config, mmPerCount, true, &plan) || plan.totalTravelMm > O1_MAX_TRAVEL_MM)
            return false;
        Task2RoutePose end;
        if (!Integrate(&plan, plan.totalTravelMm, &end))
            return false;
        float ex = targetX - end.xMm, ey = -end.yMm;
        if (hypotf(ex, ey) < 0.25f && fabsf(end.yawRad) < 0.0001f)
            return true;
        float jacobian[2][2];
        for (unsigned variable = 0; variable < 2; ++variable)
        {
            float original = route->steps[variable + 1U].signedDistanceMm;
            route->steps[variable + 1U].signedDistanceMm += 2.0f;
            route->steps[3].signedDistanceMm = r3 *
                (route->steps[2].signedDistanceMm / r2 - route->steps[1].signedDistanceMm / r1);
            Task2RoutePose perturbed;
            bool ok = Prepare(route, config, mmPerCount, true, &plan) &&
                      Integrate(&plan, plan.totalTravelMm, &perturbed);
            route->steps[variable + 1U].signedDistanceMm = original;
            if (!ok)
                return false;
            jacobian[0][variable] = (perturbed.xMm - end.xMm) * 0.5f;
            jacobian[1][variable] = (perturbed.yMm - end.yMm) * 0.5f;
        }
        float determinant = jacobian[0][0] * jacobian[1][1] - jacobian[0][1] * jacobian[1][0];
        if (!isfinite(determinant) || fabsf(determinant) < 1e-5f)
            return false;
        float change1 = (ex * jacobian[1][1] - ey * jacobian[0][1]) / determinant;
        float change2 = (ey * jacobian[0][0] - ex * jacobian[1][0]) / determinant;
        float scale = fminf(1.0f, 100.0f / fmaxf(fabsf(change1), fabsf(change2)));
        route->steps[1].signedDistanceMm += scale * change1;
        route->steps[2].signedDistanceMm += scale * change2;
        if (route->steps[1].signedDistanceMm <= 0 || route->steps[2].signedDistanceMm <= 0)
            return false;
    }
    return false;
}

static bool Validate(Obstacle1Route *route, const MotionSequencePlan *plan, bool fused)
{
    Obstacle1RouteReport *report = &route->report;
    report->minimumObstacleGapMm = report->minimumSideMarginMm =
        report->minimumParkWallMarginMm = INFINITY;
    Task2RoutePose pose = {TASK2_ROUTE_START_X_MM, 0, 0};
    float progress = 0;
    float steeringStart = plan->segments[0].signedDistanceMm - plan->junctionHalfLengthMm[0];
    for (unsigned first = 0; first < plan->count;)
    {
        MotionSequenceRun run = {plan, first, MotionSequencePlan_RunEnd(plan, first)};
        float length = 0;
        for (unsigned i = first; i <= run.last; ++i)
            length += plan->segments[i].signedDistanceMm;
        float initialYaw = pose.yawRad;
        for (float s = 0; s < length;)
        {
            float step = fminf(O1_SAMPLE_MM, length - s);
            MotionPathSample middle, end;
            if (!MotionSequencePlan_Evaluate(&run, s + 0.5f * step, &middle) ||
                !MotionSequencePlan_Evaluate(&run, s + step, &end))
                return Reject(route, "PROFILE_EVALUATION");
            pose.xMm += step * cosf(initialYaw + middle.desiredYawRad);
            pose.yMm += step * sinf(initialYaw + middle.desiredYawRad);
            pose.yawRad = initialYaw + end.desiredYawRad;
            progress += step;
            if (!Obstacle1Route_CheckPose(&route->config, &pose, report))
                return Reject(route, "CLEARANCE_OR_BOUNDARY");
            if (report->recognitionProgressMm == 0 && progress < steeringStart &&
                fabsf(pose.yMm) < 0.1f && fabsf(pose.yawRad) < 1e-4f &&
                Obstacle1Route_CameraVisible(&route->config, &pose))
            {
                report->recognitionPose = pose;
                report->recognitionProgressMm = progress;
            }
            s += step;
        }
        first = run.last + 1U;
    }
    if (report->recognitionProgressMm == 0)
        return Reject(route, "NO_CAMERA_HANDOFF");
    Task2RoutePose entry;
    float entryProgress = plan->segments[0].signedDistanceMm + plan->junctionHalfLengthMm[0];
    if (fused)
    {
        if (!Integrate(plan, entryProgress, &entry))
            return Reject(route, "PROFILE_EVALUATION");
    }
    else
        entry = (Task2RoutePose){TASK2_ROUTE_START_X_MM + entryProgress, 0, 0};
    report->entryBlendExitCameraRangeMm = Obstacle1Route_CameraRangeMm(&route->config, &entry);
    if (report->entryBlendExitCameraRangeMm < route->config.minimumApproachCameraRangeMm)
        return Reject(route, "APPROACH_RANGE");
    if (pose.xMm < route->config.parkToObstacleMm + 100 +
                   0.5f * route->config.bodyLengthMm - route->config.bodyCenterAheadOfRearAxleMm +
                   route->config.obstacleBufferMm ||
        fabsf(pose.yMm) > 0.5f || fabsf(pose.yawRad) > 1e-4f)
        return Reject(route, "EXIT_NOT_CENTRED");
    report->finishPose = pose;
    report->valid = true;
    report->reason = "OK";
    return true;
}

bool Obstacle1Route_Create(const Obstacle1RouteConfig *config,
    const MotionSequenceConfig *sequenceConfig, float mmPerCount, bool fused, Obstacle1Route *route)
{
    if (route == NULL)
        return false;
    *route = (Obstacle1Route){0};
    if (config != NULL)
        route->config = *config;
    if (config == NULL || sequenceConfig == NULL || !ValidConfig(config))
        return Reject(route, "INVALID_PARAMETERS");
    float anchorRange = config->recognitionCameraRangeMm - 0.5f * config->entryBlendMm -
                        config->recognitionLeadMm;
    float startX = config->parkToObstacleMm - CameraOffset(config) - anchorRange;
    float approachLength = startX - TASK2_ROUTE_START_X_MM;
    if (approachLength < 0.5f * config->entryBlendMm + config->recognitionLeadMm)
        return Reject(route, "NO_APPROACH_SPACE");
    float a = config->departureAngleDeg * O1_PI / 180.0f;
    float cosineB = 1.0f - (config->departureRadiusMm + config->bypassRadiusMm) *
        (1.0f - cosf(a)) / (config->bypassRadiusMm + config->exitRadiusMm);
    if (cosineB <= 0 || cosineB >= 1)
        return Reject(route, "INVALID_GEOMETRY");
    float b = acosf(cosineB);
    float radii[] = {0, config->arrow * config->departureRadiusMm,
                    -config->arrow * config->bypassRadiusMm, config->arrow * config->exitRadiusMm};
    float lengths[] = {approachLength, config->departureRadiusMm * a,
                       config->bypassRadiusMm * (a + b), config->exitRadiusMm * b};
    for (unsigned i = 0; i < OBSTACLE1_ROUTE_STEP_COUNT; ++i)
        route->steps[i] = (MotionSequenceSegment){lengths[i], radii[i] == 0 ? 0 : 1.0f / radii[i],
                                                config->cruiseSpeedCps, !fused};
    route->blendLimitMm[0] = config->entryBlendMm;
    route->blendLimitMm[1] = route->blendLimitMm[2] = config->bypassBlendMm;
    float targetX = startX + (config->departureRadiusMm + config->bypassRadiusMm) * sinf(a) +
                   (config->bypassRadiusMm + config->exitRadiusMm) * sinf(b);
    if (fused && !AlignFusedEndpoint(route, sequenceConfig, mmPerCount, targetX))
        return Reject(route, "BLEND_ALIGNMENT");
    MotionSequencePlan plan;
    if (!Prepare(route, sequenceConfig, mmPerCount, fused, &plan) || plan.totalTravelMm > O1_MAX_TRAVEL_MM)
        return Reject(route, "PROFILE_LIMITS");
    return Validate(route, &plan, fused);
}
