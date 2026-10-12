#ifndef OBSTACLE1_ROUTE_H
#define OBSTACLE1_ROUTE_H

#include "Task2Route.h"

#define OBSTACLE1_ROUTE_STEP_COUNT 4U

typedef struct
{
    Task2Arrow arrow;
    float parkToObstacleMm;       /* Park mouth to O1 front face. */
    float obstacleBufferMm;       /* Nominal outer-body separation. */
    float arenaHalfWidthMm;
    float approachSpeedCps;
    float cruiseSpeedCps;
    float recognitionCameraRangeMm;
    float minimumApproachCameraRangeMm; /* Range at the end of the entry blend. */
    float recognitionLeadMm;      /* Recognition precedes entry steering. */
    float bodyLengthMm;
    float bodyWidthMm;
    float cameraBehindFrontMm;
    float cameraHorizontalFovDeg;
    float recognitionTargetWidthMm;

    float departureRadiusMm;
    float bypassRadiusMm;
    float departureAngleDeg;
    float exitRadiusMm;
    float entryBlendMm;
    float bypassBlendMm;
} Obstacle1RouteConfig;

typedef struct
{
    bool valid;
    const char *reason;
    float minimumObstacleGapMm;
    float minimumSideMarginMm;
    float minimumParkWallMarginMm;
    Task2RoutePose recognitionPose;
    float recognitionProgressMm;
    float entryBlendExitCameraRangeMm;
    Task2RoutePose finishPose;
} Obstacle1RouteReport;

typedef struct
{
    Obstacle1RouteConfig config;
    MotionSequenceSegment steps[OBSTACLE1_ROUTE_STEP_COUNT];
    float blendLimitMm[OBSTACLE1_ROUTE_STEP_COUNT];
    Obstacle1RouteReport report;
} Obstacle1Route;

extern Obstacle1RouteConfig obstacle1RouteConfig;
extern const char *const obstacle1RouteStepNames[OBSTACLE1_ROUTE_STEP_COUNT];

bool Obstacle1Route_Create(const Obstacle1RouteConfig *config,
    const MotionSequenceConfig *sequenceConfig, float mmPerCount,
    bool fused, Obstacle1Route *route);
bool Obstacle1Route_CheckPose(const Obstacle1RouteConfig *config,
    const Task2RoutePose *pose, Obstacle1RouteReport *report);
float Obstacle1Route_CameraRangeMm(const Obstacle1RouteConfig *config,
    const Task2RoutePose *pose);
bool Obstacle1Route_CameraVisible(const Obstacle1RouteConfig *config,
    const Task2RoutePose *pose);

#endif
