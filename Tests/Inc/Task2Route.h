#ifndef TASK2_ROUTE_H
#define TASK2_ROUTE_H

#include <stdbool.h>
#include <stdint.h>

#define TASK2_ROUTE_STEP_COUNT 8U
#define TASK2_ROUTE_START_X_MM (-250.0f)

typedef enum
{
    TASK2_ARROW_RIGHT = -1,
    TASK2_ARROW_LEFT = 1,
} Task2Arrow;

/* Test inputs only. Poses reference the rear axle midpoint. Startup ranging
 * is handled by the harness; there is no live in-motion sensor endpoint here. */
typedef struct
{
    Task2Arrow firstArrow;
    Task2Arrow secondArrow;
    float obstacle2WidthMm;
    float parkToObstacle1Mm;       /* Park mouth to O1 front face. */
    float obstacleFaceGapMm;       /* O1 back face to O2 front face. */
    float obstacleBufferMm;        /* Minimum nominal outer-body clearance. */
    float obstacle2EndSpaceMm;     /* Worst-case available space at each end. */

    float cruiseSpeedCps;
    float approachSpeedCps;        /* Before O1 recognition; whole O2 straight. */
    float bodyLengthMm;
    float bodyWidthMm;
    float bodyCenterAheadOfRearAxleMm;
    float frontProtrusionMm;       /* Included conservatively at full body width. */
    float cameraBehindFrontMm;
    float recognitionCameraRangeMm;
    float cameraHorizontalFovDeg;
    float recognitionTargetWidthMm; /* Assumed centred arrow; measure to replace. */

    float firstArcCameraRangeMm;   /* Anchored nominal O1 entry. */
    float firstRejoinBeyondFaceMm; /* O1 centre-line rejoin, from its front. */
    float firstRadiusMm;
    float secondArcCameraRangeMm;  /* Anchored nominal O2 entry. */
    float secondDepartureRadiusMm;
    float secondBackExtraRadiusMm; /* Back radius = W2/2 + this value. */
    float returnJoinHeadingDeg;    /* Magnitude, measured from initial +x. */

} Task2RouteConfig;

typedef struct
{
    float xMm;
    float yMm;
    float yawRad;
} Task2RoutePose;

typedef struct
{
    bool valid;
    const char *reason;
    float minimumObstacleGapMm;
    float minimumSideMarginMm;
    float minimumParkWallMarginMm;
    /* Earliest eligible centred pose, before steering begins toward an arrow. */
    Task2RoutePose recognitionPose[2];
    float recognitionProgressMm[2];
    Task2RoutePose finishPose;
} Task2RouteReport;

typedef struct
{
    float distanceMm;
    float radiusMm; /* Zero: straight; positive: left; negative: right. */
    float speedCps; /* Hard ceiling; first approach uses a soft speed request. */
} Task2RouteStep;

typedef struct
{
    Task2RouteConfig config; /* Frozen inputs for this standalone run. */
    Task2RouteStep steps[TASK2_ROUTE_STEP_COUNT];
    float totalTravelMm;
    float totalYawRad;
    Task2RouteReport report;
} Task2Route;

/* Edit the grouped initializer in Tests/Src/Task2Route.c, or set before Run(). */
extern Task2RouteConfig task2RouteConfig;
extern const char *const task2RouteStepNames[TASK2_ROUTE_STEP_COUNT];

/* Pure 1 mm swept-footprint preflight of stopped circular/straight primitives. */
bool Task2Route_Create(const Task2RouteConfig *config, Task2Route *route);

/* Instantaneous recognition simulation: whole horizontal O1 target in view. */
bool Task2Route_Obstacle1CameraVisible(const Task2RouteConfig *config,
    const Task2RoutePose *pose);

/* Independent footprint checks, also usable for host ideal execution checks. */
/* O1-only footprint check; also useful for host geometry regressions. */
bool Task2Route_CheckObstacle1Pose(const Task2RouteConfig *config,
    float arenaHalfWidthMm, const Task2RoutePose *pose, Task2RouteReport *report);

bool Task2Route_CheckPose(const Task2RouteConfig *config,
    const Task2RoutePose *pose, Task2RouteReport *report);

#endif
