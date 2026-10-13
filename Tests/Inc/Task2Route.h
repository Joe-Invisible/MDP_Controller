#ifndef TASK2_ROUTE_H
#define TASK2_ROUTE_H

#include <stdbool.h>
#include <stdint.h>

#define TASK2_ROUTE_STEP_COUNT 16U /* Storage capacity; use route.stepCount. */
#define TASK2_ROUTE_O2_APPROACH_STEP 3U
#define TASK2_ROUTE_O2_DEPARTURE_STEP 4U
#define TASK2_ROUTE_O2_IR_STRAIGHT_STEP 5U
#define TASK2_ROUTE_START_X_MM (-250.0f)

typedef enum
{
    TASK2_ARROW_RIGHT = -1,
    TASK2_ARROW_LEFT = 1,
} Task2Arrow;

/* Test inputs only. Poses reference the rear axle midpoint. Startup ranging
 * is handled by the harness; sensed O2 edges can replace the planned suffix. */
typedef struct
{
    Task2Arrow firstArrow;
    Task2Arrow secondArrow;
    float obstacle2WidthMm;
    float parkToObstacle1Mm;       /* Park mouth to O1 front face. */
    float obstacleFaceGapMm;       /* O1 back face to O2 front face. */
    float obstacleBufferMm;        /* Minimum nominal outer-body clearance. */
    float parkBackWallBufferMm;
    float boundaryBufferMm;         /* Nominal margin from arena/park walls. */
    float obstacle2EndSpaceMm;     /* Worst-case available space at each end. */

    float cruiseSpeedCps;
    float approachSpeedCps;        /* Soft request before either arrow is recognised. */
    float bodyLengthMm;
    float bodyWidthMm;
    float bodyCenterAheadOfRearAxleMm;
    float frontProtrusionMm;       /* Included conservatively at full body width. */
    float cameraBehindFrontMm;
    float recognitionCameraRangeMm;
    float cameraHorizontalFovDeg;
    float recognitionTargetWidthMm; /* Assumed centred arrow; measure to replace. */

    float firstArcCameraRangeMm;   /* Anchored nominal O1 entry. */
    float firstRejoinBeyondFaceMm; /* Virtual original rejoin anchor: fixes O1 sweep angle. */
    float firstRadiusMm;
    float secondArcCameraRangeMm;  /* Minimum camera range at departure, normally 500. */
    float obstacle1ExitYawDeg;     /* Inward magnitude, mirrored by firstArrow. */
    float obstacle2ApproachMaxMm;
    float obstacle2DepartureWidthReserveMm; /* Whole-width reserve for sensing delay/model. */
    float obstacle2DepartureSpeedCps;
    float obstacle2IRAcquisitionSpeedCps; /* Brief soft cap near the IR yaw gate, before presence is armed. */
    float secondDepartureRadiusMm;
    float secondIRDepartureRadiusMm; /* Opposite arrows: looser, IR-terminated arc. */
    float secondIRDepartureGuardExtraMm; /* Beyond confirmation and braking estimates. */
    float secondBackExtraRadiusMm; /* Back radius = W2/2 + this value. */

} Task2RouteConfig;

typedef struct
{
    float xMm;
    float yMm;
    float yawRad;
} Task2RoutePose;

typedef struct
{
    float aheadMm;
    float leftMm;
    float yawDeg; /* Beam direction relative to chassis +x. */
} Task2SensorMount;

typedef struct
{
    Task2SensorMount mount;
    float guardTravelMm; /* Confirmation, braking and additional travel reserve. */
    float edgeUncertaintyMm;
    float acquisitionMinYawDeg;
    float acquisitionTravelMm; /* Fresh-sample confirmation window at departure speed. */
} Task2RouteIRDeparture;

typedef struct
{
    bool valid;
    const char *reason;
    float minimumObstacleGapMm;
    float minimumSideMarginMm;
    float minimumParkWallMarginMm;
    float minimumParkBackMarginMm;
    float minimumParkSideMarginMm;
    /* Earliest eligible camera pose, including the angled O2 approach. */
    Task2RoutePose recognitionPose[2];
    float recognitionProgressMm[2];
    Task2RoutePose finishPose;
} Task2RouteReport;

typedef struct
{
    float distanceMm;
    float radiusMm; /* Zero: straight; positive: left; negative: right. */
    float speedCps; /* Hard ceiling; both approaches use soft speed requests. */
} Task2RouteStep;

typedef struct
{
    Task2RouteConfig config; /* Frozen inputs for this standalone run. */
    Task2RouteStep steps[TASK2_ROUTE_STEP_COUNT];
    uint32_t stepCount;
    float totalTravelMm;
    float totalYawRad;
    bool obstacle2IRDeparture;
    bool obstacle2ReturnPending;
    Task2RouteReport report;
} Task2Route;

/* Edit the grouped initializer in Tests/Src/Task2Route.c, or set before Run(). */
extern Task2RouteConfig task2RouteConfig;
extern const char *const task2RouteStepNames[TASK2_ROUTE_STEP_COUNT];

/* Pure 1 mm swept-footprint preflight of stopped circular/straight primitives. */
bool Task2Route_Create(const Task2RouteConfig *config, Task2Route *route);
/* Opposite-arrow prefix: guard is checked now; suffix is built at the IR stop. */
bool Task2Route_CreateIR(const Task2RouteConfig *config,
    const Task2RouteIRDeparture *ir, Task2Route *route);

/* Instantaneous recognition simulation: whole horizontal O1 target in view. */
bool Task2Route_ObstacleCameraVisible(const Task2RouteConfig *config,
    const Task2RoutePose *pose, unsigned obstacle);
bool Task2Route_Obstacle1CameraVisible(const Task2RouteConfig *config,
    const Task2RoutePose *pose);

/* Replace the unexecuted suffix after an edge stop. Width is an observed
 * conservative planning bound; configured width remains the test footprint. */
bool Task2Route_ReplanObstacle2(Task2Route *route, const Task2RoutePose *stoppedPose,
    float observedWidthMm, unsigned completedDepartureStep);

/* Independent footprint checks, also usable for host ideal execution checks. */
/* O1-only footprint check; also useful for host geometry regressions. */
bool Task2Route_CheckObstacle1Pose(const Task2RouteConfig *config,
    float arenaHalfWidthMm, const Task2RoutePose *pose, Task2RouteReport *report);

bool Task2Route_CheckPose(const Task2RouteConfig *config,
    const Task2RoutePose *pose, Task2RouteReport *report);

#endif
