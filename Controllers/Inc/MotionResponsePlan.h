#ifndef MOTION_RESPONSE_PLAN_H
#define MOTION_RESPONSE_PLAN_H
#include "MotionPathProfile.h"
#include "RobotKinematics.h"
#include <stdint.h>

#define MOTION_RESPONSE_NODE_CAPACITY 160U
#define MOTION_RESPONSE_SPEED_CPS 4500.0f
#define MOTION_RESPONSE_RADIUS_MM 525.0f
#define MOTION_RESPONSE_RAW_RATE_PER_SEC 200.0f
#define MOTION_RESPONSE_ACCELERATION_MMPS2 2000.0f

typedef struct
{
    float xMm, yMm, yawRad;
} MotionResponsePose;
typedef struct
{
    float progressMm;
    MotionResponsePose pose;
} MotionResponseNode;
typedef struct
{
    MotionResponseNode nodes[MOTION_RESPONSE_NODE_CAPACITY];
    uint32_t nodeCount;
    float totalDistanceMm;
    float extraArcMm;
    float requestedYawRad;
    bool prepared;
} MotionResponsePlan;

/* Bounded experimental mode: forward, +R525 actuator schedule, 4500 CPS.
 * Only the constant arc may grow (0..500 mm); the measured lead, asymmetric
 * connectors, zero-command recovery and slowing portion retain their lengths.
 * Extended steady response is an assumption to validate, not a calibration.
 * Returns false for unsupported heading/kinematics; never silently extrapolates.
 * Does not fit an independently specified (x,y) endpoint. Pose() reports the
 * resulting geometry for a planner/operator to accept before execution. */
bool MotionResponsePlan_Prepare(MotionResponsePlan *plan,
                                const RobotKinematics *kinematics,
                                float finalYawRad);
bool MotionResponsePlan_Evaluate(const void *context, float progressMm, MotionPathSample *sample);
bool MotionResponsePlan_Pose(const MotionResponsePlan *plan,
                             float progressMm,
                             MotionResponsePose *pose);
/* Same phases 1..6 as the characterization (6 includes the slowing portion). */
uint8_t MotionResponsePlan_Phase(const MotionResponsePlan *plan, float progressMm);
#endif
