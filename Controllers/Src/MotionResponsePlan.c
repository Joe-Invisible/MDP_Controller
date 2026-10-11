#include "MotionResponsePlan.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define PI 3.14159265358979323846f
#define LEAD_MM 350.0f
#define RAMP_MM 135.378571f
#define ARC_END_MM 785.378571f
#define EXIT_END_MM 920.757142f
#define RECOVERY_END_MM 1270.757142f
#define BASE_END_MM 1420.75708f
#define END_RAW (-45.6142807f)

typedef struct { float progressMm, yawRad; } ResponseHeadingNode;
#include "MotionResponseModel.inc"

/* Wheel reference is the static-table policy used during characterization,
 * deliberately independent of predicted body curvature. */
static const struct { float raw, curvature; } wheelKnots[] = {
    {0.0f, 0.0f}, {-5.625f, .000054f}, {-11.25f, .000183f},
    {-16.875f, .000436f}, {-22.5f, 1.0f/1500.0f},
    {-27.7000008f, 1.0f/1000.0f}, {END_RAW, 1.0f/525.0f}
};

static float Clamp(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }

/* Exact displacement on each constant-curvature interpolation interval.
 * Mid-heading + sinc avoids cancellation for nearly straight intervals. */
static MotionResponsePose Advance(MotionResponsePose start, float distance, float yaw)
{
    float half = (yaw-start.yawRad)*0.5f;
    float sinc = fabsf(half) < 0.0001f ? 1.0f-half*half/6.0f : sinf(half)/half;
    float heading = start.yawRad+half;
    return (MotionResponsePose){start.xMm+distance*sinc*cosf(heading),
        start.yMm+distance*sinc*sinf(heading), yaw};
}

bool MotionResponsePlan_Prepare(MotionResponsePlan *plan,
    const RobotKinematics *kinematics, float finalYawRad)
{
    if (plan == NULL) return false;
    memset(plan, 0, sizeof(*plan));
    if (kinematics == NULL || !isfinite(finalYawRad) ||
        kinematics->rearEncoderCountsPerRev != 1560U ||
        fabsf(kinematics->rearWheelDiameterMm-65.5f) > .001f ||
        fabsf(kinematics->rearTrackWidthMm-165.0f) > .001f ||
        fabsf(kinematics->wheelbaseMm-145.0f) > .001f ||
        !isfinite(kinematics->rearWheelDiameterMm) ||
        !isfinite(kinematics->rearTrackWidthMm) || !isfinite(kinematics->wheelbaseMm))
        return false;
    const uint32_t count = sizeof(responseHeading)/sizeof(responseHeading[0]);
    float extra = (finalYawRad-responseHeading[count-1U].yawRad)/RESPONSE_STEADY_CURVATURE;
    if (extra < -0.001f || extra > 500.0f) return false;
    extra = extra < .001f ? 0.0f : extra;
    bool inserted = false;
    for (uint32_t i = 0; i < count; ++i)
    {
        if (plan->nodeCount >= MOTION_RESPONSE_NODE_CAPACITY) return false;
        float s = responseHeading[i].progressMm;
        float yaw = responseHeading[i].yawRad;
        if (inserted) { s += extra; yaw += extra*RESPONSE_STEADY_CURVATURE; }
        MotionResponseNode *node = &plan->nodes[plan->nodeCount];
        node->progressMm = s;
        node->pose = plan->nodeCount == 0 ? (MotionResponsePose){0,0,yaw}
            : Advance(plan->nodes[plan->nodeCount-1U].pose,
                      s-plan->nodes[plan->nodeCount-1U].progressMm, yaw);
        ++plan->nodeCount;
        if (!inserted && fabsf(s-ARC_END_MM) < .001f)
        {
            inserted = true;
            if (extra > 0.0f)
            {
                if (plan->nodeCount >= MOTION_RESPONSE_NODE_CAPACITY) return false;
                MotionResponseNode *extension = &plan->nodes[plan->nodeCount++];
                extension->progressMm = s+extra;
                extension->pose = Advance(node->pose, extra, yaw+extra*RESPONSE_STEADY_CURVATURE);
            }
        }
    }
    if (!inserted) return false;
    plan->extraArcMm = extra;
    plan->totalDistanceMm = BASE_END_MM+extra;
    plan->requestedYawRad = finalYawRad;
    plan->prepared = true;
    return true;
}

static uint32_t Interval(const MotionResponsePlan *plan, float progress)
{
    uint32_t lo = 0U, hi = plan->nodeCount-1U;
    while (hi-lo > 1U)
    {
        uint32_t mid = lo+(hi-lo)/2U;
        if (plan->nodes[mid].progressMm <= progress) lo = mid; else hi = mid;
    }
    return lo;
}

bool MotionResponsePlan_Pose(const MotionResponsePlan *plan, float progressMm,
                             MotionResponsePose *pose)
{
    if (plan == NULL || !plan->prepared || pose == NULL || !isfinite(progressMm)) return false;
    float s = Clamp(progressMm, 0, plan->totalDistanceMm);
    uint32_t i = Interval(plan, s);
    const MotionResponseNode *a = &plan->nodes[i], *b = &plan->nodes[i+1U];
    float fraction = (s-a->progressMm)/(b->progressMm-a->progressMm);
    *pose = Advance(a->pose, s-a->progressMm,
        a->pose.yawRad+fraction*(b->pose.yawRad-a->pose.yawRad));
    return true;
}

uint8_t MotionResponsePlan_Phase(const MotionResponsePlan *plan, float progressMm)
{
    if (plan == NULL || !plan->prepared || !isfinite(progressMm)) return 0;
    if (progressMm < LEAD_MM) return 1;
    if (progressMm < LEAD_MM+RAMP_MM) return 2;
    if (progressMm < ARC_END_MM+plan->extraArcMm) return 3;
    if (progressMm < EXIT_END_MM+plan->extraArcMm) return 4;
    if (progressMm < RECOVERY_END_MM+plan->extraArcMm) return 5;
    return 6;
}

bool MotionResponsePlan_Evaluate(const void *context, float progressMm,
                                MotionPathSample *sample)
{
    const MotionResponsePlan *plan = context;
    MotionResponsePose pose;
    if (sample == NULL || !MotionResponsePlan_Pose(plan, progressMm, &pose)) return false;
    float s = Clamp(progressMm, 0, plan->totalDistanceMm);
    uint32_t i = Interval(plan, s);
    const MotionResponseNode *a = &plan->nodes[i], *b = &plan->nodes[i+1U];
    uint8_t phase = MotionResponsePlan_Phase(plan, s);
    float raw = phase == 2 ? END_RAW*(s-LEAD_MM)/RAMP_MM
        : phase == 3 ? END_RAW
        : phase == 4 ? END_RAW*(EXIT_END_MM+plan->extraArcMm-s)/RAMP_MM : 0;
    raw = Clamp(raw, END_RAW, 0);
    float wheelCurvature = 0;
    for (uint32_t k = 1U; k < sizeof(wheelKnots)/sizeof(wheelKnots[0]); ++k)
        if (raw >= wheelKnots[k].raw)
        {
            float f = (raw-wheelKnots[k-1U].raw)/(wheelKnots[k].raw-wheelKnots[k-1U].raw);
            wheelCurvature = wheelKnots[k-1U].curvature+
                f*(wheelKnots[k].curvature-wheelKnots[k-1U].curvature);
            break;
        }
    *sample = (MotionPathSample){
        .curvaturePerMm = (b->pose.yawRad-a->pose.yawRad)/(b->progressMm-a->progressMm),
        .desiredYawRad = pose.yawRad,
        .speedLimitCps = MOTION_RESPONSE_SPEED_CPS,
        .straightTuningWeight = 1.0f-wheelCurvature*MOTION_RESPONSE_RADIUS_MM,
        .actuationProvided = true, .steeringFeedforwardCommand = raw,
        .wheelReferenceCurvaturePerMm = wheelCurvature,
        .profileLimitsProvided = true,
        .accelerationMmps2 = MOTION_RESPONSE_ACCELERATION_MMPS2,
        .decelerationMmps2 = MOTION_RESPONSE_ACCELERATION_MMPS2,
        .brakingSpeedLimitCps = sqrtf(2.0f*MOTION_RESPONSE_ACCELERATION_MMPS2*
            fmaxf(0.0f, plan->totalDistanceMm-s))/(PI*65.5f/1560.0f)
    };
    sample->straightTuningWeight = Clamp(sample->straightTuningWeight, 0, 1);
    return true;
}
