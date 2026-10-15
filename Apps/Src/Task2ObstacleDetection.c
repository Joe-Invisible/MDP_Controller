#include "Task2ObstacleDetection.h"
#include <math.h>

#ifndef TASK2_IR_TREND_EDGE
#define TASK2_IR_TREND_EDGE 0
#endif
#define PI 3.14159265358979323846f
/* Consecutive qualifying IR samples whose smallest hit counts as proof. A USS sample already needs its
 * median window and its own range to match the face, and uses the cone's innermost ray. */
#ifndef FACE_PROOF_IR_SAMPLES
#define FACE_PROOF_IR_SAMPLES 3U
#endif
#ifndef FACE_PROOF_USS_SAMPLES
#define FACE_PROOF_USS_SAMPLES 1U
#endif
/* Largest half-length the USS alone may prove; unlimited unless an experiment sets it. */
#ifndef FACE_PROOF_USS_MAX_HALF_MM
#define FACE_PROOF_USS_MAX_HALF_MM INFINITY
#endif
_Static_assert(FACE_PROOF_IR_SAMPLES >= 1U && FACE_PROOF_IR_SAMPLES <= 8U, "faceHitMm holds 8 hits");
_Static_assert(FACE_PROOF_USS_SAMPLES >= 1U && FACE_PROOF_USS_SAMPLES <= 8U, "faceHitMm holds 8 hits");
/* Rules: O2 is at least 300 mm long and nothing stands within 500 mm past its ends. */
#define RULE_O2_MIN_HALF_MM 150.0f
#define RULE_O2_END_CLEARANCE_MM 500.0f
/* Side-IR range to the face within which our sensors read a face well above the present level (about 750
 * at 450 mm on the 30 Sep calibration). */
#define IR_FACE_RANGE_MM 450.0f
/* Odometry error allowed between a face sample's capture pose and the true robot pose. */
#ifndef FACE_EXTENT_POSE_MARGIN_MM
#define FACE_EXTENT_POSE_MARGIN_MM 20.0f
#endif

bool Task2ObstacleDetection_ValidConfig(const Task2ObstacleSensorConfig *c)
{
    if (!c || !c->ultrasonicEnabled || !c->samplePeriodMs || !c->acquisitionTimeoutMs ||
        !c->presentSamples || !c->absentSamples || c->presentSamples > 100U || c->absentSamples > 100U ||
        !isfinite(c->ultrasonicBeamHalfAngleDeg) || c->ultrasonicBeamHalfAngleDeg < 0.0f ||
        c->ultrasonicBeamHalfAngleDeg >= 45.0f || !isfinite(c->edgeUncertaintyMm) ||
        c->edgeUncertaintyMm < 0.0f || !isfinite(c->ultrasonicRangeToleranceMm) ||
        c->ultrasonicRangeToleranceMm <= 0.0f || c->irPresentAdc > 4095U ||
        c->irPresentAdc <= c->irAbsentAdc || c->irFaceDropAdc > 4095U || !isfinite(c->irAcquisitionMinYawDeg) ||
        c->irAcquisitionMinYawDeg < 0.0f || c->irAcquisitionMinYawDeg > 90.0f ||
        !c->irTrendSamples || c->irTrendSamples > 100U || !c->irTrendMinDropAdc ||
        c->irTrendMinDropAdc > 4095U || c->irTrendReboundAdc >= c->irTrendMinDropAdc ||
        !isfinite(c->irTrendMinTravelMm) || c->irTrendMinTravelMm <= 0.0f)
        return false;
    if (!c->ultrasonicApproachMedianSamples ||
        c->ultrasonicApproachMedianSamples > TASK2_USS_HISTORY_CAPACITY ||
        !(c->ultrasonicApproachMedianSamples & 1U) ||
        !c->ultrasonicDepartureMedianSamples ||
        c->ultrasonicDepartureMedianSamples > TASK2_USS_HISTORY_CAPACITY ||
        !(c->ultrasonicDepartureMedianSamples & 1U) ||
        !isfinite(c->ultrasonicJumpMm) || c->ultrasonicJumpMm <= c->ultrasonicRangeToleranceMm ||
        !isfinite(c->ultrasonicMaxDepartureYawDeg) ||
        c->ultrasonicMaxDepartureYawDeg <= 0.0f || c->ultrasonicMaxDepartureYawDeg >= 90.0f)
        return false;
    const Task2SensorMount *mounts[] = {&c->ultrasonic, &c->leftIR, &c->rightIR};
    for (unsigned i = 0U; i < TASK2_SENSOR_COUNT; ++i)
        if (!isfinite(mounts[i]->aheadMm) || !isfinite(mounts[i]->leftMm) ||
            !isfinite(mounts[i]->yawDeg) || fabsf(mounts[i]->yawDeg) > 180.0f)
            return false;
    return true;
}

static Task2RoutePose SensorPose(Task2RoutePose p, const Task2SensorMount *m)
{
    float c = cosf(p.yawRad), s = sinf(p.yawRad);
    p.xMm += m->aheadMm * c - m->leftMm * s;
    p.yMm += m->aheadMm * s + m->leftMm * c;
    p.yawRad += m->yawDeg * PI / 180.0f;
    return p;
}

/* A face sample along this ray shows O2 reaches this far from its centre line. The smallest hit of
 * `samples` consecutive qualifying samples counts, so one optimistic sample cannot raise the bound. */
static void ProveFaceExtent(Task2ObstacleDetection *s, const Task2ObstacleSensorConfig *c,
    const Task2RouteConfig *r, Task2RoutePose ray, unsigned samples, float maxHalfMm)
{
    float face = r->parkToObstacle1Mm + 100.0f + r->obstacleFaceGapMm;
    if (cosf(ray.yawRad) <= 0.05f || face <= ray.xMm)
    {
        s->faceHitCount = 0U;
        return;
    }
    float hit = r->secondArrow * (ray.yMm + (face - ray.xMm) * tanf(ray.yawRad)) -
        c->edgeUncertaintyMm - FACE_EXTENT_POSE_MARGIN_MM;
    if (!isfinite(hit))
    {
        s->faceHitCount = 0U;
        return;
    }
    s->faceHitMm[s->faceHitCount++ % samples] = hit;
    if (s->faceHitCount < samples)
        return;
    float proven = INFINITY;
    for (unsigned i = 0U; i < samples; ++i)
        proven = fminf(proven, s->faceHitMm[i]);
    proven = fminf(proven, maxHalfMm);
    if (proven > s->faceExtentLowerMm)
        s->faceExtentLowerMm = proven;
}

void Task2ObstacleDetection_BeginUSSDeparture(Task2ObstacleDetection *s,
    const Task2ObstacleSensorConfig *c)
{
    if (!s || !c)
        return;
    s->ultrasonicDepartureStarted = true;
    s->ultrasonicHistoryCount = c->ultrasonicDepartureMedianSamples;
    for (unsigned i = 0U; i < c->ultrasonicDepartureMedianSamples; ++i)
        s->ultrasonicHistoryResidualMm[i] = s->ultrasonicBaselineResidualMm;
    s->ultrasonicJumpSamples = s->ultrasonicNoEchoSamples = 0U;
    s->ultrasonicJumpCandidateValid = false;
    s->absent[TASK2_SENSOR_ULTRASONIC] = 0U;
}

static bool EstimateUSSEdge(Task2ObstacleDetection *s, const Task2ObstacleSensorConfig *c,
    const Task2RouteConfig *r, Task2RoutePose pose, Task2EdgeMethod method)
{
    Task2RoutePose p = SensorPose(pose, &c->ultrasonic);
    float face = r->parkToObstacle1Mm + 100.0f + r->obstacleFaceGapMm;
    float ray = p.yawRad - r->secondArrow * c->ultrasonicBeamHalfAngleDeg * PI / 180.0f;
    if (cosf(ray) <= 0.05f || face <= p.xMm)
        return false;
    float edge = r->secondArrow * (p.yMm + (face - p.xMm) * tanf(ray));
    if (!isfinite(edge) || edge <= 0.0f)
        return false;
    s->observedWidthMm = fmaxf(300.0f, 2.0f * (edge + c->edgeUncertaintyMm));
    s->edgePose = pose;
    s->edgeMethod = method;
    s->source = TASK2_SENSOR_ULTRASONIC;
    s->detected = true;
    return true;
}

static void ObserveUSS(Task2ObstacleDetection *s, const Task2ObstacleSensorConfig *c,
    const Task2RouteConfig *r, const Task2ObstacleObservation *o, bool departing)
{
    if (o->valid)
    {
        s->lastUltrasonicRangeMm = o->value;
        if (o->value < 20.0f || o->value > 4000.0f)
        {
            s->fault = true;
            return;
        }
    }
    Task2RoutePose p = SensorPose(o->pose, &c->ultrasonic);
    bool active = departing && s->ultrasonicDepartureStarted;
    if (active && fabsf(p.yawRad) * 180.0f / PI > c->ultrasonicMaxDepartureYawDeg)
    {
        s->ultrasonicHeadingExceeded = s->fault = true;
        return; /* Oblique readings must not masquerade as an O2 edge. */
    }
    if (!o->valid)
    {
        s->present[0] = 0U;
        if (!active || !s->armed[0])
        {
            s->ultrasonicHistoryCount = 0U;
            s->ultrasonicNoEchoSamples = 0U;
            return;
        }
        if (s->ultrasonicNoEchoSamples++ == 0U)
            s->ultrasonicNoEchoPose = s->firstAbsentPose[0] =
                s->ultrasonicJumpCandidateValid ? s->ultrasonicJumpPose : o->pose;
        s->absent[0] = s->ultrasonicNoEchoSamples;
        if (s->ultrasonicNoEchoSamples >= c->absentSamples)
            EstimateUSSEdge(s, c, r, s->ultrasonicNoEchoPose, TASK2_EDGE_USS_NO_ECHO);
        return; /* No echo is not a fabricated long-range measurement. */
    }
    bool precedingNoEcho = s->ultrasonicNoEchoSamples > 0U;
    Task2RoutePose noEchoPose = s->ultrasonicNoEchoPose;
    s->ultrasonicNoEchoSamples = s->absent[0] = 0U;
    float face = r->parkToObstacle1Mm + 100.0f + r->obstacleFaceGapMm;
    float dx = face - p.xMm, half = c->ultrasonicBeamHalfAngleDeg * PI / 180.0f;
    float near = fmaxf(0.0f, fabsf(p.yawRad) - half), far = fabsf(p.yawRad) + half;
    if (dx <= 0.0f || far >= PI * 0.5f)
        return;
    float expected = dx / cosf(near), upper = dx / cosf(far);
    /* Median of range residuals: every sample uses its own trigger pose. This
     * avoids mixing older, longer approach ranges with the current pose. */
    float residual = o->value - expected;
    unsigned n = active ? c->ultrasonicDepartureMedianSamples : c->ultrasonicApproachMedianSamples;
    s->ultrasonicHistoryResidualMm[s->ultrasonicHistoryCount % n] = residual;
    ++s->ultrasonicHistoryCount;
    if (s->ultrasonicHistoryCount < n)
        return;
    float sorted[TASK2_USS_HISTORY_CAPACITY];
    for (unsigned i = 0U; i < n; ++i)
    {
        unsigned j = i;
        float value = s->ultrasonicHistoryResidualMm[i];
        while (j && sorted[j - 1U] > value)
        {
            sorted[j] = sorted[j - 1U];
            --j;
        }
        sorted[j] = value;
    }
    float filtered = sorted[n / 2U];
    s->ultrasonicFilteredResidualMm = filtered;
    bool present = filtered >= -c->ultrasonicRangeToleranceMm &&
        filtered <= upper - expected + c->ultrasonicRangeToleranceMm;
    if (active && s->armed[0])
    {
        bool rawJump = residual >= s->ultrasonicBaselineResidualMm + c->ultrasonicJumpMm;
        if (rawJump && !s->ultrasonicJumpCandidateValid)
        {
            s->ultrasonicJumpPose = s->firstAbsentPose[0] = precedingNoEcho ? noEchoPose : o->pose;
            s->ultrasonicJumpCandidateValid = true;
        }
        else if (!rawJump)
            s->ultrasonicJumpCandidateValid = false;
        if (rawJump && filtered >= s->ultrasonicBaselineResidualMm + c->ultrasonicJumpMm)
        {
            s->absent[0] = ++s->ultrasonicJumpSamples;
            if (s->ultrasonicJumpSamples >= c->absentSamples)
                EstimateUSSEdge(s, c, r, s->ultrasonicJumpPose, TASK2_EDGE_USS_RANGE_JUMP);
            return;
        }
        s->ultrasonicJumpSamples = 0U;
    }
    if (present)
    {
        if (s->present[0] < c->presentSamples)
            ++s->present[0];
        s->armed[0] |= s->present[0] >= c->presentSamples;
        s->ultrasonicBaselineResidualMm = filtered;
        bool rawPresent = residual >= -c->ultrasonicRangeToleranceMm &&
            residual <= upper - expected + c->ultrasonicRangeToleranceMm;
        /* Past the proven end plus the rules' clearance a wall or obstacle may stand. If any ray of
         * the cone reaches there at this range, the echo may be that, not O2's face. */
        float proven = fmaxf(s->faceExtentLowerMm, RULE_O2_MIN_HALF_MM);
        float outward = fminf(0.5f * PI, r->secondArrow * p.yawRad + half);
        bool unambiguous = r->secondArrow * p.yMm + o->value * sinf(outward) <
            proven + RULE_O2_END_CLEARANCE_MM - FACE_EXTENT_POSE_MARGIN_MM;
        if (s->armed[0] && rawPresent && unambiguous)
        {
            Task2RoutePose inner = p;
            inner.yawRad -= r->secondArrow * half;
            ProveFaceExtent(s, c, r, inner, FACE_PROOF_USS_SAMPLES, FACE_PROOF_USS_MAX_HALF_MM);
        }
        else
            s->faceHitCount = 0U;
    }
    else
    {
        s->present[0] = 0U;
        s->faceHitCount = 0U;
    }
}

void Task2ObstacleDetection_BeginIRTraverse(Task2ObstacleDetection *s,
    const Task2RoutePose *pose)
{
    if (!s || !pose)
        return;
    s->irTraverseStarted = true;
    s->irTraverseStartPose = *pose;
    s->irHistoryCount = s->irPeakAdc = s->irDeclineSamples = 0U;
    s->irTrendCandidateValid = false;
    s->irFaceRecentCount = s->irFaceMedianCount = s->irFaceLevelAdc = 0U;
    s->reacquire = false;
    for (unsigned i = TASK2_SENSOR_LEFT_IR; i < TASK2_SENSOR_COUNT; ++i)
    {
        s->armed[i] = false;
        s->present[i] = s->absent[i] = 0U;
    }
}

#if TASK2_IR_TREND_EDGE
static void ObserveIRTrend(Task2ObstacleDetection *s, const Task2ObstacleSensorConfig *c,
    const Task2ObstacleObservation *o)
{
    unsigned index = s->irHistoryCount % 3U;
    s->irHistoryAdc[index] = (uint16_t)o->value;
    s->irHistoryPose[index] = o->pose;
    ++s->irHistoryCount;
    if (s->irHistoryCount < 3U)
        return;
    uint16_t a = s->irHistoryAdc[0], b = s->irHistoryAdc[1], d = s->irHistoryAdc[2];
    uint16_t median = a > b ? (b > d ? b : a > d ? d : a) : (a > d ? a : b > d ? d : b);
    Task2RoutePose middle = s->irHistoryPose[(s->irHistoryCount - 2U) % 3U];
    uint16_t previous = s->irFilteredAdc;
    s->irFilteredAdc = median;
    uint16_t high = a > b ? a : b;
    if (d > high) high = d;
    uint16_t low = a < b ? a : b;
    if (d < low) low = d;
    const Task2RoutePose *oldest = &s->irHistoryPose[(s->irHistoryCount - 3U) % 3U];
    float windowTravel = hypotf(o->pose.xMm - oldest->xMm, o->pose.yMm - oldest->yMm);
    /* Only near-peak settling may replace the reference. A lower plateau
     * within the minimum-drop threshold can still be part of the same edge. */
    bool newPlateau = !s->irTrendCandidateValid && median > c->irAbsentAdc &&
        s->irPeakAdc - median <= c->irTrendReboundAdc &&
        high - low <= c->irTrendReboundAdc && windowTravel >= c->irTrendMinTravelMm;
    if (!s->irPeakAdc || median >= s->irPeakAdc || newPlateau)
    {
        s->irPeakAdc = median;
        s->irPeakPose = middle;
        s->irDeclineSamples = 0U;
        s->irTrendCandidateValid = false;
        return;
    }
    if (median > previous + c->irTrendReboundAdc ||
        median + c->irTrendReboundAdc >= s->irPeakAdc)
    {
        s->irDeclineSamples = 0U;
        s->irTrendCandidateValid = false;
        return;
    }
    /* Stationary repetitions do not establish a spatial decline. */
    float travel = hypotf(middle.xMm - s->irPeakPose.xMm, middle.yMm - s->irPeakPose.yMm);
    if (travel < 1.0f)
        return;
    /* Confirm a sustained lower level, including equal medians after an abrupt
     * drop. Rebound/near-peak samples above have already reset confirmation. */
    if (s->irDeclineSamples < c->irTrendSamples)
        ++s->irDeclineSamples;
    if (s->irDeclineSamples >= c->irTrendSamples && travel >= c->irTrendMinTravelMm &&
        s->irPeakAdc - median >= c->irTrendMinDropAdc)
    {
        s->irTrendCandidateValid = true;
        s->irTrendCandidatePose = s->irPeakPose;
    }
}
#endif

static void ObserveIRTraverse(Task2ObstacleDetection *s, const Task2ObstacleSensorConfig *c,
    const Task2RouteConfig *r, Task2ObstacleSensor sensor, const Task2ObstacleObservation *o,
    const Task2SensorMount *mount)
{
    if (!o->valid || o->value < 0.0f || o->value > 4095.0f)
    {
        s->fault = true;
        return;
    }
    Task2RoutePose rayPose = SensorPose(o->pose, mount);
    float face = r->parkToObstacle1Mm + 100.0f + r->obstacleFaceGapMm;
    if (cosf(rayPose.yawRad) <= 0.05f || face <= rayPose.xMm)
        return;
    /* Width is unknown: acquisition is not restricted to the nominal face span. The IR is used as present or
     * absent only: its reading also changes with range and angle, so a falling value is no evidence of the
     * end. TASK2_IR_TREND_EDGE=1 restores the falling-trend end and the slower confirmation it brings. */
#if TASK2_IR_TREND_EDGE
    ObserveIRTrend(s, c, o);
#endif
    /* Background noise past the end reaches the fixed absent level, so once the face is seen the end is
     * any reading well below the face level: the highest of the last eight medians, so a slow drift in
     * range or angle moves it along and the drop at the end does not. A present reading is never the end:
     * the face itself can read lower in places, and the rules keep 50 cm past O2's ends clear. */
    float gone = c->irAbsentAdc;
    if (s->armed[sensor] && c->irFaceDropAdc && s->irFaceLevelAdc > c->irAbsentAdc + c->irFaceDropAdc)
        gone = fminf((float)(s->irFaceLevelAdc - c->irFaceDropAdc), (float)c->irPresentAdc - 1.0f);
    if (o->value >= c->irPresentAdc)
    {
        s->absent[sensor] = 0U;
        if (s->present[sensor] < c->presentSamples)
            ++s->present[sensor];
        s->armed[sensor] |= s->present[sensor] >= c->presentSamples;
        s->irFaceRecentAdc[s->irFaceRecentCount++ % 3U] = (uint16_t)o->value;
        if (s->armed[sensor] && s->irFaceRecentCount >= 3U)
        {
            uint16_t a = s->irFaceRecentAdc[0], b = s->irFaceRecentAdc[1], d = s->irFaceRecentAdc[2];
            uint16_t median = a > b ? (b > d ? b : a > d ? d : a) : (a > d ? a : b > d ? d : b);
            s->irFaceMedianAdc[s->irFaceMedianCount++ % 8U] = median;
            unsigned kept = s->irFaceMedianCount < 8U ? s->irFaceMedianCount : 8U;
            s->irFaceLevelAdc = 0U;
            for (unsigned i = 0U; i < kept; ++i)
                if (s->irFaceMedianAdc[i] > s->irFaceLevelAdc)
                    s->irFaceLevelAdc = s->irFaceMedianAdc[i];
        }
        if (s->armed[sensor])
            ProveFaceExtent(s, c, r, rayPose, FACE_PROOF_IR_SAMPLES, INFINITY);
        return;
    }
    s->present[sensor] = 0U;
    s->faceHitCount = 0U;
    s->irFaceRecentCount = 0U;
    /* A same-side traverse can start with the IR already past the end. Where its ray meets the face beyond
     * the face already proven, within the range at which a face reads well above present, three readings
     * short of present (the last above the absent level) ask the app to back onto the proven face and look
     * again. They set no end: beyond the
     * proof is unobserved, and a darker part of the face reads lower too. The background past the end can
     * sit above the fixed absent level, so that level alone would let the robot crawl outward to the side
     * limit. Farther out the IR cannot tell a face from the background, so nothing is concluded there. */
    float hit = r->secondArrow * (rayPose.yMm + (face - rayPose.xMm) * tanf(rayPose.yawRad));
    float range = (face - rayPose.xMm) / cosf(rayPose.yawRad);
    bool unproven = !s->armed[sensor] && r->firstArrow == r->secondArrow && isfinite(hit) &&
        range <= IR_FACE_RANGE_MM && hit > fmaxf(s->faceExtentLowerMm, RULE_O2_MIN_HALF_MM) + c->edgeUncertaintyMm;
    if (o->value > gone && !unproven)
    {
        s->absent[sensor] = 0U;
        return;
    }
    if (s->absent[sensor]++ == 0U)
        s->firstAbsentPose[sensor] = o->pose;
    if (s->absent[sensor] < c->absentSamples)
        return;
    /* At or below the fixed absent level the initial-absence bound already applies; only a reading between
     * that and present, where the robot would otherwise crawl on, asks to look again. */
    if (unproven && o->value > c->irAbsentAdc)
    {
        s->reacquire = true;
        s->absent[sensor] = 0U;
        return;
    }
    if (!s->armed[sensor])
    {
        s->edgePose = s->irTraverseStartPose;
        s->edgeMethod = TASK2_EDGE_IR_INITIAL_ABSENCE_BOUND;
    }
    else
    {
        /* The falling-trend point refines the end only if it lies beyond the face the sensors have
         * already seen; a reading can fall for other reasons (range, angle). Otherwise take the first
         * absent sample: with valid face readings before it, that is at or past the end, so it does not
         * shorten O2 (an early loss of reflection could; the app widens any end shorter than the proof). */
        s->edgePose = s->firstAbsentPose[sensor];
        s->edgeMethod = TASK2_EDGE_THRESHOLD;
        if (s->irTrendCandidateValid)
        {
            Task2RoutePose trend = SensorPose(s->irTrendCandidatePose, mount);
            float trendEdge = r->secondArrow * (trend.yMm + (face - trend.xMm) * tanf(trend.yawRad));
            if (isfinite(trendEdge) && trendEdge >= s->faceExtentLowerMm)
            {
                s->edgePose = s->irTrendCandidatePose;
                s->edgeMethod = TASK2_EDGE_IR_TREND;
            }
        }
    }
    rayPose = SensorPose(s->edgePose, mount);
    float edge = r->secondArrow * (rayPose.yMm + (face - rayPose.xMm) * tanf(rayPose.yawRad));
    if (!isfinite(edge) || edge <= 0.0f)
        return;
    s->observedWidthMm = fmaxf(300.0f, 2.0f * (edge + c->edgeUncertaintyMm));
    s->source = sensor;
    s->detected = true;
}

void Task2ObstacleDetection_Observe(Task2ObstacleDetection *s,
    const Task2ObstacleSensorConfig *c, const Task2RouteConfig *r,
    Task2ObstacleSensor sensor, const Task2ObstacleObservation *o, bool departing)
{
    if (!s || !c || !r || !o || sensor >= TASK2_SENSOR_COUNT || s->detected || s->fault)
        return;
    /* LL/RR use the forward USS. Oblique opposite-arrow departure uses IR. */
    if (sensor != TASK2_SENSOR_ULTRASONIC && r->firstArrow == r->secondArrow && !s->irTraverseStarted)
        return;
    /* Side returns during the straight approach cannot identify O2. */
    if (sensor != TASK2_SENSOR_ULTRASONIC &&
        (!departing || !c->sideIREnabled || !s->irTraverseStarted || r->secondArrow * o->pose.yawRad * 180.0f / PI < c->irAcquisitionMinYawDeg ||
         sensor != (r->secondArrow == TASK2_ARROW_LEFT ? TASK2_SENSOR_RIGHT_IR : TASK2_SENSOR_LEFT_IR)))
        return;
    if (sensor != TASK2_SENSOR_ULTRASONIC)
    {
        unsigned i = sensor - TASK2_SENSOR_LEFT_IR;
        s->irObserved[i] = true;
        s->irValid[i] = o->valid && !o->fault && isfinite(o->value) && o->value >= 0.0f && o->value <= 4095.0f;
        s->lastIRAdc[i] = s->irValid[i] ? (uint16_t)o->value : 0U;
    }
    else
    {
        s->ultrasonicObserved = true;
        s->ultrasonicValid = o->valid && !o->fault && isfinite(o->value) &&
            o->value >= 20.0f && o->value <= 4000.0f;
        if (s->ultrasonicValid)
        {
            s->lastUltrasonicRangeMm = o->value;
            s->lastUltrasonicPose = o->pose;
        }
    }
    if (o->fault || !isfinite(o->pose.xMm) || !isfinite(o->pose.yMm) ||
        !isfinite(o->pose.yawRad) || (o->valid && !isfinite(o->value)))
    {
        s->fault = true;
        return;
    }
    if (sensor == TASK2_SENSOR_ULTRASONIC && r->firstArrow == r->secondArrow)
    {
        ObserveUSS(s, c, r, o, departing);
        return;
    }
    if (sensor != TASK2_SENSOR_ULTRASONIC)
    {
        const Task2SensorMount *mount = sensor == TASK2_SENSOR_LEFT_IR ? &c->leftIR : &c->rightIR;
        ObserveIRTraverse(s, c, r, sensor, o, mount);
        return;
    }
    /* Opposite arrows retain USS acquisition on approach, but it cannot
     * terminate their oblique departure. */
    Task2RoutePose p = SensorPose(o->pose, &c->ultrasonic);
    float face = r->parkToObstacle1Mm + 100.0f + r->obstacleFaceGapMm;
    float half = c->ultrasonicBeamHalfAngleDeg * PI / 180.0f;
    float dx = face - p.xMm;
    float near = fmaxf(0.0f, fabsf(p.yawRad) - half), far = fabsf(p.yawRad) + half;
    float minRange = dx / cosf(near);
    float maxRange = far < PI * 0.5f ? dx / cosf(far) : INFINITY;
    bool present = o->valid && dx > 0.0f && near < PI * 0.5f && o->value >= 20.0f && o->value <= 4000.0f &&
        o->value >= minRange - c->ultrasonicRangeToleranceMm &&
        o->value <= maxRange + c->ultrasonicRangeToleranceMm;
    if (present)
    {
        s->lastUltrasonicRangeMm = o->value;
        if (s->present[0] < c->presentSamples)
            ++s->present[0];
        s->armed[0] |= s->present[0] >= c->presentSamples;
    }
    else
        s->present[0] = 0U;
}
