#include "Task2ObstacleDetection.h"
#include <math.h>
#define PI 3.14159265358979323846f

bool Task2ObstacleDetection_ValidConfig(const Task2ObstacleSensorConfig *c)
{
    if (!c || !c->ultrasonicEnabled || !c->samplePeriodMs || !c->acquisitionTimeoutMs ||
        !c->presentSamples || !c->absentSamples || c->presentSamples > 100U || c->absentSamples > 100U ||
        !isfinite(c->ultrasonicBeamHalfAngleDeg) || c->ultrasonicBeamHalfAngleDeg < 0.0f ||
        c->ultrasonicBeamHalfAngleDeg >= 45.0f || !isfinite(c->edgeUncertaintyMm) ||
        c->edgeUncertaintyMm < 0.0f || !isfinite(c->ultrasonicRangeToleranceMm) ||
        c->ultrasonicRangeToleranceMm <= 0.0f || c->irPresentAdc > 4095U ||
        c->irPresentAdc <= c->irAbsentAdc)
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

void Task2ObstacleDetection_Observe(Task2ObstacleDetection *s,
    const Task2ObstacleSensorConfig *c, const Task2RouteConfig *r,
    Task2ObstacleSensor sensor, const Task2ObstacleObservation *o, bool departing)
{
    if (!s || !c || !r || !o || sensor >= TASK2_SENSOR_COUNT || s->detected || s->fault)
        return;
    if (o->fault || !isfinite(o->pose.xMm) || !isfinite(o->pose.yMm) ||
        !isfinite(o->pose.yawRad) || (o->valid && !isfinite(o->value)))
    {
        s->fault = true;
        return;
    }
    if (sensor != TASK2_SENSOR_ULTRASONIC &&
        sensor != (r->secondArrow == TASK2_ARROW_LEFT ? TASK2_SENSOR_RIGHT_IR : TASK2_SENSOR_LEFT_IR))
        return;
    const Task2SensorMount *m = sensor == TASK2_SENSOR_ULTRASONIC ? &c->ultrasonic :
        sensor == TASK2_SENSOR_LEFT_IR ? &c->leftIR : &c->rightIR;
    Task2RoutePose p = SensorPose(o->pose, m);
    float face = r->parkToObstacle1Mm + 100.0f + r->obstacleFaceGapMm;
    bool present = false, absent = false;
    if (sensor == TASK2_SENSOR_ULTRASONIC)
    {
        float half = c->ultrasonicBeamHalfAngleDeg * PI / 180.0f;
        float angle = fabsf(p.yawRad), dx = face - p.xMm;
        float near = fmaxf(0.0f, angle - half), far = angle + half;
        float minRange = dx / cosf(near);
        float maxRange = far < PI * 0.5f ? dx / cosf(far) : INFINITY;
        present = o->valid && dx > 0.0f && near < PI * 0.5f && o->value >= 20.0f && o->value <= 4000.0f &&
            o->value >= minRange - c->ultrasonicRangeToleranceMm &&
            o->value <= maxRange + c->ultrasonicRangeToleranceMm;
        absent = !present;
        if (present)
            s->lastUltrasonicRangeMm = o->value;
    }
    else
    {
        /* An ADC error cannot be used as an absent-face measurement. */
        if (!o->valid || o->value < 0.0f || o->value > 4095.0f)
        {
            s->fault = true;
            return;
        }
        if (cosf(p.yawRad) <= 0.05f || face <= p.xMm)
            return;
        present = o->value >= c->irPresentAdc;
        absent = o->value <= c->irAbsentAdc;
    }
    if (present)
    {
        s->absent[sensor] = 0U;
        if (s->present[sensor] < c->presentSamples)
            ++s->present[sensor];
        s->armed[sensor] |= s->present[sensor] >= c->presentSamples;
        return;
    }
    s->present[sensor] = 0U;
    if (!absent || !departing || !s->armed[sensor])
    {
        s->absent[sensor] = 0U;
        return;
    }
    if (s->absent[sensor]++ == 0U)
        s->firstAbsentPose[sensor] = o->pose;
    if (s->absent[sensor] < c->absentSamples)
        return;
    p = SensorPose(s->firstAbsentPose[sensor], m);
    /* The inward edge of the cone is the last ray able to see the face.
     * A pencil-ray model is obtained by setting half-angle to zero. */
    float ray = p.yawRad - r->secondArrow * (sensor == TASK2_SENSOR_ULTRASONIC
        ? c->ultrasonicBeamHalfAngleDeg * PI / 180.0f : 0.0f);
    if (cosf(ray) <= 0.05f || face <= p.xMm)
        return;
    float edge = r->secondArrow * (p.yMm + (face - p.xMm) * tanf(ray));
    if (!isfinite(edge) || edge < 150.0f)
        return;
    s->observedWidthMm = 2.0f * (edge + c->edgeUncertaintyMm);
    s->edgePose = s->firstAbsentPose[sensor];
    s->source = sensor;
    s->detected = true;
}
