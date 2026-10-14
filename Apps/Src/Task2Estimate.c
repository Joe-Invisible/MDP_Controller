#include "Task2Estimate.h"
#include <math.h>

void Task2Estimate_Init(Task2Estimate *e, const Task2RouteConfig *c) {
    *e = (Task2Estimate){
        .obstacle2Width = {true, true, c->obstacle2WidthMm, 0.0f, TASK2_ESTIMATE_CONFIGURED},
    };
}
bool Task2Estimate_SetGap(Task2Estimate *e, float mm, float uncertainty, Task2EstimateSource source) {
    if (!isfinite(mm) || mm < 599.0f || mm > 1501.0f || !isfinite(uncertainty) || uncertainty < 0.0f)
        return false;
    e->faceGap = (Task2Dimension){true, false, fminf(1500.0f, fmaxf(600.0f, mm)), uncertainty, source};
    return true;
}
bool Task2Estimate_SetFace(Task2Estimate *e, unsigned obstacle, float face, float uncertainty) {
    if (!isfinite(face) || !isfinite(uncertainty) || uncertainty < 0.0f) return false;
    if (obstacle == 1U) {
        if (face < 600.0f || face > 1500.0f) return false;
        e->obstacle1Face = (Task2Dimension){true, false, face, uncertainty, TASK2_ESTIMATE_ULTRASONIC};
        return true;
    }
    if (obstacle != 2U || !e->obstacle1Face.valid) return false;
    return Task2Estimate_SetGap(e, face - e->obstacle1Face.mm - 100.0f,
        uncertainty + e->obstacle1Face.uncertaintyMm, TASK2_ESTIMATE_ULTRASONIC);
}
void Task2Estimate_SetSpan(Task2Estimate *e, float mm) {
    if (isfinite(mm) && mm > 0.0f)
        e->cameraSpan = (Task2Dimension){true, true, mm, 0.0f, TASK2_ESTIMATE_CAMERA};
}
bool Task2Estimate_SetEdge(Task2Estimate *e, const Task2ObstacleDetection *d) {
    if (!d->detected || !isfinite(d->observedWidthMm) || d->observedWidthMm < 300.0f) return false;
    float mm = d->observedWidthMm;
    /* Span is a lower bound only. Retain a larger configured planning footprint. */
    if (e->cameraSpan.valid) mm = fmaxf(mm, e->cameraSpan.mm);
    e->obstacle2Width = (Task2Dimension){true, true, mm, 20.0f,
        d->source == TASK2_SENSOR_ULTRASONIC ? TASK2_ESTIMATE_ULTRASONIC : TASK2_ESTIMATE_IR};
    return true;
}
bool Task2Estimate_Apply(const Task2Estimate *e, Task2RouteConfig *c) {
    if (!e->obstacle1Face.valid || !e->faceGap.valid || !e->obstacle2Width.valid) return false;
    float arenaHalfWidth = 0.5f * c->obstacle2WidthMm + c->obstacle2EndSpaceMm;
    c->parkToObstacle1Mm = e->obstacle1Face.mm;
    c->obstacleFaceGapMm = e->faceGap.mm;
    c->obstacle2WidthMm = fmaxf(c->obstacle2WidthMm, e->obstacle2Width.mm);
    if (e->cameraSpan.valid) c->obstacle2WidthMm = fmaxf(c->obstacle2WidthMm, e->cameraSpan.mm);
    c->obstacle2EndSpaceMm = arenaHalfWidth - 0.5f * c->obstacle2WidthMm;
    return c->obstacle2EndSpaceMm > c->boundaryBufferMm;
}
