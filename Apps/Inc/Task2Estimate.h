#ifndef TASK2_ESTIMATE_H
#define TASK2_ESTIMATE_H
#include "Task2Route.h"
#include "Task2ObstacleDetection.h"

typedef enum { TASK2_ESTIMATE_CONFIGURED, TASK2_ESTIMATE_ULTRASONIC,
    TASK2_ESTIMATE_IR, TASK2_ESTIMATE_CAMERA } Task2EstimateSource;
typedef struct {
    bool valid;
    bool bound; /* A bound must never be reported as an exact measurement. */
    float mm, uncertaintyMm;
    Task2EstimateSource source;
} Task2Dimension;
typedef struct {
    Task2Dimension obstacle1Face, faceGap, obstacle2Width, cameraSpan;
} Task2Estimate;

/* Estimators consume observations; the planner receives only geometry. */
void Task2Estimate_Init(Task2Estimate *e, const Task2RouteConfig *defaults);
bool Task2Estimate_SetFace(Task2Estimate *e, unsigned obstacle, float faceMm, float uncertaintyMm);
bool Task2Estimate_SetGap(Task2Estimate *e, float gapMm, float uncertaintyMm, Task2EstimateSource source);
void Task2Estimate_SetSpan(Task2Estimate *e, float spanMm);
bool Task2Estimate_SetEdge(Task2Estimate *e, const Task2ObstacleDetection *edge);
bool Task2Estimate_Apply(const Task2Estimate *e, Task2RouteConfig *geometry);
#endif
