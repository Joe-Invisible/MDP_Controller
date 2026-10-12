/* Host-only fixture: wheel targets and geometric yaw are tracked perfectly.
 * This checks execution plumbing, not physical steering/tyre accuracy. */
static Task2RoutePose o1IdealPose;
static Obstacle1RouteReport o1IdealReports[2];
static FILE *o1Trace;
static void trackObstacle1Ideal(float distance, float curvature)
{
    unsigned run = hardwareStartCount - 1;
    float middle = o1IdealPose.yawRad + 0.5f * distance * curvature;
    o1IdealPose.xMm += distance * cosf(middle);
    o1IdealPose.yMm += distance * sinf(middle);
    o1IdealPose.yawRad += distance * curvature;
    (void)Obstacle1Route_CheckPose(&obstacle1RouteConfig, &o1IdealPose, &o1IdealReports[run]);
    o1IdealReports[run].finishPose = o1IdealPose;
    if (o1Trace)
        fprintf(o1Trace, "%u,%u,%.6f,%.6f,%.6f\n", run, fakeTick,
                o1IdealPose.xMm, o1IdealPose.yMm, o1IdealPose.yawRad);
}
static void checkObstacle1Ready(void)
{
    unsigned run = hardwareStartCount;
    const Obstacle1Route *route = &motionSequenceFusionTestObstacle1Routes[run];
    assert(route->report.valid);
    assert(motionSequenceFusionTestSequence.plan.count == 4);
    assert(strstr(oledRows[1], "O1:") && strstr(oledRows[3], "front:"));
    o1IdealPose = (Task2RoutePose){TASK2_ROUTE_START_X_MM, 0, 0};
    o1IdealReports[run] = (Obstacle1RouteReport){
        .minimumObstacleGapMm = INFINITY, .minimumSideMarginMm = INFINITY,
        .minimumParkWallMarginMm = INFINITY,
    };
}
static void resetO1Hardware(unsigned scenario)
{
    hardwareScenario = scenario;
    hardwareStartCount = 0;
    hardwareTrialStartTick = fakeTick = 0;
    leftCountFraction = rightCountFraction = 0;
    hardwareFixture = NULL;
}
static void testO1(void)
{
    const Obstacle1RouteConfig original = obstacle1RouteConfig;
    const Task2RouteConfig oldTask2 = task2RouteConfig;
    const float positions[] = {900, 1050, 1200};
    const float buffers[] = {100, 150};
    unsigned pairs = 0;
    for (unsigned position = 0; position < 3; ++position)
    for (unsigned buffer = 0; buffer < 2; ++buffer)
    for (int arrow = -1; arrow <= 1; arrow += 2)
    {
        obstacle1RouteConfig = original;
        obstacle1RouteConfig.parkToObstacleMm = positions[position];
        obstacle1RouteConfig.obstacleBufferMm = buffers[buffer];
        obstacle1RouteConfig.arrow = arrow;
        motionSequenceFusionTestExperiment = MOTION_SEQUENCE_FUSION_TEST_OBSTACLE1_BYPASS;
        resetO1Hardware(0);
        const char *traceDir = getenv("ANCHORED_TRACE_DIR");
        if (traceDir && positions[position] == original.parkToObstacleMm && buffers[buffer] == original.obstacleBufferMm)
        {
            char filename[1024];
            snprintf(filename, sizeof(filename), "%s/%c.csv", traceDir, arrow == 1 ? 'L' : 'R');
            o1Trace = fopen(filename, "w");
            assert(o1Trace);
        }
        MotionSequenceFusionTestRun();
        if (o1Trace) { fclose(o1Trace); o1Trace = NULL; }
        if (!motionSequenceFusionTestPassed)
            fprintf(stderr, "Rejected O1 %.0f buffer%.0f %d: %s / %s status%u\n",
                positions[position], buffers[buffer], arrow,
                motionSequenceFusionTestObstacle1Routes[0].report.reason,
                motionSequenceFusionTestObstacle1Routes[1].report.reason,
                motionSequenceFusionTestStatus);
        assert(motionSequenceFusionTestPassed && motionSequenceFusionTestComparisonValid);
        assert(hardwareStartCount == 2 && motionSequenceFusionTestArcExitCount == 3);
        assert(motionSequenceFusionTestResults[0].completedRuns == 1);
        assert(motionSequenceFusionTestResults[1].completedRuns == 4);
        for (unsigned run = 0; run < 2; ++run)
        {
            const MotionSequenceFusionTestResult *result = &motionSequenceFusionTestResults[run];
            const Obstacle1RouteReport *report = &motionSequenceFusionTestObstacle1Routes[run].report;
            assert(report->minimumObstacleGapMm >= buffers[buffer]);
            assert(report->entryBlendExitCameraRangeMm >= 500);
            assert(fabsf(report->finishPose.yMm) < 0.5f && fabsf(report->finishPose.yawRad) < 1e-4f);
            assert(result->approachRecognitionReceived && result->obstacle1ExitPoseEstimateValid);
            assert(Obstacle1Route_CameraVisible(&obstacle1RouteConfig, &result->approachRecognitionPoseEstimate));
            assert(o1IdealReports[run].minimumObstacleGapMm > 0);
            assert(o1IdealReports[run].minimumSideMarginMm > 0);
            assert(o1IdealReports[run].minimumParkWallMarginMm > 0);
            assert(o1IdealReports[run].finishPose.xMm > obstacle1RouteConfig.parkToObstacleMm + 250);
            assert(fabsf(o1IdealReports[run].finishPose.yMm) < 25);
            assert(fabsf(o1IdealReports[run].finishPose.yawRad) < 0.04f);
            bool accelerated = false;
            for (unsigned i = 0; i < motionSequenceFusionTestLogCount; ++i)
            {
                const MotionSequenceFusionTestSample *sample = &motionSequenceFusionTestLog[i];
                if (sample->comparisonRunIndex != run) continue;
                if (sample->timeMs < result->approachRecognitionElapsedMs)
                    assert(sample->targetSpeedCps <= 2000.5f);
                if (sample->timeMs > result->approachRecognitionElapsedMs &&
                    sample->sequenceTravelMm < motionSequenceFusionTestObstacle1Routes[run].steps[0].signedDistanceMm + 250 &&
                    sample->targetSpeedCps > 2100) accelerated = true;
            }
            assert(accelerated);
        }
        for (unsigned j = 0; j < 3; ++j)
            near(motionSequenceFusionTestFusedPlan.junctionHalfLengthMm[j], 250, 0.01f);
        printf("O1 front%.0f buffer%.0f %c: A=%lu B=%lu ms; nominal gap %.2f / %.2f; entry-end range %.2f\n",
            positions[position], buffers[buffer], arrow == 1 ? 'L' : 'R',
            (unsigned long)motionSequenceFusionTestResults[0].elapsedMs,
            (unsigned long)motionSequenceFusionTestResults[1].elapsedMs,
            motionSequenceFusionTestObstacle1Routes[0].report.minimumObstacleGapMm,
            motionSequenceFusionTestObstacle1Routes[1].report.minimumObstacleGapMm,
            motionSequenceFusionTestObstacle1Routes[0].report.entryBlendExitCameraRangeMm);
        if (positions[position] == 1050 && buffers[buffer] == 100 && arrow == 1)
        {
            for (unsigned i = 0; i < 4; ++i)
                printf("Default step %u: A %.2f mm / B %.2f mm, curvature %.6f\n", i,
                    motionSequenceFusionTestObstacle1Routes[0].steps[i].signedDistanceMm,
                    motionSequenceFusionTestObstacle1Routes[1].steps[i].signedDistanceMm,
                    motionSequenceFusionTestObstacle1Routes[0].steps[i].curvaturePerMm);
            printf("Default finish: nominal (%.2f, %.2f), ideal (%.2f, %.2f) / (%.2f, %.2f) mm\n",
                motionSequenceFusionTestObstacle1Routes[0].report.finishPose.xMm,
                motionSequenceFusionTestObstacle1Routes[0].report.finishPose.yMm,
                o1IdealReports[0].finishPose.xMm, o1IdealReports[0].finishPose.yMm,
                o1IdealReports[1].finishPose.xMm, o1IdealReports[1].finishPose.yMm);
        }
        ++pairs;
    }
    obstacle1RouteConfig = original;
    assert(memcmp(&task2RouteConfig, &oldTask2, sizeof(oldTask2)) == 0);
    for (unsigned invalid = 0; invalid < 6; ++invalid)
    {
        obstacle1RouteConfig = original;
        switch (invalid)
        {
        case 0: obstacle1RouteConfig.departureRadiusMm = 275; break;
        case 1: obstacle1RouteConfig.parkToObstacleMm = 600; break;
        case 2: obstacle1RouteConfig.recognitionCameraRangeMm = NAN; break;
        case 3: obstacle1RouteConfig.obstacleBufferMm = 600; break;
        case 4: obstacle1RouteConfig.arenaHalfWidthMm = 200; break;
        default: obstacle1RouteConfig.entryBlendMm = 1000; break;
        }
        resetO1Hardware(0);
        MotionSequenceFusionTestRun();
        assert(!motionSequenceFusionTestPassed && hardwareStartCount == 0);
    }
    obstacle1RouteConfig = original;
    for (unsigned scenario = 1; scenario <= 2; ++scenario)
    {
        resetO1Hardware(scenario);
        MotionSequenceFusionTestRun();
        assert(!motionSequenceFusionTestPassed && hardwareStartCount == 1);
        assert(scenario == 1 ? motionSequenceFusionTestCancelled : motionSequenceFusionTestTimedOut);
    }
    printf("PASS: %u O1 A/B pairs, both arrows, camera, acceleration, footprint, rejected routes and aborts\n", pairs);
}
int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    testO1();
    motionSequenceFusionTestExperiment = MOTION_SEQUENCE_FUSION_TEST_OBSTACLE1_BYPASS;
    return legacyRegressionMain();
}
