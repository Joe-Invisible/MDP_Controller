/* Appended to the existing host fixture by test_task2_anchored.py.
 * This file is not compiled into the STM32 firmware. */

static Task2RoutePose anchoredIdealPose;
static Task2RouteReport anchoredIdealReports[2];
static FILE *anchoredTrace;
static bool anchoredCameraSeen[2][2];

static void trackAnchoredIdeal(float distance, float curvature)
{
    unsigned run = hardwareStartCount - 1U;
    float middle = anchoredIdealPose.yawRad + 0.5f * distance * curvature;
    anchoredIdealPose.xMm += distance * cosf(middle);
    anchoredIdealPose.yMm += distance * sinf(middle);
    anchoredIdealPose.yawRad += distance * curvature;
    Task2RouteReport *report = &anchoredIdealReports[run];
    (void)Task2Route_CheckPose(&task2RouteConfig, &anchoredIdealPose, report);
    /* The fixture stops instantly, whereas the production yaw predictor
     * expects braking coast. Check collision/boundaries separately from the
     * nominal buffer; do not pretend this plant establishes an error bound. */
    bool clear = report->minimumObstacleGapMm > 0.0f &&
        report->minimumSideMarginMm > 0.0f && report->minimumParkWallMarginMm > 0.0f;
    if (!clear)
    {
        fprintf(stderr, "Ideal %c W%.0f gap %.3f side %.3f park %.3f at x%.1f y%.1f yaw%.2f\n",
                run == 0U ? 'A' : 'B', task2RouteConfig.obstacle2WidthMm,
                report->minimumObstacleGapMm, report->minimumSideMarginMm,
                report->minimumParkWallMarginMm, anchoredIdealPose.xMm,
                anchoredIdealPose.yMm, anchoredIdealPose.yawRad * 180.0f / PI);
        assert(clear);
    }
    if (fabsf(curvature) < 1e-8f && cosf(anchoredIdealPose.yawRad) > 0.99f)
    {
        const Task2RouteConfig *c = &task2RouteConfig;
        float cosine = cosf(anchoredIdealPose.yawRad), sine = sinf(anchoredIdealPose.yawRad);
        float offset = 0.5f * c->bodyLengthMm - c->cameraBehindFrontMm;
        float cameraX = anchoredIdealPose.xMm + offset * cosine;
        float cameraY = anchoredIdealPose.yMm + offset * sine;
        for (unsigned obstacle = 0; obstacle < 2; ++obstacle)
        {
            float face = c->parkToObstacle1Mm +
                (obstacle ? 100.0f + c->obstacleFaceGapMm : 0.0f);
            float dx = face - cameraX;
            bool visible = dx > 0.0f && hypotf(dx, cameraY) <= c->recognitionCameraRangeMm;
            for (int sign = -1; sign <= 1; sign += 2)
            {
                float dy = sign * 0.5f * c->recognitionTargetWidthMm - cameraY;
                float forward = dx * cosine + dy * sine;
                float lateral = -dx * sine + dy * cosine;
                visible = visible && forward > 0.0f && fabsf(lateral) <
                    forward * tanf(c->cameraHorizontalFovDeg * PI / 360.0f);
            }
            anchoredCameraSeen[run][obstacle] |= visible;
        }
    }
    report->finishPose = anchoredIdealPose;
    if (anchoredTrace)
        fprintf(anchoredTrace, "%u,%u,%.6f,%.6f,%.6f\n", run, fakeTick,
                anchoredIdealPose.xMm, anchoredIdealPose.yMm, anchoredIdealPose.yawRad);
}

static void checkAnchoredReady(void)
{
    unsigned run = hardwareStartCount;
    const Task2Route *route = &motionSequenceFusionTestTask2Routes[run];
    const MotionSequencePlan *plan = &motionSequenceFusionTestSequence.plan;
    assert(route->report.valid && plan->count == TASK2_ROUTE_STEP_COUNT);
    assert(strstr(oledRows[3], "O1:") && strstr(oledRows[3], "W:"));
    assert(strstr(oledRows[0], run == 0U ? "A fused" : "B stopped"));
    for (unsigned i = 0U; i < TASK2_ROUTE_STEP_COUNT; ++i)
    {
        near(plan->segments[i].signedDistanceMm, route->steps[i].signedDistanceMm, 0.001f);
        near(plan->segments[i].curvaturePerMm, route->steps[i].curvaturePerMm, 1e-7f);
        assert(plan->segments[i].stopAfter == (run == 1U));
    }
    anchoredCameraSeen[run][0] = anchoredCameraSeen[run][1] = false;
    anchoredIdealPose = (Task2RoutePose){TASK2_ROUTE_START_X_MM, 0.0f, 0.0f};
    anchoredIdealReports[run] = (Task2RouteReport){
        .minimumObstacleGapMm = INFINITY,
        .minimumSideMarginMm = INFINITY,
        .minimumParkWallMarginMm = INFINITY,
    };
}

static void resetHardwareScenario(unsigned scenario)
{
    hardwareScenario = scenario;
    hardwareStartCount = 0U;
    hardwareTrialStartTick = fakeTick = 0U;
    leftCountFraction = rightCountFraction = 0.0f;
    hardwareFixture = NULL;
}

static void assertParked(const Task2RouteConfig *config, const Task2RoutePose *pose)
{
    float ex = 0.5f * (config->bodyLengthMm * fabsf(cosf(pose->yawRad)) +
                      config->bodyWidthMm * fabsf(sinf(pose->yawRad)));
    float ey = 0.5f * (config->bodyLengthMm * fabsf(sinf(pose->yawRad)) +
                      config->bodyWidthMm * fabsf(cosf(pose->yawRad)));
    assert(pose->xMm - ex > -500.0f && pose->xMm + ex < 0.0f);
    assert(fabsf(pose->yMm) + ey < 300.0f);
}


static void testJunctionOverrides(void)
{
    Fixture f;
    MotionSequence sequence;
    begin(&f, &sequence);
    sequence.config.allowBlendOverlap = true;
    sequence.config.blendLengthMm = 500.0f;
    assert(MotionSequence_AddStraight(&sequence, 2000, 5000, false) == 0);
    assert(MotionSequence_AddArc(&sequence, 1000, 500, 5000, false) == 0);
    assert(MotionSequence_AddStraight(&sequence, 2000, 5000, false) == 0);
    sequence.plan.junctionBlendLimitMm[0] = 40.0f;
    assert(MotionSequencePlan_Prepare(&sequence.plan, &sequence.config,
                                     mmPerCount(), 3000, 3000, 2000, 2000, 900000, 480));
    near(sequence.plan.junctionHalfLengthMm[0], 20, 0);
    near(sequence.plan.junctionHalfLengthMm[1], 250, 0);
    sequence.plan.junctionBlendLimitMm[0] = 0.0f;
    assert(MotionSequencePlan_Prepare(&sequence.plan, &sequence.config,
                                     mmPerCount(), 3000, 3000, 2000, 2000, 900000, 480));
    near(sequence.plan.junctionHalfLengthMm[0], 250, 0);
    const float invalid[] = {-1.0f, NAN, INFINITY};
    for (unsigned i = 0; i < 3; ++i)
    {
        sequence.plan.junctionBlendLimitMm[0] = invalid[i];
        assert(!MotionSequencePlan_Prepare(&sequence.plan, &sequence.config,
                                          mmPerCount(), 3000, 3000, 2000, 2000, 900000, 480));
    }
    sequence.plan.junctionBlendLimitMm[0] = 40;
    sequence.plan.segments[0].stopAfter = true;
    assert(MotionSequencePlan_Prepare(&sequence.plan, &sequence.config,
                                     mmPerCount(), 3000, 3000, 2000, 2000, 900000, 480));
    near(sequence.plan.junctionHalfLengthMm[0], 0, 0);
}

static void testRejectedRoutes(void)
{
    const Task2RouteConfig original = task2RouteConfig;
    for (unsigned test = 0; test < 9; ++test)
    {
        task2RouteConfig = original;
        switch (test)
        {
        case 0: task2RouteConfig.firstArrow = (Task2Arrow)0; break;
        case 1: task2RouteConfig.obstacle2WidthMm = NAN; break;
        case 2: task2RouteConfig.obstacle2WidthMm = 299; break;
        case 3: task2RouteConfig.parkToObstacle1Mm = 599; break;
        case 4: task2RouteConfig.obstacleFaceGapMm = 800; break;
        case 5: task2RouteConfig.recognitionCameraRangeMm = 501; break;
        case 6: task2RouteConfig.cameraJunctionBlendMm = 500; break;
        case 7: task2RouteConfig.obstacle2EndSpaceMm = 100; break;
        case 8: task2RouteConfig.obstacle2WidthMm = 900; break;
        }
        resetHardwareScenario(0);
        MotionSequenceFusionTestRun();
        assert(hardwareStartCount == 0 && hardwareFixture == NULL);
        assert(!motionSequenceFusionTestPassed && !motionSequenceFusionTestComparisonValid);
        assert(motionSequenceFusionTestStatus == MOTIONCONTROLLER_STATUS_PROFILE_ERROR);
        assert(motionSequenceFusionTestResultCount == 0);
        assert(!motionSequenceFusionTestTask2Routes[0].report.valid);
        assert(!motionSequenceFusionTestTask2Routes[1].report.valid);
        assert(motionSequenceFusionTestTask2Routes[0].report.reason != NULL);
    }
    task2RouteConfig = original;
    for (unsigned scenario = 1; scenario <= 2; ++scenario)
    {
        resetHardwareScenario(scenario);
        MotionSequenceFusionTestRun();
        assert(!motionSequenceFusionTestPassed && !motionSequenceFusionTestComparisonValid);
        assert(hardwareStartCount == 1 && motionSequenceFusionTestResultCount == 1);
        assert(scenario == 1 ? motionSequenceFusionTestCancelled : motionSequenceFusionTestTimedOut);
    }
}

static void testAnchoredHarness(void)
{
    const Task2RouteConfig original = task2RouteConfig;
    const float widths[] = {300.0f, 350.0f, 400.0f, 500.0f};
    const float initialGaps[] = {600.0f, 1050.0f};
    motionSequenceFusionTestExperiment = MOTION_SEQUENCE_FUSION_TEST_TASK2_PARAMETERISED;
    const float faceGaps[] = {1050.0f, 1100.0f};
    for (unsigned spacing = 0U; spacing < 2U; ++spacing)
    {
        for (unsigned gap = 0U; gap < 2U; ++gap)
        {
            for (unsigned width = 0U; width < 4U; ++width)
            {
                for (int arrow1 = -1; arrow1 <= 1; arrow1 += 2)
                {
                    for (int arrow2 = -1; arrow2 <= 1; arrow2 += 2)
                    {
                        task2RouteConfig = original;
                        task2RouteConfig.parkToObstacle1Mm = initialGaps[gap];
                        task2RouteConfig.obstacleFaceGapMm = faceGaps[spacing];
                        task2RouteConfig.obstacle2WidthMm = widths[width];
                        task2RouteConfig.firstArrow = (Task2Arrow)arrow1;
                        task2RouteConfig.secondArrow = (Task2Arrow)arrow2;
                        resetHardwareScenario(0U);
                        const char *traceDirectory = getenv("ANCHORED_TRACE_DIR");
                        if (traceDirectory && spacing == 0U && gap == 1U && widths[width] == 500.0f)
                        {
                            char path[1024];
                            snprintf(path, sizeof(path), "%s/%c%c.csv", traceDirectory,
                                     arrow1 == 1 ? 'L' : 'R', arrow2 == 1 ? 'L' : 'R');
                            anchoredTrace = fopen(path, "w");
                            assert(anchoredTrace);
                        }
                        MotionSequenceFusionTestRun();
                        if (anchoredTrace)
                        {
                            fclose(anchoredTrace);
                            anchoredTrace = NULL;
                        }
                        if (!motionSequenceFusionTestPassed)
                        {
                            fprintf(stderr, "Rejected start%.0f gap%.0f W%.0f %d/%d: %s / %s\n",
                                    initialGaps[gap], faceGaps[spacing], widths[width], arrow1, arrow2,
                                    motionSequenceFusionTestTask2Routes[0].report.reason,
                                    motionSequenceFusionTestTask2Routes[1].report.reason);
                        }
                        assert(motionSequenceFusionTestPassed && motionSequenceFusionTestComparisonValid);
                        assert(hardwareStartCount == 2U && motionSequenceFusionTestResultCount == 2U);
                        assert(motionSequenceFusionTestResults[0].completedRuns == 1U);
                        assert(motionSequenceFusionTestResults[1].completedRuns == 8U);
                        assert(motionSequenceFusionTestArcExitCount == 6U);
                        for (unsigned run = 0U; run < 2U; ++run)
                        {
                            const MotionSequenceFusionTestResult *result = &motionSequenceFusionTestResults[run];
                            const Task2RouteReport *nominal = &motionSequenceFusionTestTask2Routes[run].report;
                            const Task2RouteReport *ideal = &anchoredIdealReports[run];
                            assert(nominal->valid && result->passed && !result->logTruncated);
                            assert(anchoredCameraSeen[run][0] && anchoredCameraSeen[run][1]);
                            assert(result->obstacle1ExitPoseEstimateValid);
                            assert(fabsf(result->obstacle1ExitPoseEstimate.yMm) < 20.0f);
                            assert(fabsf(result->obstacle1ExitPoseEstimate.yawRad) < 0.04f);
                            assert(nominal->minimumObstacleGapMm >= task2RouteConfig.obstacleBufferMm);
                            assert(ideal->minimumObstacleGapMm > 0.0f);
                            assert(ideal->minimumSideMarginMm > 0.0f);
                            assertParked(&task2RouteConfig, &ideal->finishPose);
                        }
                        printf("Anchored start%.0f gap%.0f W%.0f %c%c: A=%lu B=%lu ms; ideal gaps %.2f / %.2f\n",
                               initialGaps[gap], faceGaps[spacing], widths[width], arrow1 == 1 ? 'L' : 'R', arrow2 == 1 ? 'L' : 'R',
                               (unsigned long)motionSequenceFusionTestResults[0].elapsedMs,
                               (unsigned long)motionSequenceFusionTestResults[1].elapsedMs,
                               anchoredIdealReports[0].minimumObstacleGapMm,
                               anchoredIdealReports[1].minimumObstacleGapMm);
                    }
                }
            }
        }
    }
    task2RouteConfig = original;
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    testJunctionOverrides();
    testAnchoredHarness();
    puts("PASS: 64 anchored A/B combinations, camera views and footprints");
    testRejectedRoutes();
    puts("PASS: rejected routes, cancellation and timeout");
    motionSequenceFusionTestExperiment = MOTION_SEQUENCE_FUSION_TEST_OBSTACLE1_BYPASS;
    return legacyRegressionMain();
}
