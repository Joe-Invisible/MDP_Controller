/* Appended to the existing hardware fixture; never compiled into firmware. */
static const MotionControllerArcFeedforwardPoint localNegative[] = {
    {-0.004f, 80}, {-0.0005f, 10},
};
static const MotionControllerArcFeedforwardPoint localPositive[] = {
    {0.00005f, -5}, {0.0005f, -10}, {0.004f, -80},
};
static const MotionControllerArcConfig localCalibration = {
    .negativePoints = localNegative, .negativePointCount = 2,
    .positivePoints = localPositive, .positivePointCount = 3,
};

static void testLocalTable(void)
{
    MotionSequencePlan plan = {0};
    MotionSequenceConfig config = motionSequenceConfig;
    config.blendLengthMm = 500;
    config.allowBlendOverlap = true;
    plan.count = 2;
    plan.segments[0] = (MotionSequenceSegment){500, 0, 5000, false};
    plan.segments[1] = (MotionSequenceSegment){2500, 0.002f, 5000, false};
    MotionSequencePlan_SetFeedforward(&plan, &localCalibration, 0);
    /* An unrelated scalar worst case must not leak into a table-based plan. */
    assert(MotionSequencePlan_Prepare(&plan, &config, 1, 3000, 3000, 2000, 2000, 1e8f, 480));
    MotionSequenceRun run = {&plan, 0, 1};
    MotionPathSample a, b;
    assert(MotionSequencePlan_Evaluate(&run, 253, &a));
    near(a.speedLimitCps, 600, 0.01f); /* Actual steep positive centre bridge. */
    assert(MotionSequencePlan_Evaluate(&run, 500, &b));
    near(b.speedLimitCps, 3000, 0.1f); /* Gentler piece is released afterwards. */

    Fixture fixture;
    setup(&fixture, false);
    fixture.config.arcConfig = &localCalibration;
    assert(MotionController_Init(&fixture.motion, &fixture.left, &fixture.right,
                                 &fixture.steering, &fixture.imu, &fixture.config) == 0);
    for (int direction = -1; direction <= 1; direction += 2)
    {
        for (unsigned overlap = 0; overlap < 2; ++overlap)
        {
            plan = (MotionSequencePlan){0};
            plan.count = 3;
            plan.segments[0] = (MotionSequenceSegment){direction * 500, -0.002f, 5000, false};
            plan.segments[1] = (MotionSequenceSegment){direction * 180, 0.001f, 5000, false};
            plan.segments[2] = (MotionSequenceSegment){direction * 1500, 0.002f, 5000, false};
            config.allowBlendOverlap = overlap;
            MotionSequencePlan_SetFeedforward(&plan, &localCalibration, 0);
            assert(MotionSequencePlan_Prepare(&plan, &config, 1, 3000, 3000, 2000, 2000, 1e8f, 480));
            run = (MotionSequenceRun){&plan, 0, 2};
            for (float s = 0; s < 2180; s += 0.1f)
            {
                assert(MotionSequencePlan_Evaluate(&run, s, &a));
                assert(MotionSequencePlan_Evaluate(&run, s + 0.1f, &b));
                float rawA, rawB;
                assert(MotionController_GetProfileFeedforward(&fixture.motion, a.curvaturePerMm, &rawA) == 0);
                assert(MotionController_GetProfileFeedforward(&fixture.motion, b.curvaturePerMm, &rawB) == 0);
                float rawRate = fabsf(rawB - rawA) / 0.1f * a.speedLimitCps;
                assert(rawRate <= 240.5f);
                /* Includes exact local-knot braking and overlap reinforcement. */
                assert(a.speedLimitCps * a.speedLimitCps <=
                       b.speedLimitCps * b.speedLimitCps + 2 * 3000 * 0.1f + 2);
            }
        }
    }
    MotionControllerArcConfig invalid = localCalibration;
    invalid.positivePointCount = 0;
    MotionSequencePlan_SetFeedforward(&plan, &invalid, 0);
    assert(!MotionSequencePlan_Prepare(&plan, &config, 1, 3000, 3000, 2000, 2000, 1e8f, 480));
    MotionSequencePlan_SetFeedforward(&plan, &localCalibration, NAN);
    assert(!MotionSequencePlan_Prepare(&plan, &config, 1, 3000, 3000, 2000, 2000, 1e8f, 480));
    puts("PASS: local FF relief, exact knot/zero lookahead, overlap, reverse, sampled raw slew and validation");
}

static void testCruiseRequests(void)
{
    MotionProfile profile;
    assert(MotionProfile_Init(&profile, 100, 200, 0));
    assert(MotionProfile_Start(&profile, 10000, 1000));
    assert(MotionProfile_RequestCruiseSpeed(&profile, 200));
    for (unsigned i = 0; i < 40; ++i)
        MotionProfile_Update(&profile, 0, 0.1f);
    near(profile.targetSpeedMmps, 200, 0);
    assert(MotionProfile_RequestCruiseSpeed(&profile, 800));
    near(MotionProfile_Update(&profile, 0, 0.1f), 210, 0);
    for (unsigned i = 0; i < 60; ++i)
        MotionProfile_Update(&profile, 0, 0.1f);
    near(profile.targetSpeedMmps, 800, 0);
    assert(MotionProfile_RequestCruiseSpeed(&profile, 100));
    near(MotionProfile_Update(&profile, 0, 0.1f), 780, 0);
    near(profile.targetDistanceMm, 10000, 0);
    assert(profile.active);
    const float invalid[] = {0, -1, NAN, INFINITY};
    for (unsigned i = 0; i < 4; ++i)
        assert(!MotionProfile_RequestCruiseSpeed(&profile, invalid[i]));
    /* A soft cruise change cannot override a stopping or path ceiling. */
    assert(MotionProfile_RequestCruiseSpeed(&profile, 800));
    near(MotionProfile_UpdateWithBrakingLimit(&profile, 0, 0.1f, 30), 30, 0);
    MotionProfile_Stop(&profile);
    assert(!MotionProfile_RequestCruiseSpeed(&profile, 200));
    assert(MotionProfile_Start(&profile, 10000, 1000));
    near(profile.requestedCruiseSpeedMmps, 0, 0);

    for (int direction = -1; direction <= 1; direction += 2)
    {
        for (unsigned arc = 0; arc < 2; ++arc)
        {
            Fixture f;
            setup(&f, false);
            assert((arc ? MotionController_MoveArc(&f.motion, direction * 4000, 500, 5000)
                        : MotionController_MoveStraight(&f.motion, direction * 4000, 5000)) == 0);
            assert(MotionController_RequestCruiseSpeed(&f.motion, 2000) == 0);
            prepare(&f, arc ? MOTIONCONTROLLER_ARC_PREPARING : MOTIONCONTROLLER_STRAIGHT_PREPARING,
                    arc ? MOTIONCONTROLLER_ARC : MOTIONCONTROLLER_STRAIGHT);
            for (unsigned i = 0; i < 100; ++i)
                assert(MotionController_Update(&f.motion, DT) == 0);
            near(fabsf(f.motion.targetSpeedCps), 2000, 0.01f);
            float before = f.motion.motionProfile.targetSpeedMmps;
            assert(MotionController_RequestCruiseSpeed(&f.motion, 5000) == 0);
            assert(MotionController_Update(&f.motion, DT) == 0);
            near(f.motion.motionProfile.targetSpeedMmps,
                 before + f.motion.motionProfile.accelerationMmps2 * DT, 0.01f);
            near(f.motion.targetDistanceMm, 4000, 0);
            assert(f.motion.motionDirection == direction);
            assert(MotionController_RequestCruiseSpeed(&f.motion, 5001) == MOTIONCONTROLLER_STATUS_INVALID_SPEED);
            assert(MotionController_Brake(&f.motion) == 0);
            assert(MotionController_RequestCruiseSpeed(&f.motion, 2000) == MOTIONCONTROLLER_STATUS_INVALID_STATE);
            assert(MotionController_Stop(&f.motion) == 0);
        }
    }
    Fixture f;
    MotionSequence sequence;
    begin(&f, &sequence);
    assert(MotionSequence_AddStraight(&sequence, 2000, 5000, false) == 0);
    assert(MotionSequence_AddArc(&sequence, 2000, 500, 5000, false) == 0);
    assert(MotionSequence_Execute(&sequence) == 0);
    assert(MotionController_RequestCruiseSpeed(&f.motion, 2000) == 0);
    prepareSequence(&f, &sequence);
    for (unsigned i = 0; i < 100; ++i)
        sequenceUpdate(&f, &sequence);
    near(fabsf(f.motion.targetSpeedCps), 2000, 0.01f);
    assert(MotionController_RequestCruiseSpeed(&f.motion, 5000) == 0);
    sequenceUpdate(&f, &sequence);
    assert(fabsf(f.motion.targetSpeedCps) > 2000 && sequence.completedRuns == 0);
    assert(MotionController_Stop(&f.motion) == 0);
    puts("PASS: approach-to-cruise requests, bounded slowdown, preparation, standalone/fused, reverse and brake rejection");
}

int main(void)
{
    testLocalTable();
    testCruiseRequests();
    return historicalMain();
}
