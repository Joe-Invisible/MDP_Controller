#!/usr/bin/env python3
"""Production planner + controller tests with encoder/gyro hardware stubs.
Run: python3 Tests/Host/test_fused_motion.py. No physical plant simulation.
"""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
from test_unified_motion import HAL_STUB, HARNESS as BASE_HARNESS

ROOT = Path(__file__).resolve().parents[2]
PREFIX = BASE_HARNESS.split("static void testStraightSignsAndGeometry")[0]
HARNESS = PREFIX + r'''
#include "MotionSequence.h"
#include <float.h>

static float mmPerCount(void)
{ return PI * kinematics.rearWheelDiameterMm / kinematics.rearEncoderCountsPerRev; }

static void sequenceUpdate(Fixture *f, MotionSequence *s)
{
    (void)f;
    assert(MotionSequence_Update(s, DT) == 0);
}

static void seek(Fixture *f, MotionSequence *s, float progress)
{
    /* Synthetic samples exercise software; these are not a dynamic model. */
    int counts = (int)roundf(progress / mmPerCount());
    f->leftMotor.state.encCount = f->motion.previousLeftEncoderCount +
        f->motion.motionDirection * (counts - (int)roundf(
            f->motion.motionDirection * f->motion.travelledDistanceMm / mmPerCount()));
    f->rightMotor.state.encCount = f->leftMotor.state.encCount;
    sequenceUpdate(f, s);
}

static void begin(Fixture *f, MotionSequence *s)
{
    setup(f, false);
    assert(MotionSequence_Begin(s, &f->motion, &motionSequenceConfig) == 0);
    /* Fixed geometry for these unit checks; the robot harness uses current defaults. */
    s->config.blendLengthMm = 100;
    s->config.junctionSpeedCps = 2000;
}

static void prepareSequence(Fixture *f, MotionSequence *s)
{
    assert(f->motion.mode == MOTIONCONTROLLER_PROFILE_PREPARING);
    unsigned oldUpdates = wheelUpdates;
    for (unsigned i = 0; i < 49; ++i) {
        sequenceUpdate(f, s);
        assert(f->motion.mode == MOTIONCONTROLLER_PROFILE_PREPARING);
        near(f->left.targetSpeedCps, 0, 0);
        near(f->right.targetSpeedCps, 0, 0);
    }
    assert(wheelUpdates == oldUpdates);
    sequenceUpdate(f, s);
    assert(f->motion.mode == MOTIONCONTROLLER_PROFILE);
    near(f->motion.travelledDistanceMm, 0, 0);
}

static void testBlendGeometry(void)
{
    for (int direction = -1; direction <= 1; direction += 2) {
        Fixture f; MotionSequence s;
        begin(&f, &s);
        assert(MotionSequence_AddStraight(&s, direction * 300, 2000, false) == 0);
        assert(MotionSequence_AddArc(&s, direction * 500, -500, 1500, false) == 0);
        assert(MotionSequence_AddStraight(&s, direction * 300, 2000, false) == 0);
        assert(MotionSequence_Execute(&s) == 0);
        near(s.plan.totalTravelMm, 1100, 0);
        near(s.plan.nominalFinalYawRad, -direction, 1e-6f);
        assert(s.run.last == 2);
        float half = s.plan.junctionHalfLengthMm[0];
        near(half, 50, 0);
        MotionPathSample a, b;
        assert(MotionSequencePlan_Evaluate(&s.run, 300 - half, &a));
        near(a.curvaturePerMm, 0, 1e-8f);
        assert(MotionSequencePlan_Evaluate(&s.run, 300, &a));
        near(a.curvaturePerMm, -0.001f, 1e-8f);
        near(a.desiredYawRad, direction * -0.025f, 1e-6f);
        near(a.straightTuningWeight, 0.5f, 1e-6f);
        assert(a.speedLimitCps <= s.plan.junctionSpeedCps[0] + 1e-4f);
        assert(MotionSequencePlan_Evaluate(&s.run, 300 + half, &b));
        near(b.curvaturePerMm, -0.002f, 1e-8f);
        near(b.desiredYawRad, direction * -0.10f, 1e-6f);
        assert(MotionSequencePlan_Evaluate(&s.run, 1100, &a));
        near(a.desiredYawRad, -direction, 1e-6f);
        near(a.curvaturePerMm, 0, 0);
        assert(MotionSequencePlan_Evaluate(&s.run, 1200, &b));
        near(a.desiredYawRad, b.desiredYawRad, 0);
        /* Independent numerical integration verifies the analytic heading. */
        float integral = 0;
        for (unsigned i = 0; i < 11000; ++i) {
            assert(MotionSequencePlan_Evaluate(&s.run, (i + 0.5f) * 0.1f, &a));
            integral += a.curvaturePerMm * 0.1f;
        }
        near(integral, -1, 0.0001f);
        for (unsigned i = 0; i < 1100; ++i) {
            assert(MotionSequencePlan_Evaluate(&s.run, i, &a));
            assert(MotionSequencePlan_Evaluate(&s.run, i + 1, &b));
            /* Distance-domain deceleration envelope: v_prev^2 cannot
             * exceed v_next^2 + 2*d*dx, even before a lower-speed segment. */
            float va = a.speedLimitCps * mmPerCount();
            float vb = b.speedLimitCps * mmPerCount();
            assert(va * va <= vb * vb + 2 * f.config.straightDecelerationMmps2 + 0.1f);
        }
    }
}

static void testSeparateProfileLimits(void)
{
    for (int direction = -1; direction <= 1; direction += 2) {
        for (unsigned stopped = 0; stopped < 2; ++stopped) {
            Fixture f; MotionSequence s;
            begin(&f, &s);
            f.config.straightAccelerationMmps2 = 3000;
            f.config.straightDecelerationMmps2 = 3000;
            f.config.arcAccelerationMmps2 = 2000;
            f.config.arcDecelerationMmps2 = 2000;
            assert(initMotion(&f) == 0);
            assert(MotionSequence_AddStraight(&s, direction * 300, 7800, stopped) == 0);
            assert(MotionSequence_AddArc(&s, direction * 500, -500, 7800, stopped) == 0);
            assert(MotionSequence_AddStraight(&s, direction * 300, 7800, stopped) == 0);
            assert(MotionSequence_Execute(&s) == 0);
            near(f.motion.motionProfile.accelerationMmps2, 3000, 0);
            prepareSequence(&f, &s);
            near(f.motion.motionProfile.targetSpeedMmps, 3000 * DT, 1e-4f);
            if (stopped) {
                seek(&f, &s, 301);
                assert(f.motion.mode == MOTIONCONTROLLER_BRAKING);
                for (unsigned i = 0; i < f.config.stopStableSampleCount; ++i)
                    sequenceUpdate(&f, &s);
                assert(s.run.first == 1 && s.run.last == 1);
                near(f.motion.motionProfile.accelerationMmps2, 2000, 0);
                near(f.motion.motionProfile.decelerationMmps2, 2000, 0);
                prepareSequence(&f, &s);
                near(f.motion.motionProfile.targetSpeedMmps, 2000 * DT, 1e-4f);
                seek(&f, &s, 501);
                for (unsigned i = 0; i < f.config.stopStableSampleCount; ++i)
                    sequenceUpdate(&f, &s);
                assert(s.run.first == 2);
                near(f.motion.motionProfile.accelerationMmps2, 3000, 0);
                near(f.motion.motionProfile.decelerationMmps2, 3000, 0);
                continue;
            }
            MotionPathSample sample;
            assert(MotionSequencePlan_Evaluate(&s.run, 0, &sample));
            /* Constant straights: 250+250 mm; arc + blends: 400+100+100 mm. */
            float expectedBudget = 3000 * 500 + 2000 * 600;
            near(powf(sample.brakingSpeedLimitCps * mmPerCount(), 2),
                 2 * expectedBudget, 1.0f);
            const float positions[] = {100, 300, 400, 800, 1000};
            const float limits[] = {3000, 2000, 2000, 2000, 3000};
            for (unsigned j = 0; j < 5; ++j) {
                f.motion.motionProfile.targetSpeedMmps = 200;
                seek(&f, &s, positions[j]);
                near(f.motion.motionProfile.accelerationMmps2, limits[j], 0);
                near(f.motion.motionProfile.decelerationMmps2, limits[j], 0);
                near(fabsf(f.motion.targetSpeedCps) * mmPerCount(),
                     200 + limits[j] * DT, 1e-4f);
                assert(MotionSequencePlan_Evaluate(&s.run, positions[j], &sample));
                /* Independent midpoint integration checks the full stopping
                 * envelope, including crossing both kinds of limit. */
                float numericBudget = 0;
                for (float x = positions[j] + 0.5f; x < 1100; x += 1.0f) {
                    MotionPathSample next;
                    assert(MotionSequencePlan_Evaluate(&s.run, x, &next));
                    numericBudget += next.decelerationMmps2;
                }
                near(powf(sample.brakingSpeedLimitCps * mmPerCount(), 2),
                     2 * numericBudget, 1.0f);
            }
            /* Arc-to-straight execution uses the remaining stronger straight
             * budget, instead of imposing arc deceleration across the run. */
            assert(MotionSequencePlan_Evaluate(&s.run, 900, &sample));
            near(sample.brakingSpeedLimitCps * mmPerCount(), sqrtf(2*3000*200), 1e-4f);
            MotionController_Stop(&f.motion);
            begin(&f, &s);
            f.config.straightDecelerationMmps2 = 3000;
            f.config.arcDecelerationMmps2 = 2000;
            s.config.blendLengthMm = 200;
            assert(MotionSequence_AddStraight(&s, direction * 500, 7800, false) == 0);
            assert(MotionSequence_AddArc(&s, direction * 500, -500, 1000, false) == 0);
            assert(MotionSequence_Execute(&s) == 0);
            assert(MotionSequencePlan_Evaluate(&s.run, 350, &sample));
            near(sample.speedLimitCps * mmPerCount(),
                 sqrtf(powf(1000 * mmPerCount(), 2) + 2*3000*50), 1e-4f);
            /* A final arc's yaw-priority approach consumes the arc budget,
             * even though the continuous run started on a stronger straight. */
            MotionController_Stop(&f.motion);
            begin(&f, &s);
            f.config.straightDecelerationMmps2 = 3000;
            f.config.arcDecelerationMmps2 = 2000;
            assert(MotionSequence_AddStraight(&s, direction * 500, 7800, false) == 0);
            assert(MotionSequence_AddArc(&s, direction * 500, -500, 7800, false) == 0);
            assert(MotionSequence_Execute(&s) == 0);
            prepareSequence(&f, &s);
            f.motion.motionProfile.targetSpeedMmps = 1000;
            seek(&f, &s, 850);
            float toEntry = f.motion.pathProfile.terminalEntryProgressMm -
                fabsf(f.motion.travelledDistanceMm);
            near(fabsf(f.motion.targetSpeedCps) * mmPerCount(),
                 sqrtf(powf(800 * mmPerCount(), 2) + 2*2000*toEntry), 0.001f);
            /* Opposite arc blend crosses zero but keeps the arc limits. */
            MotionController_Stop(&f.motion);
            begin(&f, &s);
            f.config.straightAccelerationMmps2 = 3000;
            f.config.arcAccelerationMmps2 = 2000;
            assert(MotionSequence_AddArc(&s, direction * 300, 500, 2000, false) == 0);
            assert(MotionSequence_AddArc(&s, direction * 300, -500, 2000, false) == 0);
            assert(MotionSequence_Execute(&s) == 0);
            assert(MotionSequencePlan_Evaluate(&s.run, 300, &sample));
            near(sample.curvaturePerMm, 0, 1e-8f);
            near(sample.accelerationMmps2, 2000, 0);
        }
    }
    /* A changing path's supplied envelope replaces the constant-local-d
     * envelope without losing the usual distance-completion semantics. */
    MotionProfile profile;
    assert(MotionProfile_Init(&profile, 2000, 2000, 0.5f));
    assert(MotionProfile_Start(&profile, 1100, 2000));
    profile.targetSpeedMmps = 1500;
    float pathStopSpeed = sqrtf(2 * (2000*50 + 3000*250));
    near(MotionProfile_UpdateWithBrakingLimit(&profile, 800, DT, pathStopSpeed),
         pathStopSpeed, 1e-4f);
    near(MotionProfile_UpdateWithBrakingLimit(&profile, 1100, DT, 0), 0, 0);
    assert(!MotionProfile_IsActive(&profile));
    /* All four planner limits are validated even for an empty plan. */
    const float invalid[] = {0, -1, NAN, INFINITY};
    for (unsigned field = 0; field < 4; ++field) {
        for (unsigned v = 0; v < 4; ++v) {
            MotionSequencePlan plan = {0};
            float limits[] = {3000, 3000, 2000, 2000};
            limits[field] = invalid[v];
            assert(!MotionSequencePlan_Prepare(&plan, &motionSequenceConfig,
                mmPerCount(), limits[0], limits[1], limits[2], limits[3], 60000, 120));
        }
    }
}

static void testContinuityAndCompletion(void)
{
    for (int direction = -1; direction <= 1; direction += 2) {
        Fixture f; MotionSequence s;
        begin(&f, &s);
        /* Isolate reference and state continuity from feedback response. */
        f.config.straightHeadingKpPerSec = f.config.arcHeadingKpPerSec = 0;
        f.config.straightYawRateKp = f.config.arcYawRateKp = 0;
        f.config.straightYawRateKi = f.config.arcYawRateKi = 0;
        f.config.straightYawRateKd = f.config.arcYawRateKd = 0;
        f.config.wheelSyncKpCpsPerMm = 0;
        f.config.arcYawRateFilterTauSec = 0.1f;
        assert(initMotion(&f) == 0);
        assert(MotionSequence_AddStraight(&s, direction * 300, 2000, false) == 0);
        assert(MotionSequence_AddArc(&s, direction * 500, -500, 2000, false) == 0);
        assert(MotionSequence_AddStraight(&s, direction * 300, 2000, false) == 0);
        assert(MotionSequence_Execute(&s) == 0);
        prepareSequence(&f, &s);
        for (unsigned i = 0; i < 200; ++i) sequenceUpdate(&f, &s);
        assert(fabsf(f.motion.targetSpeedCps) > 500);
        f.left.measuredSpeedCps = f.right.measuredSpeedCps = direction * 500;
        gyroDps = direction * -1;
        float previousYaw = f.motion.yawDeg;
        float previousFilter = f.motion.filteredMeasuredCentreSpeedMmps;
        float previousTravel = f.motion.travelledDistanceMm;
        for (unsigned i = 1; i <= 219; ++i) {
            float oldRaw = f.steering.command;
            seek(&f, &s, i * 5);
            assert(f.motion.mode == MOTIONCONTROLLER_PROFILE);
            assert(f.motion.targetSpeedCps * direction > 0);
            assert(f.motion.travelledDistanceMm * direction > previousTravel * direction);
            assert(f.motion.yawDeg * -direction > previousYaw * -direction);
            assert(f.motion.filteredMeasuredCentreSpeedMmps * direction >= previousFilter * direction);
            assert(fabsf(f.steering.command - oldRaw) <=
                s.config.steeringCommandRatePerSec * DT + 1e-4f);
            near(f.motion.arcDesiredYawRad, f.motion.pathSample.desiredYawRad, 1e-6f);
            near(f.motion.rightBaseTargetCps - f.motion.leftBaseTargetCps,
                 f.motion.targetSpeedCps * kinematics.rearTrackWidthMm *
                 f.motion.targetCurvaturePerMm, 0.001f);
            previousTravel = f.motion.travelledDistanceMm;
            previousYaw = f.motion.yawDeg;
            previousFilter = f.motion.filteredMeasuredCentreSpeedMmps;
        }
        seek(&f, &s, 1101);
        assert(f.motion.mode == MOTIONCONTROLLER_BRAKING);
        /* Nonzero measured speed prevents an early sequence completion. */
        for (unsigned i = 0; i < 5; ++i) sequenceUpdate(&f, &s);
        assert(MotionSequence_IsBusy(&s));
        f.left.measuredSpeedCps = f.right.measuredSpeedCps = 0;
        gyroDps = 0;
        for (unsigned i = 0; i < f.config.stopStableSampleCount; ++i) sequenceUpdate(&f, &s);
        assert(s.state == MOTION_SEQUENCE_COMPLETE);
        assert(s.completedRuns == 1);
        assert(!MotionSequence_IsBusy(&s));
        assert(s.completedMeasuredTravelMm >= 1100);
        near(s.plan.nominalFinalYawRad, -direction, 1e-6f);
    }
}

/* Perfect coincident wheel/gyro samples isolate false errors introduced by
 * filtering; this is not a simulation of servo or vehicle dynamics. */
static void testMatchedYawRateReference(void)
{
    float peakExisting = 0, peakMatched = 0;
    for (int direction = -1; direction <= 1; direction += 2) {
        for (unsigned zeroTau = 0; zeroTau < 2; ++zeroTau) {
            for (unsigned matched = 0; matched < 2; ++matched) {
                Fixture f; MotionSequence s;
                begin(&f, &s);
                s.config.blendLengthMm = 200;
                f.config.useMatchedYawRateReferenceFilter = matched;
                f.config.arcYawRateFilterTauSec = zeroTau ? 0 : 0.1f;
                f.config.straightYawRateKp = f.config.arcYawRateKp = 100;
                f.config.straightYawRateKi = f.config.arcYawRateKi = 0;
                f.config.straightYawRateKd = f.config.arcYawRateKd = 0;
                f.config.straightHeadingKpPerSec = f.config.arcHeadingKpPerSec = 0;
                f.config.wheelSyncKpCpsPerMm = 0;
                assert(initMotion(&f) == 0);
                assert(MotionSequence_AddStraight(&s, direction * 300, 5000, false) == 0);
                assert(MotionSequence_AddArc(&s, direction * 525 * PI, 525, 5000, false) == 0);
                assert(MotionSequence_AddStraight(&s, direction * 600, 5000, false) == 0);
                f.motion.arcFilteredFeedforwardYawRateRadPerSec = 123;
                assert(MotionSequence_Execute(&s) == 0);
                near(f.motion.arcFilteredFeedforwardYawRateRadPerSec, 0, 0);
                prepareSequence(&f, &s);
                bool retainedExitHistory = false;
                for (unsigned i = 1; i <= 370; ++i) {
                    float progress = i * 6.5f;
                    int nextCounts = (int)roundf(progress / mmPerCount());
                    int previousCounts = (int)roundf(direction * f.motion.travelledDistanceMm / mmPerCount());
                    /* Mirror the incoming encoder increment, including float
                     * accumulation, so the gyro/reference samples coincide. */
                    float sampledProgress = direction * (f.motion.travelledDistanceMm +
                        direction * (nextCounts - previousCounts) * mmPerCount());
                    MotionPathSample sample;
                    assert(MotionSequencePlan_Evaluate(&s.run, sampledProgress, &sample));
                    f.left.measuredSpeedCps = f.right.measuredSpeedCps = direction * 5000;
                    gyroDps = sample.curvaturePerMm * direction * 5000 * mmPerCount() * 180 / PI;
                    seek(&f, &s, progress);
                    assert(f.motion.mode == MOTIONCONTROLLER_PROFILE);
                    if (matched || zeroTau) {
                        near(f.motion.arcYawRateErrorRadPerSec, 0, 0.00001f);
                        near(f.motion.arcSteeringCorrectionCommand, 0, 0.001f);
                    }
                    if (!zeroTau) {
                        float correction = fabsf(f.motion.arcSteeringCorrectionCommand);
                        if (matched) peakMatched = fmaxf(peakMatched, correction);
                        else peakExisting = fmaxf(peakExisting, correction);
                        if (sample.curvaturePerMm == 0 && sampledProgress > 2050 &&
                            fabsf(f.motion.arcFilteredFeedforwardYawRateRadPerSec) > 0.01f)
                            retainedExitHistory = true;
                    }
                }
                if (!zeroTau) assert(retainedExitHistory);
                MotionPathProfile nextProfile = f.motion.pathProfile;
                assert(MotionController_Stop(&f.motion) == 0);
                assert(MotionController_FollowProfile(&f.motion, &nextProfile) == 0);
                near(f.motion.arcFilteredFeedforwardYawRateRadPerSec, 0, 0);
                near(f.motion.arcUnfilteredFeedforwardYawRateRadPerSec, 0, 0);
            }
        }
    }
    near(peakExisting, 30, 0.001f);
    assert(peakMatched < 0.001f);
    printf("Perfect tracking filter experiment: existing peak correction %.2f, "
           "matched %.6f raw units; software signal test only\n", peakExisting, peakMatched);
    /* With constant curvature both formulations agree, including speed ramps. */
    float references[2][80];
    for (unsigned matched = 0; matched < 2; ++matched) {
        Fixture f; setup(&f, false);
        f.config.useMatchedYawRateReferenceFilter = matched;
        f.config.arcYawRateFilterTauSec = 0.1f;
        f.config.straightHeadingKpPerSec = f.config.arcHeadingKpPerSec = 0;
        assert(initMotion(&f) == 0);
        assert(MotionController_MoveArc(&f.motion, 1000, 525, 5000) == 0);
        prepare(&f, MOTIONCONTROLLER_ARC_PREPARING, MOTIONCONTROLLER_ARC);
        for (unsigned i = 0; i < 80; ++i) {
            f.left.measuredSpeedCps = f.right.measuredSpeedCps = (i < 40 ? i : 80-i) * 100;
            gyroDps = f.left.measuredSpeedCps * mmPerCount() / 525 * 180 / PI;
            update(&f, DT);
            references[matched][i] = f.motion.arcTargetYawRateRadPerSec;
            if (matched) near(references[0][i], references[1][i], 0.000001f);
        }
    }
}

static void testFeedforwardAndScheduling(void)
{
    Fixture f; MotionSequence s;
    begin(&f, &s);
    float raw;
    assert(MotionController_GetProfileFeedforward(&f.motion, 0, &raw) == 0);
    near(raw, f.config.straightSteeringFeedforwardCommand, 0);
    assert(MotionController_GetProfileFeedforward(&f.motion, -0.0002f, &raw) == 0);
    near(raw, 6, 1e-6f);
    assert(MotionController_GetProfileFeedforward(&f.motion, 1.0f / 3000, &raw) == 0);
    near(raw, -11.25f, 1e-5f);
    assert(MotionController_GetProfileFeedforward(&f.motion, -0.1f, &raw) ==
           MOTIONCONTROLLER_STATUS_UNSUPPORTED_CURVATURE);
    assert(MotionController_MoveArc(&f.motion, 100, 3000, 500) ==
           MOTIONCONTROLLER_STATUS_UNSUPPORTED_CURVATURE);
    assert(MotionSequence_AddStraight(&s, 300, 2000, false) == 0);
    assert(MotionSequence_AddArc(&s, 500, -500, 2000, false) == 0);
    assert(MotionSequence_Execute(&s) == 0);
    prepareSequence(&f, &s);
    near(f.motion.arcYawRatePID.kp, f.config.straightYawRateKp, 1e-6f);
    f.config.straightHeadingKpPerSec = f.config.arcHeadingKpPerSec = 0;
    /* Integral correction survives a gain change without resetting. */
    f.motion.arcYawRatePID.integral = 0.01f;
    f.motion.arcYawRatePID.previousError = 0.123f;
    float integralOutput = f.motion.arcYawRatePID.integral * f.motion.arcYawRatePID.ki;
    seek(&f, &s, 300);
    float w = f.motion.pathSample.straightTuningWeight;
    near(f.motion.arcYawRatePID.kp, f.config.arcYawRateKp + w *
         (f.config.straightYawRateKp - f.config.arcYawRateKp), 1e-5f);
    near(f.motion.arcYawRatePID.ki * f.motion.arcYawRatePID.integral, integralOutput, 1e-5f);
    assert(f.motion.arcYawRatePID.hasPreviousError);
    seek(&f, &s, 400);
    near(f.motion.arcYawRatePID.kp, f.config.arcYawRateKp, 0);
    near(f.motion.arcYawRatePID.ki * f.motion.arcYawRatePID.integral, integralOutput, 1e-5f);
}

static void testEqualCurvatureAndSignChange(void)
{
    Fixture f; MotionSequence s;
    begin(&f, &s);
    assert(MotionSequence_AddStraight(&s, 300, 2000, false) == 0);
    assert(MotionSequence_AddStraight(&s, 300, 2000, false) == 0);
    assert(MotionSequence_Execute(&s) == 0);
    MotionPathSample sample;
    assert(MotionSequencePlan_Evaluate(&s.run, 300, &sample));
    near(sample.speedLimitCps, 2000, 0);
    near(sample.curvaturePerMm, 0, 0);
    MotionController_Stop(&f.motion);
    begin(&f, &s);
    assert(MotionSequence_AddArc(&s, 300, 275, 2000, false) == 0);
    assert(MotionSequence_AddArc(&s, 300, -275, 2000, false) == 0);
    assert(MotionSequence_Execute(&s) == 0);
    assert(s.run.last == 1);
    float half = s.plan.junctionHalfLengthMm[0];
    float previous = 0;
    for (unsigned i = 0; i <= 1000; ++i) {
        float x = 300 - half + 2 * half * i / 1000;
        assert(MotionSequencePlan_Evaluate(&s.run, x, &sample));
        near(sample.straightTuningWeight, 0, 0);
        float raw;
        assert(MotionController_GetProfileFeedforward(&f.motion, sample.curvaturePerMm, &raw) == 0);
        if (i > 0) {
            float rawRate = fabsf(raw - previous) / (2 * half / 1000) *
                sample.speedLimitCps * mmPerCount();
            assert(rawRate <= 0.5f * s.config.steeringCommandRatePerSec + 0.1f);
        }
        previous = raw;
    }
    assert(MotionSequencePlan_Evaluate(&s.run, 600, &sample));
    near(sample.desiredYawRad, 0, 1e-6f);
    near(sample.curvaturePerMm, -1.0f/275, 1e-8f);
}

static void testFastJunctionAndRateIsolation(void)
{
    Fixture f; MotionSequence s;
    begin(&f, &s);
    assert(MotionSequence_AddStraight(&s, 300, 2000, false) == 0);
    assert(MotionSequence_AddArc(&s, 500, -500, 2000, false) == 0);
    assert(MotionSequence_AddStraight(&s, 300, 2000, false) == 0);
    assert(MotionSequence_Execute(&s) == 0);
    for (unsigned i = 0; i < 2; ++i) {
        near(s.plan.junctionHalfLengthMm[i], 50, 0);
        near(s.plan.junctionSpeedCps[i], 2000, 0);
    }
    near(f.motion.pathProfile.steeringCommandRatePerSec, 480, 0);
    prepareSequence(&f, &s);
    float oldRaw = f.steering.command;
    gyroDps = 1000; /* Large feedback request exercises the execution limiter. */
    sequenceUpdate(&f, &s);
    near(fabsf(f.steering.command - oldRaw), 480 * DT, 1e-4f);
    near(steeringCalibration.maxCommandRatePerSec, 120, 0);
    assert(MotionController_Stop(&f.motion) == 0);
    gyroDps = 0;
    assert(MotionController_MoveStraight(&f.motion, 300, 2000) == 0);
    wheelUpdates = 0;
    prepare(&f, MOTIONCONTROLLER_STRAIGHT_PREPARING, MOTIONCONTROLLER_STRAIGHT);
    oldRaw = f.steering.command;
    gyroDps = 1000;
    assert(MotionController_Update(&f.motion, DT) == 0);
    near(fabsf(f.steering.command - oldRaw), 120 * DT, 1e-4f);
    assert(MotionController_Stop(&f.motion) == 0);

    /* Zero explicitly inherits the old rate in both planning and execution. */
    begin(&f, &s);
    s.config.steeringCommandRatePerSec = 0;
    assert(MotionSequence_AddStraight(&s, 300, 2000, false) == 0);
    assert(MotionSequence_AddArc(&s, 500, -500, 2000, false) == 0);
    assert(MotionSequence_Execute(&s) == 0);
    assert(s.plan.junctionSpeedCps[0] > 500 && s.plan.junctionSpeedCps[0] < 800);
    prepareSequence(&f, &s);
    oldRaw = f.steering.command;
    gyroDps = 1000;
    sequenceUpdate(&f, &s);
    near(fabsf(f.steering.command - oldRaw), 120 * DT, 1e-4f);
    assert(MotionController_Stop(&f.motion) == 0);

    const float invalidRates[] = {-1, NAN, INFINITY};
    for (unsigned i = 0; i < 3; ++i) {
        begin(&f, &s);
        MotionSequenceConfig bad = motionSequenceConfig;
        bad.steeringCommandRatePerSec = invalidRates[i];
        assert(MotionSequence_Begin(&s, &f.motion, &bad) ==
               MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION);
        assert(!MotionSequencePlan_Prepare(&s.plan, &bad, mmPerCount(), 1500, 1000, 1500, 1000, 60000, 120));
        MotionPathProfile profile = {.signedDistanceMm=100, .maxSpeedCps=2000,
            .evaluate=MotionSequencePlan_Evaluate, .context=&s.run,
            .steeringCommandRatePerSec=invalidRates[i]};
        assert(MotionController_FollowProfile(&f.motion, &profile) ==
               MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT);
        assert(f.motion.mode == MOTIONCONTROLLER_IDLE);
    }
}

static void testFinalArcYawPriority(void)
{
    for (int direction = -1; direction <= 1; direction += 2) {
        for (int turn = -1; turn <= 1; turn += 2) {
            for (unsigned early = 0; early < 2; ++early) {
                Fixture f; MotionSequence s;
                begin(&f, &s);
                assert(MotionSequence_AddStraight(&s, direction * 300, 2000, false) == 0);
                assert(MotionSequence_AddArc(&s, direction * 500, turn * 500, 2000, false) == 0);
                assert(MotionSequence_Execute(&s) == 0);
                assert(f.motion.pathProfile.terminalYawPriority);
                near(f.motion.pathProfile.terminalEntryProgressMm, 770, 0);
                near(f.motion.pathFinalSample.desiredYawRad, direction * turn, 1e-6f);
                prepareSequence(&f, &s);
                for (unsigned i = 0; i < 150; ++i) sequenceUpdate(&f, &s);
                seek(&f, &s, 771);
                assert(f.motion.mode == MOTIONCONTROLLER_PROFILE);
                assert(fabsf(f.motion.targetSpeedCps) <= 800.001f);
                int yawDirection = direction * turn;
                float target = f.motion.pathFinalSample.desiredYawRad;
                if (early) {
                    /* Fresh raw gyro predicts final yaw despite some yaw
                     * still remaining. Filters must not drive this decision. */
                    float remaining = 0.04f;
                    gyroDps = yawDirection * 40;
                    f.motion.yawDeg = (target - yawDirection * remaining) * 180 / PI;
                    seek(&f, &s, 790);
                    assert(f.motion.mode == MOTIONCONTROLLER_BRAKING);
                    assert(f.motion.terminalYawPredictedReached);
                    assert(!f.motion.terminalDistanceLimitReached);
                    assert(fabsf(f.motion.travelledDistanceMm) < 800);
                } else {
                    /* Distance ending alone must not terminate an under-turn. */
                    gyroDps = 0;
                    f.motion.yawDeg = (target - yawDirection * 0.1f) * 180 / PI;
                    seek(&f, &s, 801);
                    assert(f.motion.mode == MOTIONCONTROLLER_PROFILE);
                    near(fabsf(f.motion.targetSpeedCps), 400, 1e-4f);
                    near(f.motion.arcDesiredYawRad, target, 1e-6f);
                    near(f.motion.targetCurvaturePerMm, turn / 500.0f, 1e-8f);
                    assert(!f.motion.terminalYawPredictedReached);
                    f.motion.yawDeg = (target + yawDirection * 0.001f) * 180 / PI;
                    seek(&f, &s, 810);
                    assert(f.motion.mode == MOTIONCONTROLLER_BRAKING);
                    assert(f.motion.terminalYawPredictedReached);
                    assert(!f.motion.terminalDistanceLimitReached);
                }
                gyroDps = 0;
                for (unsigned i = 0; i < f.config.stopStableSampleCount; ++i) sequenceUpdate(&f, &s);
                assert(s.state == MOTION_SEQUENCE_COMPLETE && s.completedRuns == 1);
            }
        }
    }
    /* Net target zero/opposite to final turn: use final curvature sign. */
    for (unsigned zero = 0; zero < 2; ++zero) {
        Fixture f; MotionSequence s;
        begin(&f, &s);
        float firstLength = zero ? 500 : 750;
        assert(MotionSequence_AddArc(&s, firstLength, -500, 2000, false) == 0);
        assert(MotionSequence_AddArc(&s, 500, 500, 2000, false) == 0);
        assert(MotionSequence_Execute(&s) == 0);
        assert(f.motion.pathProfile.terminalYawPriority);
        float target = zero ? 0 : -0.5f;
        near(f.motion.pathFinalSample.desiredYawRad, target, 1e-6f);
        prepareSequence(&f, &s);
        f.motion.yawDeg = (target - 0.05f) * 180 / PI;
        gyroDps = 0;
        seek(&f, &s, firstLength + 501);
        assert(f.motion.mode == MOTIONCONTROLLER_PROFILE);
        near(f.motion.arcDesiredYawRad, target, 1e-6f);
        f.motion.yawDeg = (target + 0.01f) * 180 / PI;
        seek(&f, &s, firstLength + 505);
        assert(f.motion.mode == MOTIONCONTROLLER_BRAKING);
        assert(f.motion.terminalYawPredictedReached);
    }
    /* Guarded termination records that yaw was not reached. */
    Fixture f; MotionSequence s;
    begin(&f, &s);
    assert(MotionSequence_AddArc(&s, 100, -500, 100, false) == 0);
    assert(MotionSequence_Execute(&s) == 0);
    prepareSequence(&f, &s);
    gyroDps = 0;
    seek(&f, &s, 101);
    assert(f.motion.mode == MOTIONCONTROLLER_PROFILE);
    /* Terminal creep cannot exceed the final segment's requested speed. */
    near(fabsf(f.motion.targetSpeedCps), 100, 1e-4f);
    seek(&f, &s, 131);
    assert(f.motion.mode == MOTIONCONTROLLER_BRAKING);
    assert(!f.motion.terminalYawPredictedReached);
    assert(f.motion.terminalDistanceLimitReached);
    /* Intermediate stopped arcs retain distance completion; only the actual
     * batch-ending arc opts in, with a new run-relative final heading. */
    begin(&f, &s);
    assert(MotionSequence_AddArc(&s, 100, -500, 500, true) == 0);
    assert(MotionSequence_AddArc(&s, 100, 500, 500, false) == 0);
    assert(MotionSequence_Execute(&s) == 0);
    assert(!f.motion.pathProfile.terminalYawPriority);
    prepareSequence(&f, &s);
    seek(&f, &s, 101);
    assert(f.motion.mode == MOTIONCONTROLLER_BRAKING);
    for (unsigned i = 0; i < f.config.stopStableSampleCount; ++i) sequenceUpdate(&f, &s);
    assert(f.motion.pathProfile.terminalYawPriority);
    near(f.motion.pathFinalSample.desiredYawRad, 0.2f, 1e-6f);
    /* A short final arc's window must follow its incoming blend. */
    begin(&f, &s);
    assert(MotionSequence_AddStraight(&s, 300, 2000, false) == 0);
    assert(MotionSequence_AddArc(&s, 5, -275, 500, false) == 0);
    assert(MotionSequence_Execute(&s) == 0);
    near(f.motion.pathProfile.terminalEntryProgressMm, 301.25f, 1e-5f);
    assert(f.motion.pathProfile.terminalYawPriority);
    begin(&f, &s);
    assert(MotionSequence_AddArc(&s, 100, -500, 500, false) == 0);
    assert(MotionSequence_AddStraight(&s, 100, 500, false) == 0);
    assert(MotionSequence_Execute(&s) == 0);
    assert(!f.motion.pathProfile.terminalYawPriority);
}

static void testStopsReversalsAndCancel(void)
{
    Fixture f; MotionSequence s;
    begin(&f, &s);
    assert(MotionSequence_AddStraight(&s, 100, 500, false) == 0);
    assert(MotionSequence_AddStraight(&s, -100, 500, true) == 0);
    assert(MotionSequence_AddArc(&s, -100, 500, 500, false) == 0);
    assert(MotionSequence_Execute(&s) == 0);
    assert(s.run.last == 0);
    assert(s.plan.junctionHalfLengthMm[0] == 0);
    assert(s.plan.junctionHalfLengthMm[1] == 0);
    prepareSequence(&f, &s);
    seek(&f, &s, 101);
    assert(f.motion.motionDirection == 1);
    assert(f.motion.mode == MOTIONCONTROLLER_BRAKING);
    for (unsigned i = 0; i < f.config.stopStableSampleCount - 1U; ++i) {
        sequenceUpdate(&f, &s);
        assert(f.motion.motionDirection == 1);
    }
    sequenceUpdate(&f, &s);
    assert(s.completedRuns == 1);
    assert(s.run.first == 1 && s.run.last == 1);
    assert(f.motion.mode == MOTIONCONTROLLER_PROFILE_PREPARING);
    assert(f.motion.motionDirection == -1);
    prepareSequence(&f, &s);
    seek(&f, &s, 101);
    for (unsigned i = 0; i < f.config.stopStableSampleCount; ++i) sequenceUpdate(&f, &s);
    assert(s.completedRuns == 2 && s.run.first == 2);
    prepareSequence(&f, &s);
    assert(MotionSequence_Brake(&s) == 0);
    assert(MotionSequence_Brake(&s) == 0);
    for (unsigned i = 0; i < f.config.stopStableSampleCount; ++i) sequenceUpdate(&f, &s);
    assert(s.state == MOTION_SEQUENCE_ABORTED);
    assert(s.completedRuns == 2);
    assert(!MotionSequence_IsBusy(&s));
    assert(MotionSequence_Update(&s, DT) == 0);
    assert(f.motion.mode == MOTIONCONTROLLER_IDLE);
}

static bool badSample(const void *context, float x, MotionPathSample *sample)
{
    (void)context;
    *sample = (MotionPathSample){.speedLimitCps=x > 1 && x < 99 ? NAN : 500,
        .straightTuningWeight=1};
    return true;
}

static void testValidationAndFaults(void)
{
    Fixture f; MotionSequence s;
    begin(&f, &s);
    assert(MotionSequence_Execute(&s) == 0 && s.state == MOTION_SEQUENCE_COMPLETE);
    begin(&f, &s);
    assert(MotionSequence_AddStraight(&s, 0, 500, false) == 0 && s.plan.count == 0);
    assert(MotionSequence_AddStraight(&s, NAN, 500, false) == MOTIONCONTROLLER_STATUS_INVALID_DISTANCE);
    assert(MotionSequence_AddStraight(&s, 100, INFINITY, false) == MOTIONCONTROLLER_STATUS_INVALID_SPEED);
    assert(MotionSequence_AddArc(&s, 100, 0, 500, false) == MOTIONCONTROLLER_STATUS_INVALID_RADIUS);
    assert(MotionSequence_AddArc(&s, 100, NAN, 500, false) == MOTIONCONTROLLER_STATUS_INVALID_RADIUS);
    assert(MotionSequence_AddArc(&s, 100, -10, 500, false) == MOTIONCONTROLLER_STATUS_UNSUPPORTED_CURVATURE);
    for (unsigned i = 0; i < MOTION_SEQUENCE_CAPACITY; ++i)
        assert(MotionSequence_AddStraight(&s, 5, 500, false) == 0);
    assert(MotionSequence_AddStraight(&s, 5, 500, false) == MOTIONCONTROLLER_STATUS_PROFILE_ERROR);
    assert(MotionSequence_Execute(&s) == 0);
    assert(s.run.last == MOTION_SEQUENCE_CAPACITY - 1U);
    assert(MotionSequence_AddStraight(&s, 5, 500, false) == MOTIONCONTROLLER_STATUS_INVALID_STATE);
    assert(MotionSequence_Execute(&s) == MOTIONCONTROLLER_STATUS_INVALID_STATE);
    assert(MotionSequence_Update(&s, NAN) == MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT);
    prepareSequence(&f, &s);
    imuOk = false;
    assert(MotionSequence_Update(&s, DT) == MOTIONCONTROLLER_STATUS_IMU_ERROR);
    assert(s.state == MOTION_SEQUENCE_FAILED && f.motion.mode == MOTIONCONTROLLER_IDLE);
    near(f.left.targetSpeedCps, 0, 0);
    /* External Stop/Brake must not advance to a pending run. */
    for (unsigned brake = 0; brake < 2; ++brake) {
        begin(&f, &s);
        assert(MotionSequence_AddStraight(&s, 100, 500, true) == 0);
        assert(MotionSequence_AddStraight(&s, 100, 500, false) == 0);
        assert(MotionSequence_Execute(&s) == 0);
        if (brake) assert(MotionController_Brake(&f.motion) == 0);
        else assert(MotionController_Stop(&f.motion) == 0);
        assert(MotionSequence_Update(&s, DT) == MOTIONCONTROLLER_STATUS_INVALID_STATE);
        assert(s.state == MOTION_SEQUENCE_FAILED && s.completedRuns == 0);
    }
    begin(&f, &s);
    f.config.useLegacyStraightSteering = true;
    assert(MotionSequence_Begin(&s, &f.motion, &motionSequenceConfig) == MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION);
    f.config.useLegacyStraightSteering = false;
    MotionPathProfile bad = {.signedDistanceMm=2, .maxSpeedCps=500, .evaluate=badSample};
    assert(MotionController_FollowProfile(&f.motion, &bad) == MOTIONCONTROLLER_STATUS_PROFILE_ERROR);
    /* Valid endpoints, invalid interior, checked after current odometry. */
    bad.signedDistanceMm = 100;
    assert(MotionController_FollowProfile(&f.motion, &bad) == 0);
    f.leftMotor.state.encCount += 100;
    f.rightMotor.state.encCount += 100;
    assert(MotionController_Update(&f.motion, DT) == MOTIONCONTROLLER_STATUS_PROFILE_ERROR);
    assert(f.motion.mode == MOTIONCONTROLLER_IDLE);
    assert(MotionSequence_Begin(NULL, &f.motion, &motionSequenceConfig) == MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT);
    assert(MotionSequence_Update(NULL, DT) == MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT);
    assert(MotionSequence_Brake(NULL) == MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT);
    MotionSequence uninit = {0};
    assert(MotionSequence_Execute(&uninit) == MOTIONCONTROLLER_STATUS_NOT_INITIALIZED);
}

static unsigned randomState = 74519;
static float randUnit(void)
{
    randomState = randomState * 1664525U + 1013904223U;
    return (float)(randomState & 0xFFFFU) / 65535.0f;
}

static void testRandomPlans(void)
{
    for (unsigned trial = 0; trial < 150; ++trial) {
        MotionSequencePlan p = {0};
        p.count = 1 + trial % MOTION_SEQUENCE_CAPACITY;
        for (unsigned i = 0; i < p.count; ++i) {
            float direction = randUnit() > 0.3f ? 1 : -1;
            float kappa = i % 3 == 0 ? 0 : (randUnit() - 0.5f) * 0.006f;
            p.segments[i] = (MotionSequenceSegment){direction * (5 + randUnit() * 500),
                kappa, 500 + randUnit() * 2000, randUnit() < 0.2f};
        }
        assert(MotionSequencePlan_Prepare(&p, &motionSequenceConfig, mmPerCount(),
                                          1500, 1000, 1500, 1000, 60000, 120));
        float total = 0, yaw = 0;
        for (unsigned first = 0; first < p.count;) {
            unsigned last = MotionSequencePlan_RunEnd(&p, first);
            MotionSequenceRun run = {&p, first, last};
            float length = 0, expectedYaw = 0;
            for (unsigned i = first; i <= last; ++i) {
                length += fabsf(p.segments[i].signedDistanceMm);
                expectedYaw += p.segments[i].signedDistanceMm * p.segments[i].curvaturePerMm;
            }
            MotionPathSample sample;
            assert(MotionSequencePlan_Evaluate(&run, length, &sample));
            near(sample.desiredYawRad, expectedYaw, 0.00001f);
            float numericArea = 0;
            for (unsigned j = 0; j < 2000; ++j) {
                assert(MotionSequencePlan_Evaluate(&run, (j + 0.5f) * length / 2000, &sample));
                numericArea += sample.curvaturePerMm * length / 2000;
                assert(sample.straightTuningWeight >= 0 && sample.straightTuningWeight <= 1);
            }
            numericArea *= p.segments[first].signedDistanceMm > 0 ? 1 : -1;
            near(numericArea, expectedYaw, 0.0001f);
            total += length; yaw += expectedYaw;
            first = last + 1;
        }
        near(total, p.totalTravelMm, 0.002f);
        near(yaw, p.nominalFinalYawRad, 0.00001f);
    }
    MotionSequencePlan p = {0};
    MotionSequenceConfig bad = {.blendLengthMm=NAN, .junctionSpeedCps=500};
    assert(!MotionSequencePlan_Prepare(&p, &bad, 1, 1, 1, 1, 1, 1, 1));
    p.count = MOTION_SEQUENCE_CAPACITY + 1;
    assert(!MotionSequencePlan_Prepare(&p, &motionSequenceConfig, 1, 1, 1, 1, 1, 1, 1));
    p.count = 1;
    p.segments[0] = (MotionSequenceSegment){NAN, 0, 100, false};
    assert(!MotionSequencePlan_Prepare(&p, &motionSequenceConfig, 1, 1, 1, 1, 1, 1, 1));
}


/* Exercise the selected on-target harness with synthetic time and samples. */
#include "MotionSequenceFusionTest.h"
#include "RobotTestFixture.h"
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
extern MotionSequence motionSequenceFusionTestSequence;
extern uint32_t motionSequenceFusionTestLogCount;
extern MotionSequenceFusionTestSample motionSequenceFusionTestLog[];
extern bool motionSequenceFusionTestPassed, motionSequenceFusionTestTimedOut;
extern bool motionSequenceFusionTestCancelled, motionSequenceFusionTestLogTruncated;
extern MotionControllerStatus motionSequenceFusionTestStatus;
static RobotTestFixture *hardwareFixture;
static uint32_t fakeTick;
static unsigned hardwareScenario;
static unsigned hardwareStartCount;
static uint32_t hardwareTrialStartTick;
static char oledRows[8][64];
static float leftCountFraction, rightCountFraction;

bool RobotTestFixture_InitMotionController(RobotTestFixture *rf,
                                           const MotionControllerConfig *config)
{
    *rf = (RobotTestFixture){0};
    hardwareFixture = rf;
    rf->leftWheelController.motor = &rf->leftRearWheel;
    rf->rightWheelController.motor = &rf->rightRearWheel;
    assert(SteeringController_Init(&rf->steeringController, &rf->steeringServo,
                                  &steeringCalibration));
    gyroDps = 0; imuOk = true;
    return MotionController_Init(&rf->motionController,
        &rf->leftWheelController, &rf->rightWheelController,
        &rf->steeringController, &rf->imu, config) == 0;
}

/* Integrate the actual on-target course using production blend evaluation.
 * A 300 x 200 mm centred rectangle is an assumed test footprint, not a
 * measured chassis specification. A circumscribing 181 mm circle plus a
 * 20 mm margin checks obstacle clearance; corners check car-park walls. */
static void testTask2RouteGeometry(void)
{
    MotionSequencePlan plan = motionSequenceFusionTestSequence.plan;
    assert(plan.count == 10);
    assert(MotionSequencePlan_Prepare(&plan, &motionSequenceFusionTestSequence.config,
        mmPerCount(), 3000, 3000, 2000, 2000, 0, 480));
    bool stopped = plan.segments[0].stopAfter;
    double x = -250, y = 0, yaw = 0;
    bool passes[2][2] = {{false, false}, {false, false}};
    const char *output = getenv("TASK2_ROUTE_CSV");
    FILE *trace = output ? fopen(output, "a") : NULL;
    if (output) assert(trace);
    for (unsigned first = 0; first < plan.count;) {
        MotionSequenceRun run = {&plan, first, MotionSequencePlan_RunEnd(&plan, first)};
        double length = 0;
        for (unsigned i = first; i <= run.last; ++i)
            length += fabsf(plan.segments[i].signedDistanceMm);
        double initialYaw = yaw;
        for (double d = 0; d < length;) {
            double step = fmin(1.0, length - d);
            MotionPathSample sample;
            assert(MotionSequencePlan_Evaluate(&run, (float)(d + 0.5 * step), &sample));
            yaw = initialYaw + sample.desiredYawRad;
            double previousX = x;
            x += step * cos(yaw); y += step * sin(yaw);
            assert(fabs(y) + 181 < 800); /* Compact 1600 mm clear width. */
            assert(x + 181 < 3300); /* 3800 mm length including park depth. */
            for (unsigned obstacle = 0; obstacle < 2; ++obstacle) {
                double ox = obstacle == 0 ? 1100 : 2250;
                double halfHeight = obstacle == 0 ? 50 : 250;
                double dx = fmax(fabs(x - ox) - 50, 0);
                double dy = fmax(fabs(y) - halfHeight, 0);
                assert(hypot(dx, dy) > 201);
                if ((previousX < ox && x >= ox) || (previousX > ox && x <= ox)) {
                    unsigned returning = cos(yaw) < 0;
                    assert(fabs(cos(yaw)) > 0.9);
                    assert(!passes[obstacle][returning]);
                    passes[obstacle][returning] = true;
                    /* Outward S: first left/second right. The one straight
                     * diagonal return passes above BOTH obstacles. */
                    int expectedSide = returning || obstacle == 0 ? 1 : -1;
                    assert(y * expectedSide > halfHeight + 201);
                    if (!returning && obstacle == 0) assert(y < 300);
                }
            }
            /* Check corners and the body's intersections with x=0: an
             * angled body can cross a park-wall tip without a corner inside. */
            double cornerX[4], cornerY[4];
            const int front[4] = {1, 1, -1, -1};
            const int side[4] = {1, -1, -1, 1};
            for (unsigned j = 0; j < 4; ++j) {
                cornerX[j] = x + front[j]*150*cos(yaw) - side[j]*100*sin(yaw);
                cornerY[j] = y + front[j]*150*sin(yaw) + side[j]*100*cos(yaw);
                if (cornerX[j] < 0)
                    assert(cornerX[j] > -500 && fabs(cornerY[j]) < 300);
            }
            for (unsigned j = 0; j < 4; ++j) {
                unsigned next = (j + 1) % 4;
                if ((cornerX[j] < 0) != (cornerX[next] < 0)) {
                    double fraction = -cornerX[j] / (cornerX[next] - cornerX[j]);
                    double mouthY = cornerY[j] + fraction*(cornerY[next] - cornerY[j]);
                    assert(fabs(mouthY) < 300);
                }
            }
            if (trace) fprintf(trace, "%u,%.6f,%.6f,%.6f\n", stopped, x, y, yaw);
            d += step;
        }
        MotionPathSample end;
        assert(MotionSequencePlan_Evaluate(&run, (float)length, &end));
        yaw = initialYaw + end.desiredYawRad;
        first = run.last + 1;
    }
    if (trace) fclose(trace);
    for (unsigned i = 0; i < 2; ++i) assert(passes[i][0] && passes[i][1]);
    float expectedYaw = PI + atan2f(50, 2775) + asinf(525 / hypotf(2775, 50));
    near((float)yaw, expectedYaw, 1e-5f);
    for (unsigned j = 0; j < 4; ++j) {
        int front = j < 2 ? 1 : -1;
        int side = j % 2 ? 1 : -1;
        double cx = x + front*150*cos(yaw) - side*100*sin(yaw);
        double cy = y + front*150*sin(yaw) + side*100*cos(yaw);
        assert(cx > -500 && cx < 0 && fabs(cy) < 300);
    }
    if (stopped) { near((float)x, -250, 0.01f); near((float)y, 0, 0.01f); }
    /* The final segment is a SINGLE straight tangent return into the park. */
    assert(plan.segments[9].curvaturePerMm == 0);
    assert(plan.segments[9].signedDistanceMm > 2700);
    printf("Task 2 %s nominal endpoint: x=%.2f y=%.2f mm, heading=%.2f deg\n",
           stopped ? "stopped" : "fused", x, y, yaw * 180 / PI);
}

uint32_t HAL_GetTick(void)
{
    ++fakeTick;
    bool trackMotion = hardwareScenario == 0 || hardwareScenario == 1 ||
        hardwareScenario == 3 || hardwareScenario == 5 ||
        (hardwareScenario == 4 && hardwareStartCount == 1);
    if (hardwareFixture && trackMotion) {
        WheelSpeedController *l = &hardwareFixture->leftWheelController;
        WheelSpeedController *r = &hardwareFixture->rightWheelController;
        float speedScale = hardwareScenario == 5 ? 0.5f : 1.0f;
        leftCountFraction += speedScale * l->targetSpeedCps * 0.001f;
        rightCountFraction += speedScale * r->targetSpeedCps * 0.001f;
        int dl = (int)leftCountFraction, dr = (int)rightCountFraction;
        leftCountFraction -= dl; rightCountFraction -= dr;
        hardwareFixture->leftRearWheel.state.encCount += dl;
        hardwareFixture->rightRearWheel.state.encCount += dr;
        l->measuredSpeedCps = speedScale * l->targetSpeedCps;
        r->measuredSpeedCps = speedScale * r->targetSpeedCps;
        gyroDps = hardwareFixture->motionController.targetCurvaturePerMm *
            0.5f * (l->measuredSpeedCps + r->measuredSpeedCps) * mmPerCount() * 180 / PI;
    }
    return fakeTick;
}
void HAL_Delay(uint32_t ms)
{ for (uint32_t i = 0; i < ms; ++i) (void)HAL_GetTick(); }
void SW1_WaitForPressAndRelease(void)
{
    if (hardwareScenario == 0 &&
        motionSequenceFusionTestExperiment != MOTION_SEQUENCE_FUSION_TEST_TASK2_TIMING) {
        bool single = motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_SINGLE_TUNING;
        bool gains = motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK1_GAINS;
        assert(strstr(oledRows[0], single ? "Tuning" :
            gains ? (hardwareStartCount == 0 ? "A reduced" : "B Task1")
                  : (hardwareStartCount == 0 ? "A existing" : "B matched")));
        const MotionControllerConfig *config = hardwareFixture->motionController.config;
        bool restored = gains && hardwareStartCount == 1;
        assert(config->useMatchedYawRateReferenceFilter == (single || gains || hardwareStartCount == 1));
        near(hardwareFixture->motionController.arcYawRatePID.kp, single ? 150 : restored ? 270 : 100, 0);
        near(hardwareFixture->motionController.arcYawRatePID.ki, restored ? 150 : 0, 0);
        assert(strstr(oledRows[6], single ? "S P150 I0 D0 H0" : restored ? "S P270 I150 D0 H0" : "S P100 I0 D0 H0"));
        assert(strstr(oledRows[7], single ? "C P150 I0 D0 H0" : restored ? "C P270 I150 D0 H0" : "C P100 I0 D0 H0"));
        assert(strstr(oledRows[1], "R525 180: 2.55m"));
        assert(strstr(oledRows[4], "Reset +x"));
    } else if (hardwareScenario == 0) {
        assert(strstr(oledRows[1], "Task2 S: 7.73m"));
        assert(strstr(oledRows[4], "Park +x"));
        testTask2RouteGeometry();
    }
    ++hardwareStartCount;
    hardwareTrialStartTick = fakeTick;
    /* A deliberate manual-start delay proves that waiting is not timed. */
    HAL_Delay(1000U);
}
int SW1_ReadState(void)
{
    return (hardwareScenario == 1 || (hardwareScenario == 3 && hardwareStartCount == 2)) &&
        fakeTick - hardwareTrialStartTick > 2000U ? 1 : 0;
}
void OLED_Clear(void) { memset(oledRows, 0, sizeof(oledRows)); }
void OLED_Refresh_Gram(void) { }
void OLED_Printf(uint8_t x, uint8_t y, const char *format, ...)
{
    (void)x;
    assert(y < 8); /* OLED_Printf takes a text row, not a pixel y coordinate. */
    va_list args;
    va_start(args, format);
    vsnprintf(oledRows[y], sizeof(oledRows[y]), format, args);
    va_end(args);
}

static void testRobotHarness(void)
{
    motionSequenceFusionTestExperiment = MOTION_SEQUENCE_FUSION_TEST_TASK2_TIMING;
    for (unsigned scenario = 0; scenario < 5; ++scenario) {
        hardwareScenario = scenario;
        hardwareStartCount = 0;
        hardwareTrialStartTick = 0;
        fakeTick = 0;
        leftCountFraction = rightCountFraction = 0;
        hardwareFixture = NULL;
        MotionSequenceFusionTestRun();
        assert(motionSequenceFusionTestLogCount > 1);
        assert(motionSequenceFusionTestStatus == 0);
        MotionSequenceFusionTestSample *final =
            &motionSequenceFusionTestLog[motionSequenceFusionTestLogCount - 1];
        MotionSequenceFusionTestResult *a = &motionSequenceFusionTestResults[0];
        MotionSequenceFusionTestResult *b = &motionSequenceFusionTestResults[1];
        if (scenario == 0) {
            assert(motionSequenceFusionTestPassed && motionSequenceFusionTestComparisonValid);
            assert(motionSequenceFusionTestResultCount == 2 && hardwareStartCount == 2);
            assert(a->passed && b->passed);
            near(hardwareFixture->motionController.config->straightAccelerationMmps2, 3000, 0);
            near(hardwareFixture->motionController.config->straightDecelerationMmps2, 3000, 0);
            near(hardwareFixture->motionController.config->arcAccelerationMmps2, 2000, 0);
            near(hardwareFixture->motionController.config->arcDecelerationMmps2, 2000, 0);
            near(hardwareFixture->leftWheelController.pid.kp, 0.03f, 0);
            near(hardwareFixture->rightWheelController.pid.kp, 0.03f, 0);
            assert(!a->stopAfterEachSegment && b->stopAfterEachSegment);
            assert(a->completedRuns == 1 && b->completedRuns == 10);
            assert(motionSequenceFusionTestSequence.plan.count == 10);
            for (unsigned i = 0; i < 10; ++i) {
                const MotionSequenceSegment *seg = &motionSequenceFusionTestSequence.plan.segments[i];
                near(seg->speedCps, 5000, 0);
                assert(seg->stopAfter && seg->signedDistanceMm > 0);
            }
            near(a->nominalTravelMm, 7732.5795f, 0.02f);
            near(a->nominalTravelMm, b->nominalTravelMm, 0);
            near(a->nominalYawRad, PI + atan2f(50, 2775) + asinf(525 / hypotf(2775, 50)), 1e-6f);
            near(b->nominalYawRad, a->nominalYawRad, 1e-6f);
            assert(a->elapsedMs < b->elapsedMs);
            assert(motionSequenceFusionTestSavedTimeMs ==
                   (int32_t)b->elapsedMs - (int32_t)a->elapsedMs);
            near(motionSequenceFusionTestSavedPercent,
                 100.0f * motionSequenceFusionTestSavedTimeMs / b->elapsedMs, 1e-5f);
            assert(!a->logTruncated && !b->logTruncated);
            assert(!motionSequenceFusionTestLogTruncated);
            assert(strstr(oledRows[0], "DONE") && strstr(oledRows[1], "A fused") &&
                   strstr(oledRows[2], "B stopped") && strstr(oledRows[3], "Saved") &&
                   strstr(oledRows[4], "Reduction"));
            unsigned taggedSamples[2] = {0};
            for (unsigned i = 0; i < motionSequenceFusionTestLogCount; ++i) {
                MotionSequenceFusionTestSample *sample = &motionSequenceFusionTestLog[i];
                assert(sample->comparisonRunIndex < 2);
                ++taggedSamples[sample->comparisonRunIndex];
                assert(sample->accelerationLimitMmps2 == 3000 ||
                       sample->accelerationLimitMmps2 == 2000);
                assert(sample->decelerationLimitMmps2 == sample->accelerationLimitMmps2);
                if (i > 0 && sample->comparisonRunIndex ==
                    motionSequenceFusionTestLog[i - 1].comparisonRunIndex)
                    assert(sample->timeMs >= motionSequenceFusionTestLog[i - 1].timeMs);
            }
            assert(taggedSamples[0] > 1 && taggedSamples[1] > 1);
            assert(fakeTick >= a->elapsedMs + b->elapsedMs + 2800U);
            near(final->sequenceTravelMm, b->measuredTravelMm, 0);
            printf("Ideal Task 2 S/straight-return A/B harness: fused %u ms, stopped %u ms, "
                   "saved %d ms (%.1f%%); software timing only\n",
                   a->elapsedMs, b->elapsedMs, motionSequenceFusionTestSavedTimeMs,
                   motionSequenceFusionTestSavedPercent);
        } else {
            bool second = scenario >= 3;
            bool timeout = scenario == 2 || scenario == 4;
            assert(!motionSequenceFusionTestPassed && !motionSequenceFusionTestComparisonValid);
            assert(motionSequenceFusionTestSavedTimeMs == 0);
            near(motionSequenceFusionTestSavedPercent, 0, 0);
            assert(motionSequenceFusionTestResultCount == (second ? 2U : 1U));
            assert(hardwareStartCount == (second ? 2U : 1U));
            MotionSequenceFusionTestResult *failed = second ? b : a;
            assert(!failed->passed);
            assert(failed->timedOut == timeout && failed->cancelled == !timeout);
            if (second) assert(a->passed);
            assert(motionSequenceFusionTestSequence.state == MOTION_SEQUENCE_ABORTED);
            assert(strstr(oledRows[0], "INVALID"));
            assert(!strstr(oledRows[3], "Saved"));
            if (timeout) assert(motionSequenceFusionTestLogTruncated == second);
        }
        assert(final->mode == MOTIONCONTROLLER_IDLE);
        assert(final->comparisonRunIndex == (scenario >= 3 || scenario == 0 ? 1U : 0U));
    }
}

static void testAccuracyRobotHarness(MotionSequenceFusionTestExperiment experiment)
{
    motionSequenceFusionTestExperiment = experiment;
    bool gains = experiment == MOTION_SEQUENCE_FUSION_TEST_TASK1_GAINS;
    for (unsigned scenario = 0; scenario < 6; ++scenario) {
        hardwareScenario = scenario;
        hardwareStartCount = 0;
        hardwareTrialStartTick = 0;
        fakeTick = 0;
        leftCountFraction = rightCountFraction = 0;
        hardwareFixture = NULL;
        MotionSequenceFusionTestRun();
        assert(motionSequenceFusionTestLogPeriodMs == 20);
        assert(motionSequenceFusionTestLogCount > 1 && motionSequenceFusionTestLogCount <= 650);
        assert(motionSequenceFusionTestSavedTimeMs == 0);
        near(motionSequenceFusionTestSavedPercent, 0, 0);
        MotionSequenceFusionTestResult *a = &motionSequenceFusionTestResults[0];
        MotionSequenceFusionTestResult *b = &motionSequenceFusionTestResults[1];
        assert(a->useMatchedYawRateReferenceFilter == gains);
        near(a->straightTuning.yawRateKp, 100, 0); near(a->arcTuning.yawRateKp, 100, 0);
        near(a->straightTuning.yawRateKi, 0, 0); near(a->arcTuning.yawRateKi, 0, 0);
        near(a->straightTuning.headingKpPerSec, 0, 0); near(a->arcTuning.headingKpPerSec, 0, 0);
        bool restored = gains && hardwareStartCount == 2;
        const MotionControllerConfig *config = hardwareFixture->motionController.config;
        near(config->straightYawRateKp, restored ? 270 : 100, 0);
        near(config->arcYawRateKp, restored ? 270 : 100, 0);
        near(config->straightYawRateKi, restored ? 150 : 0, 0); near(config->arcYawRateKi, restored ? 150 : 0, 0);
        near(config->straightYawRateKd, 0, 0); near(config->arcYawRateKd, 0, 0);
        near(config->straightHeadingKpPerSec, 0, 0);
        near(config->arcHeadingKpPerSec, 0, 0);
        if (hardwareStartCount == 2) {
            near(b->straightTuning.yawRateKp, restored ? 270 : 100, 0);
            near(b->arcTuning.yawRateKp, restored ? 270 : 100, 0);
            near(b->straightTuning.yawRateKi, restored ? 150 : 0, 0);
            near(b->arcTuning.yawRateKi, restored ? 150 : 0, 0);
            near(b->straightTuning.headingKpPerSec, 0, 0);
            near(b->arcTuning.headingKpPerSec, 0, 0);
        }
        near(config->arcYawRateFilterTauSec, 0.1f, 0);
        if (scenario == 0) {
            assert(motionSequenceFusionTestComparisonValid && motionSequenceFusionTestPassed);
            assert(hardwareStartCount == 2 && motionSequenceFusionTestResultCount == 2);
            assert(a->passed && b->passed && b->useMatchedYawRateReferenceFilter);
            assert(!a->stopAfterEachSegment && !b->stopAfterEachSegment);
            assert(a->completedRuns == 1 && b->completedRuns == 1);
            assert(!a->logTruncated && !b->logTruncated);
            near(a->nominalTravelMm, 900 + 525 * PI, 0.001f);
            near(a->nominalTravelMm, b->nominalTravelMm, 0);
            near(a->nominalYawRad, PI, 0.000001f);
            near(a->nominalYawRad, b->nominalYawRad, 0);
            assert(motionSequenceFusionTestSequence.plan.count == 3);
            for (unsigned i = 0; i < 3; ++i) {
                assert(!motionSequenceFusionTestSequence.plan.segments[i].stopAfter);
                near(motionSequenceFusionTestSequence.plan.segments[i].speedCps, 5000, 0);
            }
            assert(strstr(oledRows[0], "DONE") &&
                   strstr(oledRows[1], gains ? "A reduced" : "A existing") &&
                   strstr(oledRows[2], gains ? "B Task1" : "B matched"));
            assert(strstr(oledRows[3], "A yaw") && strstr(oledRows[4], "B yaw"));
            unsigned taggedSamples[2] = {0};
            for (unsigned i = 0; i < motionSequenceFusionTestLogCount; ++i) {
                const MotionSequenceFusionTestSample *sample = &motionSequenceFusionTestLog[i];
                assert(sample->comparisonRunIndex < 2);
                ++taggedSamples[sample->comparisonRunIndex];
                if (i && sample->comparisonRunIndex == motionSequenceFusionTestLog[i-1].comparisonRunIndex) {
                    unsigned gap = sample->timeMs - motionSequenceFusionTestLog[i-1].timeMs;
                    assert(gap <= 30); /* Includes final partial interval. */
                }
                assert(isfinite(sample->filteredGeometricYawRateRadPerSec));
                if (gains && sample->mode == MOTIONCONTROLLER_PROFILE) {
                    near(sample->targetYawRateRadPerSec,
                         sample->filteredGeometricYawRateRadPerSec, 0.000001f);
                }
            }
            assert(taggedSamples[0] > 100 && taggedSamples[0] <= 325);
            assert(taggedSamples[1] > 100 && taggedSamples[1] <= 325);
            printf("Ideal %s A/B harness: A %u ms, B %u ms, %u samples; "
                   "not a physical accuracy result\n", gains ? "Task 1 gains" : "filter", a->elapsedMs, b->elapsedMs,
                   motionSequenceFusionTestLogCount);
        } else if (scenario == 5) {
            /* Completed slow A cannot consume B's reserved log budget. */
            assert(hardwareStartCount == 2 && motionSequenceFusionTestResultCount == 2);
            assert(a->passed && b->passed && b->useMatchedYawRateReferenceFilter);
            assert(a->logTruncated && b->logTruncated);
            assert(!motionSequenceFusionTestComparisonValid && !motionSequenceFusionTestPassed);
            assert(motionSequenceFusionTestLogCount == 650);
            assert(motionSequenceFusionTestLog[324].comparisonRunIndex == 0);
            assert(motionSequenceFusionTestLog[324].mode == MOTIONCONTROLLER_IDLE);
            assert(motionSequenceFusionTestLog[325].comparisonRunIndex == 1);
            assert(strstr(oledRows[5], "LOG TRUNCATED"));
        } else {
            bool second = scenario >= 3;
            bool timeout = scenario == 2 || scenario == 4;
            assert(!motionSequenceFusionTestComparisonValid && !motionSequenceFusionTestPassed);
            assert(hardwareStartCount == (second ? 2U : 1U));
            assert(motionSequenceFusionTestResultCount == (second ? 2U : 1U));
            MotionSequenceFusionTestResult *failed = second ? b : a;
            assert(!failed->passed && failed->timedOut == timeout && failed->cancelled == !timeout);
            if (second) assert(a->passed && b->useMatchedYawRateReferenceFilter);
            assert(motionSequenceFusionTestLogTruncated == timeout);
            assert(strstr(oledRows[0], "INVALID"));
        }
        const MotionSequenceFusionTestSample *last =
            &motionSequenceFusionTestLog[motionSequenceFusionTestLogCount-1];
        assert(last->mode == MOTIONCONTROLLER_IDLE);
        assert(last->comparisonRunIndex == (scenario == 0 || scenario >= 3 ? 1U : 0U));
    }
}

static void testSingleTuningHarness(void)
{
    motionSequenceFusionTestExperiment = MOTION_SEQUENCE_FUSION_TEST_SINGLE_TUNING;
    for (unsigned scenario = 0; scenario < 6; ++scenario) {
        if (scenario == 3 || scenario == 4) continue; /* No second run exists. */
        hardwareScenario = scenario;
        hardwareStartCount = 0;
        hardwareTrialStartTick = fakeTick = 0;
        leftCountFraction = rightCountFraction = 0;
        hardwareFixture = NULL;
        MotionSequenceFusionTestRun();
        const MotionSequenceFusionTestResult *result = &motionSequenceFusionTestResults[0];
        assert(hardwareStartCount == 1 && motionSequenceFusionTestResultCount == 1);
        assert(!motionSequenceFusionTestComparisonValid && motionSequenceFusionTestSavedTimeMs == 0);
        near(motionSequenceFusionTestSavedPercent, 0, 0);
        assert(motionSequenceFusionTestLogPeriodMs == 20);
        assert(result->useMatchedYawRateReferenceFilter && !result->stopAfterEachSegment);
        near(result->straightTuning.yawRateKp, 150, 0);
        near(result->arcTuning.yawRateKp, 150, 0);
        near(result->straightTuning.yawRateKi, 0, 0);
        near(result->arcTuning.headingKpPerSec, 0, 0);
        assert(!motionSequenceFusionTestResults[1].passed);
        assert(motionSequenceFusionTestResults[1].elapsedMs == 0);
        assert(motionSequenceFusionTestSequence.plan.count == 3);
        near(result->nominalTravelMm, 900 + 525 * PI, 0.001f);
        near(result->nominalYawRad, PI, 0.000001f);
        for (unsigned i = 0; i < motionSequenceFusionTestLogCount; ++i) {
            const MotionSequenceFusionTestSample *sample = &motionSequenceFusionTestLog[i];
            assert(sample->comparisonRunIndex == 0);
            if (sample->mode == MOTIONCONTROLLER_PROFILE)
                near(sample->targetYawRateRadPerSec, sample->filteredGeometricYawRateRadPerSec, 0.000001f);
        }
        assert(motionSequenceFusionTestLog[motionSequenceFusionTestLogCount-1].mode == MOTIONCONTROLLER_IDLE);
        assert(strstr(oledRows[0], "Tuning") && !strstr(oledRows[0], "A/B"));
        if (scenario == 0 || scenario == 5) {
            assert(motionSequenceFusionTestPassed && result->passed && !result->logTruncated);
            assert(strstr(oledRows[0], "DONE") && strstr(oledRows[2], "Yaw"));
            if (scenario == 5) assert(motionSequenceFusionTestLogCount > 325);
            else printf("Ideal single tuning harness: %u ms, %u samples; not a physical accuracy result\n",
                        result->elapsedMs, motionSequenceFusionTestLogCount);
        } else {
            assert(!motionSequenceFusionTestPassed && !result->passed);
            assert(result->timedOut == (scenario == 2));
            assert(result->cancelled == (scenario == 1));
            assert(strstr(oledRows[0], "INVALID"));
            if (scenario == 2) {
                assert(result->logTruncated && motionSequenceFusionTestLogCount == 650);
                assert(strstr(oledRows[5], "LOG TRUNCATED"));
            }
        }
    }
    /* User-selected gains survive Run() and are captured independently per regime. */
    motionSequenceFusionTestStraightTuning = (MotionSequenceFusionTestTuning){160, 0, 0, 0.2f};
    motionSequenceFusionTestArcTuning = (MotionSequenceFusionTestTuning){140, 0, 0, 0.3f};
    hardwareScenario = 5; hardwareStartCount = 0; hardwareTrialStartTick = fakeTick = 0;
    leftCountFraction = rightCountFraction = 0; hardwareFixture = NULL;
    MotionSequenceFusionTestRun();
    assert(motionSequenceFusionTestPassed && hardwareStartCount == 1);
    near(motionSequenceFusionTestResults[0].straightTuning.yawRateKp, 160, 0);
    near(motionSequenceFusionTestResults[0].arcTuning.yawRateKp, 140, 0);
    near(motionSequenceFusionTestResults[0].straightTuning.headingKpPerSec, 0.2f, 0);
    near(motionSequenceFusionTestResults[0].arcTuning.headingKpPerSec, 0.3f, 0);
    near(hardwareFixture->motionController.config->straightYawRateKp, 160, 0);
    near(hardwareFixture->motionController.config->arcHeadingKpPerSec, 0.3f, 0);
    motionSequenceFusionTestStraightTuning = motionSequenceFusionTestArcTuning =
        (MotionSequenceFusionTestTuning){150, 0, 0, 0};
}

int main(void)
{
    /* Keep existing preparation helper compiled and exercised too. */
    Fixture f; setup(&f, false);
    assert(MotionController_MoveStraight(&f.motion, 100, 500) == 0);
    prepare(&f, MOTIONCONTROLLER_STRAIGHT_PREPARING, MOTIONCONTROLLER_STRAIGHT);
    testBlendGeometry();
    testSeparateProfileLimits();
    testContinuityAndCompletion();
    testMatchedYawRateReference();
    testFeedforwardAndScheduling();
    testEqualCurvatureAndSignChange();
    testFastJunctionAndRateIsolation();
    testFinalArcYawPriority();
    testStopsReversalsAndCancel();
    testValidationAndFaults();
    testRandomPlans();
    assert(motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_SINGLE_TUNING);
    testSingleTuningHarness();
    testRobotHarness();
    testAccuracyRobotHarness(MOTION_SEQUENCE_FUSION_TEST_REFERENCE_FILTER);
    testAccuracyRobotHarness(MOTION_SEQUENCE_FUSION_TEST_TASK1_GAINS);
    puts("PASS: fused geometry/heading area, lookahead, forward/reverse continuity, "
         "Task 2 obstacle/park geometry, FF bridge, separate limits/stopping envelopes, gain scheduling, slew, stops/reversals, cancellation, faults, 150 random plans, robot harness, final-arc yaw priority");
    return 0;
}
'''


def main():
    sources = ["MotionController", "MotionControllerConfig", "SteeringController",
               "SteeringControllerConfig", "PIDController", "MotionProfile",
               "RobotKinematics", "MotionSequencePlan", "MotionSequence"]
    with tempfile.TemporaryDirectory(prefix="mdp-fused-test-") as directory:
        work = Path(directory)
        (work / "stm32f4xx_hal.h").write_text(HAL_STUB +
            "\nuint32_t HAL_GetTick(void);\nvoid HAL_Delay(uint32_t ms);\n")
        (work / "oledutils.h").write_text("#include <stdint.h>\nvoid OLED_Clear(void);\n"
            "void OLED_Refresh_Gram(void);\nvoid OLED_Printf(uint8_t, uint8_t, const char *, ...);\n")
        (work / "userbutton.h").write_text("void SW1_WaitForPressAndRelease(void);\n"
            "int SW1_ReadState(void);\n#define SW1_Enabled 1\n")
        (work / "test.c").write_text(HARNESS)
        command = shlex.split(os.environ.get("CC", "gcc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic", "-O1",
            "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
            f"-I{work}", f"-I{ROOT / 'Controllers/Inc'}",
            f"-I{ROOT / 'PeripheralDrivers/Inc'}", f"-I{ROOT / 'Tests/Inc'}",
            f"-I{ROOT / 'Tests/Common'}", str(work / "test.c"),
            str(ROOT / "Tests/Src/MotionSequenceFusionTest.c"),
        ]
        command += [str(ROOT / f"Controllers/Src/{name}.c") for name in sources]
        command += ["-lm", "-o", str(work / "test")]
        subprocess.run(command, check=True)
        # Production components allocate no heap; leak tracing is unsupported
        # in some managed containers. Bounds/UB instrumentation stays enabled.
        env = os.environ.copy()
        env["ASAN_OPTIONS"] = env.get("ASAN_OPTIONS", "") + ":detect_leaks=0"
        subprocess.run([str(work / "test")], check=True, env=env)


if __name__ == "__main__":
    main()
