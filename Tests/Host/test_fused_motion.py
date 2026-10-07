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
            assert(va * va <= vb * vb + 2 * f.config.motionDecelerationMmps2 + 0.1f);
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
        assert(!MotionSequencePlan_Prepare(&s.plan, &bad, mmPerCount(), 1000, 60000, 120));
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
    *sample = (MotionPathSample){0, 0, x > 1 && x < 99 ? NAN : 500, 1};
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
                                          1000, 60000, 120));
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
    assert(!MotionSequencePlan_Prepare(&p, &bad, 1, 1, 1, 1));
    p.count = MOTION_SEQUENCE_CAPACITY + 1;
    assert(!MotionSequencePlan_Prepare(&p, &motionSequenceConfig, 1, 1, 1, 1));
    p.count = 1;
    p.segments[0] = (MotionSequenceSegment){NAN, 0, 100, false};
    assert(!MotionSequencePlan_Prepare(&p, &motionSequenceConfig, 1, 1, 1, 1));
}


/* Exercise the selected on-target harness with synthetic time and samples. */
#include "MotionSequenceFusionTest.h"
#include "RobotTestFixture.h"
#include <stdarg.h>
extern MotionSequence motionSequenceFusionTestSequence;
extern uint32_t motionSequenceFusionTestLogCount;
extern MotionSequenceFusionTestSample motionSequenceFusionTestLog[];
extern bool motionSequenceFusionTestPassed, motionSequenceFusionTestTimedOut;
extern bool motionSequenceFusionTestCancelled, motionSequenceFusionTestLogTruncated;
extern MotionControllerStatus motionSequenceFusionTestStatus;
static RobotTestFixture *hardwareFixture;
static uint32_t fakeTick;
static unsigned hardwareScenario;
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

uint32_t HAL_GetTick(void)
{
    ++fakeTick;
    if (hardwareFixture && hardwareScenario == 0) {
        WheelSpeedController *l = &hardwareFixture->leftWheelController;
        WheelSpeedController *r = &hardwareFixture->rightWheelController;
        leftCountFraction += l->targetSpeedCps * 0.001f;
        rightCountFraction += r->targetSpeedCps * 0.001f;
        int dl = (int)leftCountFraction, dr = (int)rightCountFraction;
        leftCountFraction -= dl; rightCountFraction -= dr;
        hardwareFixture->leftRearWheel.state.encCount += dl;
        hardwareFixture->rightRearWheel.state.encCount += dr;
        l->measuredSpeedCps = l->targetSpeedCps;
        r->measuredSpeedCps = r->targetSpeedCps;
        gyroDps = hardwareFixture->motionController.targetCurvaturePerMm *
            0.5f * (l->measuredSpeedCps + r->measuredSpeedCps) * mmPerCount() * 180 / PI;
    }
    return fakeTick;
}
void HAL_Delay(uint32_t ms)
{ for (uint32_t i = 0; i < ms; ++i) (void)HAL_GetTick(); }
void SW1_WaitForPressAndRelease(void) { }
int SW1_ReadState(void) { return hardwareScenario == 1 && fakeTick > 250 ? 1 : 0; }
void OLED_Clear(void) { }
void OLED_Refresh_Gram(void) { }
void OLED_Printf(uint8_t x, uint8_t y, const char *format, ...)
{ (void)x; (void)y; (void)format; }

static void testRobotHarness(void)
{
    for (unsigned scenario = 0; scenario < 3; ++scenario) {
        hardwareScenario = scenario;
        fakeTick = 0;
        leftCountFraction = rightCountFraction = 0;
        hardwareFixture = NULL;
        MotionSequenceFusionTestRun();
        assert(motionSequenceFusionTestLogCount > 1);
        assert(motionSequenceFusionTestStatus == 0);
        MotionSequenceFusionTestSample *final =
            &motionSequenceFusionTestLog[motionSequenceFusionTestLogCount - 1];
        if (scenario == 0) {
            assert(motionSequenceFusionTestPassed);
            printf("Ideal encoder/gyro harness elapsed: %u ms (software timing only)\n",
                   final->timeMs);
            assert(!motionSequenceFusionTestCancelled && !motionSequenceFusionTestTimedOut);
            assert(motionSequenceFusionTestSequence.completedRuns == 1);
            near(final->sequenceTravelMm,
                 motionSequenceFusionTestSequence.completedMeasuredTravelMm, 0);
        } else {
            assert(!motionSequenceFusionTestPassed);
            assert(motionSequenceFusionTestSequence.state == MOTION_SEQUENCE_ABORTED);
            assert(scenario == 1 ? motionSequenceFusionTestCancelled : motionSequenceFusionTestTimedOut);
            assert(scenario != 2 || motionSequenceFusionTestLogTruncated);
        }
        assert(final->mode == MOTIONCONTROLLER_IDLE);
    }
}

int main(void)
{
    /* Keep existing preparation helper compiled and exercised too. */
    Fixture f; setup(&f, false);
    assert(MotionController_MoveStraight(&f.motion, 100, 500) == 0);
    prepare(&f, MOTIONCONTROLLER_STRAIGHT_PREPARING, MOTIONCONTROLLER_STRAIGHT);
    testBlendGeometry();
    testContinuityAndCompletion();
    testFeedforwardAndScheduling();
    testEqualCurvatureAndSignChange();
    testFastJunctionAndRateIsolation();
    testFinalArcYawPriority();
    testStopsReversalsAndCancel();
    testValidationAndFaults();
    testRandomPlans();
    testRobotHarness();
    puts("PASS: fused geometry/heading area, lookahead, forward/reverse continuity, "
         "FF bridge, gain scheduling, slew, stops/reversals, cancellation, faults, 150 random plans, robot harness, final-arc yaw priority");
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
