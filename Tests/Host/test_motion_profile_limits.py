#!/usr/bin/env python3
"""Check straight/arc profile limits against production C with mocked hardware.

Run: python3 Tests/Host/test_motion_profile_limits.py
Requires Python 3 and a C compiler (CC, default gcc). Does not validate physical
tracking performance or fused junction planning.
"""
from test_unified_motion import FIXTURE_HARNESS, run_harness


HARNESS = FIXTURE_HARNESS + r'''
static MotionControllerArcConfig testArcConfig;

static void configureDistinctLimits(Fixture *f, bool legacy)
{
    setup(f, legacy);
    f->config.straightAccelerationMmps2 = 1200.0f;
    f->config.straightDecelerationMmps2 = 700.0f;
    f->config.arcAccelerationMmps2 = 600.0f;
    f->config.arcDecelerationMmps2 = 300.0f;
    /* Explicit settling times for the shared fixture's preparation checks. */
    f->config.straightSteeringSettlingTimeSec = 0.5f;
    testArcConfig = *f->config.arcConfig;
    testArcConfig.steeringSettlingTimeSec = 0.5f;
    f->config.arcConfig = &testArcConfig;
    assert(initMotion(f) == MOTIONCONTROLLER_STATUS_OK);
}

static void testModeSwitchingAndSpeedReferences(void)
{
    for (unsigned legacy = 0; legacy < 2; ++legacy) {
        for (int direction = -1; direction <= 1; direction += 2) {
            Fixture f;
            configureDistinctLimits(&f, legacy != 0);
            for (unsigned step = 0; step < 3; ++step) {
                bool straight = step != 1;
                float acceleration = straight ? 1200.0f : 600.0f;
                float deceleration = straight ? 700.0f : 300.0f;
                /* Same override protocol as MotionTask; selection must retain it. */
                float tolerance = straight ? 1.0f : 0.5f;
                assert(MotionProfile_Init(&f.motion.motionProfile,
                    f.motion.motionProfile.accelerationMmps2,
                    f.motion.motionProfile.decelerationMmps2, tolerance));
                MotionControllerStatus status = straight
                    ? MotionController_MoveStraight(&f.motion,
                        direction * 1000.0f, 7800.0f)
                    : MotionController_MoveArc(&f.motion,
                        direction * 1000.0f, direction * 500.0f, 7800.0f);
                assert(status == MOTIONCONTROLLER_STATUS_OK);
                near(f.motion.motionProfile.accelerationMmps2, acceleration, 0);
                near(f.motion.motionProfile.decelerationMmps2, deceleration, 0);
                near(f.motion.motionProfile.completionToleranceMm, tolerance, 0);
                wheelUpdates = 0;
                prepare(&f,
                    straight ? MOTIONCONTROLLER_STRAIGHT_PREPARING
                             : MOTIONCONTROLLER_ARC_PREPARING,
                    straight ? MOTIONCONTROLLER_STRAIGHT : MOTIONCONTROLLER_ARC);
                near(f.motion.motionProfile.targetSpeedMmps, acceleration * DT,
                     1e-4f);
                update(&f, DT);
                near(f.motion.motionProfile.targetSpeedMmps,
                     2.0f * acceleration * DT, 1e-4f);
                /* Test the stopping-distance envelope with real profile code. */
                f.motion.motionProfile.targetSpeedMmps = 1000.0f;
                float reference = MotionProfile_Update(&f.motion.motionProfile,
                                                        980.0f, DT);
                near(reference, sqrtf(2.0f * deceleration * 20.0f), 1e-4f);
                assert(MotionController_Stop(&f.motion) == 0);
            }
        }
    }
}

static void testInvalidLimits(void)
{
    const float invalid[] = {0.0f, -1.0f, NAN, INFINITY, -INFINITY};
    for (unsigned field = 0; field < 4; ++field) {
        for (unsigned value = 0; value < sizeof invalid / sizeof *invalid; ++value) {
            Fixture f;
            setup(&f, false);
            float *limits[] = {
                &f.config.straightAccelerationMmps2,
                &f.config.straightDecelerationMmps2,
                &f.config.arcAccelerationMmps2,
                &f.config.arcDecelerationMmps2,
            };
            *limits[field] = invalid[value];
            assert(initMotion(&f) == MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION);
        }
    }
}

static void testRejectedRequestsPreserveProfile(void)
{
    Fixture f;
    configureDistinctLimits(&f, false);
    assert(MotionController_MoveArc(&f.motion, 1000, 500, 7800) == 0);
    assert(MotionController_MoveStraight(&f.motion, 1000, 7800) ==
           MOTIONCONTROLLER_STATUS_BUSY);
    near(f.motion.motionProfile.accelerationMmps2, 600, 0);
    assert(MotionController_Brake(&f.motion) == 0);
    near(f.left.targetSpeedCps, 0, 0);
    near(f.right.targetSpeedCps, 0, 0);
    assert(!MotionProfile_IsActive(&f.motion.motionProfile));
    assert(MotionController_Stop(&f.motion) == 0);
    assert(MotionController_MoveStraight(&f.motion, 0, 7800) == 0);
    assert(MotionController_MoveStraight(&f.motion, 1000, 0) ==
           MOTIONCONTROLLER_STATUS_INVALID_SPEED);
    assert(MotionController_MoveArc(&f.motion, 1000, 10000, 7800) ==
           MOTIONCONTROLLER_STATUS_UNSUPPORTED_CURVATURE);
    near(f.motion.motionProfile.accelerationMmps2, 600, 0);
    assert(!MotionController_IsBusy(&f.motion));
}

static void testArcTerminalDeceleration(void)
{
    for (int direction = -1; direction <= 1; direction += 2) {
        Fixture f;
        configureDistinctLimits(&f, false);
        testArcConfig.steeringSettlingTimeSec = 0;
        assert(MotionController_MoveArc(&f.motion, direction * 1000.0f,
                                        500.0f, 7800.0f) == 0);
        f.motion.leftTravelMm = direction * 920.0f;
        f.motion.rightTravelMm = direction * 920.0f;
        f.motion.motionProfile.targetSpeedMmps = 1000.0f;
        update(&f, DT);
        float terminalSpeed = 800.0f * PI * kinematics.rearWheelDiameterMm /
                              kinematics.rearEncoderCountsPerRev;
        /* 50 mm to the existing 30 mm terminal window; arc deceleration 300. */
        near(f.motion.motionProfile.targetSpeedMmps,
             sqrtf(terminalSpeed * terminalSpeed + 2.0f * 300.0f * 50.0f),
             1e-4f);
    }
}

int main(void)
{
    testModeSwitchingAndSpeedReferences();
    testInvalidLimits();
    testRejectedRequestsPreserveProfile();
    testArcTerminalDeceleration();
    puts("PASS: separate straight/arc acceleration and deceleration, forward/"
         "reverse, legacy, mode switching, tolerance overrides, validation, "
         "rejected requests, full braking and arc terminal envelope");
    return 0;
}
'''


if __name__ == "__main__":
    run_harness(HARNESS)
