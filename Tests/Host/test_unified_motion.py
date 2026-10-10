#!/usr/bin/env python3
"""Exercise production motion/steering logic with mocked hardware on a host.

Run from any directory: python3 Tests/Host/test_unified_motion.py
Requires Python 3 and a C compiler (CC, default gcc). No hardware simulation or
performance claims: encoder/gyro samples are supplied explicitly by each case.
"""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HAL_STUB = """#ifndef HOST_HAL_STUB_H
#define HOST_HAL_STUB_H
#include <stdint.h>
typedef struct { uint32_t unused; } TIM_HandleTypeDef;
typedef struct { uint32_t unused; } I2C_HandleTypeDef;
#endif
"""
FIXTURE_HARNESS = r'''
#include "MotionController.h"
#include "SteeringControllerConfig.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846f
#define DT 0.01f
static float gyroDps;
static bool imuOk;
static unsigned wheelUpdates;

/* Mock only hardware and the unchanged lower-level wheel loops. */
int16_t DCMotor_GetEncoderCount(DCMotor *motor)
{ return (int16_t)motor->state.encCount; }
void Servo_SetSteering(Servo *servo, float command)
{ servo->state.steer = command; }
void Servo_Centre(Servo *servo)
{ Servo_SetSteering(servo, 0.0f); }
void WheelSpeedController_SetTarget(WheelSpeedController *wheel, float target)
{ wheel->targetSpeedCps = target; }
void WheelSpeedController_Update(WheelSpeedController *wheel, float dt)
{ (void)wheel; (void)dt; ++wheelUpdates; }
void WheelSpeedController_Stop(WheelSpeedController *wheel)
{ wheel->targetSpeedCps = 0.0f; wheel->measuredSpeedCps = 0.0f; }
bool WheelSpeedController_IsStationary(const WheelSpeedController *wheel)
{ return fabsf(wheel->measuredSpeedCps) < WHEEL_STATIONARY_THRESHOLD_CPS; }
bool ICM20948_ReadMeasurement(ICM20948 *imu, ICM20948Measurement *measurement)
{
    (void)imu;
    *measurement = (ICM20948Measurement){0};
    measurement->gyroDps.z = gyroDps;
    return imuOk;
}

typedef struct {
    MotionController motion;
    MotionControllerConfig config;
    WheelSpeedController left, right;
    DCMotor leftMotor, rightMotor;
    SteeringController steering;
    Servo servo;
    ICM20948 imu;
} Fixture;

static void near(float actual, float expected, float tolerance)
{
    if (!(fabsf(actual - expected) <= tolerance)) {
        fprintf(stderr, "actual %.9g expected %.9g tolerance %.9g\n",
                actual, expected, tolerance);
        assert(false);
    }
}

static MotionControllerStatus initMotion(Fixture *f)
{
    return MotionController_Init(&f->motion, &f->left, &f->right,
                                &f->steering, &f->imu, &f->config);
}

static void setup(Fixture *f, bool legacy)
{
    *f = (Fixture){0};
    f->config = motionControllerConfig;
    f->config.useLegacyStraightSteering = legacy;
    f->config.arcYawRateFilterTauSec = 0.0f;
    f->left.motor = &f->leftMotor;
    f->right.motor = &f->rightMotor;
    gyroDps = 0.0f;
    imuOk = true;
    wheelUpdates = 0;
    assert(SteeringController_Init(&f->steering, &f->servo,
                                  &steeringCalibration));
    assert(initMotion(f) == MOTIONCONTROLLER_STATUS_OK);
}

static void update(Fixture *f, float dt)
{ assert(MotionController_Update(&f->motion, dt) == MOTIONCONTROLLER_STATUS_OK); }

static void prepare(Fixture *f, MotionControllerMode preparing,
                    MotionControllerMode active)
{
    assert(f->motion.mode == preparing);
    for (unsigned i = 0; i < 49; ++i) {
        update(f, DT);
        assert(f->motion.mode == preparing);
        near(f->left.targetSpeedCps, 0.0f, 0.0f);
        near(f->right.targetSpeedCps, 0.0f, 0.0f);
        near(f->motion.motionProfile.targetSpeedMmps, 0.0f, 0.0f);
    }
    assert(wheelUpdates == 0);
    /* Incidental preparation movement is excluded from the motion origin. */
    f->leftMotor.state.encCount = 7;
    f->rightMotor.state.encCount = 9;
    f->motion.yawDeg = 12.0f;
    update(f, DT);
    assert(f->motion.mode == active);
    assert(wheelUpdates == 2);
    near(f->motion.travelledDistanceMm, 0.0f, 0.0f);
    near(f->motion.yawDeg, 0.0f, 0.0f);
}

'''
HARNESS = FIXTURE_HARNESS + r'''
static void testStraightSignsAndGeometry(void)
{
    for (int direction = -1; direction <= 1; direction += 2) {
        for (int yawSign = -1; yawSign <= 1; yawSign += 2) {
            Fixture f;
            setup(&f, false);
            /* Exercise both return directions from a raw arc command. */
            SteeringController_SetRawCommand(&f.steering, yawSign * 60.0f);
            assert(MotionController_MoveStraight(&f.motion, direction * 500.0f,
                                                2000.0f) == 0);
            float centre = SteeringController_GetCommand(&f.steering);
            near(f.motion.arcSteeringFeedforwardCommand, centre, 0.0f);
            assert(f.steering.effectiveAngleModelValid);
            prepare(&f, MOTIONCONTROLLER_STRAIGHT_PREPARING,
                        MOTIONCONTROLLER_STRAIGHT);
            f.motion.yawDeg = yawSign * 2.0f;
            update(&f, DT);
            float speed = f.motion.targetSpeedCps * PI *
                          kinematics.rearWheelDiameterMm /
                          kinematics.rearEncoderCountsPerRev;
            float kappa = f.motion.arcCommandedCurvaturePerMm;
            assert(f.motion.arcTargetYawRateRadPerSec * yawSign < 0.0f);
            assert((f.motion.arcSteeringTargetCommand - centre) *
                   direction * yawSign > 0.0f);
            near(kappa, -yawSign * direction *
                 f.config.maxPathCorrectionCurvaturePerMm, 1e-7f);
            near(kappa * speed, f.motion.arcTargetYawRateRadPerSec, 1e-7f);
            near(f.motion.wheelReferenceCurvaturePerMm, kappa, 0.0f);
            near(f.motion.rightBaseTargetCps - f.motion.leftBaseTargetCps,
                 f.motion.targetSpeedCps * kinematics.rearTrackWidthMm * kappa,
                 0.001f);
            assert(!f.steering.effectiveAngleModelValid);
            /* Poison the angle estimate: unified wheels must ignore it. */
            f.steering.effectiveAngleRad = 1.0f;
            float oldKappa = kappa;
            f.leftMotor.state.encCount += (uint16_t)(direction * 10);
            f.rightMotor.state.encCount += (uint16_t)(direction * 10);
            update(&f, DT);
            float displacement = direction * 10.0f * PI *
                                 kinematics.rearWheelDiameterMm /
                                 kinematics.rearEncoderCountsPerRev;
            near(f.motion.desiredWheelTravelDifferenceMm,
                 kinematics.rearTrackWidthMm * oldKappa * displacement, 1e-6f);
            near(f.motion.wheelReferenceCurvaturePerMm, kappa, 1e-7f);
        }
    }
}

static void testFixedStraightFeedforward(void)
{
    for (int direction = -1; direction <= 1; direction += 2) {
        Fixture f;
        setup(&f, false);
        f.config.arcYawRateKp = 0.0f;
        f.config.arcYawRateKi = 0.0f;
        f.config.arcYawRateKd = 0.0f;
        f.config.arcHeadingKpPerSec = 0.0f;
        assert(initMotion(&f) == MOTIONCONTROLLER_STATUS_OK);
        assert(MotionController_MoveStraight(&f.motion, direction * 1000.0f,
                                            2000.0f) == 0);
        const float candidate = -8.5f;
        f.motion.arcSteeringFeedforwardCommand = candidate;
        f.motion.arcSteeringTargetCommand = candidate;
        SteeringController_SetRawCommand(&f.steering, candidate);
        prepare(&f, MOTIONCONTROLLER_STRAIGHT_PREPARING,
                MOTIONCONTROLLER_STRAIGHT);
        gyroDps = direction * 2.0f;
        for (unsigned i = 0; i < 100; ++i) {
            f.leftMotor.state.encCount += direction * 10;
            f.rightMotor.state.encCount += direction * 10;
            update(&f, DT);
            near(f.steering.command, candidate, 0.0f);
            near(f.motion.arcSteeringCorrectionCommand, 0.0f, 0.0f);
            near(f.motion.arcCommandedCurvaturePerMm, 0.0f, 0.0f);
            near(f.motion.wheelReferenceCurvaturePerMm, 0.0f, 0.0f);
            near(f.motion.desiredWheelTravelDifferenceMm, 0.0f, 0.0f);
            near(f.left.targetSpeedCps, f.right.targetSpeedCps, 0.0f);
        }
        assert(f.motion.yawDeg * direction > 1.9f);
        assert(MotionController_Brake(&f.motion) == 0);
        assert(f.steering.effectiveAngleModelValid);
    }
}

static void testArcAndReverse(void)
{
    for (int direction = -1; direction <= 1; direction += 2) {
        for (int turn = -1; turn <= 1; turn += 2) {
            Fixture f;
            setup(&f, false);
            assert(MotionController_MoveArc(&f.motion, direction * 500.0f,
                                            turn * 500.0f, 2000.0f) == 0);
            near(f.steering.command, turn == -1 ? 47.5f : -47.5f, 1e-5f);
            prepare(&f, MOTIONCONTROLLER_ARC_PREPARING, MOTIONCONTROLLER_ARC);
            near(f.motion.arcCommandedCurvaturePerMm, turn / 500.0f, 1e-7f);
            assert(f.motion.arcFeedforwardYawRateRadPerSec * turn * direction > 0);
            float nominalKappa = f.motion.targetCurvaturePerMm;
            f.motion.yawDeg = 10.0f;
            update(&f, DT);
            near(f.motion.arcCommandedCurvaturePerMm, nominalKappa - direction *
                 f.config.maxPathCorrectionCurvaturePerMm, 1e-7f);
            assert(f.motion.arcHeadingYawRateCorrectionRadPerSec < 0.0f);
            assert(f.motion.wheelReferenceCurvaturePerMm * turn > 0.0f);
        }
    }
    /* Existing interpolation and unsupported near-zero gap remain intact. */
    Fixture f;
    setup(&f, false);
    assert(MotionController_MoveArc(&f.motion, 500, 450, 2000) == 0);
    near(f.motion.arcSteeringFeedforwardCommand, -52.083333f, 1e-4f);
    MotionController_Stop(&f.motion);
    assert(MotionController_MoveArc(&f.motion, 500, 10000, 2000) ==
           MOTIONCONTROLLER_STATUS_UNSUPPORTED_CURVATURE);
    assert(f.motion.mode == MOTIONCONTROLLER_IDLE);
}

static void testLowSpeedAndFilter(void)
{
    Fixture f;
    setup(&f, false);
    f.config.straightSteeringSettlingTimeSec = 0;
    assert(initMotion(&f) == 0);
    assert(MotionController_MoveStraight(&f.motion, -500, 2000) == 0);
    f.motion.yawDeg = 20;
    update(&f, 0.00001f); /* speed = 0.005 mm/s, below former 1 mm/s cutoff */
    near(f.motion.arcCommandedCurvaturePerMm,
         f.config.maxPathCorrectionCurvaturePerMm, 1e-7f);
    assert(isfinite(f.motion.arcCommandedCurvaturePerMm));
    MotionController_Stop(&f.motion);
    f.config.maxPathCorrectionCurvaturePerMm = 0;
    f.config.arcYawRateFilterTauSec = 0.10f;
    assert(initMotion(&f) == 0);
    assert(MotionController_MoveStraight(&f.motion, 500, 2000) == 0);
    gyroDps = 10.0f;
    update(&f, DT);
    near(f.motion.yawDeg, 0.1f, 1e-6f); /* integration remains unfiltered */
    near(f.motion.filteredYawRateDps, 10.0f / 11.0f, 1e-6f);
    near(f.motion.arcTargetYawRateRadPerSec, 0, 0);
    assert(f.motion.arcSteeringCorrectionCommand < 0); /* rate feedback active */
}

static void testLegacyAndValidation(void)
{
    Fixture f;
    setup(&f, true);
    assert(MotionController_MoveStraight(&f.motion, -500, 2000) == 0);
    prepare(&f, MOTIONCONTROLLER_STRAIGHT_PREPARING, MOTIONCONTROLLER_STRAIGHT);
    f.motion.yawDeg = 2;
    update(&f, DT);
    assert(f.steering.effectiveAngleModelValid);
    assert(f.steering.targetEffectiveAngleRad > 0);
    near(f.motion.wheelReferenceCurvaturePerMm,
         tanf(f.steering.effectiveAngleRad) / kinematics.wheelbaseMm, 1e-7f);
    assert(!f.motion.arcYawRatePID.hasPreviousError);
    setup(&f, false);
    /* Unified init does not depend on a physical steering-angle envelope. */
    f.steering.minEffectiveAngleRad = NAN;
    f.config.headingKp = NAN;
    f.config.maxHeadingSteeringAngleRad = 0;
    assert(initMotion(&f) == 0);
    f.config.useLegacyStraightSteering = true;
    assert(initMotion(&f) == MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION);
    setup(&f, false);
    f.config.maxPathCorrectionCurvaturePerMm = -1;
    assert(initMotion(&f) == MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION);
    f.config.maxPathCorrectionCurvaturePerMm = NAN;
    assert(initMotion(&f) == MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION);
}

static void testCompletionAndFaults(void)
{
    Fixture f;
    setup(&f, false);
    assert(MotionController_MoveStraight(&f.motion, 0, 2000) == 0);
    assert(!MotionController_IsBusy(&f.motion));
    assert(MotionController_MoveStraight(&f.motion, 500, 2000) == 0);
    assert(MotionController_MoveArc(&f.motion, 500, 500, 2000) ==
           MOTIONCONTROLLER_STATUS_BUSY);
    prepare(&f, MOTIONCONTROLLER_STRAIGHT_PREPARING, MOTIONCONTROLLER_STRAIGHT);
    uint16_t counts = (uint16_t)ceilf(500.0f *
        kinematics.rearEncoderCountsPerRev /
        (PI * kinematics.rearWheelDiameterMm));
    f.leftMotor.state.encCount += counts;
    f.rightMotor.state.encCount += counts;
    update(&f, DT);
    assert(f.motion.mode == MOTIONCONTROLLER_BRAKING);
    near(f.left.targetSpeedCps, 0, 0);
    near(f.right.targetSpeedCps, 0, 0);
    for (unsigned i = 0; i < f.config.stopStableSampleCount; ++i) update(&f, DT);
    assert(!MotionController_IsBusy(&f.motion));
    /* Raw steering from a preceding arc still returns physically to centre. */
    assert(MotionController_MoveArc(&f.motion, 500, -300, 2000) == 0);
    near(f.steering.command, 84.2f, 1e-4f);
    assert(MotionController_Brake(&f.motion) == 0);
    assert(f.steering.effectiveAngleModelValid);
    assert(fabsf(f.steering.command) < 12);
    for (unsigned i = 0; i < f.config.stopStableSampleCount; ++i) update(&f, DT);
    assert(MotionController_MoveStraight(&f.motion, -500, 2000) == 0);
    wheelUpdates = 0;
    prepare(&f, MOTIONCONTROLLER_STRAIGHT_PREPARING, MOTIONCONTROLLER_STRAIGHT);
    imuOk = false;
    assert(MotionController_Update(&f.motion, DT) == MOTIONCONTROLLER_STATUS_IMU_ERROR);
    assert(f.motion.mode == MOTIONCONTROLLER_IDLE);
    near(f.left.targetSpeedCps, 0, 0);
    near(f.right.targetSpeedCps, 0, 0);
}

int main(void)
{
    testStraightSignsAndGeometry();
    testFixedStraightFeedforward();
    testArcAndReverse();
    testLowSpeedAndFilter();
    testLegacyAndValidation();
    testCompletionAndFaults();
    puts("PASS: unified forward/reverse straight and arcs, curvature limits, "
         "wheel geometry/odometry, preparation, filter, legacy, braking and IMU fault");
    return 0;
}
'''


def run_harness(harness):
    sources = ["MotionController", "MotionControllerConfig", "SteeringController",
               "SteeringControllerConfig", "PIDController", "MotionProfile",
               "RobotKinematics"]
    with tempfile.TemporaryDirectory(prefix="mdp-motion-test-") as directory:
        work = Path(directory)
        (work / "stm32f4xx_hal.h").write_text(HAL_STUB)
        (work / "test.c").write_text(harness)
        command = shlex.split(os.environ.get("CC", "gcc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic", "-O1",
            "-fsanitize=undefined", "-fno-sanitize-recover=all",
            f"-I{work}", f"-I{ROOT / 'Controllers/Inc'}",
            f"-I{ROOT / 'PeripheralDrivers/Inc'}", str(work / "test.c"),
        ]
        command += [str(ROOT / f"Controllers/Src/{name}.c") for name in sources]
        command += ["-lm", "-o", str(work / "test")]
        subprocess.run(command, check=True)
        subprocess.run([str(work / "test")], check=True)


def main():
    run_harness(HARNESS)


if __name__ == "__main__":
    main()
