#!/usr/bin/env python3
"""Test ramp geometry and the robot harness with production motion control.

Hardware is mocked; these checks do not establish physical steering performance.
Run: python3 Tests/Host/test_steering_ramp.py
Optional: --gdb /path/to/gdb also checks the batch export using a host executable.
"""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
from test_unified_motion import HAL_STUB, FIXTURE_HARNESS

ROOT = Path(__file__).resolve().parents[2]
HARNESS = FIXTURE_HARNESS + r'''
#include "RobotTestFixture.h"
#include "SteeringRampResponseTest.h"
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

static RobotTestFixture *robot;
static uint32_t tick, trialStart;
static unsigned starts, scenario;
static float leftFraction, rightFraction;
static char rows[8][96];

bool RobotTestFixture_InitMotionController(RobotTestFixture *rf,
                                           const MotionControllerConfig *config)
{
    *rf = (RobotTestFixture){0};
    robot = rf;
    rf->leftWheelController.motor = &rf->leftRearWheel;
    rf->rightWheelController.motor = &rf->rightRearWheel;
    assert(SteeringController_Init(&rf->steeringController, &rf->steeringServo,
                                  &steeringCalibration));
    gyroDps = 0; imuOk = true;
    if (scenario == 4) return false;
    return MotionController_Init(&rf->motionController, &rf->leftWheelController,
        &rf->rightWheelController, &rf->steeringController, &rf->imu, config) == 0;
}
uint32_t HAL_GetTick(void)
{
    tick += scenario == 7 && starts ? 20U : 1U;
    if (robot) {
        WheelSpeedController *left = &robot->leftWheelController;
        WheelSpeedController *right = &robot->rightWheelController;
        float scale = (scenario == 2 || scenario == 7) ? 0.0f : scenario == 5 ? 0.5f : 1.0f;
        leftFraction += scale * left->targetSpeedCps * 0.001f;
        rightFraction += scale * right->targetSpeedCps * 0.001f;
        int dl = (int)leftFraction, dr = (int)rightFraction;
        leftFraction -= dl; rightFraction -= dr;
        robot->leftRearWheel.state.encCount += dl;
        robot->rightRearWheel.state.encCount += dr;
        left->measuredSpeedCps = scale * left->targetSpeedCps;
        right->measuredSpeedCps = scale * right->targetSpeedCps;
        float mmPerCount = PI * kinematics.rearWheelDiameterMm /
                           kinematics.rearEncoderCountsPerRev;
        gyroDps = robot->motionController.targetCurvaturePerMm *
            0.5f * (left->measuredSpeedCps + right->measuredSpeedCps) * mmPerCount * 180 / PI;
        if (scenario == 3 && starts && tick - trialStart > 1500) imuOk = false;
        if (scenario == 6 && robot->motionController.mode == MOTIONCONTROLLER_BRAKING)
            left->measuredSpeedCps = right->measuredSpeedCps = 1000;
    }
    return tick;
}
void HAL_Delay(uint32_t ms)
{ for (uint32_t i = 0; i < ms; ++i) (void)HAL_GetTick(); }
void OLED_Clear(void) { memset(rows, 0, sizeof(rows)); }
void OLED_Refresh_Gram(void) {}
void OLED_Printf(uint8_t x, uint8_t y, const char *format, ...)
{
    (void)x; assert(y < 8); va_list args; va_start(args, format);
    vsnprintf(rows[y], sizeof(rows[y]), format, args); va_end(args);
}
void SW1_WaitForPressAndRelease(void)
{
    assert(starts < STEERING_RAMP_TRIAL_COUNT);
    assert(strstr(rows[2], "Travel") && strstr(rows[3], "Area"));
    assert(strstr(rows[4], "S P0 I0 D0 H0.0"));
    assert(strstr(rows[5], "A P0 I0 D0 H0.0"));
    assert(strstr(rows[7], "Reset pose"));
    const MotionControllerConfig *config = robot->motionController.config;
    near(config->straightYawRateKp, 0, 0); near(config->arcYawRateKi, 0, 0);
    near(config->straightHeadingKpPerSec, 0, 0); near(config->arcHeadingKpPerSec, 0, 0);
    near(robot->leftWheelController.pid.kp, .03f, 0);
    near(robot->rightWheelController.pid.kp, .03f, 0);
    near(robot->steeringController.command, config->straightSteeringFeedforwardCommand, 0);
    assert(config->arcConfig->positivePointCount == 8);
    near(config->arcConfig->positivePoints[0].curvaturePerMm, .000054f, 0);
    assert(robot->motionController.mode == MOTIONCONTROLLER_IDLE);
    robot->leftRearWheel.state.encCount = robot->rightRearWheel.state.encCount = 32760;
    leftFraction = rightFraction = 0;
    trialStart = tick; ++starts;
}
int SW1_ReadState(void)
{ return (scenario == 1 || scenario == 6) && tick - trialStart >= 2000; }

static void checkProfile(const SteeringRampProfile *p)
{
    const float endRaw = p->knots[p->knotCount-1].rawCommand;
    near(p->rampLengthMm, -endRaw * p->nominalSpeedMmps /
         p->trial.nominalRawRatePerSec, .001f);
    MotionPathSample sample;
    assert(!SteeringRampProfile_Evaluate(NULL, 0, &sample));
    assert(!SteeringRampProfile_Evaluate(p, NAN, &sample));
    assert(!SteeringRampProfile_Evaluate(p, 0, NULL));
    float previousYaw = 0, numericalYaw = 0;
    const unsigned steps = 12000;
    float ds = p->totalDistanceMm / steps;
    for (unsigned i = 0; i <= steps; ++i) {
        float distance = i * ds;
        assert(SteeringRampProfile_Evaluate(p, distance, &sample));
        assert(sample.desiredYawRad >= previousYaw - .000001f);
        previousYaw = sample.desiredYawRad;
        assert(sample.straightTuningWeight >= 0 && sample.straightTuningWeight <= 1);
        assert(!sample.profileLimitsProvided);
        float raw;
        assert(MotionController_GetProfileFeedforward(&robot->motionController,
                    sample.curvaturePerMm, &raw) == 0);
        near(raw, SteeringRampProfile_RawCommand(p, distance), .0001f);
        if (i < steps) {
            MotionPathSample mid;
            assert(SteeringRampProfile_Evaluate(p, distance + .5f * ds, &mid));
            numericalYaw += mid.curvaturePerMm * ds;
        }
    }
    near(numericalYaw, p->nominalYawRad, .0002f);
    near(sample.desiredYawRad, p->nominalYawRad, .000001f);
    float boundaries[] = {p->trial.leadInMm, p->trial.leadInMm + p->rampLengthMm,
        p->trial.leadInMm + p->rampLengthMm + p->trial.arcHoldMm,
        p->trial.leadInMm + 2*p->rampLengthMm + p->trial.arcHoldMm};
    for (unsigned i = 0; i < 4; ++i) {
        MotionPathSample before, after;
        assert(SteeringRampProfile_Evaluate(p, boundaries[i]-.001f, &before));
        assert(SteeringRampProfile_Evaluate(p, boundaries[i]+.001f, &after));
        near(before.curvaturePerMm, after.curvaturePerMm, .000001f);
        near(before.desiredYawRad, after.desiredYawRad, .00001f);
    }
    for (unsigned i = 0; i < 2; ++i) {
        float start = boundaries[i == 0 ? 0 : 2];
        float a = SteeringRampProfile_RawCommand(p, start + .25f*p->rampLengthMm);
        float b = SteeringRampProfile_RawCommand(p, start + .75f*p->rampLengthMm);
        float rate = (b-a) / (.5f*p->rampLengthMm / p->nominalSpeedMmps);
        near(rate, (i == 0 ? -1 : 1)*p->trial.nominalRawRatePerSec, .0001f);
    }
}
static void checkSuccess(void)
{
    assert(steeringRampFinished && steeringRampBatchComplete && starts == 3);
    assert(steeringRampResultCount == 3 && steeringRampLogPeriodMs == 10);
    assert(sizeof(SteeringRampResponseSample) == 64);
    unsigned offset = 0;
    for (unsigned trial = 0; trial < 3; ++trial) {
        const SteeringRampResponseResult *r = &steeringRampResults[trial];
        const SteeringRampProfile *p = &steeringRampProfiles[trial];
        checkProfile(p);
        assert(r->outcome == STEERING_RAMP_COMPLETED && r->status == 0);
        assert(r->firstSample == offset && r->sampleCount > 100 && r->sampleCount < 500);
        near(r->measuredDistanceMm, p->totalDistanceMm, 1.0f);
        assert(r->maxUpdateIntervalMs == 10);
        unsigned phases[8] = {0};
        for (unsigned i = offset; i < offset+r->sampleCount; ++i) {
            const SteeringRampResponseSample *s = &steeringRampLog[i];
            assert(s->trialIndex == trial && s->phase < 8); ++phases[s->phase];
            if (i > offset) assert(s->timeMs - steeringRampLog[i-1].timeMs <= 11);
            if (s->mode == MOTIONCONTROLLER_PROFILE) near(s->rawCommand, s->rawReference, .0001f);
            if (s->phase == STEERING_RAMP_IN || s->phase == STEERING_RAMP_OUT) {
                float rate = p->trial.nominalRawRatePerSec;
                /* Integer encoder counts quantize a nominally constant speed. */
                assert(fabsf(s->rawCommandRatePerSec) <= rate*1.03f);
                /* A sample crossing a ramp boundary contains a partial ramp. */
                if (i > offset && steeringRampLog[i-1].phase == s->phase) {
                    assert(fabsf(s->rawCommandRatePerSec) > rate*.8f);
                    assert((s->phase == STEERING_RAMP_IN) == (s->rawCommandRatePerSec < 0));
                    const SteeringRampResponseSample *prev = &steeringRampLog[i-1];
                    float speed = (s->progressMm-prev->progressMm) /
                                  ((s->timeMs-prev->timeMs)*.001f);
                    near(s->rawCommandRatePerSec,
                         (s->phase == STEERING_RAMP_IN ? -rate : rate) *
                         speed/p->nominalSpeedMmps, .02f);
                }
            }
        }
        for (unsigned i = 0; i < 8; ++i) assert(phases[i] > 0);
        assert(steeringRampLog[offset+r->sampleCount-1].mode == MOTIONCONTROLLER_IDLE);
        printf("Trial %u: %.1f mm, %.2f s, %u samples; planned area %.0fx%.0f mm\n",
            trial+1, p->totalDistanceMm, r->elapsedMs*.001, r->sampleCount,
            r->maxXmm-r->minXmm+562, r->maxYmm-r->minYmm+562);
        offset += r->sampleCount;
    }
    assert(offset == steeringRampLogCount && strstr(rows[0], "DONE"));
    SteeringRampProfile invalid;
    SteeringRampTrial trial = steeringRampTrials[0];
    trial.nominalRawRatePerSec = 0;
    assert(!SteeringRampProfile_Build(&invalid, &robot->motionController, &trial));
    trial = steeringRampTrials[0]; trial.arcRadiusMm = -525;
    assert(!SteeringRampProfile_Build(&invalid, &robot->motionController, &trial));
    trial.arcRadiusMm = 10;
    assert(!SteeringRampProfile_Build(&invalid, &robot->motionController, &trial));
}
static void reset(unsigned selected)
{
    scenario = selected; tick = trialStart = starts = 0;
    leftFraction = rightFraction = 0; robot = NULL;
}
int main(void)
{
    Fixture f; setup(&f, false);
    assert(MotionController_MoveStraight(&f.motion, 100, 500) == 0);
    prepare(&f, MOTIONCONTROLLER_STRAIGHT_PREPARING, MOTIONCONTROLLER_STRAIGHT);
    for (unsigned selected = 0; selected <= 7; ++selected) {
        reset(selected); SteeringRampResponseTestRun();
        if (selected == 0) { checkSuccess(); continue; }
        assert(steeringRampFinished && !steeringRampBatchComplete);
        assert(steeringRampResultCount == 1 && starts == (selected == 4 ? 0U : 1U));
        SteeringRampOutcome expected = selected == 1 ? STEERING_RAMP_CANCELLED :
            selected == 3 ? STEERING_RAMP_CONTROLLER_ERROR :
            selected == 4 ? STEERING_RAMP_PREPARATION_ERROR :
            selected == 6 ? STEERING_RAMP_BRAKE_TIMED_OUT :
            selected == 7 ? STEERING_RAMP_TIMED_OUT : STEERING_RAMP_LOG_FULL;
        assert(steeringRampResults[0].outcome == expected);
        assert(steeringRampLogCount <= 500 && strstr(rows[0], "INVALID"));
        if (selected != 4) {
            assert(steeringRampLogCount > 0);
            const SteeringRampResponseSample *last = &steeringRampLog[steeringRampLogCount-1];
            assert(last->phase == STEERING_RAMP_FINISHED && last->mode == MOTIONCONTROLLER_IDLE);
            if (selected == 3) assert(steeringRampResults[0].status == MOTIONCONTROLLER_STATUS_IMU_ERROR);
        }
    }
    puts("PASS: ramp integration/continuity, FF inverse, encoder wrap, paired sweep, "
         "preparation display, cancellation, IMU/init faults, log bounds, run/brake watchdogs");
    return 0;
}
'''


EXPORT_FIXTURE = r'''
#include "SteeringRampResponseTest.h"
#include "RobotTestFixture.h"
const SteeringRampTrial steeringRampTrials[3] = {{4500,50,525,350,300,350,150}};
SteeringRampProfile steeringRampProfiles[3] = {{.rampLengthMm=541.5f}};
MotionControllerConfig steeringRampMotionConfig = {.kinematics=&kinematics, .arcConfig=&arcMotionConfig};
static RobotTestFixture fixture = {
    .leftWheelController={.pid={.kp=.03f}}, .rightWheelController={.pid={.kp=.03f}},
    .motionController={.initialized=INITIALIZED, .config=&steeringRampMotionConfig,
        .leftWheel=&fixture.leftWheelController, .rightWheel=&fixture.rightWheelController,
        .pathProfile={.steeringCommandRatePerSec=480}}
};
/* Deliberately unused: reproduce the original target linker failure. */
MotionController *const steeringRampController = &fixture.motionController;
SteeringRampResponseSample steeringRampLog[STEERING_RAMP_LOG_CAPACITY] = {
    {.trialIndex=0, .rawCommandRatePerSec=-50},
    {.trialIndex=1, .rawCommandRatePerSec=-100},
    {.trialIndex=2, .rawCommandRatePerSec=200}
};
SteeringRampResponseResult steeringRampResults[3] = {
    {.outcome=STEERING_RAMP_COMPLETED, .sampleCount=1},
    {.outcome=STEERING_RAMP_COMPLETED, .firstSample=1, .sampleCount=1},
    {.outcome=STEERING_RAMP_COMPLETED, .firstSample=2, .sampleCount=1}
};
uint32_t steeringRampLogCount = LOG_COUNT, steeringRampResultCount = RESULT_COUNT;
uint32_t steeringRampLogPeriodMs = 10;
bool steeringRampFinished = true, steeringRampBatchComplete = true;
static const void *volatile exportRoots[8];
int main(void) {
    exportRoots[0]=&fixture; exportRoots[1]=steeringRampTrials;
    exportRoots[2]=steeringRampProfiles; exportRoots[3]=steeringRampLog;
    exportRoots[4]=steeringRampResults; exportRoots[5]=&steeringRampLogCount;
    exportRoots[6]=&steeringRampResultCount; exportRoots[7]=&steeringRampLogPeriodMs;
    return steeringRampFinished && steeringRampBatchComplete;
}
'''


def check_export(gdb, work):
    # Read initialized ELF data directly: no ptrace, target or motion required.
    (work/'SteeringRampResponseTest.c').write_text(EXPORT_FIXTURE)
    for count, results, initialized in [(3, 3, 1), (1174, 3, 1), (0, 0, 1),
                                        (1501, 4, 1), (3, 3, 0)]:
        elf, export = work/'export-fixture', work/'export.txt'
        command = shlex.split(os.environ.get('CC', 'gcc')) + [
            '-std=c11', '-g', '-O0', '-fno-pie', '-no-pie',
            '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
            f'-DLOG_COUNT={count}', f'-DRESULT_COUNT={results}', f'-DINITIALIZED={initialized}',
            f'-I{work}', f'-I{ROOT/"Controllers/Inc"}', f'-I{ROOT/"PeripheralDrivers/Inc"}',
            f'-I{ROOT/"Tests/Inc"}', f'-I{ROOT/"Tests/Common"}', str(work/'SteeringRampResponseTest.c'),
            str(ROOT/'Controllers/Src/MotionControllerConfig.c'), '-o', str(elf)]
        subprocess.run(command, check=True)
        symbols = subprocess.check_output(['nm', str(elf)], text=True)
        assert 'steeringRampController' not in symbols
        export.unlink(missing_ok=True)
        result = subprocess.run([gdb, '-q', '--nx', '-batch', str(elf),
            '-ex', 'set pagination off',
            '-ex', f'source {ROOT}/exp/gdb_scripts/export_steering_ramp.gdb',
            '-ex', 'set $steering_ramp_battery_v = 11.43',
            '-ex', f'steering-ramp-export {export}'], capture_output=True, text=True)
        assert result.returncode == 0, result.stdout + result.stderr
        text = export.read_text()
        assert text.rstrip().endswith('=== EXPORT COMPLETE ===')
        assert '11.43' in text and 'rampLengthMm = 541.5' in text
        if count in (3, 1174):
            for rate in [-50, -100, 200]:
                assert f'rawCommandRatePerSec = {rate}' in text
            for trial in range(3):
                assert f'trialIndex = {trial}' in text
        else:
            assert 'sample array was not read' in text
            assert 'rawCommandRatePerSec' not in text
        if results == 3:
            assert text.count('outcome = STEERING_RAMP_COMPLETED') == 3
        else:
            assert 'result array was not read' in text
        assert ('steeringCommandRatePerSec = 480' in text) == bool(initialized)
        if initialized:
            assert text.count('kp = 0.0299999993') == 2
    print('PASS: GDB whole-batch export, calibration/PID metadata, empty/corrupt counts, init failure')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gdb')
    args = parser.parse_args()
    sources = ['MotionController', 'MotionControllerConfig', 'SteeringController',
               'SteeringControllerConfig', 'PIDController', 'MotionProfile', 'RobotKinematics']
    with tempfile.TemporaryDirectory(prefix='mdp-steering-ramp-') as directory:
        work = Path(directory)
        (work/'stm32f4xx_hal.h').write_text(HAL_STUB +
            '\nuint32_t HAL_GetTick(void);\nvoid HAL_Delay(uint32_t ms);\n')
        (work/'oledutils.h').write_text('#include <stdint.h>\nvoid OLED_Clear(void);\n'
            'void OLED_Refresh_Gram(void);\nvoid OLED_Printf(uint8_t, uint8_t, const char *, ...);\n')
        (work/'userbutton.h').write_text('void SW1_WaitForPressAndRelease(void);\n'
            'int SW1_ReadState(void);\n#define SW1_Enabled 1\n')
        (work/'test.c').write_text(HARNESS)
        command = shlex.split(os.environ.get('CC', 'gcc')) + [
            '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O1', '-g',
            f'-I{work}', f'-I{ROOT/"Controllers/Inc"}', f'-I{ROOT/"PeripheralDrivers/Inc"}',
            f'-I{ROOT/"Tests/Inc"}', f'-I{ROOT/"Tests/Common"}', str(work/'test.c'),
            str(ROOT/'Tests/Common/SteeringRampProfile.c'),
            str(ROOT/'Tests/Src/SteeringRampResponseTest.c')]
        command += [str(ROOT/f'Controllers/Src/{name}.c') for name in sources]
        env = os.environ.copy()
        env['ASAN_OPTIONS'] = env.get('ASAN_OPTIONS', '') + ':detect_leaks=0'
        subprocess.run(command + ['-fsanitize=address,undefined', '-fno-sanitize-recover=all',
                                  '-lm', '-o', str(work/'test')], check=True)
        subprocess.run([str(work/'test')], check=True, env=env)
        if args.gdb:
            check_export(args.gdb, work)



if __name__ == '__main__':
    main()
