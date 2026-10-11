#!/usr/bin/env python3
"""Verify a camera-route candidate with the production controller host harness.

Only temporary test inputs change; the on-target route remains untouched.
"""
import argparse
import sys, json, math, os, shlex, tempfile, subprocess
from pathlib import Path
import numpy as np
import test_fused_motion as t
import plan_task2_camera_route as model
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--plan', type=Path, default=t.ROOT / 'Tests/task2_camera_route_plan.json')
parser.add_argument('--traces', type=Path, required=True, help='Directory for nominal and ideal executed B CSVs')
args = parser.parse_args()
args.traces.mkdir(parents=True, exist_ok=True)
plan = json.loads(args.plan.read_text())
clearance = plan['required_body_clearance_mm']
steps = [(a['distance_mm'] / 1000, a['radius_mm'] / 1000) for a in plan['steps']]
assert len(steps) == 5 and all((r for d, r in steps)), 'This verifier covers the five-arc camera family'
h = t.HARNESS.replace('int main(void)\n{', 'int main(void)\n{\n    setvbuf(stdout, NULL, _IONBF, 0);')
h = h.replace('assert(gap > 200);', f'assert(gap > {clearance});').replace('idealTask2MinObstacle >= 200', f'idealTask2MinObstacle >= {clearance}')
h = h.replace('motionSequenceFusionTestArcExitCount == 4', 'motionSequenceFusionTestArcExitCount == 5').replace('a->nominalTravelMm < 7600', 'a->nominalTravelMm < 10000')
h = h.replace('Task2: 7.39m', f"Task2: {plan['distance_mm'] / 1000:.2f}m")
a = h.index('            if (fusedPlan->allowBlendOverlap && fusedPlan->requestedBlendLengthMm == 500.0f) {')
b = h.index('            const MotionControllerArcConfig', a)
h = h[:a] + h[b:]
h = h.replace('static double idealTask2X, idealTask2Y, idealTask2Yaw;', 'static FILE *cameraExecutionTrace;\nstatic double idealTask2X, idealTask2Y, idealTask2Yaw;')
h = h.replace('    idealTask2Yaw += step*curvature;', '    idealTask2Yaw += step*curvature;\n    if (cameraExecutionTrace) fprintf(cameraExecutionTrace,"1,%.6f,%.6f,%.6f,%u\\n",idealTask2X,idealTask2Y,idealTask2Yaw,fakeTick);')
h = h.replace('        idealTask2X = -250;', '        if (scenario == 0) cameraExecutionTrace = fopen(getenv("CAMERA_EXECUTION_CSV"),"w");\n        idealTask2X = -250;')
h = h.replace('        } else {\n            bool second = scenario >= 3;', '            if (cameraExecutionTrace) { fclose(cameraExecutionTrace); cameraExecutionTrace=NULL; }\n        } else {\n            bool second = scenario >= 3;', 1)
s = (t.ROOT / 'Tests/Src/MotionSequenceFusionTest.c').read_text()
a = s.index('    const MotionSequenceFusionTestStep fusionTestSteps[] = {')
b = s.index('    _Static_assert(sizeof(fusionTestSteps)', a)
block = '    const MotionSequenceFusionTestStep fusionTestSteps[] = {\n' + ''.join((f'        {{{d * 1000:.6f}f, {r * 1000:.6f}f, FUSION_SPEED_CPS}},\n' for d, r in steps)) + '    };\n'
s = s[:a] + block + s[b:]
with tempfile.TemporaryDirectory(prefix='camera-controller-') as directory:
    work = Path(directory)
    (work / 'stm32f4xx_hal.h').write_text(t.HAL_STUB + '\nuint32_t HAL_GetTick(void);\nvoid HAL_Delay(uint32_t ms);\n')
    (work / 'oledutils.h').write_text('#include <stdint.h>\nvoid OLED_Clear(void);\nvoid OLED_Refresh_Gram(void);\nvoid OLED_Printf(uint8_t, uint8_t, const char *, ...);\n')
    (work / 'userbutton.h').write_text('void SW1_WaitForPressAndRelease(void);\nint SW1_ReadState(void);\n#define SW1_Enabled 1\n')
    (work / 'test.c').write_text(h)
    (work / 'MotionSequenceFusionTest.c').write_text(s)
    sources = ['MotionController', 'MotionControllerConfig', 'SteeringController', 'SteeringControllerConfig', 'PIDController', 'MotionProfile', 'RobotKinematics', 'MotionSequencePlan', 'MotionSequence']
    command = shlex.split(os.environ.get('CC', 'gcc')) + ['-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O1', '-fsanitize=address,undefined', '-fno-sanitize-recover=all', f'-I{work}', f"-I{t.ROOT / 'Controllers/Inc'}", f"-I{t.ROOT / 'PeripheralDrivers/Inc'}", f"-I{t.ROOT / 'Tests/Inc'}", f"-I{t.ROOT / 'Tests/Common'}", str(work / 'test.c'), str(work / 'MotionSequenceFusionTest.c')] + [str(t.ROOT / f'Controllers/Src/{n}.c') for n in sources] + ['-lm', '-o', str(work / 'test')]
    subprocess.run(command, check=True)
    env = os.environ.copy()
    env['ASAN_OPTIONS'] = 'detect_leaks=0'
    env['TASK2_ROUTE_CSV'] = str(args.traces / 'nominal.csv')
    env['CAMERA_EXECUTION_CSV'] = str(args.traces / 'executed_b.csv')
    Path(env['TASK2_ROUTE_CSV']).unlink(missing_ok=True)
    subprocess.run([str(work / 'test')], check=True, env=env)
import numpy as np
nominal = np.loadtxt(args.traces / 'nominal.csv', delimiter=',')
executed = np.loadtxt(args.traces / 'executed_b.csv', delimiter=',')
for label, poses in [('A nominal', nominal[nominal[:, 0] == 0]), ('B nominal', nominal[nominal[:, 0] == 1]), ('B ideal execution', executed)]:
    xy = poses[:, 1:3]
    heading = poses[:, 3]
    tangent = np.column_stack((np.cos(heading), np.sin(heading)))
    normal = np.column_stack((-np.sin(heading), np.cos(heading)))
    camera = xy + 70 * tangent
    progress = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    for target_x in [1050, 2200]:
        centre = np.array([target_x, 0]) - camera
        visible = (np.linalg.norm(centre, axis=1) <= 500) & (camera[:, 0] < target_x)
        visible &= np.arctan2(abs(camera[:, 1]), target_x - camera[:, 0]) <= math.radians(50)
        for target_y in [-50, 50]:
            ray = np.array([target_x, target_y]) - camera
            forward = np.sum(ray * tangent, axis=1)
            lateral = np.sum(ray * normal, axis=1)
            visible &= abs(lateral) <= math.tan(math.pi / 6) * forward
        if target_x == 2200:
            visible &= camera[:, 0] > 1150
        indices = np.flatnonzero(visible)
        assert len(indices), (label, target_x, 'no full-target view')
        groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
        best = max(groups, key=lambda ids: progress[ids[-1]] - progress[ids[0]])
        window = progress[best[-1]] - progress[best[0]]
        assert window >= 30, (label, target_x, window)
        duration = f', {poses[best[-1], 4] - poses[best[0], 4]:.0f} ms' if poses.shape[1] > 4 else ''
        print(f'{label}, arrow {(1 if target_x == 1050 else 2)}: full 100 mm target visible for {window:.1f} mm{duration}')
print('PASS: camera range, full-target horizontal framing, front-face visibility and nonzero windows')
