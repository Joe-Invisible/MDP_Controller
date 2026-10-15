#!/usr/bin/env python3
"""Build and run Task2Wrap against the deterministic fake robot in task2_wrap_test.c, both start positions."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
sources = ['Task2Wrap', 'Task2Run', 'Task2Course', 'Task2Commands', 'Task2Diagnostics', 'SensorReading', 'Task2App', 'Task2Route',
           'Task2Estimate', 'Task2ObstacleDetection', 'Task2Wire']
with tempfile.TemporaryDirectory(prefix='task2-wrap-') as directory:
    exe = Path(directory) / 'test'
    subprocess.run(shlex.split(os.environ.get('CC', 'gcc')) + [
        '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-Wno-missing-field-initializers', '-O1',
        '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
        '-I'+str(root/'Apps/Inc'), str(root/'hosttests/task2_wrap_test.c'),
        *[str(root/f'Apps/Src/{s}.c') for s in sources], '-lm', '-o', str(exe)], check=True)
    env = os.environ.copy()
    env["ASAN_OPTIONS"] = env.get("ASAN_OPTIONS", "") + ":detect_leaks=0"
    subprocess.run([str(exe)], check=True, timeout=120, env=env, stdout=subprocess.DEVNULL)
    # The same fake-robot cases from the rear-flush start and with the five-move swerve.
    for flags in (['-DTASK2_WRAP_START_BACK_GAP_MM=0'], ['-DTASK2_WRAP_SWERVE_ARCS=5'],
                  ['-DTASK2_WRAP_START_BACK_GAP_MM=0', '-DTASK2_WRAP_SWERVE_ARCS=5'], ['-DTASK2_WRAP_FAR_BY_IR=0']):
        variant = Path(directory) / 'variant'
        subprocess.run(shlex.split(os.environ.get('CC', 'gcc')) + [
            '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-Wno-missing-field-initializers', '-O1',
            '-fsanitize=address,undefined', '-fno-sanitize-recover=all', *flags,
            '-I'+str(root/'Apps/Inc'), str(root/'hosttests/task2_wrap_test.c'),
            *[str(root/f'Apps/Src/{s}.c') for s in sources], '-lm', '-o', str(variant)], check=True)
        subprocess.run([str(variant)], check=True, timeout=120, env=env, stdout=subprocess.DEVNULL)
    for diagnostics in (0, 1):
        for gap in (0, 210, 243):
            for arcs, far in ((3, 1), (5, 1), (3, 0)):
                subprocess.run(shlex.split(os.environ.get('CC', 'gcc')) + [
                    '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', f'-DTASK2_DIAGNOSTICS={diagnostics}',
                    f'-DTASK2_WRAP_START_BACK_GAP_MM={gap}', f'-DTASK2_WRAP_SWERVE_ARCS={arcs}',
                    f'-DTASK2_WRAP_FAR_BY_IR={far}', '-c',
                    '-I'+str(root/'Apps/Inc'), str(root/'Apps/Src/Task2Wrap.c'),
                    '-o', str(Path(directory) / 'Task2Wrap.o')], check=True)
    home = Path(directory) / 'home'
    subprocess.run(shlex.split(os.environ.get('CC', 'gcc')) + [
        '-std=c11', '-Wall', '-Wextra', '-O1', '-fsanitize=address,undefined,float-cast-overflow', '-fno-sanitize-recover=all',
        '-I'+str(root/'Apps/Inc'), str(root/'hosttests/task2_wrap_home_test.c'),
        *[str(root/f'Apps/Src/{s}.c') for s in sources if s != 'Task2Wrap'], '-lm', '-o', str(home)], check=True)
    subprocess.run([str(home)], check=True, timeout=300, env=env, stdout=subprocess.DEVNULL)
    print('task2 wrap: PASS (fake-robot cases run for start gaps 210 and 0 with each swerve, and the mirrored far end; '
          'home geometry and emitted home routes at -3/0/+3 deg; '
          'builds with diagnostics on/off, start gaps 0/210/243, both swerves and both back legs)')
