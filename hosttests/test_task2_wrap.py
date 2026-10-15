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
        '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O1',
        '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
        '-I'+str(root/'Apps/Inc'), str(root/'hosttests/task2_wrap_test.c'),
        *[str(root/f'Apps/Src/{s}.c') for s in sources], '-lm', '-o', str(exe)], check=True)
    env = os.environ.copy()
    env["ASAN_OPTIONS"] = env.get("ASAN_OPTIONS", "") + ":detect_leaks=0"
    subprocess.run([str(exe)], check=True, timeout=120, env=env, stdout=subprocess.DEVNULL)
    for diagnostics in (0, 1):
        for flush in (0, 1):
            subprocess.run(shlex.split(os.environ.get('CC', 'gcc')) + [
                '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', f'-DTASK2_DIAGNOSTICS={diagnostics}',
                f'-DTASK2_WRAP_START_REAR_FLUSH={flush}', '-c', '-I'+str(root/'Apps/Inc'),
                str(root/'Apps/Src/Task2Wrap.c'), '-o', str(Path(directory) / 'Task2Wrap.o')], check=True)
    print('task2 wrap: PASS (fake-robot cases; builds with diagnostics on/off and both start positions)')
