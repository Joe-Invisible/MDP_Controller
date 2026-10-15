#!/usr/bin/env python3
"""Build and run the real Task2App/Run/Commands with asynchronous ideal hardware."""
import os
import argparse
from pathlib import Path
import shlex
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--exhaustive', action='store_true', help='Use PC 1 mm clearance sweeps instead of MCU certificates')
args = parser.parse_args()
sources = ['Task2App', 'Task2Course', 'Task2Run', 'Task2Commands', 'Task2Estimate',
           'Task2Route', 'Task2ObstacleDetection', 'Task2Diagnostics', 'Task2Wire', 'SensorReading', 'CommandParser', 'CommandSession', 'CommandMotion']
with tempfile.TemporaryDirectory(prefix='task2-app-') as directory:
    exe = Path(directory) / 'test'
    subprocess.run(shlex.split(os.environ.get('CC', 'gcc')) + [
        '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O1',
        f'-DTASK2_ROUTE_EXHAUSTIVE_CHECKS={int(args.exhaustive)}', '-DTASK2_COURSE_WRAP=0',
        '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
        '-I'+str(root/'Apps/Inc'), str(root/'hosttests/task2_app_test.c'),
        *[str(root/f'Apps/Src/{s}.c') for s in sources], '-lm', '-o', str(exe)], check=True)
    env = os.environ.copy()
    env["ASAN_OPTIONS"] = env.get("ASAN_OPTIONS", "") + ":detect_leaks=0"
    subprocess.run([str(exe)], check=True, timeout=60, env=env)
    # The firmware must also build with the planning records compiled out.
    for source in ('Task2App', 'Task2Run', 'Task2Diagnostics'):
        subprocess.run(shlex.split(os.environ.get('CC', 'gcc')) + [
            '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-DTASK2_DIAGNOSTICS=0', '-c',
            '-I'+str(root/'Apps/Inc'), str(root/f'Apps/Src/{source}.c'), '-o', str(Path(directory) / f'{source}.o')],
            check=True)
    print('PASS builds with TASK2_DIAGNOSTICS=0')
