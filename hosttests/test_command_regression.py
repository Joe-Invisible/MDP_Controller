#!/usr/bin/env python3
"""Run the command, sensor and explicit Task2 dry-run host regression targets."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CASES = {
    'parser': ['CommandParser'],
    'session': ['CommandParser', 'CommandSession', 'CommandMotion'],
    'motion': ['CommandMotion'],
    'button': [],
    'diagnostics': ['MotionDiagnostics'],
    'progress': ['CommandProgress', 'CommandParser', 'CommandSession', 'CommandMotion'],
    'sensor_reading': ['SensorReading'],
    'task2_commands': ['Task2Commands'],
    'task2_course': ['Task2Course', 'Task2Commands'],
    'task2_run': ['Task2Run', 'Task2Course', 'Task2Commands'],
    'watchdog': ['CommandParser', 'CommandSession', 'CommandMotion'],
    'endpoint_tolerance': [],
    'ultrasonic_approach': ['UltrasonicApproach', 'CommandParser', 'CommandSession', 'CommandMotion'],
    'command_link': [],
    'sensor_task': ['SensorReading'],
    'hcsr04': [],
}

def main():
    env = os.environ.copy()
    env['ASAN_OPTIONS'] = env.get('ASAN_OPTIONS', '') + ':detect_leaks=0'
    with tempfile.TemporaryDirectory(prefix='command-regression-') as directory:
        for name, sources in CASES.items():
            command = shlex.split(os.environ.get('CC', 'gcc')) + [
                '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                '-IApps/Inc', '-IControllers/Inc',
            ]
            if name in ('task2_course', 'task2_run'):
                command += ['-DTASK2_COURSE_DRY_RUN=1']
            if name == 'command_link':
                command += ['-DCOMMANDLINK_DIAGNOSTICS=1', '-Ihosttests/command_link_stubs']
            if name == 'sensor_task':
                command += ['-Ihosttests/sensor_task_stubs', '-IPeripheralDrivers/Inc']
            if name == 'hcsr04':
                command += ['-Ihosttests/hcsr04_stubs', '-IPeripheralDrivers/Inc']
            command += [f'hosttests/{name}_test.c', *[f'Apps/Src/{s}.c' for s in sources]]
            if name in ('endpoint_tolerance', 'ultrasonic_approach'):
                command += ['Controllers/Src/MotionProfile.c']
            if name == 'hcsr04':
                command += ['PeripheralDrivers/Src/hcsr04.c']
            exe = str(Path(directory) / name)
            subprocess.run(command + ['-lm', '-o', exe], cwd=ROOT, check=True)
            subprocess.run([exe], env=env, check=True)
    print(f'All {len(CASES)} host regression targets passed')

if __name__ == '__main__':
    main()
