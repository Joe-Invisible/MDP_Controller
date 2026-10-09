#!/usr/bin/env python3
"""Build and run hardware-independent sweep checks with the host C compiler."""
from pathlib import Path
import os
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix="wheel-performance-") as temp:
    binary = Path(temp) / "test_sweep"
    subprocess.run([
        "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-g",
        "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        "-Itools/wheel_performance/tests/stubs", "-IControllers/Inc", "-ITests/Inc",
        "tools/wheel_performance/tests/test_sweep.c",
        "Controllers/Src/WheelSpeedController.c", "Controllers/Src/PIDController.c",
        "Controllers/Src/WheelSpeedControllerConfig.c", "Controllers/Src/DynamicBrakeMap.c",
        "Controllers/Src/WheelBrakeConfig.c", "Controllers/Src/MotionControllerConfig.c",
        "-lm", "-o", str(binary)
    ], cwd=root, check=True)
    # The harness uses static storage; check memory access and UB, without LSAN's
    # process inspection (unavailable in some containers/debugger environments).
    environment = os.environ.copy()
    environment.setdefault("ASAN_OPTIONS", "detect_leaks=0")
    subprocess.run([str(binary)], check=True, env=environment)
subprocess.run([sys.executable, "tools/wheel_performance/tests/test_export.py"],
               cwd=root, check=True)
