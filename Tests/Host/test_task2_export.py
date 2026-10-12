#!/usr/bin/env python3
"""Check the real standalone GDB exporter without hardware or target motion."""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
from test_unified_motion import HAL_STUB

ROOT = Path(__file__).resolve().parents[2]
SOURCE = r'''
#include "Task2MotionTest.h"
Task2MotionTestConfig task2MotionTestConfig = {.measureObstacle1AtStartup=true,.sensorAheadOfRearAxleMm=210};
Task2MotionTestStartupRange task2MotionTestStartupRange = {.attempted=true,.valid=true,.sensorToObstacleMm=1090,.parkToObstacle1Mm=1050};
Task2MotionTestState task2MotionTestState = TASK2_TEST_COMPLETE;
Task2MotionTestResult task2MotionTestResult = {.passed=true,.completedSteps=8,.elapsedMs=17535,.arrow1Recognised=true};
Task2Route task2MotionTestRoute = {.config={.parkToObstacle1Mm=1050,.recognitionCameraRangeMm=1000},.totalTravelMm=7780};
Task2MotionTestStepResult task2MotionTestStepResults[8] = {{.measuredDistanceMm=790,.yawPredictedReached=true}};
MotionControllerConfig task2MotionTestControllerConfig = {.straightYawRateKp=270,.arcYawRateKp=270};
Task2MotionTestSample task2MotionTestLog[TASK2_MOTION_TEST_LOG_CAPACITY] = {{.timeMs=100,.stepIndex=0,.targetSpeedCps=2000},{.timeMs=200,.stepIndex=0,.targetSpeedCps=5000}};
uint32_t task2MotionTestLogCount = SAMPLE_COUNT;
int main(void) { return 0; }
'''

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gdb", required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="task2-export-") as directory:
        work = Path(directory)
        (work/"stm32f4xx_hal.h").write_text(HAL_STUB)
        (work/"fixture.c").write_text(SOURCE)
        for count in (0, 2, 651):
            elf, output = work/"fixture", work/f"export-{count}.txt"
            command = shlex.split(os.environ.get("CC", "gcc")) + [
                "-std=c11", "-g", "-O0", "-fno-pie", "-no-pie", f"-DSAMPLE_COUNT={count}",
                f"-I{work}", f"-I{ROOT/'Controllers/Inc'}", f"-I{ROOT/'PeripheralDrivers/Inc'}",
                f"-I{ROOT/'Tests/Inc'}", str(work/"fixture.c"), "-o", str(elf)]
            subprocess.run(command, check=True)
            result = subprocess.run([args.gdb,"-q","-batch",str(elf),"-ex","set pagination off",
                "-ex",f"source {ROOT/'exp/gdb_scripts/export_task2_motion.gdb'}",
                "-ex",f"task2-motion-export {output}"],capture_output=True,text=True)
            assert result.returncode == 0, result.stderr + result.stdout
            log = output.read_text()
            assert log.rstrip().endswith("=== EXPORT COMPLETE ===")
            assert "sensorAheadOfRearAxleMm = 210" in log
            assert "sensorToObstacleMm = 1090" in log
            assert "TASK2_TEST_COMPLETE" in log and "completedSteps = 8" in log
            assert "totalTravelMm = 7780" in log
            assert "Return park" in log and "yawPredictedReached = true" in log
            if count == 2:
                assert "targetSpeedCps = 2000" in log and "targetSpeedCps = 5000" in log
            else:
                assert "No samples, or invalid count" in log
                assert "targetSpeedCps = 5000" not in log
        print("PASS: standalone export, range/config/route/results, complete samples and invalid-count guards")

if __name__ == "__main__":
    main()
