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
Task2MotionTestConfig task2MotionTestConfig = {.measureObstacle1AtStartup=true};
Task2ObstacleSensorConfig task2MotionTestSensorConfig = {.ultrasonic={210,0,0},.leftIR={155,37.5,90},.rightIR={135,-33.5,-90},.irPresentAdc=700,.irAbsentAdc=400,.irAcquisitionMinYawDeg=80,.samplePeriodMs=60};
Task2ObstacleDetection task2MotionTestDetection = {.detected=true,.observedWidthMm=540};
Task2MotionTestStartupRange task2MotionTestStartupRange = {.attempted=true,.valid=true,.sensorToObstacleMm=1090,.parkToObstacle1Mm=1050};
Task2MotionTestState task2MotionTestState = TASK2_TEST_COMPLETE;
Task2MotionTestResult task2MotionTestResult = {.passed=true,.completedSteps=8,.elapsedMs=17535,.routeSetupMs=13,.returnSetupMs=7,.parallelReturnSetupMs=11,.startupPlannerPoseChecks=100,.arrow1Recognised=true,.arrow2Recognised=true,.arrow2RecognitionElapsedMs=3210};
Task2Route task2MotionTestRoute = {.config={.parkToObstacle1Mm=1050,.recognitionCameraRangeMm=1000},.totalTravelMm=7780,.obstacle2IRDeparture=true};
Task2MotionTestStepResult task2MotionTestStepResults[TASK2_ROUTE_STEP_COUNT] = {{.measuredDistanceMm=790,.yawPredictedReached=true}};
MotionControllerConfig task2MotionTestControllerConfig = {.straightYawRateKp=270,.arcYawRateKp=270};
Task2MotionTestSample task2MotionTestLog[TASK2_MOTION_TEST_LOG_CAPACITY] = {{.timeMs=100,.stepIndex=0,.targetSpeedCps=2000},{.timeMs=200,.stepIndex=4,.targetSpeedCps=5000,.irObserved=true,.irFresh=true,.irValid=true,.irArmed=true,.irAdc=395,.irAbsentSamples=2}};
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
            assert "aheadMm = 210" in log
            assert "irPresentAdc = 700" in log and "irAbsentAdc = 400" in log
            assert "irAcquisitionMinYawDeg = 80" in log and "irAcquisitionTimedOut = false" in log
            assert "leftMm = 37.5" in log and "leftMm = -33.5" in log
            assert "sensorToObstacleMm = 1090" in log
            assert "observedWidthMm = 540" in log and "samplePeriodMs = 60" in log
            assert "routeSetupMs = 13" in log and "returnSetupMs = 7" in log
            assert "startupPlannerPoseChecks = 100" in log
            assert "parallelReturnSetupMs = 11" in log
            assert "R275 quarter to 0" in log and "direct rear arc" in log
            assert "obstacle2ParkReturnPending" in log and "irTrendCandidateValid" in log and "edgeMethod" in log
            assert "parkingReturnSetupMs" in log and "obstacle2RearStoppedPose" in log
            assert "obstacle2ParallelReturnPending" in log and "obstacleBufferExceptionUsed" in log
            assert "arrow2RecognitionElapsedMs = 3210" in log
            assert "TASK2_TEST_COMPLETE" in log and "completedSteps = 8" in log
            assert "totalTravelMm = 7780" in log
            assert "Return park" in log and "yawPredictedReached = true" in log
            assert "IR arc (90 deg cap; IR disabled)" in log and "ADC trace:" in log
            if count == 2:
                assert "targetSpeedCps = 2000" in log and "targetSpeedCps = 5000" in log
                assert "irObserved = true" in log and "irArmed = true" in log
                assert "irFresh = true" in log and "irValid = true" in log
                assert "irAdc = 395" in log and "irAbsentSamples = 2" in log
            else:
                assert "No samples, or invalid count" in log
                assert "targetSpeedCps = 5000" not in log
        print("PASS: standalone export, range/config/route/results, complete samples and invalid-count guards")

if __name__ == "__main__":
    main()
