#!/usr/bin/env python3
"""Check the real GDB exporter against initialized host ELF data, without motion.
Run: python3 Tests/Host/test_fused_export.py --gdb /path/to/gdb
"""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
from test_unified_motion import HAL_STUB

ROOT = Path(__file__).resolve().parents[2]
SOURCE = r'''
#include "MotionSequenceFusionTest.h"
MotionSequenceFusionTestExperiment motionSequenceFusionTestExperiment =
    FILTER_EXPERIMENT ? MOTION_SEQUENCE_FUSION_TEST_REFERENCE_FILTER : MOTION_SEQUENCE_FUSION_TEST_TASK2_TIMING;
uint32_t motionSequenceFusionTestLogPeriodMs = FILTER_EXPERIMENT ? 20 : 100;
bool motionSequenceFusionTestPassed = true;
MotionControllerStatus motionSequenceFusionTestStatus = MOTIONCONTROLLER_STATUS_PROFILE_ERROR;
bool motionSequenceFusionTestTimedOut = true;
bool motionSequenceFusionTestCancelled = true;
bool motionSequenceFusionTestLogTruncated = true;
uint32_t motionSequenceFusionTestElapsedMs = 3456;
uint32_t motionSequenceFusionTestLogCount = SAMPLE_COUNT;
uint32_t motionSequenceFusionTestResultCount = 2;
bool motionSequenceFusionTestComparisonValid = true;
int32_t motionSequenceFusionTestSavedTimeMs = 1000;
float motionSequenceFusionTestSavedPercent = 25;
MotionSequenceFusionTestResult motionSequenceFusionTestResults[2] = {
    {.stopAfterEachSegment = false, .passed = true, .elapsedMs = 3000, .completedRuns = 1},
    {.stopAfterEachSegment = !FILTER_EXPERIMENT, .useMatchedYawRateReferenceFilter = FILTER_EXPERIMENT, .passed = true, .elapsedMs = 4000, .completedRuns = FILTER_EXPERIMENT ? 1 : 10}
};
MotionSequenceFusionTestSample motionSequenceFusionTestLog[2] = {
    {.timeMs = 20, .comparisonRunIndex = 0, .steeringCommand = 12, .sequenceTravelMm = 123, .accelerationLimitMmps2=3000, .decelerationLimitMmps2=3000},
    {.timeMs = 40, .unfilteredGeometricYawRateRadPerSec = 0.25f, .filteredGeometricYawRateRadPerSec = 0.75f, .comparisonRunIndex = 1, .steeringCommand = 13, .sequenceTravelMm = 234, .accelerationLimitMmps2=2000, .decelerationLimitMmps2=2000}
};
static const SteeringControllerCalibration calibration = {.maxCommandRatePerSec = 120};
static SteeringController steering = {.calibration = &calibration};
static WheelSpeedController left = {.pid = {.kp = 0.03f}};
static WheelSpeedController right = {.pid = {.kp = 0.03f}};
static const MotionControllerConfig fixtureConfig = {
    .straightAccelerationMmps2=3000, .straightDecelerationMmps2=3000,
    .arcAccelerationMmps2=2000, .arcDecelerationMmps2=2000
};
static MotionController motion = {.config = &fixtureConfig, .steering = &steering,
    .leftWheel=&left, .rightWheel=&right};
MotionSequence motionSequenceFusionTestSequence = {
    .controller = &motion, .state = MOTION_SEQUENCE_FAILED,
    .plan = {.count = 2, .totalTravelMm = 1234}
};
int main(void) { return 0; }
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gdb", required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="mdp-fusion-export-") as directory:
        work = Path(directory)
        (work / "stm32f4xx_hal.h").write_text(HAL_STUB)
        (work / "fixture.c").write_text(SOURCE)
        for filter_experiment, count in ((mode, count) for mode in (0, 1) for count in (0, 2, 3)):
            elf = work / "fixture"
            output = work / f"export-{filter_experiment}-{count}.txt"
            command = shlex.split(os.environ.get("CC", "gcc")) + [
                "-std=c11", "-g", "-O0", "-fno-pie", "-no-pie",
                f"-DSAMPLE_COUNT={count}", f"-DFILTER_EXPERIMENT={filter_experiment}", f"-I{work}",
                f"-I{ROOT / 'Controllers/Inc'}", f"-I{ROOT / 'PeripheralDrivers/Inc'}",
                f"-I{ROOT / 'Tests/Inc'}", str(work / "fixture.c"),
                str(ROOT / "Controllers/Src/MotionControllerConfig.c"),
                "-o", str(elf),
            ]
            subprocess.run(command, check=True)
            result = subprocess.run([
                args.gdb, "--batch", "--nx", str(elf),
                "-ex", "set pagination off",
                "-ex", f"source {ROOT / 'exp/gdb_scripts/export_fused_motion.gdb'}",
                "-ex", "set $mctrl_fusion_battery_v = 11.95",
                "-ex", f"mctrl-fusion-export {output}",
            ], capture_output=True, text=True)
            assert result.returncode == 0, result.stdout + result.stderr
            log = output.read_text()
            assert log.rstrip().endswith("=== EXPORT COMPLETE ==="), log
            assert "totalTravelMm = 1234" in log and "maxCommandRatePerSec = 120" in log
            assert "MOTIONCONTROLLER_STATUS_PROFILE_ERROR" in log
            assert "straightAccelerationMmps2 = 3000" in log
            assert "straightDecelerationMmps2 = 3000" in log
            assert "arcAccelerationMmps2 = 2000" in log
            assert "arcDecelerationMmps2 = 2000" in log
            assert log.count("kp = 0.0299999993") == 2
            assert "elapsedMs = 3000" in log and "elapsedMs = 4000" in log
            assert "completedRuns = 1" in log
            if filter_experiment:
                assert "A/B reference filter comparison" in log
                assert "MOTION_SEQUENCE_FUSION_TEST_REFERENCE_FILTER" in log
                assert "useMatchedYawRateReferenceFilter = true" in log
                assert "completedRuns = 10" not in log
            else:
                assert "A/B timing comparison" in log
                assert "MOTION_SEQUENCE_FUSION_TEST_TASK2_TIMING" in log
                assert "completedRuns = 10" in log
            if count == 2:
                assert "steeringCommand = 12" in log and "steeringCommand = 13" in log
                assert "comparisonRunIndex = 0" in log and "comparisonRunIndex = 1" in log
                assert "unfilteredGeometricYawRateRadPerSec = 0.25" in log
                assert "filteredGeometricYawRateRadPerSec = 0.75" in log
                assert "No samples, or invalid count" not in log
            else:
                assert "No samples, or invalid count" in log
                assert "steeringCommand = 12" not in log
    print("PASS: real fusion GDB export; complete marker, plan/tuning/flags, "
          "all samples, empty/invalid count guards; no probe or motion")


if __name__ == "__main__":
    main()
