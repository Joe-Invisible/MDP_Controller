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
    EXPERIMENT_MODE;
uint32_t motionSequenceFusionTestLogPeriodMs = EXPERIMENT_MODE ? 20 : 100;
bool motionSequenceFusionTestPassed = true;
MotionControllerStatus motionSequenceFusionTestStatus = MOTIONCONTROLLER_STATUS_PROFILE_ERROR;
bool motionSequenceFusionTestTimedOut = true;
bool motionSequenceFusionTestCancelled = true;
bool motionSequenceFusionTestLogTruncated = true;
uint32_t motionSequenceFusionTestElapsedMs = 3456;
uint32_t motionSequenceFusionTestLogCount = SAMPLE_COUNT;
uint32_t motionSequenceFusionTestResultCount = RESULT_COUNT;
bool motionSequenceFusionTestComparisonValid = EXPERIMENT_MODE != 3;
int32_t motionSequenceFusionTestSavedTimeMs = EXPERIMENT_MODE == 3 ? 0 : 1000;
float motionSequenceFusionTestSavedPercent = EXPERIMENT_MODE == 3 ? 0 : 25;
MotionSequenceFusionTestResult motionSequenceFusionTestResults[2] = {
    {.stopAfterEachSegment = false, .useMatchedYawRateReferenceFilter = EXPERIMENT_MODE >= 2, .passed = true, .elapsedMs = 3000, .completedRuns = 1,
     .straightTuning = {EXPERIMENT_MODE == 3 ? 150 : 100,0,0,0}, .arcTuning = {EXPERIMENT_MODE == 3 ? 150 : 100,0,0,0}},
    {.stopAfterEachSegment = !EXPERIMENT_MODE, .useMatchedYawRateReferenceFilter = EXPERIMENT_MODE != 0, .passed = true, .elapsedMs = 4000, .completedRuns = EXPERIMENT_MODE ? 1 : 10,
     .straightTuning = {EXPERIMENT_MODE == 2 ? 270 : 100, EXPERIMENT_MODE == 2 ? 150 : 0, 0, 0},
     .arcTuning = {EXPERIMENT_MODE == 2 ? 270 : 100, EXPERIMENT_MODE == 2 ? 150 : 0, 0, 0}}
};
MotionSequenceFusionTestSample motionSequenceFusionTestLog[2] = {
    {.timeMs = 20, .comparisonRunIndex = 0, .steeringCommand = 12, .sequenceTravelMm = 123, .accelerationLimitMmps2=3000, .decelerationLimitMmps2=3000},
    {.timeMs = 40, .unfilteredGeometricYawRateRadPerSec = 0.25f, .filteredGeometricYawRateRadPerSec = 0.75f, .comparisonRunIndex = EXPERIMENT_MODE == 3 ? 0 : 1, .steeringCommand = 13, .sequenceTravelMm = 234, .accelerationLimitMmps2=2000, .decelerationLimitMmps2=2000}
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
        cases = [(mode, count, 1 if mode == 3 else 2)
                 for mode in (0, 1, 2, 3) for count in (0, 2, 3)]
        cases += [(3, 2, 0), (3, 2, 3)]
        for experiment_mode, count, result_count in cases:
            elf = work / "fixture"
            output = work / f"export-{experiment_mode}-{count}-{result_count}.txt"
            command = shlex.split(os.environ.get("CC", "gcc")) + [
                "-std=c11", "-g", "-O0", "-fno-pie", "-no-pie",
                f"-DSAMPLE_COUNT={count}", f"-DEXPERIMENT_MODE={experiment_mode}",
                f"-DRESULT_COUNT={result_count}", f"-I{work}",
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
            valid_results = 0 < result_count <= 2
            if valid_results:
                assert "elapsedMs = 3000" in log and "completedRuns = 1" in log
                assert ("elapsedMs = 4000" in log) == (result_count == 2)
            else:
                assert "No results, or invalid count" in log
                assert "elapsedMs = 3000" not in log and "elapsedMs = 4000" not in log
            if experiment_mode == 3:
                assert "Single tuning run" in log and "A/B" not in log
                assert "MOTION_SEQUENCE_FUSION_TEST_SINGLE_TUNING" in log
                if valid_results:
                    assert "straightTuning = {yawRateKp = 150, yawRateKi = 0, yawRateKd = 0, headingKpPerSec = 0}" in log
                    assert "arcTuning = {yawRateKp = 150, yawRateKi = 0, yawRateKd = 0, headingKpPerSec = 0}" in log
                assert "completedRuns = 10" not in log
            elif experiment_mode == 2:
                assert "A/B tuning comparison" in log
                assert "MOTION_SEQUENCE_FUSION_TEST_TASK1_GAINS" in log
                assert "straightTuning = {yawRateKp = 270, yawRateKi = 150, yawRateKd = 0, headingKpPerSec = 0}" in log
                assert "arcTuning = {yawRateKp = 270, yawRateKi = 150, yawRateKd = 0, headingKpPerSec = 0}" in log
                assert "completedRuns = 10" not in log
            elif experiment_mode == 1:
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
                assert "comparisonRunIndex = 0" in log
                assert ("comparisonRunIndex = 1" in log) == (experiment_mode != 3)
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
