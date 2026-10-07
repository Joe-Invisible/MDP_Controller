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
    {.stopAfterEachSegment = true, .passed = true, .elapsedMs = 4000, .completedRuns = 12}
};
MotionSequenceFusionTestSample motionSequenceFusionTestLog[2] = {
    {.timeMs = 20, .comparisonRunIndex = 0, .steeringCommand = 12, .sequenceTravelMm = 123},
    {.timeMs = 40, .comparisonRunIndex = 1, .steeringCommand = 13, .sequenceTravelMm = 234}
};
static const SteeringControllerCalibration calibration = {.maxCommandRatePerSec = 120};
static SteeringController steering = {.calibration = &calibration};
static MotionController motion = {.config = &motionControllerConfig, .steering = &steering};
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
        for count in (0, 2, 3):
            elf = work / "fixture"
            output = work / f"export-{count}.txt"
            command = shlex.split(os.environ.get("CC", "gcc")) + [
                "-std=c11", "-g", "-O0", "-fno-pie", "-no-pie",
                f"-DSAMPLE_COUNT={count}", f"-I{work}",
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
            assert "A/B timing comparison" in log
            assert "elapsedMs = 3000" in log and "elapsedMs = 4000" in log
            assert "completedRuns = 1" in log and "completedRuns = 12" in log
            if count == 2:
                assert "steeringCommand = 12" in log and "steeringCommand = 13" in log
                assert "comparisonRunIndex = 0" in log and "comparisonRunIndex = 1" in log
                assert "No samples, or invalid count" not in log
            else:
                assert "No samples, or invalid count" in log
                assert "steeringCommand = 12" not in log
    print("PASS: real fusion GDB export; complete marker, plan/tuning/flags, "
          "all samples, empty/invalid count guards; no probe or motion")


if __name__ == "__main__":
    main()
