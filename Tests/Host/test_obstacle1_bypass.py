#!/usr/bin/env python3
"""Exercise isolated O1 bypass using production controllers and hardware stubs.

No servo, tyre, camera or ultrasonic plant is modelled. Optional traces describe
ideal execution, not measured robot accuracy. Requires a C compiler, no SciPy.
"""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

import test_fused_motion as baseline

ROOT = Path(__file__).resolve().parents[2]


def build_harness():
    """Reuse the historical hardware fixture; keep its full regression entry."""
    source = baseline.HARNESS.replace("int main(void)", "int legacyRegressionMain(void)")
    source = source.replace(
        "motionSequenceFusionTestExperiment != MOTION_SEQUENCE_FUSION_TEST_TASK2_TIMING) {",
        "motionSequenceFusionTestExperiment != MOTION_SEQUENCE_FUSION_TEST_TASK2_TIMING &&\n"
        "        motionSequenceFusionTestExperiment != MOTION_SEQUENCE_FUSION_TEST_OBSTACLE1_BYPASS) {",
    )
    extension = (ROOT / "Tests/Host/obstacle1_bypass_harness.inc").read_text()
    source = source.replace(
        "static uint32_t fakeTick;",
        "static void trackObstacle1Ideal(float distance, float curvature);\n"
        "static void checkObstacle1Ready(void);\nstatic uint32_t fakeTick;",
    )
    source = source.replace(
        "        gyroDps = hardwareFixture->motionController.targetCurvaturePerMm *",
        "        if (hardwareScenario == 0 && hardwareStartCount > 0 &&\n"
        "            motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_OBSTACLE1_BYPASS)\n"
        "            trackObstacle1Ideal(0.0005f * (l->measuredSpeedCps + r->measuredSpeedCps) * mmPerCount(),\n"
        "                               hardwareFixture->motionController.targetCurvaturePerMm);\n"
        "        gyroDps = hardwareFixture->motionController.targetCurvaturePerMm *",
    )
    source = source.replace(
        '    } else if (hardwareScenario == 0) {\n        assert(strstr(oledRows[1], "Task2: 7.39m"));',
        "    } else if (hardwareScenario == 0 &&\n"
        "        motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_OBSTACLE1_BYPASS) {\n"
        "        checkObstacle1Ready();\n"
        '    } else if (hardwareScenario == 0) {\n        assert(strstr(oledRows[1], "Task2: 7.39m"));',
    )
    return source + extension


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--traces", type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="obstacle1-bypass-") as directory:
        work = Path(directory)
        (work / "stm32f4xx_hal.h").write_text(
            baseline.HAL_STUB + "\nuint32_t HAL_GetTick(void);\nvoid HAL_Delay(uint32_t);\n")
        (work / "oledutils.h").write_text(
            "#include <stdint.h>\nvoid OLED_Clear(void);\nvoid OLED_Refresh_Gram(void);\n"
            "void OLED_Printf(uint8_t, uint8_t, const char *, ...);\n")
        (work / "userbutton.h").write_text(
            "void SW1_WaitForPressAndRelease(void);\nint SW1_ReadState(void);\n#define SW1_Enabled 1\n")
        (work / "test.c").write_text(build_harness())
        sources = [
            "MotionController", "MotionControllerConfig", "SteeringController",
            "SteeringControllerConfig", "PIDController", "MotionProfile",
            "RobotKinematics", "MotionSequencePlan", "MotionSequence",
        ]
        command = shlex.split(os.environ.get("CC", "gcc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic", "-O1",
            "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
            f"-I{work}", f"-I{ROOT / 'Controllers/Inc'}",
            f"-I{ROOT / 'PeripheralDrivers/Inc'}", f"-I{ROOT / 'Tests/Inc'}",
            f"-I{ROOT / 'Tests/Common'}", str(work / "test.c"),
            str(ROOT / "Tests/Src/MotionSequenceFusionTest.c"),
            str(ROOT / "Tests/Src/Task2Route.c"),
            str(ROOT / "Tests/Src/Obstacle1Route.c"),
        ]
        command += [str(ROOT / f"Controllers/Src/{name}.c") for name in sources]
        command += ["-lm", "-o", str(work / "test")]
        subprocess.run(command, check=True)
        env = os.environ.copy()
        env["ASAN_OPTIONS"] = env.get("ASAN_OPTIONS", "") + ":detect_leaks=0"
        if args.traces:
            args.traces.mkdir(parents=True, exist_ok=True)
            env["ANCHORED_TRACE_DIR"] = str(args.traces.resolve())
        subprocess.run([str(work / "test")], check=True, env=env)


if __name__ == "__main__":
    main()
