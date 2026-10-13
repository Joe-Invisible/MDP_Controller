#!/usr/bin/env python3
"""Exercise the stopped Task 2 harness with an ideal encoder/gyro fixture."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
from test_unified_motion import HAL_STUB, FIXTURE_HARNESS

ROOT = Path(__file__).resolve().parents[2]

def main():
    # Reuse hardware/controller fixture helpers, without the unrelated test cases.
    helpers = FIXTURE_HARNESS.split("static void testStraightSignsAndGeometry")[0]
    harness = helpers + Path(__file__).with_name("task2_motion_harness.inc").read_text()
    with tempfile.TemporaryDirectory(prefix="task2-stopped-") as directory:
        work = Path(directory)
        (work / "stm32f4xx_hal.h").write_text(HAL_STUB + "\nuint32_t HAL_GetTick(void);\nvoid HAL_Delay(uint32_t);\n")
        (work / "oledutils.h").write_text("#include <stdint.h>\nvoid OLED_Clear(void);\nvoid OLED_Refresh_Gram(void);\nvoid OLED_Printf(uint8_t,uint8_t,const char *,...);\n")
        (work / "userbutton.h").write_text("void SW1_WaitForPressAndRelease(void);\nint SW1_ReadState(void);\n#define SW1_Enabled 1\n")
        (work / "test.c").write_text(harness)
        sources = ["MotionController", "MotionControllerConfig", "SteeringController", "SteeringControllerConfig", "PIDController", "MotionProfile", "RobotKinematics"]
        command = shlex.split(os.environ.get("CC", "gcc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function", "-pedantic", "-O1",
            "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
            f"-I{work}", f"-I{ROOT/'Controllers/Inc'}", f"-I{ROOT/'PeripheralDrivers/Inc'}",
            f"-I{ROOT/'Tests/Inc'}", f"-I{ROOT/'Tests/Common'}", str(work/"test.c"),
            str(ROOT/"Tests/Src/Task2MotionTest.c"), str(ROOT/"Tests/Src/Task2Route.c"), str(ROOT/"Tests/Src/Task2ObstacleDetection.c"),
        ] + [str(ROOT/f"Controllers/Src/{name}.c") for name in sources] + ["-lm", "-o", str(work/"test")]
        subprocess.run(command, check=True)
        env = os.environ.copy()
        env["ASAN_OPTIONS"] = env.get("ASAN_OPTIONS", "") + ":detect_leaks=0"
        subprocess.run([str(work/"test")], env=env, check=True)

if __name__ == "__main__":
    main()
