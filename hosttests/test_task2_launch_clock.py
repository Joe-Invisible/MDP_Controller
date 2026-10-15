#!/usr/bin/env python3
"""MotionTask's control-loop clock around Task 2 launches, run on the host.

The lines from `uint32_t now = osKernelGetTickCount();` to `previousUpdate = now;` are taken unchanged from
Apps/Src/MotionTask.c and run against a fake tick. A Task 2 update can plan for hundreds of ms before it
launches a move; that wait must not reach the controller as time the new move has spent (it would skip the
steering settle), while time spent during an ongoing move must. Every Task 2 launch path must also flag
its launch.
"""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define MOTIONTASK_CONTROL_PERIOD_S 0.01f
static uint32_t tick;
static uint32_t osKernelGetTickCount(void) { return tick; }
static uint32_t osKernelGetTickFreq(void) { return 1000U; }
static bool task2Launched;

/* One pass of the loop's clock, after the Task 2 update; returns the dt handed to the controller. */
static float Pass(uint32_t *previousUpdatePointer) {
    uint32_t previousUpdate = *previousUpdatePointer;
    float dt;
SNIPPET
    *previousUpdatePointer = previousUpdate;
    return dt;
}

int main(void) {
    uint32_t previous = 0U;
    tick = 10U; float dt = Pass(&previous);
    assert(dt > 0.0099f && dt < 0.0101f);
    /* Idle, then a Task 2 update plans for 200 or 600 ms and launches a move: its first dt is one period. */
    const uint32_t waits[] = {200U, 600U};
    for (unsigned i = 0U; i < 2U; ++i) {
        tick += waits[i];
        task2Launched = true;
        dt = Pass(&previous);
        printf("launch after %u ms of planning: first dt %.3f s\n", waits[i], dt);
        assert(dt > 0.0099f && dt < 0.0101f && !task2Launched);
        /* The move then runs: the next passes see real elapsed time, including a slow one. */
        tick += 10U; dt = Pass(&previous); assert(dt > 0.0099f && dt < 0.0101f);
        tick += 35U; dt = Pass(&previous); assert(dt > 0.0349f && dt < 0.0351f);
    }
    /* No launch: a long pass during an ongoing move keeps its whole elapsed time. */
    tick += 250U; dt = Pass(&previous);
    assert(dt > 0.2499f && dt < 0.2501f);
    puts("task2 launch clock: PASS");
    return 0;
}
'''


def main():
    text = (ROOT / "Apps/Src/MotionTask.c").read_text()
    start = text.index("        uint32_t now = osKernelGetTickCount();")
    after_dt = text.index("float dt = ", start)     # the loop's own update follows its dt
    end = text.index("        previousUpdate = now;", after_dt) + len("        previousUpdate = now;")
    snippet = text[start:end].replace("float dt = ", "dt = ")
    assert "task2Launched" in snippet, "the loop no longer resets its clock on a Task 2 launch"
    # Every Task 2 launch path flags its launch.
    for name in ("MotionTask_Task2Move", "MotionTask_Task2Continuous"):
        body = text[text.index("static const char *%s(" % name):]
        body = body[:body.index("\n}\n")]
        assert re.search(r"MotionController_Move\w*\(", body) and "task2Launched |= " in body, name
    program = HARNESS.replace("SNIPPET", snippet)
    with tempfile.TemporaryDirectory(prefix="task2-launch-clock-") as directory:
        path, exe = Path(directory) / "clock.c", Path(directory) / "clock"
        path.write_text(program)
        subprocess.run(shlex.split(os.environ.get("CC", "gcc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined", str(path), "-o", str(exe)],
            check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
