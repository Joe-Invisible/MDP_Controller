#!/usr/bin/env python3
"""The run trace end to end: the STM's encoder (Task2Diagnostics.c) against the Pi's decoder (task2protocol.py).

Builds a small program that fills a trace with edge values (negative poses, +/-180 deg, no ultrasonic or IR reading,
full-scale ADC, the final-sample state), prints its "D R" index and every block, and decodes each line with the Pi's own parser.
Set MDP_CHECKLIST to the Pi project (default ~/workspace/mdp/mdp_checklist); without it the test is skipped.
"""
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CHECKLIST = Path(os.environ.get("MDP_CHECKLIST", os.path.expanduser("~/workspace/mdp/mdp_checklist")))

PROGRAM = r'''
#include <stdio.h>
#include "Task2Diagnostics.h"
static Task2Trace trace;
static char lines[TASK2_DIAG_PARTS][TASK2_DIAG_LINE_SIZE];
int main(void) {
    Task2Trace_Init(&trace);
    for (unsigned i = 0U; i < 61U; ++i) {
        Task2TraceSample s = {.t10Ms = (uint16_t)(5U * i), .xMm = (int16_t)(-250 + 37 * (int)i),
            .yMm = (int16_t)(i % 2U ? -301 : 299), .yawCentiDeg = (int16_t)(i == 3U ? -18000 : i == 4U ? 18000 : 4512),
            .phase = (uint8_t)(i % 12U), .step = (uint8_t)i, .flags = (uint8_t)(1U << (i % 8U)), .state = (uint8_t)(i == 60U),
            .ussMm = (uint16_t)(i == 5U ? 65535U : 20U + i), .irLeft = (uint16_t)(i == 6U ? 4095U : i == 7U ? 65535U : 300U + i),
            .irRight = (uint16_t)(1000U + i), .faceLevel = (uint16_t)(i < 10U ? 0U : 1300U)};
        Task2Trace_Add(&trace, &s);
    }
    unsigned n = Task2Trace_Reply(&trace, 12U, "D R", lines, TASK2_DIAG_PARTS);
    for (unsigned k = 0U; k < n; ++k) fputs(lines[k], stdout);
    for (unsigned b = 0U; ; ++b) {
        char request[16];
        snprintf(request, sizeof request, "D R %u", b);
        n = Task2Trace_Reply(&trace, 12U, request, lines, TASK2_DIAG_PARTS);
        if (!n) break;
        for (unsigned k = 0U; k < n; ++k) fputs(lines[k], stdout);
    }
    return 0;
}
'''


def expected(i):
    return [50 * i, -250 + 37 * i, -301 if i % 2 else 299, -180.0 if i == 3 else 180.0 if i == 4 else 45.12,
            i % 12, i, 1 << (i % 8), 1 if i == 60 else 0, None if i == 5 else 20 + i,
            4095 if i == 6 else None if i == 7 else 300 + i, 1000 + i,
            0 if i < 10 else 1300]


def main():
    if not (CHECKLIST / "robot/core/task2protocol.py").exists():
        print("trace decode: SKIP (no Pi project at %s)" % CHECKLIST)
        return
    sys.path.insert(0, str(CHECKLIST))
    from robot.core import task2protocol
    with tempfile.TemporaryDirectory(prefix="task2-trace-") as directory:
        source, exe = Path(directory) / "trace.c", Path(directory) / "trace"
        source.write_text(PROGRAM)
        subprocess.run(shlex.split(os.environ.get("CC", "gcc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined", "-I" + str(ROOT / "Apps/Inc"),
            str(source), str(ROOT / "Apps/Src/Task2Diagnostics.c"), "-lm", "-o", str(exe)], check=True)
        out = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout.splitlines()
    index = task2protocol.parse(out[0])
    assert index["kind"] == "trace_index" and index["run"] == 12 and index["n"] == 61, index
    assert index["per"] * index["lines"] == 28 and index["blocks"] == 3 and index["period"] == 50, index
    samples = []
    for line in out[1:]:
        assert len(line) <= 200, len(line)
        event = task2protocol.parse(line)
        assert event["kind"] == "trace" and event["run"] == 12 and event["first"] == len(samples), event
        samples.extend(event["samples"])
    assert len(samples) == 61
    for i, row in enumerate(samples):
        assert row == expected(i), (i, row, expected(i))
    print("trace decode: PASS %d samples in %d lines, C encoder and Pi decoder agree" % (len(samples), len(out) - 1))


if __name__ == "__main__":
    main()
