#!/usr/bin/env python3
"""MotionTask's UART line reader and the Task 2 planning stop drain, run on the host.

The functions are taken unchanged from Apps/Src/MotionTask.c (from MOTIONTASK_MAX_FRAME_LEN through
MotionTask_Task2PlanStop) and run against an in-memory UART, so a stop behind a long, discarded or
budget-splitting line is checked at the parser boundary.
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
#include <string.h>
#include "Task2PlanFrames.h"
#define TASK2_PLAN_CANCELLED ((size_t)-1)

static uint8_t uart[4096];
static size_t uartLength, uartAt;
static unsigned sent;
static uint32_t CommandLink_ReadBytes(uint8_t *out, uint32_t n, uint32_t timeout) {
    (void)timeout; uint32_t k = 0;
    while (k < n && uartAt < uartLength) out[k++] = uart[uartAt++];
    return k;
}
static uint32_t CommandLink_GetDropCount(void) { return 0; }
static void CommandLink_Send(const char *text) { (void)text; ++sent; }
static void Feed(const char *text) { size_t n = strlen(text); memcpy(uart + uartLength, text, n); uartLength += n; }
static void Reset(void) { uartLength = uartAt = 0; }
static uint32_t HAL_GetTick(void) { static uint32_t t; return t += 10; }
static void osDelay(uint32_t ms) { (void)ms; }

SOURCE

static unsigned Deferred(const char *line) {
    unsigned n = 0;
    for (unsigned i = 0; i < deferredCount; ++i)
        if (strcmp(deferredLines[(deferredHead + i) % MOTIONTASK_DEFERRED_LINES], line) == 0) ++n;
    return n;
}
static void Clear(void) { deferredHead = deferredCount = 0; }

int main(void) {
    /* A stop behind lines of every awkward length: short, exactly the frame limit, past it, very long,
     * and one with a NUL; also behind a complete reply frame for another request. */
    const unsigned lengths[] = {10, 63, 64, 65, 80, 200};
    for (unsigned i = 0; i < sizeof lengths / sizeof lengths[0]; ++i) {
        Reset(); Clear(); Task2PlanFrames_Begin(&planFrames, 7);
        char line[256]; memset(line, 'X', lengths[i]); line[lengths[i]] = '\0';
        Feed(line); Feed("\n"); Feed("S\n");
        bool stop = MotionTask_Task2PlanStop();
        printf("line %3u: stop %d unread %zu\n", lengths[i], stop, uartLength - uartAt);
        assert(stop && uartAt == uartLength && Deferred("S") == 1);
    }
    Reset(); Clear(); Task2PlanFrames_Begin(&planFrames, 7);
    Feed("ABC"); uart[uartLength++] = 0; Feed("DEF\nPR 6 0 1 00\nQ T\nS\n");
    assert(MotionTask_Task2PlanStop() && uartAt == uartLength && Deferred("S") == 1 && Deferred("Q T") == 1);
    /* Nothing waiting: no stop, nothing deferred. */
    Reset(); Clear(); Task2PlanFrames_Begin(&planFrames, 7);
    assert(!MotionTask_Task2PlanStop() && deferredCount == 0);
    /* A peer that never stops sending cannot hold the drain: it reads at most its bound. */
    Reset(); Clear(); Task2PlanFrames_Begin(&planFrames, 7);
    for (unsigned i = 0; i < 60; ++i) Feed("Q T\n");
    Feed("S\n");
    /* A full receive buffer of short lines is inspected to the end. */
    bool stop = MotionTask_Task2PlanStop();
    assert(stop && uartAt == uartLength);
    /* A full buffer of empty lines, then the stop. */
    Reset(); Clear(); Task2PlanFrames_Begin(&planFrames, 7);
    for (unsigned i = 0; i < 254; ++i) Feed("\n");
    Feed("S\n");
    assert(MotionTask_Task2PlanStop() && uartAt == uartLength && Deferred("S") == 1);
    /* A peer that never stops sending cannot hold the drain: it makes at most its bounded reads. */
    Reset(); Clear(); Task2PlanFrames_Begin(&planFrames, 7);
    for (unsigned i = 0; i < 1000; ++i) Feed("Q\n");
    (void)MotionTask_Task2PlanStop();
    assert(uartAt < uartLength && uartAt <= TASK2_PLAN_DRAIN_READS * MOTIONTASK_RX_BYTES_PER_TICK);
    printf("flood: read %zu of %zu bytes, then returned\n", uartAt, uartLength);
    puts("task2 transport: PASS");
    return 0;
}
'''


def main():
    text = (ROOT / 'Apps/Src/MotionTask.c').read_text()
    start = text.index('#define MOTIONTASK_MAX_FRAME_LEN')
    stop_fn = text.index('static bool MotionTask_Task2PlanStop(void)')
    end = text.index('\n}\n', stop_fn) + 3
    source = text[start:end]
    # The plan-host block opens with #if; close it, and give the drain the constants it uses.
    source = source.replace('#if TASK2_PLAN_HOST', '#if 1') + '\n#endif\n'
    source = re.sub(r'static (size_t|bool) MotionTask_Task2Plan\(', r'static \1 Unused(', source)
    program = HARNESS.replace('SOURCE', source)
    with tempfile.TemporaryDirectory(prefix='task2-transport-') as directory:
        path = Path(directory) / 'transport.c'
        path.write_text(program)
        exe = Path(directory) / 'transport'
        subprocess.run(shlex.split(os.environ.get('CC', 'gcc')) + [
            '-std=c11', '-Wall', '-Wextra', '-Wno-unused-function', '-fsanitize=address,undefined',
            '-I' + str(ROOT / 'Apps/Inc'), str(path), str(ROOT / 'Apps/Src/Task2PlanFrames.c'),
            str(ROOT / 'Apps/Src/Task2Wire.c'), str(ROOT / 'Apps/Src/Task2Route.c'), '-lm', '-o', str(exe)],
            check=True)
        env = os.environ.copy()
        env['ASAN_OPTIONS'] = env.get('ASAN_OPTIONS', '') + ':detect_leaks=0'
        subprocess.run([str(exe)], check=True, env=env)


if __name__ == '__main__':
    main()
