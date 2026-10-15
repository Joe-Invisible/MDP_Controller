#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "Task2PlanFrames.h"
#include "Task2Wire.h"

static Task2PlanFrames f;
static char frames[TASK2_PLAN_MAX_FRAMES][96];

/* Split bytes the way the Pi does: 24 bytes per frame, the last one shorter. */
static unsigned Split(uint16_t g, const uint8_t *bytes, size_t length) {
    unsigned n = (unsigned)((length + TASK2_PLAN_FRAME_BYTES - 1U) / TASK2_PLAN_FRAME_BYTES);
    for (unsigned k = 0U; k < n; ++k) {
        size_t at = (size_t)k * TASK2_PLAN_FRAME_BYTES, size = length - at < TASK2_PLAN_FRAME_BYTES ? length - at : TASK2_PLAN_FRAME_BYTES;
        char hex[2U * TASK2_PLAN_FRAME_BYTES + 1U];
        assert(Task2Wire_ToHex(bytes + at, size, hex, sizeof(hex)) == 2U * size);
        (void)snprintf(frames[k], sizeof(frames[k]), "PR %u %u %u %s", (unsigned)g, k, n, hex);
        assert(strlen(frames[k]) <= 64U);
    }
    return n;
}

int main(void) {
    Task2PlanFrames_Begin(&f, 7U);
    assert(Task2PlanFrames_Feed(&f, "S") == TASK2_PLAN_FRAME_STOP);
    assert(Task2PlanFrames_Feed(&f, "A 3 2 L") == TASK2_PLAN_FRAME_OTHER);
    assert(Task2PlanFrames_Feed(&f, "Q T") == TASK2_PLAN_FRAME_OTHER);
    assert(Task2PlanFrames_Feed(&f, "PR 6 0 1 00") == TASK2_PLAN_FRAME_STALE);
    assert(Task2PlanFrames_Feed(&f, "PR 7 0 0") == TASK2_PLAN_FRAME_NONE);
    const char *bad[] = {"PR 7 0 0 x", "PR -7 0 1 00", "PR 7 0", "PR 7 0 1", "PR 7 0 1 ", "PR 7 1 1 00",
        "PR 7 0 1 0", "PR 7 0 1 zz", "PR 99999999999 0 1 00", "PR 7 0 9999 00", "PR 7 0 44 00", "PR 7 0 2 00",
        "PR 7 0 1 0000000000000000000000000000000000000000000000000000", "PR 7 +0 1 00", "PR  7 0 1 00"};
    for (unsigned i = 0U; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        Task2PlanFrames_Begin(&f, 7U);
        Task2PlanFrame got = Task2PlanFrames_Feed(&f, bad[i]);
        if (got != TASK2_PLAN_FRAME_BAD) { fprintf(stderr, "accepted: %s (%d)\n", bad[i], (int)got); return 1; }
    }

    /* A real reply, frames out of order with a harmless repeat, decodes back to the same reply. */
    static Task2WireReply sent, back;
    sent = (Task2WireReply){.run = 3U, .generation = 7U, .ok = true, .checks = 1234U, .request = 0xDEADBEEFU};
    sent.route.config = task2RouteConfig;
    sent.route.stepCount = 9U;
    sent.route.steps[8] = (Task2RouteStep){.distanceMm = 431.5f, .radiusMm = -275.0f, .speedCps = 3000.0f};
    sent.route.report.reason = "OK_PARKING_RETURN";
    static uint8_t bytes[TASK2_WIRE_MAX_BYTES];
    size_t length = Task2Wire_EncodeReply(&sent, bytes, sizeof(bytes));
    assert(length > 2U * TASK2_PLAN_FRAME_BYTES);
    unsigned n = Split(7U, bytes, length);
    Task2PlanFrames_Begin(&f, 7U);
    assert(Task2PlanFrames_Feed(&f, frames[n - 1U]) == TASK2_PLAN_FRAME_MORE);
    assert(Task2PlanFrames_Feed(&f, frames[n - 1U]) == TASK2_PLAN_FRAME_MORE);   /* identical repeat */
    assert(Task2PlanFrames_Feed(&f, "PR 6 0 1 00") == TASK2_PLAN_FRAME_STALE);   /* older request */
    for (unsigned k = 0U; k + 2U < n; ++k) assert(Task2PlanFrames_Feed(&f, frames[k]) == TASK2_PLAN_FRAME_MORE);
    assert(Task2PlanFrames_Feed(&f, frames[n - 2U]) == TASK2_PLAN_FRAME_DONE);
    assert(Task2PlanFrames_Length(&f) == length);
    assert(memcmp(f.data, bytes, length) == 0);
    assert(Task2Wire_DecodeReply(f.data, length, &back));
    assert(back.generation == 7U && back.request == 0xDEADBEEFU && back.route.stepCount == 9U);
    assert(strcmp(back.route.report.reason, "OK_PARKING_RETURN") == 0);

    /* A repeat that differs, a frame count that changes, a short middle frame, a late "no host". */
    Task2PlanFrames_Begin(&f, 7U);
    assert(Task2PlanFrames_Feed(&f, frames[0]) == TASK2_PLAN_FRAME_MORE);
    char changed[96];
    memcpy(changed, frames[0], sizeof(changed));
    changed[strlen(changed) - 1U] = changed[strlen(changed) - 1U] == '0' ? '1' : '0';
    assert(Task2PlanFrames_Feed(&f, changed) == TASK2_PLAN_FRAME_BAD);
    Task2PlanFrames_Begin(&f, 7U);
    assert(Task2PlanFrames_Feed(&f, frames[0]) == TASK2_PLAN_FRAME_MORE);
    char recount[96];
    (void)snprintf(recount, sizeof(recount), "PR 7 1 %u %s", n + 1U, strrchr(frames[1], ' ') + 1);
    assert(Task2PlanFrames_Feed(&f, recount) == TASK2_PLAN_FRAME_BAD);
    assert(Task2PlanFrames_Feed(&f, "PR 7 0 0") == TASK2_PLAN_FRAME_BAD);
    char shortMiddle[96];
    (void)snprintf(shortMiddle, sizeof(shortMiddle), "PR 7 1 %u 0011", n);
    assert(Task2PlanFrames_Feed(&f, shortMiddle) == TASK2_PLAN_FRAME_BAD);
    /* The last frame repeated with a different length. */
    Task2PlanFrames_Begin(&f, 7U);
    assert(Task2PlanFrames_Feed(&f, frames[n - 1U]) == TASK2_PLAN_FRAME_MORE);
    char longer[96];
    (void)snprintf(longer, sizeof(longer), "%s00", frames[n - 1U]);
    assert(Task2PlanFrames_Feed(&f, longer) == TASK2_PLAN_FRAME_BAD);

    printf("task2_plan_frames_test: PASS\n");
    return 0;
}
