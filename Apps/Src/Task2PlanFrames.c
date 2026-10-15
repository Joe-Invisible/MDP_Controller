#include "Task2PlanFrames.h"

#include <stdio.h>
#include <string.h>

void Task2PlanFrames_Begin(Task2PlanFrames *f, uint16_t generation) {
    memset(f, 0, sizeof(*f));
    f->generation = generation;
}

/* Strict decimal: digits only, no sign, no overflow past `limit`. */
static const char *Number(const char *p, unsigned limit, unsigned *out) {
    unsigned v = 0U;
    if (*p < '0' || *p > '9') return NULL;
    while (*p >= '0' && *p <= '9') {
        v = 10U * v + (unsigned)(*p++ - '0');
        if (v > limit) return NULL;
    }
    *out = v;
    return p;
}

Task2PlanFrame Task2PlanFrames_Feed(Task2PlanFrames *f, const char *line) {
    if (strcmp(line, "S") == 0) return TASK2_PLAN_FRAME_STOP;
    if (strncmp(line, "PR ", 3U) != 0) return TASK2_PLAN_FRAME_OTHER;
    unsigned g, k, n;
    const char *p = Number(line + 3, 65535U, &g);
    if (!p || *p != ' ') return TASK2_PLAN_FRAME_BAD;
    if (g != f->generation) return TASK2_PLAN_FRAME_STALE;
    p = Number(p + 1, 1000U, &k);
    if (!p || *p != ' ') return TASK2_PLAN_FRAME_BAD;
    p = Number(p + 1, 1000U, &n);
    if (!p) return TASK2_PLAN_FRAME_BAD;
    if (n == 0U) return *p == '\0' && k == 0U && f->received == 0U ? TASK2_PLAN_FRAME_NONE : TASK2_PLAN_FRAME_BAD;
    if (*p != ' ') return TASK2_PLAN_FRAME_BAD;
    const char *hex = p + 1;
    size_t chars = strlen(hex);
    if (n > TASK2_PLAN_MAX_FRAMES || k >= n || (f->parts && n != f->parts) || chars == 0U ||
        chars > 2U * TASK2_PLAN_FRAME_BYTES || (k + 1U < n && chars != 2U * TASK2_PLAN_FRAME_BYTES))
        return TASK2_PLAN_FRAME_BAD;
    uint8_t bytes[TASK2_PLAN_FRAME_BYTES];
    size_t got = Task2Wire_FromHex(hex, chars, bytes, sizeof(bytes));
    if (!got) return TASK2_PLAN_FRAME_BAD;
    uint8_t *slot = f->data + (size_t)k * TASK2_PLAN_FRAME_BYTES;
    if (f->have[k]) {
        bool same = (k + 1U < n || got == f->lastBytes) && memcmp(slot, bytes, got) == 0;
        return same ? TASK2_PLAN_FRAME_MORE : TASK2_PLAN_FRAME_BAD;
    }
    f->parts = n;
    memcpy(slot, bytes, got);
    f->have[k] = true;
    if (k + 1U == n) f->lastBytes = got;
    return ++f->received == f->parts ? TASK2_PLAN_FRAME_DONE : TASK2_PLAN_FRAME_MORE;
}

size_t Task2PlanFrames_Length(const Task2PlanFrames *f) {
    return f->parts ? (size_t)(f->parts - 1U) * TASK2_PLAN_FRAME_BYTES + f->lastBytes : 0U;
}
