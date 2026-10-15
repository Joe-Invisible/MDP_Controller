#ifndef INC_TASK2PLANFRAMES_H_
#define INC_TASK2PLANFRAMES_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "Task2Wire.h"

/*
 * A planning host's reply arrives as frames "PR <gen> <k> <n> <hex>", each within the 64-byte line
 * limit: k of n, every frame but the last carrying TASK2_PLAN_FRAME_BYTES. "PR <gen> 0 0" means no
 * host. The assembler only sorts lines; the caller decides what a stop or any other line means.
 */
#define TASK2_PLAN_FRAME_BYTES 24U
#define TASK2_PLAN_MAX_FRAMES ((TASK2_WIRE_MAX_BYTES + TASK2_PLAN_FRAME_BYTES - 1U) / TASK2_PLAN_FRAME_BYTES)

typedef enum {
    TASK2_PLAN_FRAME_OTHER,     /* not a planning reply: keep it for the main loop */
    TASK2_PLAN_FRAME_STOP,      /* "S" */
    TASK2_PLAN_FRAME_STALE,     /* a reply to another request: dropped */
    TASK2_PLAN_FRAME_MORE,      /* accepted, more to come */
    TASK2_PLAN_FRAME_DONE,      /* the whole reply is assembled */
    TASK2_PLAN_FRAME_NONE,      /* the host has no plan: plan locally now */
    TASK2_PLAN_FRAME_BAD        /* malformed or contradictory: plan locally now */
} Task2PlanFrame;

typedef struct {
    uint16_t generation;
    unsigned parts, received;
    size_t lastBytes;
    bool have[TASK2_PLAN_MAX_FRAMES];
    uint8_t data[TASK2_PLAN_MAX_FRAMES * TASK2_PLAN_FRAME_BYTES];
} Task2PlanFrames;

void Task2PlanFrames_Begin(Task2PlanFrames *f, uint16_t generation);
Task2PlanFrame Task2PlanFrames_Feed(Task2PlanFrames *f, const char *line);
/* After DONE: the assembled reply's length. */
size_t Task2PlanFrames_Length(const Task2PlanFrames *f);

#endif
