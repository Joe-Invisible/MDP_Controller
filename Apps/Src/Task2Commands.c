#include "Task2Commands.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *const stageNames[] = {
    "IDLE", "APPROACH1", "WAIT1", "SWERVE1", "APPROACH2", "WAIT2",
    "ROUND2", "HOME", "DONE", "STOPPED", "FAULT",
};

const char *Task2Commands_StageName(Task2Stage stage) {
    return (unsigned)stage < sizeof(stageNames) / sizeof(stageNames[0])
        ? stageNames[stage] : "IDLE";
}

void Task2Commands_Init(Task2Commands *t) {
    memset(t, 0, sizeof(*t));
}

static void Queue(Task2Commands *t, const char *line) {
    if (t->outboxCount == TASK2COMMANDS_OUTBOX) {
        /* Never block the control loop; the RPi recovers the state with Q T. */
        ++t->outboxDropped;
        return;
    }
    unsigned slot = (t->outboxHead + t->outboxCount) % TASK2COMMANDS_OUTBOX;
    snprintf(t->outbox[slot], TASK2COMMANDS_REPLY_SIZE, "%s", line);
    ++t->outboxCount;
}

bool Task2Commands_NextEvent(Task2Commands *t, char line[TASK2COMMANDS_REPLY_SIZE]) {
    if (t->outboxCount == 0U)
        return false;
    memcpy(line, t->outbox[t->outboxHead], TASK2COMMANDS_REPLY_SIZE);
    t->outboxHead = (t->outboxHead + 1U) % TASK2COMMANDS_OUTBOX;
    --t->outboxCount;
    return true;
}

void Task2Commands_SetStage(Task2Commands *t, Task2Stage stage) {
    if (!t->started || stage == t->stage)
        return;
    t->stage = stage;
    char line[TASK2COMMANDS_REPLY_SIZE];
    switch (stage) {
    case TASK2_STAGE_APPROACH1:
    case TASK2_STAGE_APPROACH2:
        snprintf(line, sizeof(line), "EV %u APPROACH %u\n", (unsigned)t->run,
                 stage == TASK2_STAGE_APPROACH1 ? 1U : 2U);
        break;
    case TASK2_STAGE_WAIT1:
    case TASK2_STAGE_WAIT2:
        snprintf(line, sizeof(line), "EV %u WAIT %u\n", (unsigned)t->run,
                 stage == TASK2_STAGE_WAIT1 ? 1U : 2U);
        break;
    case TASK2_STAGE_ROUND2:
        snprintf(line, sizeof(line), "EV %u ROUND 2\n", (unsigned)t->run);
        break;
    default:
        return;
    }
    Queue(t, line);
}

void Task2Commands_Begin(Task2Commands *t, uint8_t run, Task2CommandArrow arrow1) {
    Task2Commands_Init(t);
    t->started = true;
    t->run = run;
    t->arrows[0] = arrow1;
    t->stage = TASK2_STAGE_IDLE;
    Task2Commands_SetStage(t, TASK2_STAGE_APPROACH1);
}

Task2CommandArrow Task2Commands_Preview(const Task2Commands *t) {
    return t->preview;
}

Task2CommandArrow Task2Commands_Arrow(const Task2Commands *t, unsigned obstacle) {
    return t->started && (obstacle == 1U || obstacle == 2U)
        ? t->arrows[obstacle - 1U] : TASK2_COMMAND_ARROW_NONE;
}

bool Task2Commands_Span(const Task2Commands *t, uint16_t *spanMm) {
    if (!t->started || !t->hasSpan)
        return false;
    *spanMm = t->obstacle2SpanMm;
    return true;
}

void Task2Commands_ClearEvents(Task2Commands *t) {
    t->outboxHead = 0U;
    t->outboxCount = 0U;
}

static void AppendMm(char *line, size_t size, float mm) {
    size_t used = strlen(line);
    if (isfinite(mm) && fabsf(mm) < 100000.0f)
        snprintf(line + used, size - used, " %ld", (long)lroundf(mm));
    else
        snprintf(line + used, size - used, " -");
}

void Task2Commands_Summary(Task2Commands *t, float obstacle1FaceMm,
                           float obstacle2FaceMm, float obstacle2WidthMm, float pathMm) {
    if (!t->started)
        return;
    char line[TASK2COMMANDS_REPLY_SIZE];
    snprintf(line, sizeof(line), "EV %u SUMMARY", (unsigned)t->run);
    AppendMm(line, sizeof(line), obstacle1FaceMm);
    AppendMm(line, sizeof(line), obstacle2FaceMm);
    AppendMm(line, sizeof(line), obstacle2WidthMm);
    AppendMm(line, sizeof(line), pathMm);
    size_t used = strlen(line);
    snprintf(line + used, sizeof(line) - used, "\n");
    Queue(t, line);
}

/*
 * Strict decimal field: digits only, value in [low, high], followed by the
 * expected separator. Advances *p past the separator.
 */
static bool Field(const char **p, unsigned long low, unsigned long high,
                  char separator, unsigned long *value) {
    const char *s = *p;
    unsigned long v = 0;
    unsigned digits = 0;
    while (*s >= '0' && *s <= '9') {
        if (++digits > 5U)
            return false;
        v = v * 10U + (unsigned long)(*s - '0');
        ++s;
    }
    if (digits == 0U || *s != separator || v < low || v > high)
        return false;
    *value = v;
    *p = separator == '\0' ? s : s + 1;
    return true;
}

static bool ArmedFor(Task2Stage stage, unsigned obstacle) {
    return obstacle == 1U
        ? stage == TASK2_STAGE_APPROACH1 || stage == TASK2_STAGE_WAIT1
        : stage == TASK2_STAGE_APPROACH2 || stage == TASK2_STAGE_WAIT2;
}

static bool RunActive(const Task2Commands *t) {
    return t->started && t->stage >= TASK2_STAGE_APPROACH1 && t->stage <= TASK2_STAGE_HOME;
}

/* "A - n X": arrow 1 before the run, replaceable until "r:T2" starts it. */
static void Preview(Task2Commands *t, const char *p, char *reply) {
    unsigned long obstacle;
    if (!Field(&p, 1U, 2U, ' ', &obstacle) || (p[0] != 'L' && p[0] != 'R' && p[0] != '-') || p[1] != '\0') {
        snprintf(reply, TASK2COMMANDS_REPLY_SIZE, "NAK - PARSE\n");
        return;
    }
    if (obstacle != 1U || RunActive(t)) {
        snprintf(reply, TASK2COMMANDS_REPLY_SIZE, "NAK - %lu STATE\n", obstacle);
        return;
    }
    t->preview = p[0] == 'L' ? TASK2_COMMAND_ARROW_LEFT
               : p[0] == 'R' ? TASK2_COMMAND_ARROW_RIGHT : TASK2_COMMAND_ARROW_NONE;
    snprintf(reply, TASK2COMMANDS_REPLY_SIZE, "ACK - 1 %c\n", p[0]);
}

static void Arrow(Task2Commands *t, const char *p, char *reply) {
    unsigned long run, obstacle;
    if (p[0] == '-' && p[1] == ' ') {
        Preview(t, p + 2, reply);
        return;
    }
    if (!Field(&p, 0U, 255U, ' ', &run) || !Field(&p, 1U, 2U, ' ', &obstacle) ||
        (p[0] != 'L' && p[0] != 'R') || p[1] != '\0') {
        snprintf(reply, TASK2COMMANDS_REPLY_SIZE, "NAK - PARSE\n");
        return;
    }
    char side = p[0];
    Task2CommandArrow arrow = side == 'L' ? TASK2_COMMAND_ARROW_LEFT : TASK2_COMMAND_ARROW_RIGHT;
    bool thisRun = t->started && run == t->run;
    Task2CommandArrow stored = thisRun ? t->arrows[obstacle - 1U] : TASK2_COMMAND_ARROW_NONE;
    if (thisRun && stored == arrow) {
        snprintf(reply, TASK2COMMANDS_REPLY_SIZE, "ACK %lu %lu %c\n", run, obstacle, side);
    } else if (thisRun && stored != TASK2_COMMAND_ARROW_NONE) {
        snprintf(reply, TASK2COMMANDS_REPLY_SIZE, "NAK %lu %lu ARROW\n", run, obstacle);
    } else if (!thisRun || !ArmedFor(t->stage, (unsigned)obstacle)) {
        snprintf(reply, TASK2COMMANDS_REPLY_SIZE, "NAK %lu %lu STATE\n", run, obstacle);
    } else {
        t->arrows[obstacle - 1U] = arrow;
        snprintf(reply, TASK2COMMANDS_REPLY_SIZE, "ACK %lu %lu %c\n", run, obstacle, side);
    }
}

static void Span(Task2Commands *t, const char *p, char *reply) {
    unsigned long run, span;
    if (!Field(&p, 0U, 255U, ' ', &run) || !Field(&p, 1U, 9999U, '\0', &span)) {
        snprintf(reply, TASK2COMMANDS_REPLY_SIZE, "NAK - PARSE\n");
        return;
    }
    bool open = t->started && run == t->run &&
        t->stage >= TASK2_STAGE_APPROACH1 && t->stage <= TASK2_STAGE_WAIT2;
    if (!open) {
        snprintf(reply, TASK2COMMANDS_REPLY_SIZE, "NAK %lu W STATE\n", run);
        return;
    }
    t->hasSpan = true;
    t->obstacle2SpanMm = (uint16_t)span;
    snprintf(reply, TASK2COMMANDS_REPLY_SIZE, "ACK %lu W %lu\n", run, span);
}

static char ArrowLetter(Task2CommandArrow arrow) {
    return arrow == TASK2_COMMAND_ARROW_LEFT ? 'L'
         : arrow == TASK2_COMMAND_ARROW_RIGHT ? 'R' : '-';
}

bool Task2Commands_Receive(Task2Commands *t, const char *line,
                           char reply[TASK2COMMANDS_REPLY_SIZE]) {
    reply[0] = '\0';
    if (strcmp(line, "Q T") == 0) {
        if (!t->started)
            snprintf(reply, TASK2COMMANDS_REPLY_SIZE, "T - IDLE - -\n");
        else
            snprintf(reply, TASK2COMMANDS_REPLY_SIZE, "T %u %s %c %c\n", (unsigned)t->run,
                     Task2Commands_StageName(t->stage), ArrowLetter(t->arrows[0]),
                     ArrowLetter(t->arrows[1]));
        return true;
    }
    if (strncmp(line, "A ", 2) == 0) {
        Arrow(t, line + 2, reply);
        return true;
    }
    if (strncmp(line, "W ", 2) == 0) {
        Span(t, line + 2, reply);
        return true;
    }
    return false;
}
