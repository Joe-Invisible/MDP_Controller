#include "Task2Diagnostics.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void Task2Diag_Init(Task2Diagnostics *d)
{
    memset(d, 0, sizeof(*d));
    d->nextGeneration = 1U;
}

Task2DiagRecord *Task2Diag_Begin(Task2Diagnostics *d, Task2DiagKind kind)
{
    unsigned slot = (d->head + d->count) % TASK2_DIAG_RECORDS;
    if (d->count == TASK2_DIAG_RECORDS)
    {
        d->head = (d->head + 1U) % TASK2_DIAG_RECORDS;
        ++d->overwritten;
    }
    else
    {
        ++d->count;
    }
    Task2DiagRecord *r = &d->records[slot];
    memset(r, 0, sizeof(*r));
    r->generation = d->nextGeneration++;
    if (d->nextGeneration == 0U) d->nextGeneration = 1U;
    r->kind = (uint8_t)kind;
    return r;
}

void Task2Diag_End(Task2Diagnostics *d, const Task2DiagRecord *record)
{
    if (record->kind == TASK2_DIAG_FAULT && !d->hasFault)
    {
        d->fault = *record;
        d->hasFault = true;
    }
}

/* Appends to a line; once anything does not fit, the line ends in " ~" instead. */
typedef struct
{
    char *text;
    size_t used;
    bool cut;
} Line;

static void Add(Line *l, const char *format, ...)
{
    if (l->cut) return;
    size_t room = TASK2_DIAG_LINE_SIZE - 3U - l->used; /* keep room for " ~" or "\n" */
    va_list args;
    va_start(args, format);
    int n = vsnprintf(l->text + l->used, room, format, args);
    va_end(args);
    if (n < 0 || (size_t)n >= room)
    {
        l->cut = true;
        l->used = strlen(l->text);
        return;
    }
    l->used += (size_t)n;
}

static void Finish(Line *l)
{
    snprintf(l->text + l->used, TASK2_DIAG_LINE_SIZE - l->used, l->cut ? " ~\n" : "\n");
}

static void Mm(Line *l, const char *key, float value)
{
    if (isfinite(value) && fabsf(value) < 100000.0f) Add(l, " %s=%ld", key, (long)lroundf(value));
    else Add(l, " %s=-", key);
}

/* Pose coordinates to 0.1 mm and yaw to 0.01 degree, enough to replay a plan from its record. */
static void Fine(Line *l, const char *key, float value, float scale)
{
    if (isfinite(value) && fabsf(value) < 100000.0f)
    {
        long v = lroundf(value * scale), s = (long)scale;
        Add(l, " %s=%s%ld.%0*ld", key, v < 0 ? "-" : "", labs(v) / s, s == 10 ? 1 : 2, labs(v) % s);
    }
    else
    {
        Add(l, " %s=-", key);
    }
}

static void Flag(Line *l, const char *key, bool value)
{
    Add(l, " %s=%u", key, value ? 1U : 0U);
}

static const char *Token(const char *text)
{
    return text && text[0] ? text : "-";
}

static void
Start(Line *l, char *text, unsigned run, const Task2DiagRecord *r, unsigned part, const char *tag)
{
    l->text = text;
    l->used = 0U;
    l->cut = false;
    text[0] = '\0';
    Add(l, "DT %u %u %u/%u %s", run, (unsigned)r->generation, part, TASK2_DIAG_PARTS, tag);
}

static void Steps(Line *l, const Task2DiagRecord *r, unsigned first, unsigned last)
{
    for (unsigned i = first; i < last && i < r->stepCount && i < TASK2_DIAG_STEPS; ++i)
    {
        const Task2DiagStep *s = &r->steps[i];
        Add(l, " %u:", (unsigned)r->stepFirst + i);
        if (isfinite(s->distanceMm) && fabsf(s->distanceMm) < 100000.0f)
            Add(l, "%ld", (long)lroundf(s->distanceMm));
        else Add(l, "-");
        if (isfinite(s->radiusMm) && fabsf(s->radiusMm) < 100000.0f)
            Add(l, "/%ld", (long)lroundf(s->radiusMm));
        else Add(l, "/-");
        if (isfinite(s->speedCps) && fabsf(s->speedCps) < 100000.0f)
            Add(l, "/%ld%s", (long)lroundf(s->speedCps), s->sensorTerminated ? "s" : "");
        else Add(l, "/-%s", s->sensorTerminated ? "s" : "");
    }
}

static unsigned Record(const Task2DiagRecord *r, unsigned run, char lines[][TASK2_DIAG_LINE_SIZE])
{
    static const char kinds[] = {'P', 'F', 'S'};
    Line l;
    Start(&l, lines[0], run, r, 1U, "H");
    Add(&l,
        " k=%c ok=%u w=%s ms=%lu ph=%u st=%u why=%s",
        r->kind < 3U ? kinds[r->kind] : '?',
        r->ok ? 1U : 0U,
        Token(r->what),
        (unsigned long)r->ms,
        (unsigned)r->phase,
        (unsigned)r->step,
        Token(r->reason));
    /* nx=distance/radius/speed of a reverse launched with this decision, or nx=- */
    if (r->hasNext && isfinite(r->next.distanceMm) && isfinite(r->next.radiusMm) &&
        isfinite(r->next.speedCps) && fabsf(r->next.distanceMm) < 100000.0f &&
        fabsf(r->next.radiusMm) < 100000.0f && fabsf(r->next.speedCps) < 100000.0f)
        Add(&l,
            " nx=%ld/%ld/%ld",
            (long)lroundf(r->next.distanceMm),
            (long)lroundf(r->next.radiusMm),
            (long)lroundf(r->next.speedCps));
    else Add(&l, " nx=%s", r->hasNext ? "?" : "-");
    Finish(&l);

    Start(&l, lines[1], run, r, 2U, "I");
    Fine(&l, "x", r->xMm, 10.0f);
    Fine(&l, "y", r->yMm, 10.0f);
    Fine(&l, "yaw", r->yawDeg, 100.0f);
    Add(&l, " a1=%c a2=%c", r->arrow1 ? r->arrow1 : '-', r->arrow2 ? r->arrow2 : '-');
    Mm(&l, "o1", r->obstacle1FaceMm);
    Mm(&l, "gap", r->gapMm);
    Mm(&l, "w", r->widthMm);
    Mm(&l, "wu", r->widthUncertaintyMm);
    Add(&l, " ws=%u", (unsigned)r->widthSource);
    Flag(&l, "wv", r->widthValid);
    Flag(&l, "wb", r->widthBound);
    Finish(&l);

    Start(&l, lines[2], run, r, 3U, "P");
    Fine(&l, "fx", r->fromXMm, 10.0f);
    Fine(&l, "fy", r->fromYMm, 10.0f);
    Fine(&l, "fyaw", r->fromYawDeg, 100.0f);
    Mm(&l, "cw", r->configWidthMm);
    Mm(&l, "ce", r->configEndSpaceMm);
    Mm(&l, "cl", 0.5f * r->configWidthMm + r->configEndSpaceMm);
    Flag(&l, "rf", r->returnFallbackActive);
    Mm(&l, "rob", r->returnObstacleBufferMm);
    Mm(&l, "rbb", r->returnBoundaryBufferMm);
    Add(&l, " ih=");
    if (!r->irHistoryCount) Add(&l, "-");
    for (unsigned i = 0U; i < r->irHistoryCount && i < TASK2_DIAG_IR_HISTORY; ++i)
        Add(&l, i ? ",%u" : "%u", (unsigned)r->irHistory[i]);
    Finish(&l);

    Start(&l, lines[3], run, r, 4U, "B");
    Mm(&l, "wp", r->planWidthMm);
    Mm(&l, "es", r->endSpaceMm);
    Mm(&l, "sl", r->sideLimitMm);
    Mm(&l, "ph", r->provenHalfMm);
    Flag(&l, "em", r->edgeMeasured);
    Flag(&l, "po", r->provenOnly);
    Mm(&l, "og", r->obstacleGapMm);
    Mm(&l, "sm", r->sideMarginMm);
    Mm(&l, "pm", r->parkMarginMm);
    Flag(&l, "fb", r->fallback);
    Flag(&l, "bx", r->bufferException);
    Flag(&l, "pa", r->poseAdjusted);
    Flag(&l, "rc", r->returnCertified);
    Flag(&l, "ex", r->exhausted);
    Add(&l,
        " cc=%lu tc=%lu us=%lu src=%c ref=%c hw=%lu",
        (unsigned long)r->candidateChecks,
        (unsigned long)r->callChecks,
        (unsigned long)r->callUs,
        r->planSource ? r->planSource : '-',
        r->planRefusal ? r->planRefusal : '-',
        (unsigned long)r->hostWaitMs);
    Finish(&l);

    Start(&l, lines[4], run, r, 5U, "S");
    Add(&l,
        " n=%u from=%u got=%u cut=%u",
        (unsigned)r->stepTotal,
        (unsigned)r->stepFirst,
        (unsigned)r->stepCount,
        r->stepCount < r->stepTotal ? 1U : 0U);
    Steps(&l, r, 0U, TASK2_DIAG_STEPS / 2U);
    Finish(&l);

    Start(&l, lines[5], run, r, 6U, "S");
    Steps(&l, r, TASK2_DIAG_STEPS / 2U, TASK2_DIAG_STEPS);
    Finish(&l);

    Start(&l, lines[6], run, r, 7U, "X");
    Mm(&l, "uss", r->ussRangeMm);
    Flag(&l, "uv", r->ussValid);
    Add(&l,
        " us=%u useq=%lu ucap=%lu uago=%lu il=%u ir=%u fl=%u em=%u",
        (unsigned)r->ussStatus,
        (unsigned long)r->ussSequence,
        (unsigned long)r->ussCapturedMs,
        (unsigned long)r->ussSinceObservedMs,
        (unsigned)r->irAdc[0],
        (unsigned)r->irAdc[1],
        (unsigned)r->irFaceLevelAdc,
        (unsigned)r->edgeMethod);
    Flag(&l, "dt", r->detected);
    Mm(&l, "ow", r->observedWidthMm);
    Mm(&l, "fr", r->frozenRangeMm);
    Mm(&l, "fe", r->frozenExpectedMm);
    Flag(&l, "fz", r->ussFrozen);
    Add(&l, " xe=%u rw=%u", (unsigned)r->extensions, (unsigned)r->rewinds);
    Mm(&l, "rr", r->rejectedRangeMm);
    Mm(&l, "cf", r->cameraFaceMm);
    Mm(&l, "uf", r->ussFaceMm);
    Finish(&l);
    return TASK2_DIAG_PARTS;
}

static const Task2DiagRecord *Find(const Task2Diagnostics *d, unsigned long generation)
{
    if (d->hasFault && d->fault.generation == generation) return &d->fault;
    for (unsigned i = 0U; i < d->count; ++i)
    {
        const Task2DiagRecord *r = &d->records[(d->head + i) % TASK2_DIAG_RECORDS];
        if (r->generation == generation) return r;
    }
    return NULL;
}

unsigned Task2Diag_Reply(const Task2Diagnostics *d,
                         unsigned run,
                         const char *request,
                         char lines[][TASK2_DIAG_LINE_SIZE],
                         unsigned maxLines)
{
    if (strcmp(request, "D T") == 0)
    {
        if (maxLines < 1U) return 0U;
        Line l = {lines[0], 0U, false};
        lines[0][0] = '\0';
        char firmware[sizeof(TASK2_FIRMWARE_ID)];
        memcpy(firmware, TASK2_FIRMWARE_ID, sizeof(firmware));
        for (char *c = firmware; *c; ++c)
            if (*c == ' ') *c = '_';
        Add(&l,
            "DT %u INDEX v=%u fw=%s n=%u over=%lu",
            run,
            TASK2_DIAG_FORMAT,
            firmware,
            (unsigned)d->count,
            (unsigned long)d->overwritten);
        if (d->hasFault) Add(&l, " fault=%u", (unsigned)d->fault.generation);
        else Add(&l, " fault=-");
        Add(&l, " g=");
        for (unsigned i = 0U; i < d->count; ++i)
            Add(&l,
                i ? ",%u" : "%u",
                (unsigned)d->records[(d->head + i) % TASK2_DIAG_RECORDS].generation);
        Finish(&l);
        return 1U;
    }
    if (strncmp(request, "D T ", 4) != 0 || maxLines < TASK2_DIAG_PARTS) return 0U;
    const char *p = request + 4;
    unsigned long generation = 0U;
    unsigned digits = 0U;
    while (*p >= '0' && *p <= '9' && digits < 6U)
    {
        generation = generation * 10U + (unsigned long)(*p - '0');
        ++p;
        ++digits;
    }
    if (digits == 0U || *p != '\0' || generation == 0U || generation > 65535U) return 0U;
    const Task2DiagRecord *r = Find(d, generation);
    return r ? Record(r, run, lines) : 0U;
}

void Task2Trace_Init(Task2Trace *t)
{
    memset(t, 0, sizeof(*t));
}

void Task2Trace_Add(Task2Trace *t, const Task2TraceSample *sample)
{
    unsigned slot = (t->head + t->count) % TASK2_TRACE_SAMPLES;
    if (t->count < TASK2_TRACE_SAMPLES)
    {
        ++t->count;
    }
    else
    {
        t->head = (uint16_t)((t->head + 1U) % TASK2_TRACE_SAMPLES);
        ++t->dropped;
    }
    t->samples[slot] = *sample;
}

static void Put16(uint8_t *b, unsigned *n, uint16_t v)
{
    b[(*n)++] = (uint8_t)(v & 0xFFU);
    b[(*n)++] = (uint8_t)(v >> 8);
}

static unsigned PackSample(const Task2TraceSample *s, uint8_t b[TASK2_TRACE_SAMPLE_BYTES])
{
    unsigned n = 0U;
    Put16(b, &n, s->t10Ms);
    Put16(b, &n, (uint16_t)s->xMm);
    Put16(b, &n, (uint16_t)s->yMm);
    Put16(b, &n, (uint16_t)s->yawCentiDeg);
    b[n++] = s->phase;
    b[n++] = s->step;
    b[n++] = s->flags;
    b[n++] = s->state;
    Put16(b, &n, s->ussMm);
    Put16(b, &n, s->irLeft);
    Put16(b, &n, s->irRight);
    Put16(b, &n, s->faceLevel);
    return n;
}

unsigned Task2Trace_Reply(const Task2Trace *t,
                          unsigned run,
                          const char *request,
                          char lines[][TASK2_DIAG_LINE_SIZE],
                          unsigned maxLines)
{
    if (strcmp(request, "D R") == 0)
    {
        if (maxLines < 1U) return 0U;
        Line l = {lines[0], 0U, false};
        lines[0][0] = '\0';
        unsigned blocks = (t->count + TASK2_TRACE_BLOCK_SAMPLES - 1U) / TASK2_TRACE_BLOCK_SAMPLES;
        Add(&l,
            "DR %u INDEX v=%u n=%u blocks=%u per=%u lines=%u period=%u drop=%lu",
            run,
            TASK2_TRACE_FORMAT,
            (unsigned)t->count,
            blocks,
            TASK2_TRACE_PER_LINE,
            TASK2_TRACE_BLOCK_LINES,
            TASK2_TRACE_PERIOD_MS,
            (unsigned long)t->dropped);
        Finish(&l);
        return 1U;
    }
    if (strncmp(request, "D R ", 4) != 0 || maxLines < TASK2_TRACE_BLOCK_LINES) return 0U;
    const char *p = request + 4;
    unsigned long block = 0U;
    unsigned digits = 0U;
    while (*p >= '0' && *p <= '9' && digits < 4U)
    {
        block = block * 10U + (unsigned long)(*p - '0');
        ++p;
        ++digits;
    }
    if (digits == 0U || *p != '\0') return 0U;
    unsigned first = (unsigned)block * TASK2_TRACE_BLOCK_SAMPLES;
    if (first >= t->count) return 0U;
    unsigned written = 0U;
    for (unsigned at = first; at < t->count && at < first + TASK2_TRACE_BLOCK_SAMPLES;
         at += TASK2_TRACE_PER_LINE)
    {
        Line l = {lines[written], 0U, false};
        lines[written][0] = '\0';
        Add(&l, "DR %u %u ", run, at);
        for (unsigned i = at; i < t->count && i < at + TASK2_TRACE_PER_LINE; ++i)
        {
            uint8_t b[TASK2_TRACE_SAMPLE_BYTES];
            unsigned n = PackSample(&t->samples[(t->head + i) % TASK2_TRACE_SAMPLES], b);
            for (unsigned k = 0U; k < n; ++k) Add(&l, "%02x", b[k]);
        }
        Finish(&l);
        ++written;
    }
    return written;
}
