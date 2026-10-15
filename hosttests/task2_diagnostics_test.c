#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "Task2Diagnostics.h"

static char lines[TASK2_DIAG_PARTS][TASK2_DIAG_LINE_SIZE];

/* The widest values a record can hold. */
static void Fill(Task2DiagRecord *r, const char *what, const char *reason) {
    r->hasNext = true; r->next = (Task2DiagStep){-99999.4f, -99999.4f, 99999.4f, false};
    r->ok = true; r->what = what; r->reason = reason; r->ms = 0xffffffffU; r->phase = 255U; r->step = 255U;
    r->xMm = r->yMm = r->yawDeg = -99999.4f; r->arrow1 = 'L'; r->arrow2 = 'R';
    r->obstacle1FaceMm = r->gapMm = r->widthMm = r->widthUncertaintyMm = -99999.4f; r->widthSource = 255U;
    r->widthValid = r->widthBound = true;
    r->planWidthMm = r->endSpaceMm = r->sideLimitMm = r->provenHalfMm = -99999.4f;
    r->edgeMeasured = r->provenOnly = true;
    r->fromXMm = r->fromYMm = r->fromYawDeg = r->configWidthMm = r->configEndSpaceMm = -99999.4f;
    r->returnFallbackActive = true; r->returnObstacleBufferMm = r->returnBoundaryBufferMm = -99999.4f;
    r->obstacleGapMm = r->sideMarginMm = r->parkMarginMm = -99999.4f;
    r->fallback = r->bufferException = r->poseAdjusted = r->returnCertified = r->exhausted = true;
    r->candidateChecks = r->callChecks = r->callUs = r->hostWaitMs = 0xffffffffU;
    r->planSource = 'R'; r->planRefusal = 'x'; r->rejectedRangeMm = -99999.4f;
    r->stepTotal = 32U; r->stepFirst = 20U; r->stepCount = TASK2_DIAG_STEPS;
    for (unsigned i = 0U; i < TASK2_DIAG_STEPS; ++i)
        r->steps[i] = (Task2DiagStep){-99999.4f, -99999.4f, 99999.4f, true};
    r->ussRangeMm = r->observedWidthMm = r->frozenRangeMm = r->frozenExpectedMm = -99999.4f;
    r->ussValid = r->detected = r->ussFrozen = true; r->ussStatus = 255U;
    r->ussSequence = r->ussCapturedMs = r->ussSinceObservedMs = 0xffffffffU;
    r->irAdc[0] = r->irAdc[1] = r->irFaceLevelAdc = 65535U; r->edgeMethod = 255U; r->extensions = r->rewinds = 255U;
    r->irHistoryCount = TASK2_DIAG_IR_HISTORY;
    for (unsigned i = 0U; i < TASK2_DIAG_IR_HISTORY; ++i) r->irHistory[i] = 65535U;
}

static unsigned Get(const Task2Diagnostics *d, unsigned generation) {
    char request[16];
    snprintf(request, sizeof(request), "D T %u", generation);
    return Task2Diag_Reply(d, 255U, request, lines, TASK2_DIAG_PARTS);
}

int main(void) {
    static Task2Diagnostics d;
    Task2Diag_Init(&d);

    /* Widest record and longest tokens: nothing is cut. */
    Task2DiagRecord *r = Task2Diag_Begin(&d, TASK2_DIAG_FAULT);
    Fill(r, "WWWWWWWWWWWWWWWWWWWWWWWW", "RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR");
    Task2Diag_End(&d, r);
    assert(d.hasFault && d.fault.generation == 1U);
    assert(Get(&d, 1U) == TASK2_DIAG_PARTS);
    size_t longest = 0U;
    for (unsigned k = 0U; k < TASK2_DIAG_PARTS; ++k) {
        size_t n = strlen(lines[k]);
        assert(n < TASK2_DIAG_LINE_SIZE && lines[k][n - 1] == '\n' && !strstr(lines[k], " ~"));
        if (n > longest) longest = n;
    }
    assert(strstr(lines[0], " k=F ok=1 w=WWWWWWWWWWWWWWWWWWWWWWWW ms=4294967295 "));
    assert(strstr(lines[0], " nx=-99999/-99999/99999\n") && strstr(lines[1], " x=-99999.4 y=-99999.4 yaw=-99999.40 "));
    assert(strstr(lines[4], " n=32 from=20 got=12 cut=1 20:-99999/-99999/99999s ") &&
           strstr(lines[4], " 25:-99999/-99999/99999s\n") && strstr(lines[5], " 31:-99999/-99999/99999s\n"));
    /* At 115200 baud a line must also go out within CommandLink's 20 ms send timeout: 200 bytes is 17.4 ms. */
    assert(longest <= 200U);
    printf("widest record line: %zu of %u bytes\n", longest, TASK2_DIAG_LINE_SIZE);

    /* A field that cannot fit marks its line as cut instead of ending it silently. */
    char huge[300];
    memset(huge, 'Z', sizeof(huge) - 1U);
    huge[sizeof(huge) - 1U] = '\0';
    r = Task2Diag_Begin(&d, TASK2_DIAG_PLAN);
    Fill(r, huge, "X");
    Task2Diag_End(&d, r);
    assert(Get(&d, 2U) == TASK2_DIAG_PARTS);
    size_t n = strlen(lines[0]);
    assert(n < TASK2_DIAG_LINE_SIZE && strcmp(lines[0] + n - 3U, " ~\n") == 0);

    /* Values outside the printed range read "-", never a wrong number. */
    r = Task2Diag_Begin(&d, TASK2_DIAG_PLAN);
    Fill(r, "W", "X");
    r->xMm = NAN; r->yMm = 1.0e9f; r->steps[0].distanceMm = INFINITY; r->next.distanceMm = NAN;
    Task2Diag_End(&d, r);
    assert(Get(&d, 3U) == TASK2_DIAG_PARTS && strstr(lines[1], " x=- y=- ") && strstr(lines[4], " 20:-/") &&
           strstr(lines[0], " nx=?\n"));
    /* Small negative values keep their sign and decimals. */
    r = Task2Diag_Begin(&d, TASK2_DIAG_PLAN);
    r->what = "W"; r->xMm = -0.04f; r->yMm = -12.36f; r->yawDeg = 0.004f; r->hasNext = false;
    Task2Diag_End(&d, r);
    assert(Get(&d, 4U) == TASK2_DIAG_PARTS && strstr(lines[1], " x=0.0 y=-12.4 yaw=0.00 ") && strstr(lines[0], " nx=-\n"));

    /* The newest records are kept and the first fault survives being overwritten. */
    for (unsigned i = 0U; i < 19U; ++i) {
        r = Task2Diag_Begin(&d, i == 5U ? TASK2_DIAG_FAULT : TASK2_DIAG_PLAN);
        r->what = "W";
        Task2Diag_End(&d, r);
    }
    assert(d.count == TASK2_DIAG_RECORDS && d.overwritten == 23U - TASK2_DIAG_RECORDS);
    assert(Get(&d, 1U) == TASK2_DIAG_PARTS && strstr(lines[0], " k=F ") && Get(&d, 9U) == 0U);
    assert(Task2Diag_Reply(&d, 255U, "D T", lines, 1U) == 1U);
    /* The firmware id is one token: its spaces became underscores. */
    const char *fw = lines[0] + 20;
    assert(strncmp(lines[0], "DT 255 INDEX v=4 fw=", 20) == 0 && strcspn(fw, " ") > 0 &&
           strncmp(fw + strcspn(fw, " "), " n=12 ", 6) == 0);
    assert(strstr(lines[0], " n=12 over=11 fault=1 g=12,13,14,15,16,17,18,19,20,21,22,23\n"));

    /* Malformed requests name no record. */
    assert(Task2Diag_Reply(&d, 1U, "D T 0", lines, TASK2_DIAG_PARTS) == 0U);
    assert(Task2Diag_Reply(&d, 1U, "D T 65536", lines, TASK2_DIAG_PARTS) == 0U);
    assert(Task2Diag_Reply(&d, 1U, "D T 12 ", lines, TASK2_DIAG_PARTS) == 0U);
    assert(Task2Diag_Reply(&d, 1U, "D T 12", lines, TASK2_DIAG_PARTS - 1U) == 0U);
    assert(Task2Diag_Reply(&d, 1U, "D TT", lines, TASK2_DIAG_PARTS) == 0U);

    /* Run trace: index, blocks oldest first, the newest kept on overflow, widest line within budget. */
    static Task2Trace trace;
    Task2Trace_Init(&trace);
    static char tl[TASK2_DIAG_PARTS][TASK2_DIAG_LINE_SIZE];
    assert(Task2Trace_Reply(&trace, 9U, "D R", tl, TASK2_DIAG_PARTS) == 1U);
    assert(strcmp(tl[0], "DR 9 INDEX v=2 n=0 blocks=0 per=4 lines=7 period=50 drop=0\n") == 0);
    assert(Task2Trace_Reply(&trace, 9U, "D R 0", tl, TASK2_DIAG_PARTS) == 0U);
    for (unsigned i = 0U; i < TASK2_TRACE_SAMPLES + 30U; ++i) {
        Task2TraceSample s = {.t10Ms = (uint16_t)i, .xMm = -32767, .yMm = 32767, .yawCentiDeg = -18000,
            .phase = 255U, .step = 255U, .flags = 255U, .state = 255U, .ussMm = 65535U, .irLeft = 65535U, .irRight = 65535U,
            .faceLevel = 65535U};
        Task2Trace_Add(&trace, &s);
    }
    assert(trace.count == TASK2_TRACE_SAMPLES && trace.dropped == 30U);
    assert(Task2Trace_Reply(&trace, 65535U, "D R", tl, TASK2_DIAG_PARTS) == 1U);
    char expect[96];
    snprintf(expect, sizeof expect, "DR 65535 INDEX v=2 n=%u blocks=%u per=4 lines=7 period=50 drop=30\n", TASK2_TRACE_SAMPLES,
             (TASK2_TRACE_SAMPLES + TASK2_TRACE_BLOCK_SAMPLES - 1U) / TASK2_TRACE_BLOCK_SAMPLES);
    assert(strcmp(tl[0], expect) == 0);
    size_t widestTrace = 0U;
    unsigned blocks = (TASK2_TRACE_SAMPLES + TASK2_TRACE_BLOCK_SAMPLES - 1U) / TASK2_TRACE_BLOCK_SAMPLES, seen = 0U;
    for (unsigned b = 0U; b < blocks; ++b) {
        char request[16];
        snprintf(request, sizeof request, "D R %u", b);
        unsigned n = Task2Trace_Reply(&trace, 65535U, request, tl, TASK2_DIAG_PARTS);
        assert(n >= 1U && n <= TASK2_TRACE_BLOCK_LINES);
        for (unsigned i = 0U; i < n; ++i) {
            size_t len = strlen(tl[i]);
            assert(len && tl[i][len - 1U] == '\n' && !strstr(tl[i], "~"));
            if (len > widestTrace) widestTrace = len;
            unsigned first = 0U; char hex[TASK2_DIAG_LINE_SIZE];
            assert(sscanf(tl[i], "DR 65535 %u %s", &first, hex) == 2 && first == seen);
            unsigned samples = (unsigned)strlen(hex) / (2U * TASK2_TRACE_SAMPLE_BYTES);
            assert(strlen(hex) == samples * 2U * TASK2_TRACE_SAMPLE_BYTES && samples >= 1U && samples <= TASK2_TRACE_PER_LINE);
            /* The oldest kept sample is the 31st added: t = 30, little-endian first. */
            if (seen == 0U) assert(strncmp(hex, "1e00", 4) == 0);
            seen += samples;
        }
    }
    assert(seen == TASK2_TRACE_SAMPLES);
    char past[16];
    snprintf(past, sizeof past, "D R %u", blocks);
    assert(Task2Trace_Reply(&trace, 1U, past, tl, TASK2_DIAG_PARTS) == 0U);
    assert(Task2Trace_Reply(&trace, 1U, "D R x", tl, TASK2_DIAG_PARTS) == 0U);
    assert(Task2Trace_Reply(&trace, 1U, "D R 1 ", tl, TASK2_DIAG_PARTS) == 0U);
    assert(Task2Trace_Reply(&trace, 1U, "D R 0", tl, TASK2_TRACE_BLOCK_LINES - 1U) == 0U);
    assert(widestTrace <= 200U);
    printf("widest trace line: %zu of %u bytes\n", widestTrace, TASK2_DIAG_LINE_SIZE);
    puts("PASS task2_diagnostics: widest record uncut, overflow marked, ring and first fault kept; trace blocks whole and in order");
    return 0;
}
