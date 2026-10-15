/*
 * Task2Wrap's private home geometry, natively: the body/box clearance helper against crossing, touching and separated
 * cases, and the home moves it emits (TO_X along the actual heading, the entry re-solved where that straight ends,
 * the PARK straight) executed with ideal motion every 1 mm, over D1/gap/O2 extremes, both sides and headings
 * -3/0/+3 deg: every path keeps the clearances the planner requires and ends inside the car park.
 */
#include <assert.h>
#include <stdio.h>

#include "../Apps/Src/Task2Wrap.c"
static int Geometry(void) {
    /* Codex 462: axle (-91.5, 0), heading 0 -> body x [-128.5, 128.5]; O2 box x [-50, 50], y [-530, 530] crosses it. */
    float g = BodyBoxGap(-91.5f, 0.0f, 0.0f, -50.0f, 50.0f, -530.0f, 530.0f);
    printf("crossing: %.1f (want -1)\n", g); assert(g < 0.0f);
    g = BodyBoxGap(-91.5f, 0.0f, 0.0f, 128.5f, 228.5f, -530.0f, 530.0f);   /* touching the front face: contact */
    printf("touching: %.2f (want -1 or 0)\n", g); assert(g < 0.01f);
    g = BodyBoxGap(-91.5f, 0.0f, 0.0f, 128.51f, 228.5f, -530.0f, 530.0f);  /* 0.01 mm apart */
    printf("0.01 apart: %.3f (want ~0.01)\n", g); assert(g >= 0.0f && g < 0.05f);
    g = BodyBoxGap(-91.5f, 0.0f, 0.0f, 188.5f, 288.5f, -530.0f, 530.0f);   /* 60 mm ahead */
    printf("60 ahead: %.2f (want 60)\n", g); assert(fabsf(g - 60.0f) < 0.01f);
    g = BodyBoxGap(-91.5f, 0.0f, 0.0f, -50.0f, 50.0f, 160.0f, 260.0f);      /* 60 mm to the side */
    printf("60 aside: %.2f (want 60)\n", g); assert(fabsf(g - 60.0f) < 0.01f);
    g = BodyBoxGap(0.0f, 0.0f, 0.7854f, 300.0f, 400.0f, 300.0f, 400.0f);    /* diagonal, apart */
    printf("diagonal apart: %.2f (> 0)\n", g); assert(g > 0.0f);
    g = BodyBoxGap(0.0f, 0.0f, 0.7854f, 100.0f, 110.0f, -500.0f, 500.0f);   /* thin wall through a turned body */
    printf("thin wall crossing turned body: %.1f (want -1)\n", g); assert(g < 0.0f);
    puts("geometry: PASS");
    return 0;
}
static Task2Wrap w;
static Task2AppIo io = {.mmPerCount = 3.14159265f * 65.5f / 1560.0f};
static float minO1, minO2, minPark, minO1True;
static float perturbX, perturbY, perturbQ;   /* added to the pose after the first TO_X (arrival error) */
static bool perturbEvery;                    /* ... after every TO_X instead (each re-planned straight misses too) */
static int farOne, farRechecked;
static void Check(float x, float y, float q) {
    float o2lo = fminf(w.nearEndYMm, w.farEndYMm) - O2_Y_SLACK_MM, o2hi = fmaxf(w.nearEndYMm, w.farEndYMm) + O2_Y_SLACK_MM;
    minO1 = fminf(minO1, BodyBoxGap(x, y, q, w.o1FaceXMm, w.o1FaceXMm + 100.0f, -50.0f - O1_Y_SLACK_MM, 50.0f + O1_Y_SLACK_MM));
    minO1True = fminf(minO1True, BodyBoxGap(x, y, q, w.o1FaceXMm, w.o1FaceXMm + 100.0f, -50.0f, 50.0f));
    minO2 = fminf(minO2, BodyBoxGap(x, y, q, w.o2FaceXMm, w.o2FaceXMm + 100.0f, o2lo, o2hi));
    float back = BackWallX(), ent = EntranceX(), hw = 0.5f * PARK_WIDTH_MM;
    minPark = fminf(minPark, fminf(BodyBoxGap(x, y, q, back - 30.0f, back, -hw - 30.0f, hw + 30.0f),
                    fminf(BodyBoxGap(x, y, q, back, ent, hw, hw + 30.0f), BodyBoxGap(x, y, q, back, ent, -hw - 30.0f, -hw))));
}
static void Straight1(float d) {          /* along the current heading, signed */
    float n = ceilf(fabsf(d)), c = cosf(w.pose.yawRad), s = sinf(w.pose.yawRad);
    for (int i = 0; i < (int)n; ++i) { w.pose.xMm += d / n * c; w.pose.yMm += d / n * s; Check(w.pose.xMm, w.pose.yMm, w.pose.yawRad); }
}
static void Arc1(float q1, int turn) {
    float a = Wrap(q1 - w.pose.yawRad);
    if (turn > 0 && a < 0.0f) a += 2.0f * PI;
    if (turn < 0 && a > 0.0f) a -= 2.0f * PI;
    float r = (a > 0.0f ? 1.0f : -1.0f) * RADIUS_MM, n = ceilf(fabsf(a) * RADIUS_MM), q = w.pose.yawRad;
    for (int i = 0; i < (int)n; ++i) {
        float qn = q + a / n;
        w.pose.xMm += r * (sinf(qn) - sinf(q)); w.pose.yMm -= r * (cosf(qn) - cosf(q)); q = qn;
        w.pose.yawRad = q; Check(w.pose.xMm, w.pose.yMm, q);
    }
}
/* Runs the queued home moves the way LaunchNext/HOME/PARK do (the ENTRY and DIAG branches mirror LaunchNext's); returns 1 if
 * the angled park was used. */
static int Execute(void) {
    bool perturbed = false;
    for (unsigned guard = 0; w.moveNext < w.moveCount && guard < 40; ++guard) {
        Task2WrapMove *m = &w.moves[w.moveNext];
        if (m->kind == TASK2_WRAP_S_TO_Y) { ExpandS(&w, w.moveNext); continue; }
        if (m->kind == TASK2_WRAP_ENTRY) {
            float q1;
            if (AngledEntry(&w, &q1)) { *m = (Task2WrapMove){TASK2_WRAP_ARC_TO, (int8_t)(w.pose.yMm >= 0.0f ? 1 : -1), q1, 0.0f}; w.angled = true; }
            else if (m->turn == 1 && w.replans < HOME_REPLANS) { ++w.replans; QueueHome(&w); }
            else *m = (Task2WrapMove){TASK2_WRAP_S_TO_Y, 1, 0.0f, m->speedMmps};
            continue;
        }
        if (m->kind == TASK2_WRAP_DIAG) {
            float th, len, side = w.pose.yMm >= 0.0f ? 1.0f : -1.0f;
            float q0 = PI + Wrap(w.pose.yawRad - PI);
            if ((m->turn == 1 && DiagonalExact(&w, w.pose.xMm, w.pose.yMm, q0, &th, &len)) ||
                DiagonalHomeFrom(&w, w.pose.xMm, w.pose.yMm, q0, &th, &len)) {
                Task2WrapMove out[3] = {{TASK2_WRAP_ARC_TO, (int8_t)side, PI + side * th, 0.0f}, {TASK2_WRAP_STRAIGHT, 0, len, m->speedMmps},
                                        {TASK2_WRAP_ARC_TO, (int8_t)-side, PI, 0.0f}};
                Splice(&w, w.moveNext, out, 3U);
            } else { if (w.replans < HOME_REPLANS) ++w.replans; QueueHome(&w); }
            continue;
        }
        ++w.moveNext;
        if (m->kind == TASK2_WRAP_ARC_TO) Arc1(m->value, m->turn);
        else if (m->kind == TASK2_WRAP_STRAIGHT) Straight1(m->value);
        else if (m->kind == TASK2_WRAP_TO_X) {
            Straight1((m->value - w.pose.xMm) / cosf(w.pose.yawRad));
            if (!perturbed || perturbEvery) { w.pose.xMm += perturbX; w.pose.yMm += perturbY; w.pose.yawRad += perturbQ * DEG; perturbed = true; }
        }
    }
    if (w.angled) {
        /* As HOME's end: the parking straight from the measured pose if it still fits, else square up and plan again. */
        float length, tx, ty;
        if (!ParkStraightOk(&w, &length) && w.replans < HOME_REPLANS) {
            ++w.replans; w.angled = false;
            Arc1(PI, Wrap(PI - w.pose.yawRad) >= 0.0f ? 1 : -1);
            QueueHome(&w);
            return Execute();
        }
        if (!ParkStraightOk(&w, &length)) {
            float c = cosf(w.pose.yawRad), s = sinf(w.pose.yawRad);
            EntryTarget(w.pose.yawRad, &tx, &ty);
            length = fmaxf(0.0f, (tx - w.pose.xMm) * c + (ty - w.pose.yMm) * s);
        }
        Straight1(length);
    } else {
        Straight1((ParkX() - w.pose.xMm) / cosf(w.pose.yawRad));
    }
    return w.angled;
}
/* From the far settle (behind O2 near its far end, heading along it): the one arc of 90 deg + theta when FAR_SETTLE
 * takes it, else the 90 deg turn home and the home route, as the Wrap does. */
static float arcErrDeg;   /* the first home arc ends this far past its heading (arc-end error) */
static int FarCase(float o1, float o2, float half, int side, float yawErrDeg, int print) {
    memset(&w, 0, sizeof(w)); w.io = &io;
    w.o1FaceXMm = o1; w.o2FaceXMm = o2; w.nearEndYMm = -(float)side * half; w.farEndYMm = (float)side * half;
    w.pose = (Task2RoutePose){o2 + 100.0f + RADIUS_MM - END_SHORT_MM, (float)side * (half - END_SHORT_MM),
                              ((float)side * 90.0f + yawErrDeg) * DEG};
    minO1 = minO2 = minPark = minO1True = INFINITY;
    float q1;
    int one = AngledEntryFrom(&w, w.pose.xMm, w.pose.yMm, PI + Wrap(w.pose.yawRad - PI), &q1);
    if (one) {
        Arc1(q1 + (float)side * arcErrDeg * DEG, side);
        w.angled = true;
        w.moveCount = w.moveNext = 0U;
        Execute();
    } else {
        Arc1(PI + (float)side * arcErrDeg * DEG, side);
        QueueHome(&w);
        Execute();
    }
    float inside = InsideCarPark(w.pose.xMm, w.pose.yMm, w.pose.yawRad, 0.0f);
    if (print) printf("far settle O1 %.0f O2 %.0f ends ±%.0f side %+d yaw err %+.0f arc err %+.2f: %s, replans %u, O1 %.1f (real %.1f), O2 %.1f, walls %.1f, inside %d\n",
                      o1, o2, half, side, yawErrDeg, arcErrDeg, one ? "one arc" : "turn + home", w.replans, minO1, minO1True, minO2, minPark, (int)inside);
    farOne += one;
    farRechecked += w.replans > 0;
    /* Exact arcs must keep the planner's margins; with an arc-end error the path must stay off the real obstacles and
     * walls and end inside. */
    return arcErrDeg == 0.0f ? (minO1 < 59.5f || minO2 < 59.5f || minPark < 59.5f || !inside)
                             : (minO1True < 30.0f || minO2 < 30.0f || minPark < 20.0f || !inside);
}
static int Case(float o1, float o2, float half, float x0, float y0, float yawDeg, int print) {
    memset(&w, 0, sizeof(w)); w.io = &io;
    w.o1FaceXMm = o1; w.o2FaceXMm = o2; w.nearEndYMm = half; w.farEndYMm = -half;
    w.pose = (Task2RoutePose){x0, y0, yawDeg * DEG};
    minO1 = minO2 = minPark = minO1True = INFINITY;
    QueueHome(&w);
    int angled = Execute();
    float inside = InsideCarPark(w.pose.xMm, w.pose.yMm, w.pose.yawRad, 0.0f);
    if (print) printf("O1 %.0f O2 %.0f ends ±%.0f from (%.0f, %.0f, %.1f°) arrival %+.0f/%+.0f/%+.0f: %s, replans %u, O1 %.1f (real box %.1f), O2 %.1f, car park walls %.1f, inside %d\n",
                      o1, o2, half, x0, y0, yawDeg, perturbX, perturbY, perturbQ, angled ? "angled" : "square", w.replans,
                      minO1, minO1True, minO2, minPark, (int)inside);
    /* Without an arrival error every check is the planner's own margin; with one, the path must still stay off the
     * real obstacles and walls and end inside. */
    bool exact = perturbX == 0.0f && perturbY == 0.0f && perturbQ == 0.0f;
    return exact ? (minO1 < 59.5f || minO2 < 59.5f || minPark < 59.5f || !inside)
                 : (minO1True < 30.0f || minO2 < 30.0f || minPark < 30.0f || !inside);
}
static int HomeRoutes(void) {
    Case(603.0f, 2203.0f, 150.0f, 2253.0f, -375.0f, 177.0f, 1);        /* Codex 467 */
    int n = 0, low = 0, angled = 0;
    float d1s[] = {600, 900, 1500}, gaps[] = {600, 1000, 1500}, halves[] = {150, 300, 530, 1000}, yaws[] = {-3, 0, 3};
    for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) for (int c = 0; c < 4; ++c) for (int s = -1; s <= 1; s += 2) for (int k = 0; k < 3; ++k) {
        float o1 = 3.0f + d1s[a], o2 = o1 + 100.0f + gaps[b], half = halves[c];
        float x0 = o2 + 50.0f, y0 = (float)s * (half - 50.0f + RADIUS_MM);   /* after the far settle and the 90 deg turn home */
        int bad = Case(o1, o2, half, x0, y0, 180.0f + yaws[k], 0);
        ++n; low += bad; angled += w.angled;
        if (bad) Case(o1, o2, half, x0, y0, 180.0f + yaws[k], 1);
    }
    printf("%d poses, %d angled, %d below 59.5 mm or not inside\n", n, angled, low);
    /* Codex 469: the lane entry planned before O1, the robot arriving 5 mm short. */
    perturbX = 5.0f;
    int c469 = Case(603.0f, 1303.0f, 150.0f, 1353.0f, -375.0f, 177.0f, 1);
    /* Arrival errors after the lane straight for every pose: +-5 mm along, +-5 mm across, +-1 deg. */
    static const float px[6] = {5, -5, 0, 0, 0, 0}, py[6] = {0, 0, 5, -5, 0, 0}, pq[6] = {0, 0, 0, 0, 1, -1};
    int m = 0, lowP = 0, replanned = 0;
    for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) for (int c = 0; c < 4; ++c) for (int s = -1; s <= 1; s += 2) for (int k = 0; k < 3; ++k)
        for (int j = 0; j < 6; ++j) {
            perturbX = px[j]; perturbY = py[j]; perturbQ = pq[j];
            float o1 = 3.0f + d1s[a], o2 = o1 + 100.0f + gaps[b], half = halves[c];
            float x0 = o2 + 50.0f, y0 = (float)s * (half - 50.0f + RADIUS_MM);
            int bad = Case(o1, o2, half, x0, y0, 180.0f + yaws[k], 0);
            ++m; lowP += bad; replanned += w.replans > 0;
            if (bad) Case(o1, o2, half, x0, y0, 180.0f + yaws[k], 1);
        }
    /* Codex 503: the same error after every straight, so re-plans run out: the lane past O1, never across it. */
    perturbEvery = true;
    perturbX = 0.0f; perturbY = 5.0f; perturbQ = 0.0f;
    int c503 = Case(903.0f, 2003.0f, 150.0f, 2053.0f, -375.0f, 177.0f, 1);
    for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) for (int c = 0; c < 4; ++c) for (int s = -1; s <= 1; s += 2) for (int k = 0; k < 3; ++k)
        for (int j = 0; j < 6; ++j) {
            perturbX = px[j]; perturbY = py[j]; perturbQ = pq[j];
            float o1 = 3.0f + d1s[a], o2 = o1 + 100.0f + gaps[b], half = halves[c];
            float x0 = o2 + 50.0f, y0 = (float)s * (half - 50.0f + RADIUS_MM);
            int bad = Case(o1, o2, half, x0, y0, 180.0f + yaws[k], 0);
            ++m; lowP += bad; replanned += w.replans > 0;
            if (bad) Case(o1, o2, half, x0, y0, 180.0f + yaws[k], 1);
        }
    perturbEvery = false;
    perturbX = perturbY = perturbQ = 0.0f;
    printf("%d perturbed runs (half with the error after every straight), %d re-planned, %d within 30 mm of the real O1 / O2 / "
           "walls or not inside; Codex 469 case %s, Codex 503 case %s\n", m, replanned, lowP, c469 ? "FAIL" : "ok", c503 ? "FAIL" : "ok");
    int nf = 0, lowF = 0;
    for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) for (int c = 0; c < 4; ++c) for (int s = -1; s <= 1; s += 2) for (int k = 0; k < 3; ++k) {
        float o1 = 3.0f + d1s[a], o2 = o1 + 100.0f + gaps[b];
        int bad = FarCase(o1, o2, halves[c], s, yaws[k], 0);
        ++nf; lowF += bad;
        if (bad) FarCase(o1, o2, halves[c], s, yaws[k], 1);
    }
    printf("%d far-settle poses, %d one arc, %d below 59.5 mm or not inside\n", nf, farOne, lowF);
    /* Codex 477: the merged arc 3.53 deg short at D1 600 / G 1000 / W 1060, far side -1. */
    arcErrDeg = -3.53f;
    int c477 = FarCase(603.0f, 1703.0f, 530.0f, -1, 0.0f, 1);
    int ne = 0, lowE = 0;
    farOne = farRechecked = 0;
    static const float errs[2] = {-3.53f, 3.53f};
    for (int e = 0; e < 2; ++e) for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) for (int c = 0; c < 4; ++c)
        for (int s = -1; s <= 1; s += 2) for (int k = 0; k < 3; ++k) {
            arcErrDeg = errs[e];
            float o1 = 3.0f + d1s[a], o2 = o1 + 100.0f + gaps[b];
            int bad = FarCase(o1, o2, halves[c], s, yaws[k], 0);
            ++ne; lowE += bad;
            if (bad) FarCase(o1, o2, halves[c], s, yaws[k], 1);
        }
    arcErrDeg = 0.0f;
    printf("%d far-settle runs with the first home arc off by 3.53 deg: %d one arc, %d re-checked, %d within 20/30 mm of real walls/obstacles or not inside; Codex 477 case %s\n",
           ne, farOne, farRechecked, lowE, c477 ? "FAIL" : "ok");
    return low != 0 || lowP != 0 || c469 || c503 || lowF != 0 || lowE != 0 || c477;
}

/* QueueHome from heading-home poses with heading errors up to 8 deg either way: an S that the measured heading
 * carries past the parking spot must be refused, not turned into a negative step count (built with
 * float-cast-overflow, so that would stop the test). Includes the pose Codex 501 found. */
static int HeadingErrorSweep(void) {
    unsigned n = 0U;
    for (int e = -8; e <= 8; ++e) for (float y0 = -400.0f; y0 <= 400.0f; y0 += 25.0f) for (float x0 = 300.0f; x0 <= 900.0f; x0 += 7.0f) {
        memset(&w, 0, sizeof(w)); w.io = &io;
        w.o1FaceXMm = 603.0f; w.o2FaceXMm = 1303.0f; w.nearEndYMm = -150.0f; w.farEndYMm = 150.0f;
        w.pose = (Task2RoutePose){x0, y0, PI + (float)e * DEG};
        QueueHome(&w); ++n;
    }
    memset(&w, 0, sizeof(w)); w.io = &io;
    w.o1FaceXMm = 603.0f; w.o2FaceXMm = 1303.0f; w.nearEndYMm = -150.0f; w.farEndYMm = 150.0f;
    w.pose = (Task2RoutePose){644.391113f, -375.0f, 185.0f * DEG};
    QueueHome(&w); ++n;
    printf("heading-error sweep: %u QueueHome calls: PASS\n", n);
    return 0;
}
int main(void) {
    int bad = Geometry();
    bad |= HomeRoutes();
    bad |= HeadingErrorSweep();
    puts(bad ? "task2 wrap home: FAIL" : "task2 wrap home: PASS");
    return bad;
}
