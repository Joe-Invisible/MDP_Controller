/* cc -std=c11 -Wall -Wextra -Werror -IApps/Inc -IControllers/Inc
 * hosttests/endpoint_tolerance_test.c Controllers/Src/MotionProfile.c -lm
 * -o /tmp/mdp_endpoint_tolerance_test && /tmp/mdp_endpoint_tolerance_test
 * Uses actual profile and watchdog with recorded endpoint positions.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "CommandMotion.h"
#include "MotionProfile.h"
#include "MotionWatchdog.h"

static MotionProfile Profile(float tolerance) {
    MotionProfile p;
    assert(MotionProfile_Init(&p, 500.0f, 250.0f, tolerance));
    assert(MotionProfile_Start(&p, 100.0f, 600.0f));
    return p;
}

int main(void) {
    const float observed[] = {99.2598f, 99.0619f, 99.4576f, 99.3916f, 99.3257f};
    for (unsigned i = 0; i < sizeof observed / sizeof *observed; ++i) {
        MotionProfile old = Profile(0.5f);
        for (unsigned tick = 0; tick < 300; ++tick) {
            assert(MotionProfile_Update(&old, observed[i], 0.01f) > 0.0f);
            assert(MotionProfile_IsActive(&old));
        }
        MotionProfile revised = Profile(COMMANDMOTION_STRAIGHT_COMPLETION_TOLERANCE_MM);
        assert(MotionProfile_Update(&revised, observed[i], 0.01f) == 0.0f);
        assert(!MotionProfile_IsActive(&revised));
    }
    MotionProfile p = Profile(COMMANDMOTION_STRAIGHT_COMPLETION_TOLERANCE_MM);
    assert(MotionProfile_Update(&p, 98.9f, 0.01f) > 0.0f);
    assert(MotionProfile_IsActive(&p));
    assert(MotionProfile_Update(&p, 99.0f, 0.01f) == 0.0f);
    /* Arc/shared tolerance restored through the same existing init API. */
    p = Profile(0.5f);
    assert(MotionProfile_Update(&p, 99.0f, 0.01f) > 0.0f);
    assert(MotionProfile_Update(&p, 99.5f, 0.01f) == 0.0f);
    /* A real stall farther from the target remains a fault. */
    MotionWatchdog w;
    assert(MotionWatchdog_Start(&w, 0, 100, 100, WATCH_MOVE));
    assert(MotionWatchdog_Check(&w, 10, WATCH_MOVE, 98) == NULL);
    assert(!strcmp(MotionWatchdog_Check(&w, 2010, WATCH_MOVE, 98), "NO_PROGRESS"));
    puts("endpoint tolerance tests passed");
}
