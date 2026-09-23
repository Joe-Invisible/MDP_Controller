/* Run: cc -Wall -Wextra -Werror -fsanitize=address,undefined -IApps/Inc
 * hosttests/watchdog_test.c Apps/Src/CommandParser.c Apps/Src/CommandSession.c
 * Apps/Src/CommandMotion.c -o /tmp/mdp_watchdog_test && /tmp/mdp_watchdog_test
 * Exercises stalls, progress, phase deadlines, timer wrap and batch abort.
 */
#include "MotionWatchdog.h"
#include "CommandSession.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>

int main(void) {
    MotionWatchdog w;
    assert(MotionWatchdog_Start(&w, 100, 18.8f, 100, WATCH_MOVE));
    assert(!MotionWatchdog_Check(&w, 2099, WATCH_MOVE, 0));
    assert(!strcmp(MotionWatchdog_Check(&w, 2100, WATCH_MOVE, 0), "NO_PROGRESS"));

    /* A real short move advances then brakes normally. */
    assert(MotionWatchdog_Start(&w, 0, 18.8f, 100, WATCH_MOVE));
    for (unsigned t = 10; t <= 400; t += 10)
        assert(!MotionWatchdog_Check(&w, t, WATCH_MOVE, 18.8f * t / 400));
    assert(!MotionWatchdog_Check(&w, 410, WATCH_BRAKE, 18.8f));
    assert(!MotionWatchdog_Check(&w, 500, WATCH_BRAKE, 18.8f));
    assert(!strcmp(MotionWatchdog_Check(&w, 2410, WATCH_BRAKE, 18.8f), "BRAKE_TIMEOUT"));

    /* Time spent settling must not eat the active movement's grace period. */
    assert(MotionWatchdog_Start(&w, 0, 1100, 80, WATCH_PREPARE));
    assert(!MotionWatchdog_Check(&w, 500, WATCH_PREPARE, 0));
    assert(!MotionWatchdog_Check(&w, 510, WATCH_MOVE, 0));
    assert(!MotionWatchdog_Check(&w, 2509, WATCH_MOVE, 0));
    assert(!strcmp(MotionWatchdog_Check(&w, 2510, WATCH_MOVE, 0), "NO_PROGRESS"));
    assert(MotionWatchdog_Start(&w, 0, 1100, 80, WATCH_PREPARE));
    assert(!strcmp(MotionWatchdog_Check(&w, 2000, WATCH_PREPARE, 0), "PREP_TIMEOUT"));

    /* Signed progress for B is converted by the caller: -1 * -distance. */
    assert(MotionWatchdog_Start(&w, 0, -628, 100, WATCH_MOVE));
    for (unsigned t = 10; t <= 6280; t += 10)
        assert(!MotionWatchdog_Check(&w, t, WATCH_MOVE, t * 0.1f));

    /* Noise, backward drift, and oscillation are not forward progress. */
    assert(MotionWatchdog_Start(&w, 0, 18.8f, 100, WATCH_MOVE));
    for (unsigned t = 10; t < 2000; t += 10)
        assert(!MotionWatchdog_Check(&w, t, WATCH_MOVE, t % 20 ? 0.9f : -2));
    assert(!strcmp(MotionWatchdog_Check(&w, 2000, WATCH_MOVE, 0), "NO_PROGRESS"));

    /* Even slow continuing progress has a finite overall command deadline. */
    assert(MotionWatchdog_Start(&w, 0, 100, 100, WATCH_MOVE));
    for (unsigned t = 1000; t < 9000; t += 1000)
        assert(!MotionWatchdog_Check(&w, t, WATCH_MOVE, t / 1000.0f));
    assert(!strcmp(MotionWatchdog_Check(&w, 9000, WATCH_MOVE, 9), "MOVE_TIMEOUT"));

    assert(MotionWatchdog_Start(&w, UINT32_MAX - 999U, 18.8f, 100, WATCH_MOVE));
    assert(!MotionWatchdog_Check(&w, 999, WATCH_MOVE, 0));
    assert(!strcmp(MotionWatchdog_Check(&w, 1000, WATCH_MOVE, 0), "NO_PROGRESS"));
    assert(!MotionWatchdog_Start(&w, 0, NAN, 100, WATCH_MOVE));
    assert(!MotionWatchdog_Start(&w, 0, 100, 0, WATCH_MOVE));
    assert(!MotionWatchdog_Start(&w, 0, 1e30f, 100, WATCH_MOVE));
    assert(MotionWatchdog_Start(&w, 0, 18.8f, 100, WATCH_MOVE));
    assert(!strcmp(MotionWatchdog_Check(&w, 10, WATCH_MOVE, NAN), "ODOMETRY"));

    /* Fault cancels the remaining route; neither duplicate nor completion
     * can restart it. S still requires confirmed stationary before STOPPED.
     */
    CommandSession s;
    char reply[COMMANDSESSION_REPLY_SIZE];
    CommandSession_Init(&s);
    assert(!CommandSession_Receive(&s, "31:F18.8;L180", reply));
    assert(CommandSession_Current(&s));
    CommandSession_Fault(&s, "NO_PROGRESS", reply);
    assert(!strcmp(reply, "FAULT 31 NO_PROGRESS\n"));
    assert(!CommandSession_Current(&s));
    CommandSession_CommandDone(&s, reply);
    assert(s.state == COMMANDSESSION_FAULT && s.next == 0);
    assert(!CommandSession_Receive(&s, "31:F18.8;L180", reply));
    assert(!strcmp(reply, "FAULT 31 NO_PROGRESS\n"));
    assert(CommandSession_Receive(&s, "S", reply));
    assert(s.state == COMMANDSESSION_STOPPING);
    CommandSession_Stopped(&s, reply);
    assert(!strcmp(reply, "STOPPED 31\n"));
    assert(!CommandSession_Current(&s));
    puts("watchdog tests passed");
}
