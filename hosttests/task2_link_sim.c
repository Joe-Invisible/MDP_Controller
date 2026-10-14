/*
 * The STM's Task 2 protocol on the host: the real CommandSession, CommandParser,
 * Task2Commands, dry-run Task2Course and Task2Run, wired as MotionTask wires them
 * (10 ms ticks, A/W/Q T before the session, events drained every tick, a
 * 200 ms brake after S). Lines in on
 * stdin, replies out on stdout, so the RPi's own code can talk to it.
 *
 * cc -std=c11 -Wall -Wextra -Werror -DTASK2_COURSE_DRY_RUN=1 -IApps/Inc hosttests/task2_link_sim.c
 *    Apps/Src/Command{Parser,Session,Motion}.c Apps/Src/Task2{Commands,Course,Run}.c
 *    -lm -o /tmp/task2_link_sim
 *
 * Only the protocol and course logic are real; there is no motion, UART or RTOS.
 */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

#include "CommandSession.h"
#include "Task2Run.h"

static CommandSession session;
static Task2Run task2;
static bool commandRunning, braking;
static uint32_t brakeUntilMs;
static char reply[COMMANDSESSION_REPLY_SIZE];

static uint32_t NowMs(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000U + ts.tv_nsec / 1000000U);
}

static void Send(const char *line) {
    if (line[0] != '\0') {
        fputs(line, stdout);
        fflush(stdout);
    }
}

static void Drain(void) {
    char event[TASK2COMMANDS_REPLY_SIZE];
    while (Task2Run_NextEvent(&task2, event))
        Send(event);
}

static const Task2CourseIo io = {0};   /* the dry run never moves */

static void HandleLine(char *line) {
    if (Task2Run_Receive(&task2, line, reply)) {
        Send(reply);
        return;
    }
    if (CommandSession_Receive(&session, line, reply)) {
        braking = true;                       /* S: brake for 200 ms */
        brakeUntilMs = NowMs() + 200U;
        commandRunning = false;
        Task2Run_Stop(&task2);
    }
    Send(reply);
}

static void Tick(uint32_t now) {
    bool busy = braking && (int32_t)(now - brakeUntilMs) < 0;
    if (!busy) {
        braking = false;
        Task2Run_Stationary(&task2);
        CommandSession_Stopped(&session, reply);
        Send(reply);
    }
    if (!commandRunning && !busy) {
        const Command *cmd = CommandSession_Current(&session);
        if (cmd != NULL) {
            if (cmd->type != COMMAND_TASK2) {
                CommandSession_Fault(&session, "SIM_T2_ONLY", reply);
                Send(reply);
            } else {
                Task2CommandArrow arrow1 = cmd->param > 0.0f ? TASK2_COMMAND_ARROW_LEFT
                                         : cmd->param < 0.0f ? TASK2_COMMAND_ARROW_RIGHT : TASK2_COMMAND_ARROW_NONE;
                const char *fault = Task2Run_Start(&task2, session.seq, arrow1, &io, now);
                if (fault) {
                    CommandSession_Fault(&session, fault, reply);
                    Send(reply);
                } else {
                    commandRunning = true;
                }
            }
        }
    }
    if (commandRunning && task2.active) {
        const char *fault = NULL;
        Task2RunStatus status = Task2Run_Update(&task2, busy, now, &fault);
        if (status == TASK2RUN_DONE) {
            Drain();
            commandRunning = false;
            CommandSession_CommandDone(&session, reply);
            Send(reply);
        } else if (status == TASK2RUN_FAULT) {
            Drain();
            commandRunning = false;
            Task2Run_Fault(&task2);
            CommandSession_Fault(&session, fault, reply);
            Send(reply);
        }
    }
    Drain();
}

int main(void) {
    CommandSession_Init(&session);
    Task2Run_Init(&task2);
    Send("READY\n");
    char buffer[128];
    size_t used = 0;
    uint32_t next = NowMs();
    for (;;) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(0, &fds);
        struct timeval timeout = {0, 2000};
        if (select(1, &fds, NULL, NULL, &timeout) > 0) {
            char c;
            ssize_t n = read(0, &c, 1);
            if (n <= 0)
                return 0;
            if (c == '\n') {
                buffer[used] = '\0';
                if (used && buffer[used - 1] == '\r')
                    buffer[used - 1] = '\0';
                HandleLine(buffer);
                used = 0;
            } else if (used < sizeof(buffer) - 1U) {
                buffer[used++] = c;
            }
        }
        uint32_t now = NowMs();
        if ((int32_t)(now - next) >= 0) {
            Tick(now);
            next += 10U;
        }
    }
}
