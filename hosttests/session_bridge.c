/* Host-only bridge for exercising the real C session from Pi-side tests.
 * Each input returns one line; '-' means the firmware sent no reply.
 * @finish simulates physical completion, @count reports completed commands.
 */
#include "CommandSession.h"
#include <stdio.h>
#include <string.h>

int main(void) {
    CommandSession session;
    char line[128], reply[COMMANDSESSION_REPLY_SIZE];
    unsigned completed = 0;
    CommandSession_Init(&session);
    while (fgets(line, sizeof(line), stdin)) {
        line[strcspn(line, "\r\n")] = '\0';
        reply[0] = '\0';
        if (strcmp(line, "@finish") == 0) {
            while (CommandSession_Current(&session)) {
                ++completed;
                CommandSession_CommandDone(&session, reply);
            }
        } else if (strcmp(line, "@count") == 0) {
            snprintf(reply, sizeof(reply), "%u\n", completed);
        } else if (CommandSession_Receive(&session, line, reply)) {
            CommandSession_Stopped(&session, reply);
        }
        fputs(reply[0] ? reply : "-\n", stdout);
        fflush(stdout);
    }
    return 0;
}
