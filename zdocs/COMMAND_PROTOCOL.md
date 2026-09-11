# STM command protocol

USART3 runs at 115200 baud, 8N1. Each request/reply ends in `\n`; `\r\n`
requests are also accepted. Commands are case-sensitive with no spaces.
Wait for `READY` after power-up before sending movement batches.

## One batch at a time

Send up to eight movements in one numbered batch, for example:

```text
Pi:   12:F100;B50
STM:  DONE 12
```

`F` and `B` take positive distances in **millimetres**, including fractions.
Acceptance is silent. STM executes every movement in order, stopping between
movements. `DONE` is emitted only after the final movement is stationary.
The Pi must wait for that outcome before sending the next batch. There is no
queue of future batches, and accepting the next batch replaces the previous
batch record. A new batch parsed during motion receives `NAK <id> BUSY`.

Requests are limited to 64 characters before the line ending. `L`/`R` are
reserved but currently rejected with `UNSUPPORTED`; a batch containing either
is rejected **in full before any movement**. `S` must be a standalone,
unnumbered request; embedding it in a batch is rejected.

## Status and stopping

`STATUS` does not execute or change anything. It reports the current/latest
batch, or `READY` if no batch has been accepted since boot:

| Reply | Meaning |
| --- | --- |
| `READY` | Initialized, no batch recorded yet |
| `BUSY 12` | Batch 12 is executing |
| `DONE 12` | Batch 12 completed and the robot stopped |
| `FAULT 12 UPDATE` | Batch 12 failed; braking was requested and further motion is blocked |
| `STOPPING 12` | Standalone stop is braking/cancelling batch 12 |
| `STOPPED 12` | Braking finished; batch 12 will not resume; new batches are allowed |

Send `S` at any time during execution to cancel the remainder of the batch
and brake. STM replies `STOPPED <id>` once the controller confirms stationary
wheels. Retrying `S` while braking reports `STOPPING <id>` without restarting
the brake. If already idle, it replies `STOPPED` and preserves the last batch
outcome. `S` also clears a latched execution fault after braking completes;
the Pi must decide the next route from the robot's actual position.

On a controller failure STM sends `FAULT <id> <reason>` and never starts the
remaining movements. `FAULT` is **not** confirmation that braking has finished.
Initialization failure sends `FAULT - INIT`; subsequent requests receive the
same response and no movement is accepted. Restart after correcting hardware.

## IDs, rejection, and recovery

IDs run from 0 to 255, wrapping to 0. The first numbered batch after boot may
use any ID. Each accepted batch consumes one ID even if later cancelled or
faulted. Rejections consume no ID. Use exactly the next ID for a new batch.

`NAK <id> <reason>` rejects a request. `NAK <expected-id> SEQ` specifically
reports the ID expected next. `NAK - FRAME` means a malformed/oversized line
or detected byte loss; STM discards that line through its newline.

STM remembers **one** batch, not a history table. Repeating that batch's ID
and movements reports its current outcome without executing again. Reusing
that ID with different movements returns `ID_REUSE`. Older IDs receive a
sequence rejection; do not keep old retries across a 256-ID wrap.

If a reply is missed, send `STATUS` first. Retry a batch only when its
acceptance is uncertain and STM has not restarted. `READY` at boot (or in
response to `STATUS`) means previous execution/ID state has been lost: do not
automatically replay the old route. Re-establish the robot's position first.
For initial integration, a timeout or unexpected status should pause the Pi's
run for inspection rather than automatically resend movements.

Unnumbered movement batches remain available for manual bench testing and
use `-` in replies, but they have no duplicate protection. Once a numbered
batch is accepted, unnumbered movements are rejected until restart.

## Implementation and review

`MotionTask` owns the controllers and the `CommandSession` record. The session
module is plain C with no HAL/RTOS dependencies; it validates/records a whole
batch and exposes its next movement. There is no command task, command queue,
frame-history table, or status/TX mutex. The UART ISR only writes bytes into
the existing stream buffer. The OLED task remains separate.

The control loop polls at most one line and 64 bytes per tick. Normal stop
and status requests are serviced during motion; this is a software stop, not
a hardware emergency stop. Do not flood the UART or pipeline future batches.
Detected receive loss discards a line rather than interpreting partial data.
The polling budget is intended for batches and occasional status requests,
not continuous full-baud streaming. Deadline overruns use elapsed time for
controller updates and skip missed wakeups instead of running catch-up ticks.

Host checks:

```sh
cc -Wall -Wextra -Werror hosttests/parser_test.c Apps/Src/CommandParser.c -IApps/Inc -o /tmp/mdp_parser_test
/tmp/mdp_parser_test
cc -Wall -Wextra -Werror hosttests/session_test.c Apps/Src/CommandParser.c Apps/Src/CommandSession.c -IApps/Inc -o /tmp/mdp_session_test
/tmp/mdp_session_test
```

Hardware review should check stop distance, completion timing, IMU-fault
braking, UART bursts/line recovery, and the 10 ms loop budget. Refresh/rebuild
in CubeIDE so its generated makefiles discover `CommandSession.c` and remove
the deleted `CommandTask.c`.
