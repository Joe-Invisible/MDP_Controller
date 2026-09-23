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

`L` and `R` take positive heading changes in **degrees**, up to 360 per
command. They drive forward arcs with a 275 mm rear-axle-centre radius at
2,000 encoder counts/sec. For example, `L90` travels about 432.0 mm along a
left quarter-circle; it does not turn in place. `F`/`B` retain a 5,000
counts/sec cruise speed. The arc controller prepositions the steering for
0.5 seconds, then profiles the movement and brakes to a stop. `STATUS` and
standalone `S` remain available during steering preparation and motion.

The angle is converted to path length (`radius * degrees * pi / 180`); the
existing controller uses encoder distance and IMU feedback. `DONE` confirms
completion and stationary wheels, not independently measured angular
accuracy. Validate left/right direction and 90/180/360-degree accuracy on
the floor before assessment, and account for the arc in obstacle routes.

Requests are limited to 64 characters before the line ending. A turn above
360 degrees rejects the whole batch with `NAK <id> RANGE` before any movement.
`S` must be a standalone, unnumbered request; embedding it in a batch is
rejected. Limits and conversion live in `Apps/Inc/CommandMotion.h` and
`Apps/Src/CommandMotion.c`.

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

## Movement watchdog and optional diagnostics

The UART runtime guards each movement independently of the calibrated motor
and arc controllers. `Apps/Inc/MotionWatchdog.h` is always enabled:

- `NO_PROGRESS`: centre encoder distance fails to advance at least 1 mm in
  the commanded direction for 2 seconds. Noise/backward motion does not reset
  the timer. This detects lack of encoder progress, not every collision:
  spinning wheels can still count as progress.
- `PREP_TIMEOUT` / `BRAKE_TIMEOUT`: a command spends 2 seconds in steering
  preparation or braking without completing that phase.
- `MOVE_TIMEOUT`: elapsed command time exceeds 5 seconds plus four times
  distance divided by nominal cruise speed. This also bounds slow progress.
- `ODOMETRY`: centre distance is not finite.

These produce `FAULT <id> <reason>`, actively request braking, and cancel
the remaining batch. They never manufacture `DONE` or `STOPPED`. `S` still
requires the normal stationary-wheel check. They are software guards, not
a fix for the uncharacterised physical stall or obstacle-turn collision.
The braking guard applies to execution of a movement; it does not force
completion of a standalone stop or of fault recovery.

`Apps/Inc/MotionDiagnostics.h` and `Apps/Src/MotionDiagnostics.c` own optional
diagnostic data/formatting. Build with `-DMOTION_DIAGNOSTICS=0` to remove
snapshot capture, storage, and the standalone `D` query. Default is `1` for
floor diagnosis in both Debug and Release. Turning it off does not remove
the watchdog. There is no automatic UART telemetry stream.

Send `D\n` through the sole serial owner (not a second reader competing with
A.5). It returns current command data during motion, or the saved last
completion/fault/stop snapshot afterward; before any movement, `D NONE`.
An explicit `S` after a fault preserves the fault snapshot. Starting another
movement replaces it, and resetting the STM clears it. Example:

```text
D 31 step=1 F18.8 NO_PROGRESS mode=STRAIGHT target=18.8 travel=0 left=0 right=0 vl=0 vr=0 steer=0 yaw=0
```

`step` is one-based within the batch. `target`, `travel`, `left`, and `right`
are millimetres; travel values are signed odometry. `vl`/`vr` are measured
encoder counts/second. `steer` is the raw commanded steering value, **not a
measured front-wheel angle**. `yaw` is gyro heading in degrees relative to
the current command. The event token distinguishes `LIVE`, `DONE`, `STOP`,
or the fault reason; a saved snapshot does not claim to be the current pose.
Poll sparingly (at most once a second during diagnosis); prefer a query after
stopping. The bounded diagnostic line is longer than a normal status reply.
The existing Pi movement client continues to consume unchanged FAULT/DONE
replies; its normal reply parser does not yet expose the D query.

Host regression checks (no motors):

```sh
cc -Wall -Wextra -Werror -fsanitize=address,undefined -IApps/Inc hosttests/watchdog_test.c Apps/Src/CommandParser.c Apps/Src/CommandSession.c Apps/Src/CommandMotion.c -o /tmp/mdp_watchdog_test
/tmp/mdp_watchdog_test
cc -Wall -Wextra -Werror -fsanitize=address,undefined -IApps/Inc hosttests/diagnostics_test.c Apps/Src/MotionDiagnostics.c -o /tmp/mdp_diagnostics_test
/tmp/mdp_diagnostics_test
```

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

The UART runtime inherits geometry, wheel synchronisation, motion profiling,
and arc calibration from `motionControllerConfig`. Its straight-heading
tuning stays at `Kp=0.80`, `Ki=0`, `Kd=0`, with a `0.010 rad` steering limit.
`MotionTask` keeps this configuration in static storage because the controller
retains its pointer. Calibration tests use the shared configuration directly;
changing their straight-heading gains does not change the UART runtime.

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
cc -Wall -Wextra -Werror hosttests/session_test.c Apps/Src/CommandParser.c Apps/Src/CommandSession.c Apps/Src/CommandMotion.c -IApps/Inc -o /tmp/mdp_session_test
/tmp/mdp_session_test
cc -Wall -Wextra -Werror hosttests/motion_test.c Apps/Src/CommandMotion.c -IApps/Inc -o /tmp/mdp_motion_test
/tmp/mdp_motion_test
```

Hardware review should check stop distance, completion timing, IMU-fault
braking, UART bursts/line recovery, and the 10 ms loop budget. Refresh/rebuild
in CubeIDE so its generated makefiles discover `CommandSession.c` and
`CommandMotion.c`, and remove
the deleted `CommandTask.c`.
# User button query (A.5 start)

`BUTTON\n` returns `BUTTON <count> <held>\n`. The unsigned counter advances
once per debounced press and release of the physical USERBTN (called SW1 in
the driver). `held` is 1 while pressed or while release is being debounced.
The counter resets at boot; a button held at boot is not a start event.

The Pi takes a baseline after its camera and recognition connection are
ready, then polls every 200 ms for a fresh action. A button already held
when the Pi arms must be released before a new press can start the run.
The query neither starts motion nor changes the active command batch.
This is a start input, not a hardware emergency-stop implementation.

Host debounce check:

```sh
cc -Wall -Wextra -Werror -IApps/Inc hosttests/button_test.c -o /tmp/button_test
/tmp/button_test
```
