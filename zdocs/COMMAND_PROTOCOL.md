# STM command protocol

USART3 runs at 115200 baud, 8N1. Each request/reply ends in `\n`; `\r\n`
requests are also accepted. Movement batches are case-sensitive with no spaces; Q subqueries use one space.
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

`Q` (legacy alias `STATUS`) does not execute or change anything. It reports the current/latest
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
Initialization failure sends `FAULT - INIT_*`; ordinary requests receive the
same response and no movement is accepted. Sensor queries and standalone G
remain available. Fix the hardware issue before retrying initialization.

## Explicit reboot (`G`)

Standalone, unnumbered `G` uses the same letter as Team 37's Checklist reset.
It is not a movement and cannot appear inside a batch. MotionTask accepts it
only when neither the session nor the controller is busy/stopping; otherwise
it replies `NAK - BUSY`. Pi must first send S and confirm STOPPED when necessary.
The initialization-failure loop also accepts G without calling uninitialized
controller objects. G disables rear PWM outputs, sends `RESETTING`, and invokes
CMSIS `NVIC_SystemReset`. Ordinary startup then centres the steering, initializes
sensors/controllers and emits READY (or a new INIT fault). No route resumes.
No new task, periodic UART traffic or motor calibration is introduced.

A real CPU HardFault or stuck UART task may never process G. This command does
not install a hardware watchdog or a Pi-to-NRST connection. Use hardware reset
or ST-LINK for an unresponsive CPU. S clears runtime fault latches, not failed
controller initialization or an unrecoverable CPU exception.

Reboot clears volatile STM batch/progress state; the next batch ID is zero.
Before G, the Pi reset helper saves available Q P and its in-memory last batch/
reply to a JSON checkpoint. Saving is only during an explicit reset, not every
movement or control tick. The current Pi process remembers its own last batch;
a new process cannot reconstruct a full old batch from Q P. If UART is dead,
last observed progress may be stale or absent and must remain labelled so.
Only RESETTING followed by READY confirms success; no blind retries or replay.
Run `mdp_checklist/checks/reset_stm.py` on Pi with other serial clients closed.

## Ultrasonic approach (`U200`)

A numbered batch such as `60:U200` approaches forward to a **200 mm remaining
ultrasonic gap**, measured from the transducer front faces. It does not mean
travel 200 mm. `Q U` remains a read-only sensor query. U targets must be
100..1000 mm; out-of-range targets reject the whole batch with `NAK ... RANGE`.
U can share a batch with F/B/L/R. Each U has its own time/travel bound; choose
an appropriate whole-batch Pi timeout (up to 15 seconds per U, plus other moves).

`UltrasonicApproach.c` is application policy with no hardware access. MotionTask
consumes SensorTask snapshots and uses the existing straight controller and
braking APIs. There are no repeated short F commands, new threads, heap
allocations or Pi round trips in the stop decision. The only live profile
adjustment is U's encoder endpoint, not motor/steering calibration. The same
MotionProfile used for F controls acceleration and gradual deceleration.
`UltrasonicApproachConfig.h` owns the initial limits:

- Forward only: no automatic reverse or retry if the robot starts too close.
- Default cruise cap 100 mm/s; an identified test build may override
  `ULTRASONIC_APPROACH_CRUISE_MMPS` at compile time (300 mm/s is software-tested).
- Range must be valid and at most 150 ms old, including before movement starts.
- Each new echo updates the profile endpoint to encoder travel + measured gap
  minus desired gap, compensating capture age using current encoder speed.
  Between echoes, keep that endpoint fixed and let encoder travel consume it.
  Acceleration/deceleration come from the unchanged MotionProfile. No profile
  restart or separate predicted-distance full-brake trigger is used.
- The U profile stops within a 10 mm estimated endpoint band, then enters
  normal controller braking. New echoes during braking cannot restart it.
- Maximum travel is initial range minus target plus 50 mm, capped at 1000 mm.
  Initial gaps needing more than 950 mm travel are rejected before movement.
  Reaching the encoder bound never counts as successful sensor completion.
- Overall limit 15 seconds, plus the existing progress/brake watchdogs.
- After stationary confirmation, require **two distinct post-stop readings**
  within target +/-20 mm before advancing the batch or returning DONE.
  A cached pre-brake sample cannot confirm success. Verification is bounded
  to 1 second; no autonomous corrective retry is performed.

Failure tokens: `US_NOT_READY`, `US_STALE`, `US_ECHO` (echo timeout),
`US_SENSOR` (invalid reading), `US_TARGET`, `US_TOO_CLOSE`, `US_TRAVEL`,
`US_TIMEOUT`, `US_ODOMETRY`, `US_STATE`, `US_VERIFY` (verification timeout),
`US_RANGE` (stopped outside the accepted upper range). They use the existing
`FAULT <id> <reason>` and S recovery. FAULT requests braking; only STOPPED or
successful DONE confirms stationary wheels. The limits are initial software
policy; actual approach accuracy and stopping distances need floor tests.
F/B retain the tested 1 mm straight tolerance; arcs retain shared 0.5 mm.
U uses a separate 10 mm control endpoint band and 20 mm final sensor band.
Capture-age compensation is an approximation; real stopping accuracy requires
floor measurements. A short/spurious echo can still cause an early stop/fault.

## Partial batch progress (`Q P`)

Query the current/latest batch without consuming an ID or altering execution:

```text
Pi:  Q P
STM: P 60 STOPPED 1 3 2 B 100 -48 -2.2 IDLE
```

Fields: `P id state completed total step command parameter travel_mm yaw_deg phase`.
Here batch 60 contained three commands, one completed, and command 2 (B100)
was interrupted after -48 mm of measured travel and -2.2 degrees of relative
heading change. Command 3 never started. `completed` counts whole commands
only; `step` is the 1-based most recently attempted command, or 0 before any.
If stopped between commands, `step == completed` refers to the completed one.
Parameter has the original command's units: F/B travel mm, L/R degrees,
U desired remaining sensor gap mm. Float fields can use decimal or exponent
notation. Travel is signed vehicle-centre encoder odometry and yaw is relative
to that command's start; neither is a global position or an accuracy guarantee.

Phases: `NOT_STARTED`, `ACTIVE` (includes steering preparation), `BRAKING`,
`IDLE`. A start precondition failure reports the attempted command but uses
`-` for unavailable travel/yaw. Invalid odometry also uses `-`, never a fake
zero. Startup: `P - READY 0 0 0 - - - - NOT_STARTED`. A fault reason remains
available through Q/STATUS. Read progress **after confirmed STOPPED** to obtain
the final post-braking snapshot; ACTIVE/BRAKING measurements may still change.
Do not automatically replay a partial command or a route after a reset.

`CommandProgress` is a fixed current/latest snapshot, not history. MotionTask
owns it; sampling occurs at command start, on a Q P request, and when the
controller becomes idle. Formatting/transmission happens only on an explicit
query. No telemetry stream, extra queue or normal-driving UART traffic was
added. UART transmission remains bounded but blocking, so an explicit Q P
query still costs formatting and wire time; prefer querying after stop rather
than polling it during motion. This is protocol functionality independent of
`MOTION_DIAGNOSTICS`; D/D W/D H remain optional debug snapshots.

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
in CubeIDE so its generated makefiles discover all Apps/Src modules, including `UltrasonicApproach.c` and
`CommandProgress.c`, and remove
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
