# Task 2 integration build

`feature/task2-integration` is based on `yash/task2-commands` (`b420ec7`). It replaces the timed course placeholder with `Task2App`, using the existing numbered RPi protocol. This is an integration candidate: firmware and host tests do not establish reliable floor routes, ultrasonic geometry, or wall clearance on the physical robot.

## Build and select the mode

Import the repository's existing project into STM32CubeIDE, clean, and build Debug or Release. New application files are under the existing `Apps` source root; no additional task needs registration.

The checked-in settings select actual motion:

| Setting | Value | Purpose |
| --- | --- | --- |
| `Core/Src/main.c`: `BRANCH_TO_TESTS` | `0` | Start the RTOS applications |
| `Apps/Inc/Task2Course.h`: `TASK2_COURSE_DRY_RUN` | `0` | Run the real Task2App |
| MotionTask stack | 6144 bytes | Accommodate stationary geometry planning |
| FreeRTOS heap | 24576 bytes | Accommodate application stacks and RTOS objects |
| USART3 | 115200, 8N1 | Existing Pi command link |

For a protocol-only board test, define `TASK2_COURSE_DRY_RUN=1` for the entire firmware build. That mode preserves the collaborator's timed stages, never launches Task2 movement, and displays `T2 DRY RUN`. Remove the define for actual motion. CubeMX regeneration must preserve the heap setting and the application task stack.

## RPi test sequence

Send newline-terminated ASCII lines over the existing command link. A run number is a batch sequence ID, not an independent counter: after accepting a numbered batch, the next new batch must use the next ID modulo 256. The first ID after reboot can be any value.

1. Wait for `READY`. Use `Q U` and `Q I` to inspect sensor freshness, status and raw IR ADCs.
2. Place the robot at the configured rear-axle start `(-250, 0)` facing out of the carpark. Each run resets its local pose and requires fresh startup ranging. A restart assumes the robot has been placed back at this start.
3. Send `12:T2L` for an already recognised left O1 arrow, `12:T2R` for right, or `12:T2` to provide arrow 1 later. A newly accepted batch has **no immediate ACK**; expect `EV 12 APPROACH 1`. `Q` reports `BUSY 12`. Do not confuse this with `A`/`W`, which do receive ACKs.
4. For a plain `T2`, send `A 12 1 L` or `A 12 1 R` during `APPROACH1`/`WAIT1`. Expect `ACK 12 1 L` or `R`. Without arrow 1, the robot stops at the O1 decision point in `WAIT1`.
5. After `EV 12 APPROACH 2`, send `A 12 2 L` or `A 12 2 R`. Earlier arrow-2 messages receive `NAK 12 2 STATE`. The stored arrow is immutable; identical retries are acknowledged, conflicting retries are rejected. Without arrow 2, the robot makes a bounded recognition approach and stops in `WAIT2`.
6. Optionally send `W 12 450` while the run is still before `ROUND2`. This is the camera bullseye span, a **lower bound**, not an exact width. It can enlarge the planning footprint; it never makes the robot treat the bullseyes as the full obstacle. The latest accepted span is consumed again at the measured departure stop, including during stationary USS acquisition. After `EV 12 ROUND 2`, width messages receive `NAK 12 W STATE`.
7. Use `Q T` for `T 12 STAGE a1 a2`; `Q`, `Q U`, `Q I`, `Q P` and existing diagnostics remain available. On success expect `EV 12 SUMMARY x1 x2 w2 path`, then `DONE 12`. Summary distances are rounded millimetres in the run's local frame; an initially absent IR supplies a planning bound and reports `w2` as `-`.
8. Send standalone `S` to cancel. `STOPPED 12` is reported after the wheels become stationary. A fault reports `FAULT 12 reason` and remains latched until `S`; the next new run uses ID 13. Repeating ID 12 queries the previous batch outcome, rather than restarting the robot.

`A - 1 L/R/-` remains supported by the protocol as a replaceable preview. The production app does not pre-plan from that preview; the run's arrow comes from `T2L/T2R` or `A r 1 X`. The preview is cleared at run start.

## Execution and estimation

`MotionTask` remains the sole owner of the motors, steering and controller. `Task2Run` owns terminal protocol states; `Task2Course` selects the real app or explicit dry run. `Task2AppIo` starts primitives, requests speed, brakes, and reads snapshots. `SensorTask` remains the only owner of periodic USS/ADC acquisition. The app counts each sequence once and associates its acquisition timestamp with a short pose history.

| Phase | Production behaviour |
| --- | --- |
| O1 ranging | Stationary median of 11 fresh valid USS readings; 2500 ms deadline |
| O1 approach | Drive to the camera entry anchor, accelerate on actual arrow receipt, stop before committing to the bypass |
| O1 bypass | Two stopped R275 arcs, using the actual RPi arrow |
| O2 gap ranging | Stationary median of 11 face-coordinate estimates at the measured O1 exit; reject a missing/out-of-range gap |
| O2 recognition | Approach only if arrow 2 is missing; brake early when it arrives |
| LL/RR departure | Skip further recognition approach if the known arrow and measured pose admit a certified departure; acquire a face baseline, then execute a bounded outward arc and diagonal leg |
| LR/RL departure | Plan the approach from the measured gap; ignore IR during the arc to the side-facing heading; run the next straight continuously until confirmed IR absence |
| Edge and return | Brake on confirmed edge, then plan lateral/quarter-turn, rear bypass and parking from their actual stopped poses |
| Parking | Prefer the rear arc plus one straight when feasible; use bounded correction/alignment fallbacks otherwise; require the measured full body inside the buffered park before DONE |

`Task2Estimate` separates dimensions, source, uncertainty and bound semantics from `Task2Route` geometry and `Task2App` execution. A future camera gap can enter through `Task2Estimate_SetGap(..., TASK2_ESTIMATE_CAMERA)` before O2 planning; the stationary gap stage consumes an already valid supplied estimate. Adding a Pi message/provider for that is separate work. No camera range/width message is invented in this branch.

Current USS face projection assumes a flat face and the nearest ray within the configured 7.5-degree half cone. Its oblique-heading accuracy is provisional. O1 position and O1–O2 gap accept the configured 600–1500 mm course range; gap validation tolerates 1 mm of endpoint rounding. There is no silent nominal-gap fallback. Width starts with the **configured 500 mm planning footprint**, which is an assumption for integration, not an observed upper bound. A span or sensed edge can enlarge the return footprint while preserving the fixed arena width.

## Configuration and diagnostic points

| File / object | What to adjust or inspect |
| --- | --- |
| `Apps/Src/Task2Route.c`: `task2RouteConfig` | Geometry and nominal width; snapshotted per run |
| `Apps/Src/Task2App.c`: `task2AppSensorDefaults` | Sensor mounts, filters, ADC thresholds, acquisition deadlines |
| `Apps/Src/MotionTask.c`: private Task2 configuration | PI gains 200/100, heading gain 0.2; straight acceleration/deceleration 3000 and arc 2000 mm/s²; Task2 wheel P gain 0.03 |
| `Apps/Inc/SensorConfig.h` | Existing USS 70 ms / IR 50 ms sampling policy and 250 ms freshness limits |
| Debugger: `task2.course.app` in MotionTask | Phase, step, measured pose, estimates, edge method, route report and failure reason |

The normal command-mode tuning is restored at the next stopped batch boundary. Shared motion primitives now support continuous straight motion, live cruise requests, per-primitive acceleration limits and the experimentally exercised arc terminal policy. Existing standalone test initializers are migrated to the split acceleration fields.

Default speed ceilings are 7000 CPS cruise, 3000 approach and 2000 departure; the IR straight is capped at 500 until acquired and 750 after a trend candidate while absence confirmation settles. Every finite primitive retains the movement watchdog. The continuous IR straight has a width-independent 30-second watchdog, fresh-sensor and measured-clearance checks; its execution has no inferred-width distance endpoint. Active app time is limited to 120 seconds, excluding stationary arrow waits.

Carpark side walls and mouth tips retain 100 mm clearance, the back wall 75 mm, and the arena boundary 35 mm. The existing LR/RL obstacle-buffer exception of 1 mm remains in the certified quarter-turn/rear-bypass sections; LL/RR use the normal 100 mm obstacle buffer. A successful edge stop can still fail later measured-pose planning. This branch does not promise a return path for every width, sensor error or pose.

## Host validation

Python 3 and GCC with AddressSanitizer/UndefinedBehaviorSanitizer are sufficient:

```sh
python3 hosttests/test_task2_app.py
python3 hosttests/test_task2_app.py --exhaustive
python3 hosttests/test_command_regression.py
python3 Tests/Host/test_unified_motion.py
```

The real-app harness exercises the real parser/session, Task2Run, estimator, detector and planner with asynchronous ideal sensor snapshots and finite brake delays. It covers all four arrow pairs at 600/1050/1500 mm gaps, the 300 mm initial-absence case, fresh-sample medians with isolated USS outliers, late arrows, timer wrap, cancellation/restart, sensor failure/staleness, a stuck USS finite cap, and a span received during acquisition. Successful cases check the measured full-body parking predicate and stationary summary/DONE ordering. These are software contracts and ideal geometry tests, not physical controller or sensing validation.

For a Pi test without firmware or motors, retain the collaborator's POSIX link simulator and explicitly select dry run:

```sh
cc -std=c11 -Wall -Wextra -Werror -DTASK2_COURSE_DRY_RUN=1 -IApps/Inc \
  hosttests/task2_link_sim.c Apps/Src/CommandParser.c Apps/Src/CommandSession.c \
  Apps/Src/CommandMotion.c Apps/Src/Task2Commands.c Apps/Src/Task2Course.c \
  Apps/Src/Task2Run.c -lm -o /tmp/task2_link_sim
/tmp/task2_link_sim
```
