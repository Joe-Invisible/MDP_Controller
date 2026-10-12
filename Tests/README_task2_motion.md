# Standalone Task 2 route test

`TestMain.c` runs `Task2MotionTestRun()` once. Each step calls the existing
`MotionController_MoveStraight()` or `MotionController_MoveArc()` API and stops
before preparing the next step. Arc exits retain the existing yaw-priority
policy and the previous stopped-run tuning. No experimental motion sequence
planner is used by the test or its geometric preflight.

Edit `task2RouteConfig` in `Src/Task2Route.c`: both arrows, O2 width, obstacle
spacing, buffer, speeds, camera range and the route geometry are grouped there.
The current eight-step route is retained as a baseline for subsequent primitive
planning work. O2 width remains a configured value; IR-based edge termination
is future work. Peripheral and standalone controller tests remain available.

`task2MotionTestConfig` in `Src/Task2MotionTest.c` controls startup ultrasonic
measurement, sensor offset, update/log periods and timeouts. The sensor offset
is 210 mm from the rear axle; `parkToObstacle1Mm = sensorRangeMm - 40` for the
fixed starting axle pose x=-250. A failed/invalid reading or failed preflight
aborts before motor initialization. Disable `measureObstacle1AtStartup` to use
the manual route distance.

At the measurement screen press SW1 once, then press SW1 at the ready screen to
start the single run. SW1 during motion cancels with braking. The test simulates
instant recognition when O1's complete target is in camera view/range, raising
the soft cruise request from approach speed to cruise speed without restarting
the primitive. O2 retains its constant approach-speed policy. No live camera or
RPi command is connected yet.

The chassis is 227 mm long, from -37 to +190 mm relative to the rear axle. The
20 mm sensor protrusion is included conservatively at full body width. Width
remains an assumed 200 mm; the camera is +110 mm ahead of the axle. The preflight
samples the complete rotated footprint at 1 mm spacing along circular/straight
primitives. This is nominal geometry, not a physical pose-error guarantee.

Break at `Task2MotionTestFinished`, then export while suspended:

```gdb
source exp/gdb_scripts/export_task2_motion.gdb
set $task2_battery_v = 12.0
task2-motion-export task2_run01.txt
```

The export contains the frozen route, startup measurement, controller tuning,
recognition event, checkpoint/final pose estimates, per-step arc exits and all
speed/steering samples. Use a fresh filename. Old debugger breakpoints and
export commands should be replaced with the names above.

Host checks:

```sh
python Tests/Host/test_task2_motion.py
python Tests/Host/test_unified_motion.py
python Tests/Host/test_motion_profile_limits.py
python Tests/Host/test_task2_export.py --gdb /path/to/gdb
```

The host fixture models ideal wheel/gyro tracking. It verifies mirrored routes,
parameter changes, startup ranging, speed changes, stopping, cancellation,
timeouts and exports; it does not model servo lag, slip or camera latency.

## Route graph

![Current configured stopped route](task2_motion_route.svg)

Regenerate from the actual C planner after changing the route configuration
(requires a host C compiler, NumPy and Matplotlib):

```sh
python Tests/Host/plot_task2_motion.py
```

The plotter also accepts `--first-arrow`, `--second-arrow`, `--park-to-o1`,
`--gap` and `--width` overrides. Use `--park-to-o1` for the captured startup
distance when it differs from the configured layout. Each plotted route must
pass the same geometric preflight. This is nominal geometry, not a run trace.
