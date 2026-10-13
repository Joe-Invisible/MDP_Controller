# Standalone Task 2 route test

`TestMain.c` runs `Task2MotionTestRun()` once. Every motion uses the existing
`MotionController_MoveStraight()` or `MotionController_MoveArc()` and finishes
stationary before the next primitive. Fusion remains abandoned. Arc exits keep
the stopped-run yaw-priority policy and tuning.

## Approach and departure

The O1 departure retains its original R275 sweep. Its second arc now ends at
19 degrees inward: -19 after LEFT, +19 after RIGHT. The O2 approach is a straight
at that heading, initially at approach speed, with a 1000 mm maximum length. Both
approaches raise their soft speed request to cruise after simulated recognition.
The complete
assumed arrow must fit the camera range and field of view. The normal departure
endpoint is 500 mm camera-to-arrow-centre range; the planner tries earlier
endpoints in 25 mm range increments if turning clearance requires it.

The departure uses live ultrasonic measurements. A sensor is armed only after
three fresh O2-consistent readings. Three consecutive absent/inconsistent
readings during departure confirm a candidate edge; a renewed return resets
the absence count. An acquisition timeout or reaching the departure guard
endpoint without a confirmed edge fails the run. Trigger/ADC faults are distinct
from completed ultrasonic no-echo timeouts. SW1 cancellation and run/brake
timeouts remain active.

Measurements carry the pose at trigger time. The candidate edge uses the
**first** absent measurement's pose, so confirmation delay does not get added
to the inferred obstacle width. The ultrasonic model projects the inward cone
boundary onto the configured O2 front plane, adds a configurable uncertainty
margin, and assumes O2 is centred on the original centreline. After detection,
the controller brakes; the remaining route is rebuilt from the **stopped pose**,
not the detection pose. Live departure defaults to **1000 CPS** for this first
sensor experiment. It can be raised after checking the edge logs.

The planner first tries tangent circular joins and a tangent straight return.
When that fails, it tries a direct return arc or a straight return with stopped
centreline-recovery arcs. Opposite-side cases that cannot clear the front corner
use a front traverse, R275 end cap, back traverse and exit turn, followed by the
return and parking alignment. These are bounded geometry searches, not a fused
route. Storage capacity is 16 steps; `route.stepCount` is the actual length.

Both the initial guard route and the replanned suffix have full rotated-body
preflight at 1 mm spacing. The suffix checks the larger of the configured test
width and the observed width bound, while retaining the original arena side
limits. Defaults reserve 100 mm of whole O2 width for sensing/model delay in
the initial guard; require 100 mm obstacle clearance, 35 mm side/park-mouth
clearance, and 15 mm park back-wall clearance. An infeasible suffix leaves the
robot stopped and reports failure.

## Configuration

Edit `task2RouteConfig` in `Src/Task2Route.c` for both arrows, the reference O2
width, O1 distance, the valid 600–1500 mm face gap, available end space (default
500 mm per side), speeds, camera geometry, clearances and approach/departure
parameters. The original virtual O1 rejoin anchor only determines its retained
sweep angle; the robot no longer completes that rejoin before O2.

Edit **`task2ObstacleSensorConfig` in `Src/Task2MotionTest.c`** for mounts:

| Sensor | Ahead of rear axle | Left of rear axle | Beam yaw relative to chassis |
|---|---:|---:|---:|
| Ultrasonic | 210 mm, measured | 0 mm | 0 degrees |
| Left IR | 110 mm, assumed | +100 mm, assumed | +90 degrees, assumed |
| Right IR | 110 mm, assumed | -100 mm, assumed | -90 degrees, assumed |

The same ultrasonic mount is used for startup ranging and moving measurements.
`task2MotionTestConfig` controls startup ranging, control/log periods and
run/brake timeouts. All sensor configuration is frozen at run start and exported.

Ultrasonic is enabled by default. **Side IR is disabled by default** because its
positions and voltage thresholds have not been measured. After configuring them,
enable `sideIREnabled`: LEFT departure observes the right IR, RIGHT observes the
left IR. Only a beam facing the O2 front plane can acquire the face. The IR path
uses voltage hysteresis and the same acquired-then-lost confirmation logic.

The 15-degree ultrasonic half-angle, 80 mm range-association tolerance, 20 mm
edge uncertainty, 60 ms sample period and IR thresholds are explicit test
assumptions. They are not calibrated accuracy guarantees. Confirm them using the
mock obstacle and exported observations before increasing departure speed.

O2 width and face gap still describe the **test reference layout**, used for
preflight and echo association. Sensor events control departure termination and
infer a width bound; this is not yet a production planner for an unknown O2
position. Configured arrows and camera visibility simulate recognition; no live
camera/RPi command is connected. Startup ultrasonic remains one fresh O1 reading.

## Running and exporting

Press SW1 at the measurement screen, then at the ready screen to start. A failed
startup reading or preflight aborts before motor initialization. Disable
`measureObstacle1AtStartup` to use the configured O1 distance. The default startup
geometry gives `parkToObstacle1Mm = ultrasonicRangeMm - 40`.

O1 and O2 recognition raise their straight's soft speed request from approach to
cruise speed without restarting the primitive or changing its endpoint. Normal
profile deceleration still stops the approach before the separate departure.
Both recognition times/poses, acquired sensors,
confirmed edge, observed width and stopped departure pose are recorded.

Break at `Task2MotionTestFinished`, then export while suspended:

```gdb
source exp/gdb_scripts/export_task2_motion.gdb
set $task2_battery_v = 12.0
task2-motion-export task2_run01.txt
```

The export includes frozen mounts/thresholds, detector state, route/step count,
startup range, controller tuning, per-step exits, stopped/final pose estimates,
and speed/steering samples with ultrasonic acquisition/loss state. Use a fresh
filename.

Host checks:

```sh
python Tests/Host/test_task2_motion.py
python Tests/Host/test_unified_motion.py
python Tests/Host/test_motion_profile_limits.py
python Tests/Host/test_task2_export.py --gdb /path/to/gdb
```

The host fixture models wheel/gyro tracking and an ideal ultrasonic cone. It
checks 128 layouts (both O1 positions, all arrows, four gaps and four widths),
edge acquisition/confirmation, mounted sensor geometry, IR hysteresis, stopping,
cancellation, absent/stuck/faulted sensors and exports. It does not model real
acoustic reflections, servo lag, wheel slip or camera latency.

## Route graph

![Current configured stopped route](task2_motion_route.svg)

This is the nominal guarded route. A live edge stop inserts a departure stop and
replans the suffix, so an executed route can have more primitives. Regenerate
from the actual C planner (host compiler, NumPy and Matplotlib required):

```sh
python Tests/Host/plot_task2_motion.py
```

Overrides: `--first-arrow`, `--second-arrow`, `--park-to-o1`, `--gap`, `--width`
and `--output`. Each plotted route passes the same preflight. The path references
the rear axle; body checks include the measured chassis (-37 to +190 mm) and the
20 mm ultrasonic protrusion conservatively at full 200 mm width. Camera offset
is +110 mm. Startup pose remains rear axle x=-250; place the visible chassis tail
213 mm from the park back wall.
