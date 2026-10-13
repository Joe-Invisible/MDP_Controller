# Standalone Task 2 route test

`TestMain.c` runs `Task2MotionTestRun()` once. Every motion uses the existing
`MotionController_MoveStraight()`, `MotionController_MoveArc()`, or the
caller-terminated `MotionController_MoveStraightContinuous()` and finishes
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

Same-arrow departure keeps its existing ultrasonic/IR behaviour. Opposite arrows
use side IR on the straight only: LEFT selects right IR and RIGHT selects left IR.
The preferred R325 departure arc completes to mirrored +/-90 degrees with IR
polling disabled. The continuous straight then begins at that measured stopped
heading, with no distance endpoint.

Three initial ADC readings <=400 confirm that O2 has already ended. The robot
brakes and uses the beam intersection at the straight-start pose as a conservative
width bound; preceding presence is not required in this case. Otherwise three
readings >=700 acquire O2, and three consecutive readings <=400 confirm eventual
disappearance. Readings 401..699 confirm neither state. An unresolved initial
state times out after 1500 ms; cancellation and overall/braking timeouts remain.

The edge estimate and braking event are distinct. During the straight, a three-
sample median rejects isolated spikes. The detector retains the plateau pose
before a sustained decline: default minimum filtered drop 200 ADC counts, three
sustained lower filtered observations (equal values count) and 20 mm travel. Upward excursions >50 counts
cancel a candidate; a stable new plateau can replace an unconfirmed one. Final
absence confirms the event, but width is calculated from the retained onset
pose. If there is no usable trend candidate, the first absent pose remains an
explicit threshold fallback. These are provisional signal parameters for physical
validation; ADC is used for presence/trend, not converted to a range.

After braking, the route has three separately planned stopped stages:

1. R275 quarter-turn opposite to departure, ending parallel at heading zero.
2. Rear bypass from the measured quarter-turn stop. Its radius follows from
   obstacle clearance and available arena side space. Prefer a tangent rear arc
   aimed through the carpark's body-centre target, so one straight can finish the
   return. Entry collisions get a bounded positioning-straight refinement. The
   predicted tangent straight must clear O1, O2 and the buffered carpark. If it
   cannot, retain the parallel rear-arc/alignment fallback.
3. From the measured rear-arc stop, recheck the straight with that actual heading.
   Its length places the whole body's centre closest to (-250, 0), accounting for
   the rear axle's offset. If the straight cannot fit, use the existing four-choice
   parking join with unequal alignment angles. A direct return can park diagonally;
   full-body containment and wall buffers, rather than exact 180-degree yaw, decide
   acceptance. `obstacle2TangentRearArc` and `obstacle2StraightParkReturn` identify
   these choices in the export.

The R275 turn, positioning straight and rear arc retain the agreed reduced
obstacle-buffer exception. Body overlap and side-boundary violations are still
rejected. Straight return and parking keep the normal 100 mm obstacle buffer,
35 mm arena side margin. Carpark walls have separate margins: **100 mm for
both side walls and their entrance tips**, and **75 mm for the back wall**.
Distances to the finite wall segments are checked even while the chassis is
outside the park mouth, so corner approaches cannot evade the side buffer. The larger of nominal
and observed width is checked, with the original arena side boundaries retained.

The fixed O1 template is validated offline, with no online sweep. Variable O2
primitives use bounded interval certificates and reuse accepted clearance results.
Every planning call has a 12,288 footprint-check budget. Prefix straight distances
are nominal geometry previews, not IR execution limits. `sensorTerminated=true`
tells `LaunchStep()` to use continuous motion; measured lengths replace previews
when the robot stops. Full +/-90-degree departure and a bounded initial-absence
estimate must fit the configured approach and camera limits; some narrow, long-gap
layouts that formerly stopped inside the arc are now rejected at setup.

Exports include `routeSetupMs`, `returnSetupMs` (R275), `parallelReturnSetupMs`
(rear bypass), and `parkingReturnSetupMs`, plus the measured stops at each stage.
Pending flags identify the stage: `obstacle2ReturnPending`,
`obstacle2ParallelReturnPending`, and `obstacle2ParkReturnPending`.

The initial rear-axle coordinate remains -250 mm. The finish target instead
centres the entire collision footprint, including its front protrusion. At a
180-degree heading, the rear axle finishes at -163.5 mm, with 126.5 mm nominal
back-wall clearance and 200 mm side clearance. The tangent straight accounts for
its diagonal heading when targeting the body centre. Stopping and sensor/control
errors still require physical verification.

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
| Left IR | 155 mm, measured | +37.5 mm, measured | +90 degrees |
| Right IR | 135 mm, measured | -33.5 mm, measured | -90 degrees |

The lateral IR offsets use the measured 189 mm width between the outer sides of
the front wheels: left inset 57 mm gives `189/2 - 57 = 37.5`, and right inset
61 mm gives `-(189/2 - 61) = -33.5`. This reference does not change the 200 mm
collision footprint.

The same ultrasonic mount is used for startup ranging and moving measurements.
`task2MotionTestConfig` controls startup ranging, control/log periods and
run/brake timeouts. All sensor configuration is frozen at run start and exported.

Ultrasonic and side IR are enabled by default. Mount coordinates remain
configurable. `aheadMm` is measured parallel to the chassis centreline from the
rear-axle midpoint to the receiver-lens centre; `leftMm` is the separate lateral
offset. Opposite-arrow IR acquisition is not restricted to the nominal O2 width.
IR never acquires during approach or the opposite-arrow departure arc. The legacy
80-degree yaw gate remains applicable to same-arrow observations.

Every fresh selected IR conversion is logged with `irFresh=true`. The trace
includes raw ADC, `irFilteredAdc`, `irPeakAdc`, `irDeclineSamples`,
`irTrendCandidateValid` and `edgeMethod`. The final detector state includes the
onset, first-absence and straight-start poses. Edge methods distinguish threshold
fallback, trend onset and an initial-absence width bound. Regular motion entries
retain the latest reading with `irFresh=false`; ADC faults stop the run.

The 15-degree ultrasonic half-angle, 80 mm range-association tolerance, 20 mm
edge uncertainty and 60 ms sample period are explicit test
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

The host fixture models wheel/gyro tracking, an ideal ultrasonic cone and IR
face intersections. It
checks 128 layouts (both O1 positions, all arrows, four gaps and four widths),
edge acquisition/confirmation, mounted sensor geometry, IR hysteresis, stopping,
cancellation, absent/stuck/faulted sensors and exports in both planner modes.
Both modes check all 128 layouts: 122 complete and six narrow, 1500 mm-gap cases
reject before motors initialize under the full-90-degree policy. All eight faster
mirrored cases at 7000/3000/2000 CPS and current published tuning complete. Tests
also replay run08 ADC (about 507 mm from onset versus the former 751 mm estimate),
its original 751 mm return bound and mirror, run10's abrupt drop settling at an
unchanged median (about 554 mm from onset versus its former 651 mm fallback),
noise/rebounds, initial absence, and
an actual 300 mm obstacle while the planner still assumes 500 mm. Recorded
run09/run10 quarter-turn poses and their mirrors use one parking straight after
the rear arc, with the entire simulated robot inside: minimum wall gaps in the
controller fixture exceed 110 mm at the sides/tips and 102 mm at the back.
Explicit checks reject the old rear-axle finish target and close approaches to
an entrance tip even when the whole chassis is still outside the park.
The run03 recorded stop with a 647 mm width bound and its mirror execute through
the real stopped controller fixture, verifying the measured quarter-turn stop,
direct back arc, obstacle crossings, full-body clearance and parking.
It does not model real
acoustic reflections, servo lag, wheel slip or camera latency.

## Route graph

![Current configured stopped route](task2_motion_route.svg)

For opposite arrows this shows the nominal preview through the IR straight;
the remaining stages are created after their measured stops. Same-arrow plots show the full
nominal guarded route. An executed route can have more primitives. Regenerate
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
