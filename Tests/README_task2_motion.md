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

Same-arrow departure retains ultrasonic edge detection. Opposite-arrow departure
uses **side IR only** for its edge: LEFT observes right IR, RIGHT observes left IR.
Its preferred configurable **R325 arc** stops on a confirmed IR disappearance or at mirrored
**90 degrees**, whichever comes first. If the cap is reached without an edge,
the next primitive travels straight at the stopped heading until IR confirms an
edge. Initial absence cannot count as an edge: three present samples must first
acquire O2, followed by three consecutive absent samples. IR is never read during
the straight O2 approach or below the configurable **80-degree mirrored chassis
yaw gate** (`irAcquisitionMinYawDeg`, adjustable toward 85 degrees). Near that gate,
the arc briefly requests **500 CPS** until three present samples arm IR, then
restores departure speed. Slowing begins early enough to decelerate before the
gate. If IR has not acquired within **1500 ms after the gate**, it reports
`irAcquisitionTimedOut` and stops. Ultrasonic still acquires the face before departure,
but its later echo losses/background returns cannot end opposite-arrow departure.

The straight has a full-footprint-checked guard based on the reference edge plus
sample-confirmation, braking and additional travel reserves. Reaching the guard
without a confirmed edge fails the run. Trigger/ADC faults are distinct from
completed ultrasonic no-echo timeouts. SW1 cancellation and run/brake timeouts
remain active.

The planner checks a fresh-sample acquisition window inside the uncertainty
margin as well as the footprint. It tries the preferred IR radius at earlier
approach endpoints, then a radius 25 mm tighter and R275 if needed. The selected
radius is exported in step 5. These remain stopped circular/straight primitives.

Measurements carry the pose at trigger time. The candidate edge uses the
**first** absent measurement's pose, so confirmation delay does not get added
to the inferred obstacle width. The ultrasonic model projects the inward cone
boundary onto the configured O2 front plane, adds a configurable uncertainty
margin, and assumes O2 is centred on the original centreline. After detection,
the controller brakes; the remaining route is rebuilt from the **stopped pose**,
not the detection pose. Live departure defaults to **1000 CPS** for this first
sensor experiment. It can be raised after checking the edge logs.

For same-arrow departure, the planner first tries tangent circular joins and a tangent straight return.
When that fails, it tries a direct return arc or a straight return with stopped
centreline-recovery arcs. Opposite-side cases that cannot clear the front corner
use a front traverse, R275 end cap, back traverse and exit turn, followed by the
return and parking alignment. These are bounded geometry searches, not a fused
route. After an IR stop near 90 degrees, the return uses a checked end-cap arc,
back straight, exit arc and return/parking alignment. An earlier IR stop can use
the tangent return search when it fits. The initial IR plan contains six guarded
prefix primitives (`obstacle2ReturnPending=true`); the actual stopped pose and
width determine its suffix. Storage capacity is 16 steps; `route.stepCount` is
the actual length.

Firmware trusts the fixed O1 template validated offline: steps 0-2 have no
online footprint sweep. It certifies only the variable O2 approach, departure
and generated return using conservative interval bounds. Shorter approach
candidates reuse a checked longer straight. Accepted candidates retain their
clearance margins; there is no duplicate final startup or return sweep. Each
planning call has a hard limit of 12,288 pose checks; exhaustion reports `PLANNING_BUDGET` and leaves the robot stopped.
IR startup uses fixed-cost face intersections and a return-space screen; the
actual return search runs only after the IR edge stop. Host tests and the route
graph additionally use exhaustive 1 mm validation (`TASK2_ROUTE_EXHAUSTIVE_CHECKS=1`).
Candidates must cross O2 outward and back; a clear loop in front is rejected.
The OLED shows `Route setup`, `Sensor init`, then `Controller init` after the O1
reading. Exports include `routeSetupMs`, `returnSetupMs`,
`startupPlannerPoseChecks` and the final report's `plannerPoseChecks`.
`clearanceFromStep` identifies where the reported geometry coverage begins
(zero-based step 3 for normal MCU routes). Earliest camera previews are computed
only offline (`recognitionPreviewValid`); the test still records actual simulated
recognition poses during motion. The suffix checks the larger of the configured
test width and the observed width bound, while retaining the original arena side
limits. Defaults reserve 100 mm of whole O2 width for sensing/model delay in
the ultrasonic initial guard. IR guard travel uses the configured departure
speed, sample period/count, controller deceleration and
`secondIRDepartureGuardExtraMm` (30 mm). Both require 100 mm obstacle clearance, 35 mm side/park-mouth
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
| Left IR | 155 mm, measured | +37.5 mm, measured | +90 degrees |
| Right IR | 135 mm, measured | -33.5 mm, measured | -90 degrees |

The lateral IR offsets use the measured 189 mm width between the outer sides of
the front wheels: left inset 57 mm gives `189/2 - 57 = 37.5`, and right inset
61 mm gives `-(189/2 - 61) = -33.5`. This reference does not change the 200 mm
collision footprint.

The same ultrasonic mount is used for startup ranging and moving measurements.
`task2MotionTestConfig` controls startup ranging, control/log periods and
run/brake timeouts. All sensor configuration is frozen at run start and exported.

Ultrasonic and **side IR are enabled by default**. Mount coordinates and beam
angles remain configurable. `aheadMm` is the receiver-lens centre's
forward distance from the rear-axle midpoint, parallel to the chassis centreline;
`leftMm` is its separate lateral offset. LEFT departure observes the right IR,
RIGHT observes the left IR. IR is read only during the moving departure, and
approach observations cannot arm its detector. Acquisition additionally requires
the modelled beam to intersect the configured O2 face within its width; merely
facing its plane does not identify O2. This gate uses the test reference layout
and cannot prove that a nearer object is absent. Opposite-arrow departures require
IR enabled and use it exclusively for the edge. Same-arrow departures allow
ultrasonic or the selected IR to confirm the edge. The IR path
uses raw ADC hysteresis: **>=700 counts present, <=400 absent**. Counts 401..699
confirm neither state and reset consecutive-sample confirmation. Three fresh
present samples must acquire the face before three consecutive absent samples
can report its edge. These are initial thresholds from oblique passes across O2;
trend-based edge detection is deferred.

Every fresh selected IR read creates a log entry marked **`irFresh=true`**,
including during the departure arc after the yaw gate and the straight. `irAdc` is valid when `irValid`
is true; `irObserved` distinguishes no readings yet. Entries include `irArmed`,
`irPresentSamples` and `irAbsentSamples`. Regular motion entries contain the last
reading with `irFresh=false`. Faulted reads are logged before stopping. The
minimum 300 mm width check applies to the uncertainty-expanded width bound,
rather than rejecting a slightly underestimated 150 mm edge coordinate.

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
Both modes complete 112 layouts, reject 12 before motors start, and reject four
infeasible actual brake-stop returns while stationary at the IR edge. The
reference-edge startup screen cannot guarantee the later stopped pose has a
return; suffix preflight remains mandatory. The difficult synthetic return also
completes within the MCU work limit. All eight faster mirrored IR layouts at
7000/3000/2000 CPS complete, including the current published configuration.
It does not model real
acoustic reflections, servo lag, wheel slip or camera latency.

## Route graph

![Current configured stopped route](task2_motion_route.svg)

For opposite arrows this shows the guarded prefix through the IR straight;
the return is created after the live edge stop. Same-arrow plots show the full
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
