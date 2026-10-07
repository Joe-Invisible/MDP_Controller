# Experimental fused motion sequence

Branch: `experiment/fused-motion-sequence`.
Base: `69a661ea8536fc8ef161897878dcdc8f21c5b057` on
`feature/unified-calibration-free-motion` (latest unified control at creation).
The separate IMU-initialisation-hardening branch is not included.

## First implementation

`MotionSequence` owns the batch, lookahead, blend/stop policy and completion.
It uses the new `MotionController_FollowProfile()` capability for each continuous
run. The original straight/arc APIs and their tuning/terminal behaviour are
retained. No heap allocation; capacity is 16 nonzero segments.

Each curvature change gets a symmetric linear curvature ramp. Its full length
is at most `blendLengthMm`; each neighbour supplies at most one quarter of its
own length. Thus blends on both sides of a short segment cannot overlap.
Equal-curvature segments join without a curvature change or an extra speed cap.

The ramp consumes existing distance, adds zero nominal distance, and preserves
the integral of curvature over signed travel. Desired heading is evaluated by
analytic integration, not by multiplying current curvature by total travel.
Intermediate junctions are virtual: their exact position and heading are not
required. The final **nominal** heading and distance are preserved, while the
XY trajectory differs from the original piecewise straight/arc path.

Straight-ending profiles and intermediate stopped runs complete by distance
using the inherited 0.5 mm tolerance. If the batch ends in an arc exceeding the
inherited 0.5-degree minimum, its final run instead retains **yaw-priority arc
termination** for camera heading:

* The final target is the analytically integrated heading of the entire current
  fused run. It includes preceding segments and their blends, so yaw error
  accumulated within that run remains visible to termination.
* The approach speed is limited toward 800 CPS before the terminal window.
  The window begins in the final constant-curvature section, after its incoming
  blend, and at most 30 mm before nominal distance completion.
* The braking decision predicts stopping yaw from the freshest raw gyro rate
  using the inherited 75 ms horizon. Direction comes from final arc curvature
  and travel direction, even when mixed turns give zero or opposite-sign net yaw.
* The controller may brake before nominal distance completion if predicted yaw
  reaches the target, or continue at up to 400 CPS when distance finishes first.
  Creep is also capped by the final segment's requested speed.
* Heading and curvature references freeze at the nominal endpoint during
  overrun. The inherited maximum 30 mm overrun guard still forces braking.

Both termination paths use the existing centred-steering braking and stationary
sample completion. The desired geometric path still preserves nominal distance
and heading area, but final-arc actual distance may now differ to prioritise
camera orientation. Actual tracking and the empirical stopping-yaw prediction
still require hardware validation. No final XY-pose guarantee is provided.

`terminalYawPredictedReached` and `terminalDistanceLimitReached` distinguish
heading-triggered braking from the distance guard; reaching the guard alone does
not demonstrate successful camera heading. Final measured yaw includes braking
and may differ from predicted yaw. Completed-run errors across earlier stops
remain diagnostic and are not carried into the final run's heading reference.

Direction changes and `stopAfter=true` split the batch into rest-to-rest runs.
The next run starts only after braking reports stationary for the configured
sample count, then performs its initial steering preparation. Zero-distance
commands are no-ops; a zero-distance stop marks the preceding segment.

Within a run, odometry, yaw, measured-speed/yaw-rate filters, synchronisation
history and yaw-rate PI state continue across junctions. Gain weights interpolate
between the straight and arc endpoint regimes over each blend. Integral state
is rescaled to preserve its output contribution when Ki changes; Ki=0 clears
that contribution. Opposite-sign arc blends use arc gains throughout. The
profile's raw steering slew limiter remains in effect. A per-run rate override
is copied from the sequence configuration; zero inherits the normal steering
calibration rate. Standalone motions always retain their normal rate, including
when issued after a fused run on the same controller.

## Experimental feedforward bridge and junction speed

The calibrated arc branches stop short of zero curvature. Profiles alone use a
linear bridge from the configured straight raw feedforward to the nearest
point in each branch. This is an **unvalidated near-centre model**, needed for a
continuous straight/arc or opposite-sign blend. Standalone `MoveArc` still
rejects the uncalibrated near-zero gap. The outer calibrated limits are never
extrapolated. Existing servo clamping and the base's limited-headroom arc
experiment remain; feedback may run out of headroom at extreme commands.

Defaults in `Controllers/Src/MotionSequencePlan.c`:

| Setting | Value |
| --- | ---: |
| Maximum full blend length | 100 mm |
| Curvature-changing junction speed ceiling | 2000 CPS |
| Fused-profile raw steering slew limit | 480 raw units/s |
| Feedforward allocation of configured raw slew rate | 50% |

These deliberately faster defaults replace the initial 200 mm / 500 CPS
experiment. The sequence's slew override is used by both the planner and the
steering executor; it does not modify `steeringCalibration` (120 raw units/s).
Set `steeringCommandRatePerSec=0` to inherit that rate. For the original slow
profile use `{.blendLengthMm=200, .junctionSpeedCps=500,
.steeringCommandRatePerSec=0}`. Overrides must be finite and nonnegative.

The planner also limits junction speed using the largest slope of the complete
piecewise-linear feedforward map (including the centre bridge), curvature ramp
length, and effective per-sequence raw slew rate. The remaining slew allowance is reserved
for feedback; it is not a guaranteed physical servo margin. Speed lookahead
uses the configured deceleration to approach each lower segment/junction speed
before its boundary. `MotionProfile` supplies acceleration and final stopping.
These are configurable software protections, not a measured servo speed limit.

## API example

Keep the sequence object and its controller alive until execution finishes.
Check every returned status in application code. Configuration, calibration and
the sequence plan must remain unchanged while running. Do not reuse a running
sequence object or issue separate commands to its controller.

```c
static MotionSequence sequence;

MotionSequence_Begin(&sequence, &robot.motionController, &motionSequenceConfig);
MotionSequence_AddStraight(&sequence, 300.0f, 2000.0f, false);
MotionSequence_AddArc(&sequence, 500.0f, -500.0f, 2000.0f, false);
MotionSequence_AddStraight(&sequence, 300.0f, 2000.0f, false);
MotionSequence_Execute(&sequence); /* asynchronous */

/* At each control period: */
MotionSequence_Update(&sequence, dt);
```

Call only `MotionSequence_Update`, not also `MotionController_Update`.
`MotionSequence_Brake` cancels the remaining batch and enters braking; continue
updates until `MotionSequence_IsBusy` is false. IMU/profile errors retain the
base controller's immediate Stop/coast error semantics and mark the sequence
failed; remaining runs are not launched. `completedMeasuredTravelMm` and
`completedMeasuredYawRad` accumulate completed runs including braking drift.
They are separate from `plan.totalTravelMm` and `plan.nominalFinalYawRad`.

## Software-only checks

Run with Python 3 and GCC on Linux/WSL:

```sh
python3 Tests/Host/test_unified_motion.py
python3 Tests/Host/test_fused_motion.py
# Optional: exporter check using installed host GDB
python3 Tests/Host/test_fused_export.py --gdb /path/to/gdb
```

The fusion test compiles production planner/controller/steering/PID/profile
sources and the actual robot test harness against hardware stubs. It uses
`-Wall -Wextra -Werror -pedantic` and address/undefined-behaviour sanitizers.
Leak tracing is disabled for managed-container compatibility; tested components
perform no heap allocation.

Coverage includes analytic and independently integrated curvature area, total
distance, 150 deterministic generated plans, asymmetric/short segments,
forward/reverse signs, both arc signs, opposite-sign ramps, speed lookahead,
software slew bounds, gain/integral continuity, one preparation per continuous
run, reversals/stop waypoints, capacity/invalid inputs, cancellation, faults,
and the selected hardware harness's completion/cancellation/timeout/final log.
Final-arc cases cover early predicted-yaw braking, continued motion after nominal
arc distance, frozen references on overrun, bounded distance-guard braking,
forward/reverse travel with both turn signs, zero/opposite-sign net targets,
short final arcs, low requested speeds and final-only policy selection.
The faster-profile checks also verify that both original straight/arc test junctions retain
2000 CPS, the executor uses the 480 raw units/s override, a subsequent standalone
motion returns to 120 raw units/s, zero inherits that limit in planner/executor,
and invalid rate overrides are rejected before motion.

The original 300 mm / R=-500 mm arc / 300 mm experiment completed in 4885 ms
in the ideal encoder/gyro harness, versus 8526 ms with the previous defaults
(about 43% less time). The current harness uses the longer A/B course below.
This measures software speed/preparation/braking scheduling with synthetic
perfect speed tracking, not physical servo response or robot accuracy.

The real GDB exporter also passed checks against initialized host ELF data:
plan/config/result fields, all samples, empty/invalid-count guards and the
complete marker. It did not require a running process or a probe.

The standalone suite was brought up to date for the base's explicit raw-zero
straight FF, regime-specific gains and measured-speed FF. The updated suite
also passes against the untouched base `MotionController.c`.

Changed firmware sources passed host syntax checks against the real STM32 HAL
headers, suppressing only 64-bit-host versus 32-bit-HAL cast/overflow warnings.
There is no ARM toolchain/CubeIDE here; a full target build, link/RAM check,
control-period timing and physical robot trials remain local. Stubbed samples
are software checks, not an identified servo/vehicle model or evidence of
hardware tracking performance.

## Robot experiment

`Tests/Src/TestMain.c` selects `MotionSequenceFusionTestRun()` on this branch.
CubeIDE's existing Controllers/Tests source folders include the new `.c` files;
refresh the project and clean/rebuild before flashing. Existing production
UART/RPi integration is outside this first experiment.

The harness runs a mixed-turn course twice at **5000 CPS requested for every
straight and arc**. It contains six forward 500 mm straights and these turns,
with one straight preceding each turn:

| Turn | Radius | Angle |
| --- | ---: | ---: |
| 1 | -275 mm | -90 degrees |
| 2 | +275 mm | +90 degrees |
| 3 | +275 mm | +180 degrees |
| 4 | -275 mm | -90 degrees |
| 5 | +275 mm | +90 degrees |
| 6 | -275 mm | -180 degrees |

Nominal travel is about 6455.75 mm and nominal net yaw is zero. The course is
not a closed square and need not return to its starting position. All travel
is forward: mandatory stops from direction reversals cannot mask the fusion
comparison. Edit `fusionTestSteps[]` to change the course; zero radius selects
straight motion. Each row contains signed distance, radius and requested speed.

**A** sets `stopAfter=false` on every segment and executes one continuous run.
**B** sets `stopAfter=true` on every segment and executes 12 rest-to-rest runs.
The path, controller configuration and sequence configuration are otherwise
identical. The configured 2000 CPS junction ceiling and feedforward slew budget
still apply to A; 5000 CPS is a requested segment ceiling, not a guaranteed speed
through a transition. Each curvature blend is 100 mm for this course.

Both batches end in an arc and use yaw-priority final termination. B's
intermediate arcs finish by distance, as discussed above; B is a stopped-profile
comparison rather than an exact replay of standalone `MoveArc` semantics.

Keep the robot stationary through normal IMU startup. Press/release SW1 to
start A. After A completes, the robot remains stopped and prompts for B.
Return it to the same starting pose and press/release SW1 to start B. Each
button wait and 400 ms hand-clearance delay is excluded from timing. Each run's
timer starts just before `MotionSequence_Execute()` and ends after completion
including controller steering preparation and final braking.

Press SW1 during either trace to cancel and brake. Each trace has a 60 s
watchdog and a further 3 s braking timeout. Failure/cancellation of A prevents B
from starting; failure of either trace prevents a valid timing comparison.
`Passed` means both batches completed without an API error, not that their
physical path/heading met an accuracy criterion.

The final OLED screen shows A's fused time, B's stopped time, milliseconds saved
(`B-A`) and percentage reduction (`100*(B-A)/B`). A negative saving is retained
if the fused trace is slower. Incomplete trials show `INVALID` and no savings.
Debugger-visible `motionSequenceFusionTestResults[0]` and `[1]` retain per-trace
status, times, run counts, nominal/measured travel and yaw, and truncation flags.
`motionSequenceFusionTestComparisonValid` guards the saved-time/percentage
fields. The existing `motionSequenceFusionTestElapsedMs` is the last trace's time.

One shared log holds 500 samples at 100 ms (about 50 s of combined running time)
without duplicating the large sample buffer. Each sample's `comparisonRunIndex`
is 0 for A or 1 for B; `timeMs` restarts at zero for each trace. Manual waiting is
not logged. If full, the final slot is replaced with the latest trace's final
sample and truncation is flagged globally and in that trace's result. Summary
results remain available even if detailed logging is truncated.

In the ideal encoder/gyro host harness, A takes 18250 ms and B takes 22342 ms:
4092 ms saved, or about 18.3%. This is a software scheduling check, not a physical
servo/vehicle performance prediction. The harness also verifies cancellation
and timeout in either A or B, withholding savings for incomplete comparisons,
manual-wait exclusion, per-trace logging and OLED text-row coordinates.

To run the original standalone/yaw-priority sequence harness instead, restore
`MotionControllerSequenceTestRun()` in `TestMain.c`.

Set a breakpoint on `MotionSequenceFusionTestFinished` before starting. Once
the target is suspended there, run these in the GDB console (cwd = project):

```text
source exp/gdb_scripts/export_fused_motion.gdb
set $mctrl_fusion_battery_v = 11.95
mctrl-fusion-export fusion_run01.txt
```

Enter the actual measured voltage. Choose a new output name for each run;
export appends if the name already exists. The command exports both A/B results and the timing comparison, the last
trace's complete plan, inherited tuning, final heading policy/termination flags
and tagged samples from both traces. It neither resumes
the target nor initiates motion. The old automatic sequence-export breakpoint
does not apply to this harness.

Inspect the saved timing comparison together with speed/raw-command tracking,
absence of intermediate preparation/braking in A, and measured final travel/yaw
for both traces. Physical tracking may require longer blends, lower junction
speeds or centre-region calibration; the host tests cannot settle those choices.
