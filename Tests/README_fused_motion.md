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

Profile completion is distance-based using the inherited 0.5 mm tolerance,
followed by the existing centred-steering braking/stationary-sample completion.
The standalone arc's yaw-priority terminal extension is deliberately not applied
to profiles: it could change the requested fused distance. There is no new
final-yaw correction phase. Actual heading/distance accuracy and braking drift
remain hardware measurements, particularly with the inherited arc heading gain
of zero. This implementation does not guarantee final XY pose.

Direction changes and `stopAfter=true` split the batch into rest-to-rest runs.
The next run starts only after braking reports stationary for the configured
sample count, then performs its initial steering preparation. Zero-distance
commands are no-ops; a zero-distance stop marks the preceding segment.

Within a run, odometry, yaw, measured-speed/yaw-rate filters, synchronisation
history and yaw-rate PI state continue across junctions. Gain weights interpolate
between the straight and arc endpoint regimes over each blend. Integral state
is rescaled to preserve its output contribution when Ki changes; Ki=0 clears
that contribution. Opposite-sign arc blends use arc gains throughout. The
configured raw steering slew limiter remains in effect.

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
| Maximum full blend length | 200 mm |
| Curvature-changing junction speed ceiling | 500 CPS |
| Feedforward allocation of configured raw slew rate | 50% |

The planner also limits junction speed using the largest slope of the complete
piecewise-linear feedforward map (including the centre bridge), curvature ramp
length, and configured raw slew rate. The remaining slew allowance is reserved
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

The selected path is straight +300 mm, arc +500 mm at R=-500 mm, straight
+300 mm, all requested at 2000 CPS. Nominal travel is 1100 mm and nominal yaw
is -1 rad (-57.30 degrees). Each blend is 150 mm long for this path (75 mm
from each neighbour). The plan records the actual junction speed caps.

Keep the robot stationary through normal IMU startup. Press/release SW1 to
start. Press SW1 during execution to cancel and brake. A 20 s watchdog also
cancels; braking has a further 3 s timeout. The test does not restart itself.
`Passed` means the sequence completed without an API error, not that its
physical path/heading met an accuracy criterion.

The log holds 500 samples at 20 ms (about 10 s). A longer run sets
`motionSequenceFusionTestLogTruncated`; the final slot is replaced with the
final sample. Inspect the flag before treating a log as a complete time series.

For a comparison with the same profile endpoint semantics, set
`FUSION_STOP_EACH_SEGMENT=true` in `MotionSequenceFusionTest.c`, rebuild, and run
the identical path. This comparison stops/prepares at every junction. To run
the original standalone/yaw-priority sequence harness instead, restore
`MotionControllerSequenceTestRun()` in `TestMain.c`.

Set a breakpoint on `MotionSequenceFusionTestFinished` before starting. Once
the target is suspended there, run these in the GDB console (cwd = project):

```text
source exp/gdb_scripts/export_fused_motion.gdb
set $mctrl_fusion_battery_v = 11.95
mctrl-fusion-export fusion_run01.txt
```

Enter the actual measured voltage. Choose a new output name for each run;
export appends if the name already exists. The command exports the complete
plan, inherited tuning, result flags and available samples. It neither resumes
the target nor initiates motion. The old automatic sequence-export breakpoint
does not apply to this harness.

First inspect speed and raw-command tracking through both blends, absence of
intermediate preparation/braking, final travel/yaw, and time versus the stopped
comparison. Extend to reverse travel, both turning signs, opposite-sign arcs
and short segments only after the initial path is characterised. Physical
tracking may require longer blends, lower junction speeds or centre-region
calibration; the host tests cannot settle those choices.
