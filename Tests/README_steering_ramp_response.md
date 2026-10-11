# Open-loop steering ramp response

`SteeringRampResponseTestRun()` is selected in `Tests/Src/TestMain.c`.
Restore `MotionSequenceFusionTestRun()` there to resume fusion testing.
No production controller logic or fusion gain settings are changed.

This experiment measures the moving robot's response to a raw steering command
ramp, including entry/exit asymmetry and the return toward straight. It is open
loop in steering: both straight and arc yaw-rate P/I/D and heading gains are
zero. Rear-wheel speed feedback remains enabled at P=0.03, I=D=0. Wheel
synchronization remains at the values copied from `motionControllerConfig`
(currently 10 CPS/mm, limited to ±100 CPS). Thus the response includes the
servo/linkage, tyre/chassis dynamics, and the geometric rear-wheel references;
it is not a direct measurement of physical servo angle.

## Default sweep

All three trials use **4,500 centre CPS** (about 594 mm/s) and the same
straight → positive-curvature arc → straight sequence. Positive curvature uses
negative raw commands. The endpoint is +R525, currently raw approximately
−45.61. Only the ramp rate changes:

| Trial | Nominal raw units/s | Length of each ramp | Total travel | Planned area with body/clearance |
|---|---:|---:|---:|---:|
| 1 | 50 | 541.5 mm | 2,233.0 mm | 1,881 × 1,833 mm |
| 2 | 100 | 270.8 mm | 1,691.5 mm | 1,872 × 1,358 mm |
| 3 | 200 | 135.4 mm | 1,420.8 mm | 1,773 × 1,126 mm |

The footprint is relative to a rear-axle centre starting at (0,0), heading +x,
with the turn toward +y. It adds a conservative 181 mm body radius and 100 mm
clearance on each side. It estimates the nominal path; actual tracking can
depart from it. Exact centre-path extents are included in each result.

Each trial contains:

1. Deterministic centering, then raw straight feedforward and 0.5 s stationary
   settling before the preparation display.
2. A manual SW1 start, followed by 0.5 s stationary profile preparation.
3. A 350 mm straight lead-in to accelerate and reach cruise speed.
4. A linear **raw-command** ramp into the arc, with curvature interpolated
   through the measured feedforward table.
5. A 300 mm constant-command arc hold (about 0.5 s).
6. The reverse raw ramp back to straight, without recentering or a stop between
   entry and exit.
7. A 350 mm straight hold to observe the response tail.
8. A 150 mm stopping allowance. Deceleration and the controller's final
   centering/braking are marked separately; exclude them when comparing ramps.

Acceleration and deceleration are both 2,000 mm/s². With these defaults the
lead-in and final allowance keep the ramps and straight hold outside the normal
speed-change regions. The ideal host fixture takes approximately 4.55, 3.64,
and 3.19 seconds including profile preparation; physical runs may take longer.
The 300 mm arc hold is an observation window, not proof that the plant settles
within 0.5 seconds.

## Run and export

Set a breakpoint on `SteeringRampResponseTestFinished`, build/flash the Debug
test firmware, and start. Before **each** trial the OLED shows rate, CPS,
estimated travel/area, and gains read from the active controller configuration.
Reset the robot to the same initial pose, then press/release SW1. A press during
motion cancels the trial and ends the batch after braking. Do not restart the
firmware between trials: all samples are retained together.

After the third trial, or after an aborted batch, export once with the target
suspended. Run these commands from the project root, replacing voltage and
filename with the current measurement and a fresh batch filename:

```gdb
source exp/gdb_scripts/export_steering_ramp.gdb
set $steering_ramp_battery_v = 11.43
steering-ramp-export steering_ramp_run01.txt
```

The voltage above is an example, not a measurement taken by the firmware.
The script exports the entire sweep, actual calibration/configuration,
profiles, result codes and samples. It appends if the filename already exists.
`steeringRampBatchComplete` means all three trials completed with a complete
log; it does not mean that tracking was accurate or the steering settled.

Sampling is once per motion-controller update, nominally 10 ms. Each trial
has its own time origin and a reserved 500-sample budget, including a final
sample. The default successful sweep uses about 1,141 samples. A full log,
controller/IMU fault, cancellation, or timeout ends the batch; earlier data
remain exportable. A braking watchdog stops the controller after 3 seconds if
the normal stationary condition never arrives. Result `outcome` names the
reason; `status` preserves the last controller return value.

## Timing and interpretation

The existing fusion controller evaluates profiles by **measured travel**. This
test keeps that API: raw command is linear in travel, and nominal raw units/s
are obtained at the requested cruise speed. It does not enforce an exact
timer-based raw slew when actual speed differs. `rawCommandRatePerSec` records
the command change divided by the actual update interval. Integer encoder
counts and boundary-crossing samples produce small rate variations even at
constant physical speed. The 480 units/s software slew ceiling is deliberately
above the sweep rates; it is not an assumed physical servo capability.

Use `trialIndex`, `mode` and `phase` to separate the ramps and observation
windows. Phase values are 0=preparing, 1=lead-in, 2=ramp-in, 3=arc hold,
4=ramp-out, 5=straight hold, 6=stopping, and 7=finished. Log quantities include:

- `rawReference` and `rawCommand`: planned command and command sent to the
  servo. Neither is measured wheel angle.
- `yawRateDps`: unfiltered gyro response; use this for response delay/ringing.
  `filteredYawRateDps` also retains the controller's 0.10 s filter output.
- `curvaturePerMm` and `measuredCentreSpeedMmps`: the geometric expectation is
  `curvaturePerMm * measuredCentreSpeedMmps * 180/pi` degrees/s.
- Left/right target and measured CPS and wheel-sync correction, to check
  whether speed tracking or synchronization changed the response.

Compare entry and exit separately and compare repetitions before fitting a
delay/rate model. If the slow ramp still shows systematic curvature error at
nearly steady speed, revisit the static table or direction dependence. If the
error grows primarily with ramp rate, dynamic lag becomes a stronger candidate.
This sweep supplies evidence for response-aware planning/feedforward; it does
not automatically change or validate the fused architecture.

## Maintenance and verification

`Tests/Common/SteeringRampProfile.c` owns the pure profile math. It splits each
raw ramp at the static calibration knots, interpolates curvature piecewise,
and integrates desired yaw analytically. This avoids assuming linear raw
command gives linear curvature. `Tests/Src/SteeringRampResponseTest.c` owns the
trial table, hardware preparation, sampling, lifecycle and display.

The experiment-local positive-curvature table starts with the measured loose
points `(0.000054,-5.625)`, `(0.000183,-11.25)` and `(0.000436,-16.875)`, followed
by the existing larger-curvature points from the public table. Public points
at or below the last loose point are replaced by this snapshot. Production
calibration stays unchanged, and the actual table is exported. Update these
three values in `ConfigureExperiment()` if they are recalibrated.

To change the sweep, edit `steeringRampTrials`; its fields are documented in
`SteeringRampProfile.h`. Check travel/area, speed-change clearance and log/RAM
budget when changing speed, distances, rates, trial count or sample fields.
Unsupported endpoints, non-monotonic raw/curvature knots, and capacity overflow
fail preparation before any trial starts.

Run `python3 Tests/Host/test_steering_ramp.py`. It compiles the real motion,
steering and profile code with hardware mocks under address/undefined-behavior
sanitizers. It checks integrated yaw and continuity, inverse feedforward,
encoder wrap, displays, the three-trial lifecycle, cancellation, faults,
log bounds and the brake watchdog. `--gdb /path/to/gdb` additionally exercises
the real export script and invalid-count guards. Hardware response remains to
be measured on the robot.
