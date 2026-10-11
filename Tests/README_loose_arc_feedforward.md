# Loose-arc feedforward calibration

Branch: `experiment/loose-arc-feedforward`.
Base: `ea5750f43309609591a91058b90ca7895ef49d18`, immediately before
`f65608d` introduced yaw-priority arc termination. The controller uses ordinary
profile-distance completion, without a yaw-based stop, extension or terminal
approach envelope. This is an isolated calibration branch, not a fusion branch.
Later straight tuning, separate straight/arc limits, fusion and IMU-hardening
changes are not part of this base. Transfer measured table points back to the
fusion branch; do not merge this older controller wholesale.

`Tests/Src/TestMain.c` selects `MotionControllerArcTestRun()`, implemented by
`Tests/Src/MotionControllerArcTest_original.c` (the other arc source is excluded
from Debug builds). Defaults:

- Forward distance: 1000 mm; requested speed: 5000 CPS.
- Acceleration and deceleration: 2000 mm/s², allowing a cruise section at 5000 CPS.
- `ARC_TEST_FF_EXP = 1`: yaw-rate P/I/D and outer heading gains are zero.
- Wheel PID: P0.03, I0; synchronization: P10, maximum 100 CPS correction.
- Deterministic initial centring; candidate steering is applied while stopped,
  then held for the configured 500 ms arc preparation time before driving.
- Nominal rear-wheel ratio follows requested curvature throughout acceleration,
  cruise and deceleration; actual speed targets include synchronization correction.
- Steering recentres during braking. Use matched ARC-exit yaw and centre distance
  for calibration, rather than mixing exit yaw with final stopped distance.

The table in `Controllers/Src/MotionControllerConfig.c` contains three explicitly
**provisional**, unmeasured positive-curvature entries:

| Requested radius | Raw command |
| ---: | ---: |
| +2000 mm (default) | -16.875 |
| +3000 mm | -11.25 |
| +6000 mm | -5.625 |

Edit `ARC_TEST_RADIUS_MM` for each candidate, rebuild/flash and run separately.
All three nominal 1 m paths fit approximately within a 1.4 × 0.8 m area under an
assumed centred 300 × 200 mm chassis; allow actual chassis offsets and tracking
error. Retain the same preparation procedure. Inspect measured wheel speeds and
raw yaw rate to confirm cruise and consistency. Zero command can be checked with
`MotionControllerStraightFeedforwardTest` in open-loop mode separately.

Replace each provisional curvature with:

`measured exit yaw [degrees] * pi / 180 / measured exit centre distance [mm]`

Keep entries sorted by increasing measured curvature. If measurements change
that order/sign or depend strongly on approach history, investigate rather than
forcing the results into the expected radii. Once curvature is replaced, the
original requested radius may no longer select the same raw candidate: keep
acquisition notes and avoid unintentionally recalibrating another command.

## Export one run

Set a breakpoint at `MotionControllerArcTestFinished`, reached after final
results and the final sample have been stored. After stopping there, source the
script with GDB's working directory at the project root:

```gdb
source exp/gdb_scripts/export_arc_feedforward.gdb
set $arc_ff_battery_v = 11.43
arc-ff-export arc_ff_r2000_run01.txt
```

Use the actual voltage and a fresh filename every time; logging appends.
The script exports requested distance/radius/speed, actual prepared raw command,
configuration, exit/final measurements, timeout diagnostics and the 20 ms sample
array, including gyro and wheel speeds. Failed starts do not reach the finished
hook. Check command acceptance, timeout, exit capture and log count before using
results. `ARC_TEST_LOG_CAPACITY` holds 500 samples (about 10 s); timeout is 10 s.

Host checks exercise all three candidates with deliberately over/underturned
headings, fixed steering and nominal wheel ratios: heading cannot stop a run
before its distance, and insufficient yaw cannot extend it beyond distance.
These are software checks; they do not validate physical curvature or slip.
