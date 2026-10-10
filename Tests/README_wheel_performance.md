# Wheel speed and acceleration limits

Branch: `experiment/wheel-performance-limits`, based on `main` (`30558cc`).
`TestMain.c` selects `WheelPerformanceTestRun()`. Use the STM32CubeIDE **Debug**
configuration, which already includes Tests/Common and Tests/Inc.
This is a standalone 100 Hz hardware harness, independent of MotionController,
heading feedback, wheel synchronisation, and fused sequencing. Existing wheel
gains and feedforward calibration are reused without modifications.

## One sweep, one export

1. Build/flash, set a breakpoint on `WheelPerformanceTest_Complete`, then run.
2. At the initial `Set config; SW1` screen, edit `wheelPerformanceConfig` in the
   debugger if required. Enter measured `batteryVoltage`; zero means unrecorded.
3. Choose `experiment=1` for top speed or `experiment=2` for acceleration. Press
   and release SW1 to freeze the configuration for the entire sweep.
4. Each trial displays its direction, requested PWM/acceleration, estimated
   **drive distance**, and the drive-distance guard before movement begins.
   Reposition the robot at the start of the runway and press/release SW1.
   These pauses do not clear the cumulative log. No export is needed between
   trials. The motor outputs are neutral after verified standstill, so the robot
   can be moved by hand. Set `waitBetweenTrials=false` only when the entire
   continuous sweep fits the available test space.
5. SW1 during movement aborts the sweep and brakes both wheels. Distance, timing,
   buffer and stopping faults also terminate the sweep, retaining partial data.
6. At the completion breakpoint, export once using the matching GDB script:

   ```gdb
   source exp/wheel_performance/export_top_speed.gdb
   ```

   or:

   ```gdb
   source exp/wheel_performance/export_acceleration.gdb
   ```

   Paths are relative to GDB's working directory (`pwd` / `cd` in its console).
   Each script writes one whole-sweep `.txt` file there. It overwrites the same
   filename on the next export, so rename it before exporting another sweep.
7. Export **before resetting**. Reset and choose the other experiment for the
   second sweep. The two experiments share one buffer; a reset clears it.

The format uses standard GDB print/logging commands; no embedded GDB Python is
required. The analyzer accepts plain GDB text and CubeIDE's GDB/MI console-stream
wrapping (`~"..."`). Offline conversion uses only Python's standard library:

```sh
python exp/wheel_performance/analyze_sweep.py wheel_top_speed.txt
python exp/wheel_performance/analyze_sweep.py wheel_acceleration.txt
```

This creates trace and trial-summary CSVs from the single hardware export.

## Default sweeps

| Experiment | Points in each direction | Repeats | Trials |
|---|---|---:|---:|
| Top speed | 80%, 90%, 100% direct PWM | 2 | 12 |
| Acceleration | 500, 1000, 1500, 3000, 6000, 10000 mm/s² | 2 | 24 |

Order is the complete forward table, then the reverse table, repeated twice.
Both wheels are driven together; their responses are logged separately. No
automatic direction reversal occurs until both wheels have stopped. Steering
is fixed at raw zero by default; `steeringRawCommand` permits a known straight
setting without changing the experiment's motor commands.

The preparation screen's `Drive ~... mm` excludes braking. In the speed sweep,
it uses the faster wheel's existing direction-specific speed/PWM calibration
and assumes that steady speed for the full drive duration. In the acceleration
sweep, it integrates the requested ramp plus hold: `v²/(2a) + v * holdTime`.
These are planning estimates; unknown top-speed behaviour, tracking error and
slip can change actual travel. The screen explicitly says `Plus stopping space`.
The guard is shown separately, with `Guard may cut run` when the estimate reaches
or exceeds it. Estimates are retained in each trial's exported metadata.
The same screen is refreshed before automatic trials when button pauses are
disabled; the existing 500 ms inter-trial delay and 300 ms steering delay follow.

Top speed applies each PWM step for 1500 ms and retains the startup response.
The last 500 ms is the proposed steady window. The analyzer checks that the
window is present and fitted speed changes by no more than 5% across it; it
does not declare a ceiling from a truncated or unsettled response. Extend
`speedDriveMs` if the full-PWM response has not settled. Use the minimum repeated
100%-PWM mean for each wheel and direction, then the slower wheel, when deriving
a shared speed budget. Apply the chosen margin afterward.

Acceleration commands equal, independent wheel-speed ramps from zero to
5000 CPS, followed by a 200 ms hold. Change `accelerationTargetCps` after the
speed experiment to a value comfortably below the measured loaded ceiling.
Add larger points to `accelerationTable` if all tested values track well.
The range deliberately extends to 10000 mm/s² to expose loss of tracking rather
than stopping the sweep near the existing 1500 mm/s² setting. Logging uses every
10 ms control sample during acceleration so short, aggressive ramps are retained.
The analyzer fits measured speed versus time over each ramp and reports tracking
error and PWM saturation. Startup delay is included in that overall fit; the
trace allows analysis of individual speed ranges separately. Sustained lag at
100% drive and lag with remaining PWM authority have different causes.

## Distance, stopping, and load

Run on the intended floor with the complete robot for loaded limits. Suspended
wheels establish an unloaded ceiling only. Direct PWM does not correct unequal
wheel speeds, so trajectory drift is expected; there is no heading controller.
Visible slip invalidates an encoder-derived chassis acceleration result.

`maxDriveDistanceMm=2000` is a drive-phase guard on either wheel's **absolute**
travel. Braking begins at that limit; it is NOT a total-distance guarantee.
Provide additional stopping space. A distance-limited trial stops the entire
sweep and is marked invalid for a steady-speed ceiling. Choose drive durations
and distance limits for the actual runway before starting.

At each drive endpoint the harness applies full dynamic braking, recording until
both wheels remain below 100 CPS for five 10 ms samples. The 1000 ms brake
timeout leaves the brakes engaged and ends the sweep. Stop distance is retained
per wheel. The stopping phase is measured independently of the requested
acceleration and is not used to certify a deceleration profile.

## Logging and status

Control/encoder observations run at 100 Hz using actual elapsed time. Acceleration
ramp/hold logging runs at 100 Hz; speed and brake logging run at 50 Hz, plus
trial-start and final-stop samples, into a single 60,000-byte
CCM-RAM buffer. `.ccmram` is explicitly cleared by the harness, because startup
does not initialize this section. No linker or startup changes are required.
The frozen configuration and all trial metadata remain in ordinary SRAM.
Configuration validation reserves worst-case drive and brake samples for the
whole sweep. Oversized sweeps are rejected before enabling the motors.

Each sample stores the command applied during the preceding measurement
interval, measured speeds, actual control `dtMs`, trial-relative time, phase,
and both motor modes. Speeds/targets are rounded to 1 CPS; signed PWM is stored
in hundredths of one percent. BRAKE mode has positive braking PWM in either
travel direction; use the mode field when interpreting it. At 50 Hz each speed
measurement still covers its most recent 10 ms control interval. Repositioning
is excluded from trial time and encoder baselines are reset afterward.

Status codes: 0 not started, 1 running, 2 complete, 3 button abort, 4 drive
distance limit, 5 stop timeout, 6 log full, 7 invalid configuration, 8 init
failure, 9 control gap above 50 ms, 10 telemetry out of int16 range.
An aborted sweep may still contain earlier valid trials, but must not be treated
as a fully completed dataset.

## Software validation

Run `python tools/wheel_performance/run_host_checks.py` on a machine with GCC
and Python. The host model exercises complete sweeps, repeated forward/reverse
trials, encoder wrap, manual repositioning, and configuration/distance/button/
stopping/timing/buffer faults. Address and undefined-behaviour sanitizers are
enabled. Export-parser checks cover complete and partial data, plateau rejection,
and acceleration fits in both directions. These are software checks, not motor
measurements or proof of physical stopping performance.

Before the branch was pushed, all 82 Debug-config source files were compiled with
ARM GCC 13.2.1 and the firmware linked with the existing FLASH linker script.
The 60,000-byte CCM section and approximately 29 KB of ordinary SRAM fit their
respective memory regions. Both GDB scripts were also exercised against memory
fixtures and their output read by the analyzer. Hardware sweeps remain pending.
