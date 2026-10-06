# Developer Notes

This document records known issues, implementation quirks, observed hardware behaviour, and design decisions that may be useful during debugging and future development.

These notes are intended primarily for behaviours that are not obvious from the public API or source code alone, especially where an apparently unusual implementation choice exists because of experimental observations.

---

## Wheel Speed Controller

### Transient wheel-speed spikes

**Observed behaviour**

* During some `WheelSpeedController` tests, an individual measured wheel-speed sample may briefly spike to a physically implausible value.

**Cause**

* `WheelSpeedController_Update()` computes wheel speed using the elapsed time `dt` supplied by its caller.
* The encoder count is maintained independently by the underlying `rwdriver`.
* If the supplied `dt` does not correspond to the actual interval over which the encoder count increment was accumulated, the count/time ratio can produce an unrealistically large speed estimate.

**Impact**

* A single anomalous measurement can temporarily affect the controller output.
* Logged plots may therefore appear significantly worse than the actual mechanical response.

**Notes / mitigation**

* Do not immediately interpret an isolated speed spike as real wheel acceleration.
* This behaviour was traced to incorrect timing supplied by the controller consumer rather than a motor or encoder fault.
* Consumers of the controller must ensure that `dt` accurately represents the elapsed time between control updates.
* If timing problems become difficult to manage at the caller level, the controller could eventually maintain its own update timing.

---

### Zero-speed behaviour

When `targetCps == 0`, the controller deliberately handles the motor differently from normal closed-loop speed control.

**Current intended behaviour**

* actively brake while the wheel is still moving;
* transition to coast once the robot has reached a stable stationary state.

**Rationale**

* active braking reaches zero speed significantly faster than simply allowing the wheel to coast;
* continuously braking a stationary motor is unnecessary.

**Implementation responsibility**

* `WheelSpeedController` actively brakes when given a zero-speed target, bypassing the normal speed-control calculation.
* `MotionController` determines when the robot has reached a sufficiently stable stationary state and can be allowed to coast.

This division is intentional: `WheelSpeedController` is responsible for attaining the requested wheel speed, while robot-level stopping behaviour belongs to `MotionController`.

---

## Steering Controller

### Effective steering angle is not the raw servo command

The steering command used by the motion controller represents an **effective wheel steering angle**, not the raw command sent directly to the servo.

The relationship between raw servo command and effective steering angle is experimentally calibrated and is not assumed to be linear or symmetric.

`SteeringController` therefore performs inverse calibration-table mapping when converting a requested effective steering angle into the corresponding raw steering command.

**Implications**

* Do not replace the effective-angle conversion with a simple linear scale without recalibrating the steering mechanism.
* A raw command of zero and an effective steering angle of zero are conceptually different quantities, even when they happen to correspond under the current calibration.
* Legacy effective-angle clients use this conversion. Unified motion control instead commands raw steering and coordinates the rear wheels from its IMU-based curvature request; it must not consume the stale effective-angle estimate during raw control.

The currently calibrated raw steering-command range is:

```text
[-12, +12]
```

---

### Steering command slew-rate limiting

Normal heading-control steering changes are deliberately rate-limited before being applied to the servo.

The current limit is:

```text
maxCommandRatePerSec = 60 raw command units / s
```

For this reason, `SteeringController_SetEffectiveAngleRad()` receives the control-loop elapsed time `dt`.

`dt` is part of the controller behaviour and must represent the actual time since the preceding steering update.

**Rationale**

Without rate limiting, heading feedback can request abrupt raw servo-command changes that are mechanically unrealistic and may excite steering backlash or oscillation.

**Important**

Do not remove the `dt` parameter or bypass the rate limiter simply because the desired steering angle can be computed instantaneously. The commanded steering angle and the physically applied steering movement are intentionally treated separately.

---

### Reversal deadband and steering backlash

The steering linkage exhibits mechanical backlash when the direction of steering motion reverses.

`SteeringController` contains reversal-handling logic to compensate for this behaviour.

Very small requested steering changes around zero can otherwise be incorrectly interpreted as genuine direction reversals. To suppress this, the current implementation uses:

```text
reversalDeadbandRad = 0.0005 rad
```

Changes smaller than this threshold should not trigger reversal behaviour.

**Important**

This value and the associated reversal logic are not arbitrary numerical cleanup. They were introduced to make the calibrated steering behaviour stable around the centre position.

If steering calibration or mechanical linkage is changed, the backlash behaviour and reversal deadband should be re-evaluated experimentally.

---

### Deterministic steering centering at motion startup

The physical steering state at the beginning of a motion command must not be assumed to be known solely from the controller's internal command state.

For repeatable motion experiments, the steering controller therefore establishes the centre position deterministically before normal motion control begins.

**Why this matters**

A small difference in the initial physical steering position can produce a measurable heading error even if the subsequent heading controller behaves correctly.

This is particularly important when comparing repeated straight-line runs, because otherwise differences attributed to controller tuning may actually originate from different initial steering states.

**Testing note**

Do not remove or bypass the deterministic centering behaviour when performing repeatability or controller-tuning tests unless the initial steering state is being deliberately investigated.

---

## Sensors and Hardware

### ICM-20948 / debugger partial-power issue

An intermittent peripheral-initialisation problem has been observed while the debugger is connected.

**Observed behaviour**

* The first ICM-20948 I²C register write unexpectedly returned `HAL_BUSY`.
* Firmware that had previously initialized the IMU correctly could therefore fail immediately during startup.
* Disconnecting the debugger from the robot completely and leaving it disconnected for approximately 10 seconds cleared the problem.

With the battery switched off but the debugger still attached, approximately:

```text
V_IN ≈ 2.8 V
```

was measured on the robot.

**Possible explanation**

The debugger may partially power parts of the system through its connection, leaving peripherals or buses in an unexpected electrical state.

This has **not been proven to be the root cause**, so it should be treated as a troubleshooting observation rather than a confirmed hardware diagnosis.

**Troubleshooting procedure**

If an otherwise unexplained `HAL_BUSY` or similar peripheral-initialisation failure occurs while debugging:

1. switch off the robot battery;
2. completely disconnect the debugger from the robot;
3. leave the system unpowered for several seconds;
4. reconnect and retry initialization.

Avoid immediately modifying otherwise working peripheral-driver code until this possibility has been excluded.

---

### HC-SR04 ultrasonic sensor

The HC-SR04 ultrasonic sensor is connected using:

```text
TRIG : PB14
ECHO : PB15 / TIM12_CH2 input capture
VCC  : J connector 5V5 rail
```

The 5V5 rail used during testing measured approximately:

```text
5.1 V
```

The implemented driver has been functionally verified on hardware.

During initial testing, measured distances were reasonably accurate for targets within approximately:

```text
200 mm
```

**Important**

This does **not** establish a ±200 mm operating limit for the HC-SR04. It only describes the range over which this particular implementation has so far been experimentally checked.

Accuracy, repeatability, minimum measurable distance, outlier behaviour, and performance at longer ranges have not yet been formally characterised.

Any motion-control logic that eventually uses ultrasonic measurements should therefore avoid assuming a known measurement uncertainty until a dedicated sensor-characterisation experiment has been performed.

---

## Experimental Testing and Data

### Record actual acquisition conditions

Controller behaviour has been shown to depend not only on firmware parameters but also on mechanical condition, initial steering state, battery condition, and other hardware factors.

Experimental datasets should therefore be accompanied, where available, by:

* actual date and time of acquisition;
* firmware revision or Git commit;
* relevant controller parameters;
* commanded motion and target speed;
* battery voltage;
* important mechanical configuration or recent mechanical changes;
* any unusual debugger or power configuration.

The **actual acquisition time** should be distinguished from:

* file creation time;
* debugger export time;
* copy time;
* Git commit time;
* the time at which the dataset was later analysed or shared.

Do not infer an exact experiment time from a file timestamp unless the relationship between the timestamp and acquisition has been established.

---

### Mechanical condition can invalidate run-to-run comparisons

Mechanical changes must be treated as experimental interventions rather than incidental details.

A significant example occurred on **1 September 2026 at approximately 21:08 SGT**, when the attachment between the front-left wheel and its servo axis was found to be loose. The wheel could wobble noticeably until the attachment screw was tightened.

Earlier straight-motion datasets had shown a fairly consistent positive yaw error, while subsequent datasets tended toward negative yaw error.

This does not by itself prove that the loose wheel caused the earlier yaw behaviour, but it means data taken before and after the repair should not automatically be treated as samples from an unchanged system.

**General rule**

When controller behaviour changes unexpectedly, check for mechanical, electrical, battery, sensor, and test-procedure changes before attributing the difference solely to control parameters.

---

## Maintaining These Notes

Add an entry here when:

* behaviour is surprising or easy to misinterpret during debugging;
* an unusual implementation choice exists for an experimentally established reason;
* removing a workaround could reintroduce a known problem;
* hardware behaviour has been observed but its underlying cause is not yet fully understood;
* an experimental procedure is necessary for obtaining reproducible results.

Routine implementation details that are already obvious from the module interface or source code generally do not need to be duplicated here.

Proposed architecture that has not yet settled should normally remain in design discussions or planning notes rather than being documented here as established behaviour.

---

## Unified calibration-free motion (30 September 2026)

Straight motion is now the zero-curvature case of the same path controller used
for arcs. The public `MoveStraight` and `MoveArc` APIs, signed-distance/radius
conventions, motion profiles, braking, and mechanical preparation states remain.

For the internally signed rear-axle-centre reference speed v—whose direction is 
derived solely from the signed distance—and signed displacement `s`, and requested 
curvature
`kappa_ref` (zero for straight motion):

* desired yaw is `kappa_ref * s`;
* nominal yaw rate is `kappa_ref * v`;
* heading feedback adds `arcHeadingKpPerSec * headingError`, limited to
  `abs(v) * maxPathCorrectionCurvaturePerMm`;
* the yaw-rate PID adds a direction-corrected raw steering correction around the
  prepared feedforward command;
* both rear-wheel base speeds and accumulated wheel-travel difference use the
  corrected requested curvature, `targetYawRate / v`. At zero speed the nominal
  curvature is retained without division.

The shared correction bound replaces `2 * abs(nominalYawRate)`, which would
otherwise disable heading recovery when nominal curvature is zero. The former
1 mm/s curvature fallback is removed: the speed-scaled correction remains
bounded at every nonzero speed, including slow profile starts and finishes.

Initial shared tuning is yaw-rate `Kp=10`, `Ki=Kd=0`, raw correction limit ±5,
outer heading gain 1/s, filter time constant 0.10 s, and correction-curvature
limit `0.001/mm`. The curvature limit is a starting value requiring hardware
validation, not a measured calibration result. It also changes the outer-loop
saturation envelope for existing arcs; recheck arc behavior with these limits.

Straight feedforward captures the actual raw command immediately after
`SteeringController_Centre`. It does not assume raw zero is physically straight
and does not interpolate across the unsupported gap between arc-table branches.
The validated immediate centering/prepositioning and 0.5 s mechanical holds are
preserved. Active feedback remains rate limited. The motion origin is still
established at the preparation boundary.

Set `MotionControllerConfig.useLegacyStraightSteering=true` to compare the
previous straight heading PID and effective-angle wheel coordination. The
legacy `headingKp/Ki/Kd` and `maxHeadingSteeringAngleRad` apply only to that
baseline. The retained `arcYawRate*`, `arcHeadingKpPerSec`, and
`maxArcSteeringCommandCorrection` now tune both unified straight motion and arcs.
Their historical names and the `arc*` diagnostics are intentionally retained
until hardware comparison supports removing the legacy implementation.

`MotionControllerTest` has a `MOTION_TEST_USE_LEGACY_STEERING` switch and records
raw feedforward/target/actual commands, yaw-rate feedback, commanded curvature,
and effective-angle model validity. Sequence logs retain the existing arc
fields for straight commands too and now record raw feedforward and model
validity. Ignore effective-angle diagnostics whenever the model-valid flag is
false. Every custom motion configuration must set the new curvature bound;
zero explicitly disables outer heading correction while retaining rate feedback.

Run the host control checks with:

```sh
python3 Tests/Host/test_unified_motion.py
```

They compile the production motion, profile, PID, kinematics, steering and
configuration sources with warnings treated as errors and undefined-behavior
checks. Only peripheral access and the unchanged wheel-speed loops are mocked.
The checks cover both straight travel directions and yaw-error signs, both arc
curvature signs and travel directions, preparation timing/origin reset, wheel
geometry and odometry, low-speed bounds, gyro filtering, legacy selection,
configuration validation, braking and IMU failure. They establish control-law
and state-machine behavior, not physical trajectory accuracy.

Hardware validation should begin with isolated forward and reverse straights
at 2000 CPS, followed by the existing mixed sequence and square test. Compare
endpoint distance/yaw and the new rate/raw-curvature traces against the legacy
baseline under the same speed, preparation and battery conditions. Recheck arcs
before changing gains or deleting the baseline. Re-establish the questionable
R=-275 mm feedforward point separately; this implementation leaves the table
unchanged.

## Sequence-test export automation (1 October 2026)

The shared Debug launch now loads a GDB command file that exports sequence
results and samples automatically at `MotionControllerSequenceTest_ShowFinal`,
after motion/braking and final bookkeeping. Filenames are unique and include
the host export timestamp; this timestamp must not be mistaken for acquisition
start time. The target stays suspended, and `EXPORTED:` confirms a complete
dump. No firmware or controller tuning changes are involved.

Windows setup and the optional build/flash/run command for local Codex are in
[`exp/gdb_scripts/README.md`](../exp/gdb_scripts/README.md). The standalone
runner archives the exact flashed ELF and its hash, and retains the SW1 start
for each physical experiment. Export commands were checked with host GDB and
PowerShell, and runner orchestration with mock build/debug tools. CubeIDE,
Windows PowerShell 5.1, and ST-LINK hardware integration still require local
verification.


## Experimental fused motion sequence (6 Oct 2026)

`experiment/fused-motion-sequence` inherits unified-control commit `69a661e`.
`MotionSequence` generates bounded same-direction profiles above
`MotionController_FollowProfile`, retaining controller state through virtual
junctions. Symmetric curvature ramps consume neighbouring distances and
preserve nominal heading area. Reversals and explicit stops split continuous
runs; profile endpoints use distance completion and existing braking.

The profile-only near-centre raw feedforward bridge is an experimental model,
and straight/arc gains are scheduled through blends while preserving integral
output. Standalone commands retain their existing behaviour. Defaults are a
200 mm maximum blend and 500 CPS changing-curvature junction ceiling, further
limited by the configured raw slew rate. Equal curvature has no extra slowdown.

The selected `MotionSequenceFusionTest` logs a straight/arc/straight experiment;
`FUSION_STOP_EACH_SEGMENT` enables the stopped profile comparison. Host tests
cover geometry, state continuity, speed lookahead, faults and the harness.
Full target build and hardware validation remain local. Details and GDB export:
[Tests/README_fused_motion.md](../Tests/README_fused_motion.md).
