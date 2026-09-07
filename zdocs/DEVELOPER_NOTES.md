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
* Motion-control and kinematic code should operate in terms of effective steering angle rather than depending directly on servo calibration values.

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
