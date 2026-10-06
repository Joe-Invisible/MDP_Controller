# MDP Controller

Firmware for the STM32F407VET6-based controller of a four-wheeled MDP robot.

The robot uses two independently driven rear DC motors with Hall encoders for propulsion, a servo-actuated front steering axle, and an ICM-20948 IMU for yaw-rate and heading feedback. The firmware is developed with STM32CubeIDE, STM32 HAL, and FreeRTOS.

> **Status:** Active development. The low-level hardware drivers, rear-wheel speed control, steering control, motion profiling, encoder odometry, dynamic braking, and unified straight/arc motion controller are implemented and have been exercised on hardware. This branch is focused on motion-controller tuning and regression testing before reintegration with the production UART/RPi application layer.

## Experimental Motion Fusion

This branch adds `MotionSequence` above the unified controller and
`MotionController_FollowProfile()` for continuous straight/arc batches. Internal
curvature ramps preserve nominal total distance and heading area; reversals and
explicit stop waypoints remain stopped boundaries. A batch-ending arc retains
yaw-priority completion for camera heading. Near-zero feedforward
interpolation and physical transition tracking are experimental.

`TestMain.c` selects the new fusion harness. See
[the fusion experiment guide](Tests/README_fused_motion.md) for API usage,
software-only tests, endpoint policy, logging and the stopped comparison.
Hardware validation and a full CubeIDE target build are pending.

## Robot Geometry

The current controller configuration uses:

| Parameter | Value |
| --- | ---: |
| Wheelbase | 145 mm |
| Rear wheel centre-to-centre track | 165 mm |
| Effective rear wheel diameter | 65.5 mm |
| Rear encoder resolution | 1560 counts/rev |

The authoritative values are defined in `Controllers/Src/MotionControllerConfig.c`.

## Control Architecture

The control stack is separated into hardware, actuator control, robot motion control, and application layers:

```text
                Application / RPi command layer
                           |
                    MotionController
                           |
            +--------------+--------------+
            |                             |
     Path / yaw control             Motion profile
            |                             |
            +--------------+--------------+
                           |
             Rear-wheel reference speeds
                           |
             +-------------+-------------+
             |                           |
      WheelSpeedController        SteeringController
         left / right                front servo
             |                           |
       DC motors + encoders             Servo

                    ICM-20948 gyro Z
                           |
                 yaw-rate / heading feedback
```

`MotionController` exposes robot-level primitives such as straight motion, constant-curvature arcs, braking, and stopping. It owns encoder odometry, motion profiling, steering/path feedback, and the transition between active motion and braking.

## Motion Controller

### Unified Straight and Arc Control

Straight and curved motion use the same path-control structure:

1. A distance-based motion profile produces the centre-speed reference.
2. Rear encoder measurements provide travelled distance and measured centre speed.
3. Desired yaw is derived from commanded curvature and travelled distance.
4. Feedforward yaw rate is computed from commanded curvature and filtered measured centre speed.
5. A heading outer loop may add a bounded yaw-rate correction.
6. A yaw-rate PI controller converts yaw-rate error into a raw steering-command correction.
7. Rear-wheel target speeds are generated from the commanded curvature and corrected by the wheel synchronisation loop.

The measured centre speed and measured yaw rate use matching first-order low-pass filtering so feedforward and feedback have comparable bandwidth.

The software control law is shared between straight and arc motion, but the steering plant behaves differently around the centre operating point and at finite curvature. The controller therefore selects separate gain sets when a command begins.

Current production-candidate tuning:

| Regime | Steering feedforward | Yaw-rate Kp | Yaw-rate Ki | Heading Kp [/s] |
| --- | ---: | ---: | ---: | ---: |
| Straight | raw 0.0 | 370 | 450 | 2.0 |
| Arc | curvature table | 270 | 150 | 0.0 |

The straight raw-zero feedforward is an empirical bias-cancellation operating point. It should not be interpreted as the legacy steering model's geometric zero-angle calibration.

The controller currently allows up to +/-30 raw steering-command units of feedback correction around the selected feedforward value.

### Arc Feedforward Calibration

Arc motion does not depend on the legacy steering-angle model for normal path control. Instead, it uses an empirical mapping from path curvature to absolute raw steering command.

The positive- and negative-curvature branches are stored separately and interpolated independently. This intentionally avoids assuming left/right symmetry or interpolating across the uncalibrated region around zero curvature.

Representative calibrated points include radii from approximately 2500 mm down to 275 mm. The calibration table is defined in `MotionControllerConfig.c` and should be treated as hardware-specific.

### Rear-Wheel Synchronisation

Rear-wheel synchronisation is defined relative to the wheel-travel difference required by the commanded path rather than blindly forcing both wheels to travel equal distances.

For straight motion the desired difference is zero. For an arc, the desired difference is derived from the rear track width and path curvature. A proportional synchronisation correction is then applied symmetrically to the two wheel-speed references.

### Motion Profile

The motion profile provides acceleration, cruise, and deceleration references while respecting the requested travel distance.

Current production-candidate values are:

```text
acceleration = 1500 mm/s^2
deceleration = 1000 mm/s^2
completion tolerance = 0.5 mm
```

These values were updated during the terminal-arc validation campaign and are configurable through `MotionControllerConfig`.

### Yaw-Priority Arc Completion

For arc commands, final orientation is more important than ending at exactly the nominal path distance. Near the end of an arc, the controller therefore switches to a yaw-sensitive terminal approach.

The current strategy is:

```text
fast bulk motion
      -> guaranteed low-speed terminal approach
      -> predict braking yaw from measured gyro rate
      -> begin braking at the predicted yaw threshold
      -> allow a small bounded overrun if nominal distance is reached first
```

The present terminal approach uses an 800 CPS approach reference and a 75 ms empirical braking-yaw prediction horizon. The desired heading reference is frozen at the nominal endpoint during any terminal distance overrun.

This mechanism exists only for finite-curvature arc motion; straight motion continues to use distance completion.

### Dynamic Braking

The rear-wheel controllers support speed-dependent active dynamic braking. Active braking is entered when measured wheel speed exceeds the requested speed by a configured threshold and uses hysteresis before returning to normal propulsion control.

When `MotionController` enters its braking state, the steering is returned to centre while the rear-wheel controllers drive the measured speeds toward zero. Multiple consecutive stationary samples are required before the motion is declared complete.

## Wheel Speed Controller

Each rear wheel has an independent controller operating in encoder counts per second (CPS).

Motor characterization showed an approximately linear relationship between PWM and wheel speed outside the effective dead zone:

```text
v ~= k(P - P0)
```

The wheel-speed controller combines a calibrated motor feedforward model with PI feedback:

```text
Target Speed
     |
     +----> Motor Model / Feedforward ----+
     |                                    |
     +----> PI Feedback ------------------+----> PWM / Brake ----> Motor
                    ^
                    |
              Encoder Speed
```

Separate calibration parameters are supported for left/right motors and forward/reverse operation.

## Steering Controller

`SteeringController` owns the servo interface and the historical steering calibration model. It also provides raw-command control used by the calibration-free motion controller.

During active unified motion, steering commands are rate-limited in software. Motion preparation may pre-position the steering feedforward command before translation begins so the servo has time to settle mechanically.

The legacy effective-angle path remains in the code as a temporary A/B baseline for straight motion, but the production-candidate unified controller does not require it for normal straight or arc path feedback.

## IMU

The ICM-20948 driver provides accelerometer and gyroscope measurements. The Z-axis gyroscope is used for robot yaw-rate feedback and integrated relative heading during each motion primitive.

Gyroscope bias calibration is performed during initialization. Correct stationary initialization is important because yaw-rate bias directly affects heading and path control.

## FreeRTOS and RPi Integration

Application-level RTOS code is kept under `Apps/`.

This tuning branch primarily exercises the motion controller through standalone hardware test harnesses. The fuller UART command/session integration is developed separately on the `merge/uart-command-calibration-free` integration branch, where an RPi can submit motion-command batches and receive completion/fault acknowledgements.

The intended responsibility split is:

```text
RPi / top-layer application:
    path planning, sequencing, replanning, task logic

STM32:
    deterministic motion execution, wheel control,
    steering control, braking, odometry and IMU feedback
```

Endpoint-result reporting for higher-level replanning is a natural next extension of this interface.

## Test and Calibration Infrastructure

Standalone hardware and controller tests live under `Tests/`. `Tests/Src/TestMain.c` selects the currently executed test when `BRANCH_TO_TESTS` is enabled in `Core/Src/main.c`.

Motion-controller test harnesses include:

* `MotionControllerTest` - isolated straight-motion regression and diagnostics;
* `MotionControllerStraightFeedforwardTest` - zero-curvature feedforward and straight feedback experiments;
* `MotionControllerArcTest` - isolated constant-curvature arc experiments;
* `MotionControllerSequenceTest` - multi-primitive sequence regression;
* `MotionProfileTest` - profile validation;
* lower-level wheel-speed, steering, dynamic-brake, IMU, sensor, and motor tests.

The motion tests expose debugger-visible logs so measured CPS, steering commands, yaw rate, yaw, wheel travel, synchronisation error, braking state, and controller internals can be exported for analysis.

Experimental scripts and captured datasets are retained under `exp/` where useful so tuning decisions remain traceable to hardware measurements.

## Project Structure

```text
MDP_Controller/
|- Apps/               # FreeRTOS application tasks and managers
|- Controllers/        # Motion, speed, steering and control algorithms
|  |- Inc/
|  `- Src/
|- Core/               # STM32CubeMX-generated application/startup code
|- Drivers/            # STM32 HAL and CMSIS drivers
|- Middlewares/        # FreeRTOS middleware
|- PeripheralDrivers/  # Hardware abstraction and peripheral drivers
|  |- Inc/
|  `- Src/
|- Tests/              # Hardware/controller test harnesses
|  |- Inc/
|  `- Src/
|- exp/                # Experiment and tuning data
|- zdocs/              # Datasheets and hardware references
`- MDP_Controller.ioc  # STM32CubeMX configuration
```

Application-specific code is kept separate from CubeMX-generated code where possible:

* `PeripheralDrivers/` contains low-level hardware interfaces.
* `Controllers/` contains reusable feedback, motion, and kinematic algorithms.
* `Apps/` contains FreeRTOS tasks and application-level resource management.
* `Tests/` contains standalone bring-up, calibration, and regression routines.
* `zdocs/` contains board, actuator, sensor, MCU, and IMU references.

## Development Progress

### Implemented and Hardware-Tested

* [x] Rear DC motor and Hall encoder drivers
* [x] Front steering-servo driver
* [x] ICM-20948 IMU driver and gyroscope bias calibration
* [x] OLED and user-interface support
* [x] Generic PID controller
* [x] Rear motor characterization and feedforward + PI speed control
* [x] Speed-dependent dynamic braking
* [x] Encoder-based distance odometry
* [x] Acceleration/deceleration motion profiles
* [x] Rear-wheel path synchronisation
* [x] Unified straight/arc yaw-rate path controller
* [x] Empirical raw-steering curvature feedforward table
* [x] Mode-dependent straight/arc feedback tuning
* [x] Straight feedforward bias compensation
* [x] Yaw-priority terminal arc completion
* [x] Multi-command sequence regression harness

### Integration / Cleanup Remaining

* [ ] Reintegrate the tuned controller configuration with the UART/RPi production application
* [ ] Expose useful settled endpoint information to the top-layer application
* [ ] Resolve the temporary sharp-arc limited-headroom regression exemption
* [ ] Remove or retire the legacy straight A/B control path once production regression is complete
* [ ] Clean up historical `arc*` diagnostic names that are now shared by straight and arc control
* [ ] Keep documentation and regression datasets synchronized with production changes

## Building and Running Tests

1. Clone the repository.
2. Import the project into STM32CubeIDE as an existing project.
3. Build using the STM32CubeIDE toolchain.
4. Flash the STM32F407VET6 using the configured debugger/programmer.
5. For standalone controller tests, select the desired harness in `Tests/Src/TestMain.c` and ensure the test branch in `Core/Src/main.c` is enabled.

The STM32CubeMX peripheral configuration is stored in `MDP_Controller.ioc`.

## Known Development Note

The current controller still defines `MOTIONCONTROLLER_TEST_ALLOW_LIMITED_ARC_HEADROOM` for the sharp +/-275 mm hardware-regression cases. At these extreme feedforward commands there is not a full symmetric +/-30 raw units of physical steering headroom, although the lower-level steering controller still clamps the actual command to the servo range. This exemption should be resolved deliberately during production cleanup rather than left as an undocumented test artifact.

## Hardware Documentation

Relevant board, motor-driver, motor/encoder/servo, MCU, sensor, and IMU documentation is stored under `zdocs/`.

## Development Notes

The control architecture is intentionally layered so that peripheral access, low-level feedback control, robot-level motion control, and the RPi-facing application can be developed and tested independently.

The project favors measured hardware behavior over purely geometric assumptions: wheel-speed models, curvature feedforward points, controller gains, braking behavior, and terminal-motion parameters are derived and validated experimentally, with test logs retained where useful for regression and reproducibility.
