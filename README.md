# MDP Controller

Firmware for the STM32F407VET6-based controller of a four-wheeled MDP robot.

The robot uses two independently driven rear DC motors with Hall encoders for propulsion, while the front wheels are steered by a servo motor. An ICM-20948 IMU provides inertial measurements for heading feedback.

The firmware is developed using STM32CubeIDE, STM32 HAL, and FreeRTOS.

> **Status:** Active development. Hardware drivers, RTOS application infrastructure, and closed-loop rear-wheel speed control are implemented. Development is currently progressing toward the higher-level robot motion controller.

## Control Architecture

The control software is organized into several layers:

```text
                    Motion Controller
                       (in development)
                              |
                 +------------+------------+
                 |                         |
          Wheel Speed Control        Steering Control
           Left / Right Rear          Front Servo
                 |
          +------+------+
          |             |
      Rear Motor     Rear Motor
          |             |
       Encoder         Encoder

                 IMU (Gyro Z)
                     |
                Heading Feedback
```

The intended motion controller will coordinate:

* rear-wheel velocity;
* travelled distance from encoder measurements;
* front-wheel steering angle;
* yaw/heading feedback from the IMU.

This allows robot-level commands such as straight-line movement and controlled turns to be implemented separately from the low-level motor control loops.

## Current Features

### Peripheral Drivers

Drivers are currently provided for:

* rear DC motors and Hall encoders;
* front-wheel steering servo;
* ICM-20948 IMU;
* OLED display;
* user button;
* onboard LED.

Hardware-specific control is kept in `PeripheralDrivers/` rather than directly in application code.

### Wheel Speed Controller

Each rear wheel has an independent closed-loop speed controller operating in encoder counts per second (CPS).

Motor characterization showed an approximately linear relationship between PWM and wheel speed outside the motor dead zone:

$$
v \approx k(P-P_0)
$$

where:

* \(v\) is wheel speed in counts per second;
* \(P\) is PWM duty cycle;
* \(P_0\) represents the effective motor dead zone;
* \(k\) is the experimentally determined motor slope.

The wheel-speed controller therefore uses:

```text
Target Speed
     |
     +----> Motor Model / Feedforward ----+
     |                                    |
     +----> PI Feedback ------------------+----> PWM ----> Motor
                    ^
                    |
              Encoder Speed
```

The feedforward term estimates the PWM required to achieve the requested speed, while the PI controller compensates for model error and disturbances.

Separate calibration parameters are supported for:

* left and right motors;
* forward and reverse motion;
* start-up PWM;
* minimum running PWM.

This accounts for motor asymmetry and the static friction/dead-zone behavior observed experimentally.

Controller tuning and characterization were performed using measured step responses. The associated experiment scripts, captured data, and derived response metrics are retained in the repository for reference and reproducibility.

### IMU

The ICM-20948 driver provides accelerometer and gyroscope measurements.

Gyroscope bias calibration is performed during initialization, with particular emphasis on the Z-axis gyroscope measurement because it corresponds to robot yaw and will be used by the motion controller for heading feedback.

### FreeRTOS Application Layer

Application-level RTOS functionality is kept under `Apps/`.

The current implementation includes a thread-safe OLED manager and a dedicated OLED task, allowing other tasks to submit display updates without directly sharing the OLED peripheral.

## Project Structure

```text
MDP_Controller/
├── Apps/               # FreeRTOS application tasks and managers
├── Controllers/        # Feedback and motion-control algorithms
│   ├── Inc/
│   └── Src/
├── Core/               # STM32CubeMX-generated application/startup code
├── Drivers/            # STM32 HAL and CMSIS drivers
├── Middlewares/        # FreeRTOS middleware
├── PeripheralDrivers/  # Hardware abstraction and peripheral drivers
│   ├── Inc/
│   └── Src/
├── Tests/              # Hardware and controller test routines
│   ├── Inc/
│   └── Src/
├── exp/                # Experiment/Tuning data for reference
├── zdocs/              # Hardware datasheets and reference documentation
└── MDP_Controller.ioc  # STM32CubeMX configuration
```

Application-specific code is kept separate from CubeMX-generated code where possible:

* `PeripheralDrivers/` contains low-level interfaces to robot hardware.
* `Controllers/` contains reusable control algorithms operating above the hardware drivers.
* `Apps/` contains FreeRTOS tasks and application-level resource management.
* `Tests/` contains standalone routines used during hardware bring-up, calibration, and controller development.
* `zdocs/` contains relevant hardware datasheets and board documentation.

## Development Progress

### Implemented

* [x] Rear-wheel DC motor driver
* [x] Hall encoder acquisition
* [x] Front steering servo driver
* [x] OLED driver and optimized rendering
* [x] Thread-safe OLED manager and FreeRTOS OLED task
* [x] ICM-20948 accelerometer/gyroscope driver
* [x] Gyroscope bias calibration
* [x] Generic PID controller
* [x] Rear-wheel motor characterization
* [x] Feedforward + PI wheel-speed controller
* [x] Wheel-speed controller tuning and response analysis

### In Progress / Planned

* [ ] Robot motion controller
* [ ] Encoder-based distance control
* [ ] Acceleration/deceleration profiles
* [ ] IMU-based straight-line heading correction
* [ ] Closed-loop steering and turning
* [ ] Higher-level robot motion commands

## Building

1. Clone the repository.
2. Import the project into STM32CubeIDE as an existing project.
3. Build using the STM32CubeIDE toolchain.
4. Flash the firmware to the STM32F407VET6 controller using the configured debugger/programmer.

The STM32CubeMX peripheral configuration is stored in `MDP_Controller.ioc`.

## Hardware Documentation

Relevant board, motor-driver, motor/encoder/servo, MCU, and IMU documentation is stored under `zdocs/`.

## Development Notes

This repository is under active development as part of the MDP robot project.

The control architecture is intentionally layered so that peripheral access, low-level feedback control, RTOS application code, and robot-level motion control can be developed and tested independently.

Experimental scripts and measured datasets are retained where useful so that controller parameters and design decisions can be traced back to hardware measurements rather than treated as unexplained constants.
