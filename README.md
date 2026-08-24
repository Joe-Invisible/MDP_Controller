# MDP Controller

Firmware for the STM32F407VET6-based controller of the MDP robot.

The project is developed using **STM32CubeIDE**, **STM32 HAL**, and **FreeRTOS**.

> **Status:** Early development. Peripheral drivers and the FreeRTOS application architecture are currently being implemented.

## Project Structure

```text
MDP_Controller/
├── Apps/               # FreeRTOS application tasks
├── Core/               # CubeMX-generated application and startup code
├── Drivers/            # STM32 HAL and CMSIS drivers
├── Middlewares/        # FreeRTOS middleware
├── PeripheralDrivers/  # Drivers for hardware peripherals
├── Tests/              # Hardware and driver test code
└── MDP_Controller.ioc  # STM32CubeMX configuration
```

Application-specific code is kept separate from CubeMX-generated code where possible:

* **`PeripheralDrivers/`** contains drivers for hardware such as the OLED display and wheel motors.
* **`Apps/`** contains FreeRTOS tasks and application-level task initialization.
* **`Tests/`** contains standalone test routines used during hardware bring-up and driver development.

## Building

1. Clone the repository.
2. Import the project into **STM32CubeIDE** as an existing project.
3. Build the project using the STM32CubeIDE toolchain.
4. Flash the firmware to the target board using the configured debugger/programmer.

## Device/Component Documentation

Copies of hardware used in this project are provided in `/zdocs`.

## Development

This repository is under active development as part of the MDP robot project. The architecture and available drivers will continue to evolve as additional hardware and robot functionality are integrated.
