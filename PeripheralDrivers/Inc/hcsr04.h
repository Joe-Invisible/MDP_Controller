/*
 * hcsr04.h
 *
 *  Created on: 2026年9月6日
 *      Author: Joe
 */

#ifndef INC_HCSR04_H_
#define INC_HCSR04_H_

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define HCSR04_TRIGGER_PULSE_US    10U
#define HCSR04_TIMEOUT_MS          50U

/*
 * Speed of sound at approximately 20 deg C:
 *
 *      343 m/s = 0.343 mm/us
 *
 * The ultrasonic pulse travels to the object and back, hence / 2.
 */
#define HCSR04_MM_PER_US           (0.343f / 2.0f)

typedef enum
{
    HCSR04_STATE_IDLE = 0,
    HCSR04_STATE_WAITING_RISING,
    HCSR04_STATE_WAITING_FALLING,
    HCSR04_STATE_READY,
    HCSR04_STATE_TIMEOUT
} HCSR04_State;

typedef struct
{
    GPIO_TypeDef *trigPort;
    uint16_t trigPin;

    TIM_HandleTypeDef *htim;
    uint32_t channel;

    volatile HCSR04_State state;

    volatile uint32_t risingCapture;
    volatile uint32_t pulseWidthUs;

    uint32_t triggerTickMs;
} HCSR04_HandleTypeDef;


/**
 * @brief Initialise an HC-SR04 instance.
 *
 * The timer must already be configured such that one timer count = 1 us.
 */
void HCSR04_Init(HCSR04_HandleTypeDef *sensor,
                 GPIO_TypeDef *trigPort,
                 uint16_t trigPin,
                 TIM_HandleTypeDef *htim,
                 uint32_t channel);

/**
 * @brief Start a new ultrasonic measurement.
 *
 * @return true if measurement was started, false if the sensor is already busy
 *         or input capture could not be started.
 */
bool HCSR04_Trigger(HCSR04_HandleTypeDef *sensor);

/**
 * @brief Perform timeout handling.
 *
 * Call periodically from task/thread context.
 */
void HCSR04_Update(HCSR04_HandleTypeDef *sensor);

/**
 * @brief Handle a HAL timer input-capture callback.
 *
 * Call this from HAL_TIM_IC_CaptureCallback().
 */
void HCSR04_HandleInputCapture(HCSR04_HandleTypeDef *sensor,
                               TIM_HandleTypeDef *htim);

/**
 * @brief Return whether the sensor is currently waiting for an echo.
 */
bool HCSR04_IsBusy(const HCSR04_HandleTypeDef *sensor);

/**
 * @brief Return whether a completed measurement is available.
 */
bool HCSR04_HasMeasurement(const HCSR04_HandleTypeDef *sensor);

/**
 * @brief Return the measured ECHO pulse width in microseconds.
 */
uint32_t HCSR04_GetPulseWidthUs(const HCSR04_HandleTypeDef *sensor);

/**
 * @brief Return the measured distance in millimetres.
 */
float HCSR04_GetDistanceMm(const HCSR04_HandleTypeDef *sensor);

/**
 * @brief Return the current driver state.
 */
HCSR04_State HCSR04_GetState(const HCSR04_HandleTypeDef *sensor);

#endif /* INC_HCSR04_H_ */
