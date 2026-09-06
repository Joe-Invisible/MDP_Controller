/*
 * hcsr04.c
 *
 *  Created on: 2026年9月6日
 *      Author: Joe
 */

#include "hcsr04.h"


static bool HCSR04_IsActiveChannel(const HCSR04_HandleTypeDef *sensor,
                                   TIM_HandleTypeDef *htim);

static uint32_t HCSR04_GetCaptureFlag(uint32_t channel);

static uint32_t HCSR04_GetElapsedCounts(TIM_HandleTypeDef *htim,
                                        uint32_t start,
                                        uint32_t end);

static void HCSR04_DelayUs(HCSR04_HandleTypeDef *sensor,
                           uint32_t delayUs);


/**
 * @brief Initialise an HC-SR04 instance.
 */
void HCSR04_Init(HCSR04_HandleTypeDef *sensor,
                 GPIO_TypeDef *trigPort,
                 uint16_t trigPin,
                 TIM_HandleTypeDef *htim,
                 uint32_t channel)
{
    sensor->trigPort = trigPort;
    sensor->trigPin = trigPin;

    sensor->htim = htim;
    sensor->channel = channel;

    sensor->state = HCSR04_STATE_IDLE;

    sensor->risingCapture = 0U;
    sensor->pulseWidthUs = 0U;
    sensor->triggerTickMs = 0U;

    /* Ensure TRIG is inactive. */
    HAL_GPIO_WritePin(sensor->trigPort,
                      sensor->trigPin,
                      GPIO_PIN_RESET);

    /* Every measurement begins by looking for a rising edge. */
    __HAL_TIM_SET_CAPTUREPOLARITY(sensor->htim,
                                  sensor->channel,
                                  TIM_INPUTCHANNELPOLARITY_RISING);
}


/**
 * @brief Start a new ultrasonic measurement.
 */
bool HCSR04_Trigger(HCSR04_HandleTypeDef *sensor)
{
    if (HCSR04_IsBusy(sensor))
    {
        return false;
    }

    sensor->risingCapture = 0U;
    sensor->pulseWidthUs = 0U;

    __HAL_TIM_SET_CAPTUREPOLARITY(sensor->htim,
                                  sensor->channel,
                                  TIM_INPUTCHANNELPOLARITY_RISING);

    /*
     * Remove any pending capture flag left from a previous measurement.
     */
    uint32_t captureFlag = HCSR04_GetCaptureFlag(sensor->channel);

    if (captureFlag != 0U)
    {
        __HAL_TIM_CLEAR_FLAG(sensor->htim, captureFlag);
    }

    sensor->state = HCSR04_STATE_WAITING_RISING;
    sensor->triggerTickMs = HAL_GetTick();

    /*
     * Starting input capture also starts the timer counter.
     */
    if (HAL_TIM_IC_Start_IT(sensor->htim, sensor->channel) != HAL_OK)
    {
        sensor->state = HCSR04_STATE_IDLE;
        return false;
    }

    /*
     * HC-SR04 trigger pulse.
     */
    HAL_GPIO_WritePin(sensor->trigPort,
                      sensor->trigPin,
                      GPIO_PIN_SET);

    HCSR04_DelayUs(sensor, HCSR04_TRIGGER_PULSE_US);

    HAL_GPIO_WritePin(sensor->trigPort,
                      sensor->trigPin,
                      GPIO_PIN_RESET);

    return true;
}


/**
 * @brief Check for a measurement timeout.
 */
void HCSR04_Update(HCSR04_HandleTypeDef *sensor)
{
    if (!HCSR04_IsBusy(sensor))
    {
        return;
    }

    uint32_t elapsedMs = HAL_GetTick() - sensor->triggerTickMs;

    if (elapsedMs >= HCSR04_TIMEOUT_MS)
    {
        HAL_TIM_IC_Stop_IT(sensor->htim, sensor->channel);

        __HAL_TIM_SET_CAPTUREPOLARITY(sensor->htim,
                                      sensor->channel,
                                      TIM_INPUTCHANNELPOLARITY_RISING);

        sensor->state = HCSR04_STATE_TIMEOUT;
    }
}


/**
 * @brief Handle the timer input-capture interrupt.
 */
void HCSR04_HandleInputCapture(HCSR04_HandleTypeDef *sensor,
                               TIM_HandleTypeDef *htim)
{
    if (htim != sensor->htim)
    {
        return;
    }

    if (!HCSR04_IsActiveChannel(sensor, htim))
    {
        return;
    }

    uint32_t capture =
        HAL_TIM_ReadCapturedValue(sensor->htim, sensor->channel);

    switch (sensor->state)
    {
        case HCSR04_STATE_WAITING_RISING:
            /*
             * Beginning of ECHO pulse.
             */
            sensor->risingCapture = capture;

            __HAL_TIM_SET_CAPTUREPOLARITY(
                sensor->htim,
                sensor->channel,
                TIM_INPUTCHANNELPOLARITY_FALLING);

            sensor->state = HCSR04_STATE_WAITING_FALLING;
            break;

        case HCSR04_STATE_WAITING_FALLING:
            /*
             * End of ECHO pulse.
             */
            sensor->pulseWidthUs =
                HCSR04_GetElapsedCounts(sensor->htim,
                                        sensor->risingCapture,
                                        capture);

            /*
             * Prepare for the next measurement.
             */
            __HAL_TIM_SET_CAPTUREPOLARITY(
                sensor->htim,
                sensor->channel,
                TIM_INPUTCHANNELPOLARITY_RISING);

            HAL_TIM_IC_Stop_IT(sensor->htim, sensor->channel);

            sensor->state = HCSR04_STATE_READY;
            break;

        default:
            /*
             * Ignore unexpected captures.
             */
            break;
    }
}


/**
 * @brief Return whether a measurement is currently in progress.
 */
bool HCSR04_IsBusy(const HCSR04_HandleTypeDef *sensor)
{
    return (sensor->state == HCSR04_STATE_WAITING_RISING) ||
           (sensor->state == HCSR04_STATE_WAITING_FALLING);
}


/**
 * @brief Return whether a valid completed measurement is available.
 */
bool HCSR04_HasMeasurement(const HCSR04_HandleTypeDef *sensor)
{
    return sensor->state == HCSR04_STATE_READY;
}


/**
 * @brief Return measured pulse width in microseconds.
 */
uint32_t HCSR04_GetPulseWidthUs(const HCSR04_HandleTypeDef *sensor)
{
    return sensor->pulseWidthUs;
}


/**
 * @brief Return measured distance in millimetres.
 */
float HCSR04_GetDistanceMm(const HCSR04_HandleTypeDef *sensor)
{
    return (float)sensor->pulseWidthUs * HCSR04_MM_PER_US;
}


/**
 * @brief Return the current sensor state.
 */
HCSR04_State HCSR04_GetState(const HCSR04_HandleTypeDef *sensor)
{
    return sensor->state;
}


/**
 * @brief Check whether the HAL callback belongs to this sensor's channel.
 */
static bool HCSR04_IsActiveChannel(const HCSR04_HandleTypeDef *sensor,
                                   TIM_HandleTypeDef *htim)
{
    switch (sensor->channel)
    {
        case TIM_CHANNEL_1:
            return htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1;

        case TIM_CHANNEL_2:
            return htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2;

        case TIM_CHANNEL_3:
            return htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3;

        case TIM_CHANNEL_4:
            return htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4;

        default:
            return false;
    }
}


/**
 * @brief Convert a TIM_CHANNEL_x value to the corresponding capture flag.
 */
static uint32_t HCSR04_GetCaptureFlag(uint32_t channel)
{
    switch (channel)
    {
        case TIM_CHANNEL_1:
            return TIM_FLAG_CC1;

        case TIM_CHANNEL_2:
            return TIM_FLAG_CC2;

        case TIM_CHANNEL_3:
            return TIM_FLAG_CC3;

        case TIM_CHANNEL_4:
            return TIM_FLAG_CC4;

        default:
            return 0U;
    }
}


/**
 * @brief Find elapsed timer counts while accounting for one counter rollover.
 *
 * For TIM12:
 *
 *      ARR = 65535
 *      counter period = 65536 us
 *
 * An HC-SR04 ECHO pulse is much shorter than this, so at most one rollover
 * can occur between the rising and falling edge.
 */
static uint32_t HCSR04_GetElapsedCounts(TIM_HandleTypeDef *htim,
                                        uint32_t start,
                                        uint32_t end)
{
    if (end >= start)
    {
        return end - start;
    }

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;

    return (period - start) + end;
}


/**
 * @brief Busy-wait using the 1 MHz input-capture timer.
 */
static void HCSR04_DelayUs(HCSR04_HandleTypeDef *sensor,
                           uint32_t delayUs)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(sensor->htim);

    while (true)
    {
        uint32_t now = __HAL_TIM_GET_COUNTER(sensor->htim);

        if (HCSR04_GetElapsedCounts(sensor->htim, start, now) >= delayUs)
        {
            break;
        }
    }
}
