/*
 * gp2y0a21yk.h
 *
 * Blocking ADC driver for the Sharp GP2Y0A21YK analogue distance sensor.
 *
 *  Created on: 2026年9月20日
 *      Author: Joe
 */


#ifndef INC_GP2Y0A21YK_H_
#define INC_GP2Y0A21YK_H_

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define GP2Y0A21YK_ADC_FULL_SCALE_COUNT    4095U

typedef enum
{
    GP2Y0A21YK_OK = 0,
    GP2Y0A21YK_ERROR_NOT_INITIALIZED,
    GP2Y0A21YK_ERROR_INVALID_ARGUMENT,
    GP2Y0A21YK_ERROR_ADC_CONFIG,
    GP2Y0A21YK_ERROR_ADC_START,
    GP2Y0A21YK_ERROR_ADC_TIMEOUT,
    GP2Y0A21YK_ERROR_ADC_CONVERSION,
    GP2Y0A21YK_ERROR_ADC_STOP
} GP2Y0A21YK_Status;

typedef struct
{
    ADC_HandleTypeDef *hadc;
    uint32_t channel;
    uint32_t samplingTime;
    float adcReferenceVoltageV;
    uint32_t conversionTimeoutMs;
} GP2Y0A21YK_Config;

typedef struct
{
    GP2Y0A21YK_Config config;
    bool initialized;
} GP2Y0A21YK;

typedef struct
{
    uint16_t rawAdc;
    float voltageV;
} GP2Y0A21YK_Measurement;

/**
 * @brief Initialize one sensor instance from an application-owned config.
 */
GP2Y0A21YK_Status GP2Y0A21YK_Init(
    GP2Y0A21YK *sensor,
    const GP2Y0A21YK_Config *config);

/**
 * @brief Perform one blocking ADC conversion.
 *
 * Instances that share an ADC peripheral must be read serially. This function
 * reconfigures regular rank 1, starts one conversion, and stops the ADC before
 * returning.
 */
GP2Y0A21YK_Status GP2Y0A21YK_Read(
    GP2Y0A21YK *sensor,
    GP2Y0A21YK_Measurement *measurement);

/**
 * @brief Convert a 12-bit ADC result to volts using the configured reference.
 */
float GP2Y0A21YK_RawToVoltage(
    const GP2Y0A21YK *sensor,
    uint16_t rawAdc);

#endif /* INC_GP2Y0A21YK_H_ */
