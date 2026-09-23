/*
 * gp2y0a21yk.c
 *
 *  Created on: 2026年9月20日
 *      Author: Joe
 */

#include "gp2y0a21yk.h"

#include <stddef.h>

GP2Y0A21YK_Status GP2Y0A21YK_Init(
    GP2Y0A21YK *sensor,
    const GP2Y0A21YK_Config *config)
{
    if ((sensor == NULL) ||
        (config == NULL) ||
        (config->hadc == NULL) ||
        (config->adcReferenceVoltageV <= 0.0f) ||
        (config->conversionTimeoutMs == 0U))
    {
        return GP2Y0A21YK_ERROR_INVALID_ARGUMENT;
    }

    sensor->config = *config;
    sensor->initialized = true;

    return GP2Y0A21YK_OK;
}

GP2Y0A21YK_Status GP2Y0A21YK_Read(
    GP2Y0A21YK *sensor,
    GP2Y0A21YK_Measurement *measurement)
{
    ADC_ChannelConfTypeDef channelConfig = {0};
    HAL_StatusTypeDef halStatus;
    uint32_t rawAdc;

    if ((sensor == NULL) || (measurement == NULL))
    {
        return GP2Y0A21YK_ERROR_INVALID_ARGUMENT;
    }

    if (!sensor->initialized)
    {
        return GP2Y0A21YK_ERROR_NOT_INITIALIZED;
    }

    channelConfig.Channel = sensor->config.channel;
    channelConfig.Rank = 1;
    channelConfig.SamplingTime = sensor->config.samplingTime;

    if (HAL_ADC_ConfigChannel(sensor->config.hadc, &channelConfig) != HAL_OK)
    {
        return GP2Y0A21YK_ERROR_ADC_CONFIG;
    }

    if (HAL_ADC_Start(sensor->config.hadc) != HAL_OK)
    {
        return GP2Y0A21YK_ERROR_ADC_START;
    }

    halStatus = HAL_ADC_PollForConversion(
        sensor->config.hadc,
        sensor->config.conversionTimeoutMs);

    if (halStatus == HAL_TIMEOUT)
    {
        (void)HAL_ADC_Stop(sensor->config.hadc);
        return GP2Y0A21YK_ERROR_ADC_TIMEOUT;
    }

    if (halStatus != HAL_OK)
    {
        (void)HAL_ADC_Stop(sensor->config.hadc);
        return GP2Y0A21YK_ERROR_ADC_CONVERSION;
    }

    rawAdc = HAL_ADC_GetValue(sensor->config.hadc);

    if (HAL_ADC_Stop(sensor->config.hadc) != HAL_OK)
    {
        return GP2Y0A21YK_ERROR_ADC_STOP;
    }

    measurement->rawAdc = (uint16_t)rawAdc;
    measurement->voltageV = GP2Y0A21YK_RawToVoltage(
        sensor,
        measurement->rawAdc);

    return GP2Y0A21YK_OK;
}

float GP2Y0A21YK_RawToVoltage(
    const GP2Y0A21YK *sensor,
    uint16_t rawAdc)
{
    if ((sensor == NULL) || !sensor->initialized)
    {
        return 0.0f;
    }

    return ((float)rawAdc * sensor->config.adcReferenceVoltageV) /
           (float)GP2Y0A21YK_ADC_FULL_SCALE_COUNT;
}
