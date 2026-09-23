/*
 * SideIRSensorConfig.c
 *
 *  Created on: 2026年9月20日
 *      Author: Joe
 */

#include "SideIRSensorConfig.h"

#include "adc.h"

#define SIDE_IR_ADC_REFERENCE_V       3.3f
#define SIDE_IR_ADC_TIMEOUT_MS        2U
#define SIDE_IR_ADC_SAMPLING_TIME     ADC_SAMPLETIME_144CYCLES

/* H1 pin 8: PC1 / ADC1_IN11. */
const GP2Y0A21YK_Config sideIRLeftConfig = {
    .hadc = &hadc1,
    .channel = ADC_CHANNEL_11,
    .samplingTime = SIDE_IR_ADC_SAMPLING_TIME,
    .adcReferenceVoltageV = SIDE_IR_ADC_REFERENCE_V,
    .conversionTimeoutMs = SIDE_IR_ADC_TIMEOUT_MS,
};

/* H1 pin 10: PC2 / ADC1_IN12. */
const GP2Y0A21YK_Config sideIRRightConfig = {
    .hadc = &hadc1,
    .channel = ADC_CHANNEL_12,
    .samplingTime = SIDE_IR_ADC_SAMPLING_TIME,
    .adcReferenceVoltageV = SIDE_IR_ADC_REFERENCE_V,
    .conversionTimeoutMs = SIDE_IR_ADC_TIMEOUT_MS,
};
