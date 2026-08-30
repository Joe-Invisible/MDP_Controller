/*
 * WheelSpeedControllerConfig.c
 *
 *  Created on: 2026年8月30日
 *      Author: Joe
 */

#include "WheelSpeedControllerConfig.h"

const WheelSpeedCalibration leftCalibration = {
    .forwardSlope      = 203.88f,
    .forwardOffset     = 53.03f,

    .reverseSlope      = 191.88f,
    .reverseOffset     = 53.44f,

    .startForwardPWM   = 55.0f,
    .startReversePWM   = 56.0f,

    .runForwardPWM     = 54.0f,
    .runReversePWM     = 55.0f
};

const WheelSpeedCalibration rightCalibration = {
    .forwardSlope      = 192.56f,
    .forwardOffset     = 53.37f,

    .reverseSlope      = 198.32f,
    .reverseOffset     = 53.49f,

    .startForwardPWM   = 56.0f,
    .startReversePWM   = 56.0f,

    .runForwardPWM     = 54.0f,
    .runReversePWM     = 55.0f
};
