/*
 * WheelSpeedController.h
 *
 * Closed-loop wheel speed controller using encoder counts as input.
 * Pre-derived motor model required. See DCMotor_Linear_Response_Summary in exp/
 *
 *  Created on: 2026年8月29日
 *      Author: Joe
 */

#ifndef INC_WHEELSPEEDCONTROLLER_H_
#define INC_WHEELSPEEDCONTROLLER_H_

#include <stdbool.h>
#include "PIDController.h"
#include "rwdriver.h"
#include <stdint.h>


/**
 * Check threshold for stationary wheel. Set this according
 * to the controller update frequency. 100Cps corresponds to 100Hz update.
 */
#define WHEEL_STATIONARY_THRESHOLD_CPS		100

/**
 * We store all calibration values as positive magnitudes.
 * The sign for DCMotor input will be
 */
typedef struct {
    float forwardSlope;
    float forwardOffset;

    float reverseSlope;
    float reverseOffset;

    float startForwardPWM;
    float startReversePWM;

    float runForwardPWM;
    float runReversePWM;
} WheelSpeedCalibration;


typedef struct {
    DCMotor *motor;

    WheelSpeedCalibration calibration;
    PIDController pid;

    int16_t previousEncoderCount;

    float targetSpeedCps;
    float measuredSpeedCps;
    float outputPWM;

    float currentOffset;
    float currentSlope;
} WheelSpeedController;


bool WheelSpeedController_Init(
    WheelSpeedController *controller,
    DCMotor *motor,
	float kp, float ki,
	float minFeedback, float maxFeedback,
    const WheelSpeedCalibration *calibration);

void WheelSpeedController_SetTarget(
    WheelSpeedController *controller,
    float speedCps);

void WheelSpeedController_Update(
    WheelSpeedController *controller,
    float dt);

/**
 * Returns whether the wheel is stationary
 * based on the measured wheel speed below a
 * set threshold to eliminate noise
 */
bool WheelSpeedController_IsStationary(
	const WheelSpeedController *controller);

/**
 * Stops commanding torque, this instruction leaves
 * the motor at a coast state.
 */
void WheelSpeedController_Stop(
    WheelSpeedController *controller);

#endif /* INC_WHEELSPEEDCONTROLLER_H_ */
