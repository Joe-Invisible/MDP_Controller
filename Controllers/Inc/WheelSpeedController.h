/*
 * WheelSpeedController.h
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
 * PID Controller parameters
 */
// These constants defines how much at most the PID controller may
// correct the model estimate
#define WHEELSPEEDCONTROLLER_MIN_FEEDBACK 	-30.0f
#define WHEELSPEEDCONTROLLER_MAX_FEEDBACK 	30.0f

#define WHEELSPEEDCONTROLLER_KP				0.0f
#define WHEELSPEEDCONTROLLER_KI				0.0f

/**
 * Check threshold for stationary wheel
 */
#define WHEEL_STATIONARY_THRESHOLD_CPS		10

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
    const WheelSpeedCalibration *calibration);

void WheelSpeedController_SetTarget(
    WheelSpeedController *controller,
    float speedCps);

void WheelSpeedController_Update(
    WheelSpeedController *controller,
    float dt);

/**
 * Stops commanding torque, this instruction leaves
 * the motor at a coast state.
 */
void WheelSpeedController_Stop(
    WheelSpeedController *controller);

#endif /* INC_WHEELSPEEDCONTROLLER_H_ */
