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
#include "DynamicBrakeMap.h"
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


typedef enum
{
    WHEEL_SPEED_ACTUATOR_COAST = 0,
    WHEEL_SPEED_ACTUATOR_DRIVE,
    WHEEL_SPEED_ACTUATOR_BRAKE

} WheelSpeedActuatorMode;


/*
 * Active-braking control policy.
 *
 * This is distinct from DynamicBrakeMap:
 *
 *   DynamicBrakeMap:
 *       normalized brake demand -> physical H-bridge PWM
 *
 *   WheelSpeedBrakeConfig:
 *       overspeed error -> normalized brake demand
 */
typedef struct
{
    const DynamicBrakeMap *map;

    /*
     * Active braking begins once overspeed reaches this value.
     */
    float engageOverspeedCps;

    /*
     * Once braking has begun, continue until overspeed falls
     * below this value.
     *
     * Must be less than engageOverspeedCps.
     */
    float releaseOverspeedCps;

    /*
     * Overspeed corresponding to brakeDemand == 1.0f.
     *
     * Must be greater than engageOverspeedCps.
     */
    float fullDemandOverspeedCps;

} WheelSpeedBrakeConfig;


typedef struct {
    DCMotor *motor;

    WheelSpeedCalibration calibration;
    PIDController pid;

    int16_t previousEncoderCount;

    float targetSpeedCps;
    float measuredSpeedCps;
    /*
     * Propulsion PWM only.
     * Set to zero whenever active braking is being applied.
     */
    float outputPWM;

    /*
     * Active-brake telemetry.
     *
     * brakeDemand:
     *   normalized controller request [0, 1]
     *
     * brakePWM:
     *   physical H-bridge brake PWM [%]
     */
    float brakeDemand;
    float brakePWM;

    WheelSpeedActuatorMode actuatorMode;

    /*
     * Used for brake hysteresis.
     *
     * This is separate from actuatorMode because target == 0
     * also uses BRAKE, but is not an overspeed-braking state.
     */
    bool activeBrakeEngaged;

    bool brakeConfigured;
    WheelSpeedBrakeConfig brakeConfig;

    float currentOffset;
    float currentSlope;
} WheelSpeedController;


bool WheelSpeedController_Init(
    WheelSpeedController *controller,
    DCMotor *motor,
	float kp, float ki,
	float minFeedback, float maxFeedback,
    const WheelSpeedCalibration *calibration);

bool WheelSpeedController_ConfigureBrake(
        WheelSpeedController *controller,
        const WheelSpeedBrakeConfig *config);

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
