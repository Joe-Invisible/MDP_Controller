/*
 * WheelSpeedContoller.c
 *
 *  Created on: 2026年8月29日
 *      Author: Joe
 */

#include "WheelSpeedController.h"
#include "rwdriver.h"
#include <math.h>

bool WheelSpeedController_Init(
        WheelSpeedController *controller,
        DCMotor *motor,
		float kp, float ki,
        const WheelSpeedCalibration *calibration)
{
    if (controller == NULL ||
        motor == NULL ||
        calibration == NULL)
    {
        return false;
    }

    /* Initialise all controller state to zero. */
    *controller = (WheelSpeedController){0};

    controller->motor = motor;
    controller->calibration = *calibration;

    /*
     * Use the current encoder count as the initial reference.
     * Otherwise the first Update() could interpret all previous
     * encoder counts as movement during the first control period.
     */
    controller->previousEncoderCount =
        DCMotor_GetEncoderCount(motor);

    controller->targetSpeedCps   = 0.0f;
    controller->measuredSpeedCps = 0.0f;
    controller->outputPWM        = 0.0f;

    controller->currentOffset 	= 0.0f;
    controller->currentSlope 	= 0.0f;

    if (!PIDController_Init(
    		&controller->pid,
		kp, ki, 0.0f,
		WHEELSPEEDCONTROLLER_MIN_FEEDBACK, WHEELSPEEDCONTROLLER_MAX_FEEDBACK))
    		return false;

    /* Ensure the motor starts in a known stopped state. */
    DCMotor_SetPWM(motor, 0.0f);

    return true;
}

/**
 * Computes the feedforward PWM magnitude.
 *
 * Through experiment, we established that the motor speed response is linear in
 * PWM value, i.e., for all $P > P_0$ where $P_0$ is the dead zone PWM value, have
 * 	$$
 * 	v \approx k (P - P_0)
 * 	$$
 * where $v$ is the motor speed in counts per second, and $k$ is a constant slope
 * of around 190-200.
 * Therefore, to compute the required $P$ for some target $v$, take
 * 	$$
 * 	P \approx P_0 + \frac{v}{k}
 * 	$$
 *
 * The two slopes (for forward and reverse motion) are not identical, and
 * the slopes for left and right motors are also not identical.
 * At 100% PWM the left count and right count will differ by ~6.5% in 500ms.
 */
static float WheelSpeedController_ComputePFF(
        WheelSpeedController *controller)
{
    float target = controller->targetSpeedCps;

    if (target == 0.0f)
        return 0.0f;

    float magnitude =
        controller->currentOffset +
        fabsf(target) / controller->currentSlope;

    return magnitude;
}

/**
 * We use PI controller here to correct the model error.
 * Theoretically a signed control should support sudden,
 * aggressive reversal / change of direction.
 */
static float WheelSpeedController_ComputePPI(
		WheelSpeedController *controller,
		float dt)
{
	float errorCps = controller->targetSpeedCps - controller->measuredSpeedCps;

	return PIDController_Update(&controller->pid, errorCps, dt);
}

void WheelSpeedController_SetTarget(
    WheelSpeedController *controller,
    float speedCps) {

	if (!controller) return;

	controller->targetSpeedCps = speedCps;

	if (speedCps == 0.0f) return;

	controller->currentOffset =
			controller->targetSpeedCps < 0.0f ?
				controller->calibration.reverseOffset :
				controller->calibration.forwardOffset;

	controller->currentSlope =
			controller->targetSpeedCps < 0.0f ?
				controller->calibration.reverseSlope :
				controller->calibration.forwardSlope;
}

/**
 * We use a motor model to estimate the required PWM value for commanded
 * speed, then use a PI controller to correct model error.
 */
void WheelSpeedController_Update(
    WheelSpeedController *controller,
    float dt) {

	if (!controller || !controller->motor || dt <= 0.0f) return;

	int16_t current = DCMotor_GetEncoderCount(controller->motor);

	int16_t delta = (int16_t)(
			(uint16_t)current -
			(uint16_t)controller->previousEncoderCount);

	controller->previousEncoderCount = current;

	controller->measuredSpeedCps = (float)delta / dt;

	float pffMagnitude = WheelSpeedController_ComputePFF(controller);
	float ppi = WheelSpeedController_ComputePPI(controller, dt);

	float pff = 0.0f;

	// Apply direction
	if (controller->targetSpeedCps > 0.0f)
	    pff = pffMagnitude;
	else if (controller->targetSpeedCps < 0.0f)
	    pff = -pffMagnitude;
	else
	    pff = 0.0f;

	float pwm = pff + ppi;

	/*
	 * Apply deadband compensation except when stopped.
	 */
	if (controller->targetSpeedCps != 0.0f)
	{
	    bool stationary =
	            fabsf(controller->measuredSpeedCps) <
	            WHEEL_STATIONARY_THRESHOLD_CPS;

	    float minPWM;

	    if (controller->targetSpeedCps > 0.0f)
	    {
	        minPWM = stationary ?
	                controller->calibration.startForwardPWM :
	                controller->calibration.runForwardPWM;
	    }
	    else
	    {
	        minPWM = stationary ?
	                controller->calibration.startReversePWM :
	                controller->calibration.runReversePWM;
	    }

	    /*
	     * Calibration values are stored as positive magnitudes.
	     */
	    if (fabsf(pwm) < minPWM)
	    {
	        pwm = copysignf(
	                minPWM,
	                controller->targetSpeedCps);
	    }
	}

	// The controller should also know about output saturation.
	if (pwm > 100.0f)
	    pwm = 100.0f;
	else if (pwm < -100.0f)
	    pwm = -100.0f;

	controller->outputPWM = pwm;

	DCMotor_SetPWM(controller->motor, controller->outputPWM);
}

void WheelSpeedController_Stop(WheelSpeedController *controller)
{
    if (controller == NULL || controller->motor == NULL)
    {
        return;
    }

    controller->targetSpeedCps = 0.0f;
    controller->measuredSpeedCps = 0.0f;
    controller->outputPWM = 0.0f;

    // PIDController_Reset(&controller->pid);

    DCMotor_SetPWM(controller->motor, 0.0f);

    /*
     * So that when control resumes, the controller does not treat
     * encoder movement while stopped as movement during one control interval.
     */
    controller->previousEncoderCount =
        DCMotor_GetEncoderCount(controller->motor);
}
