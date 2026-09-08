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
		float minFeedback, float maxFeedback,
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

    controller->actuatorMode =
        WHEEL_SPEED_ACTUATOR_COAST;

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
		minFeedback, maxFeedback))
    		return false;

    /* Ensure the motor starts in a known stopped state. */
    DCMotor_SetPWM(motor, 0.0f);

    return true;
}

bool WheelSpeedController_ConfigureBrake(
        WheelSpeedController *controller,
        const WheelSpeedBrakeConfig *config)
{
    if (controller == NULL ||
        config == NULL ||
        config->map == NULL)
    {
        return false;
    }

    if (!DynamicBrakeMap_ValidateMap(config->map))
        return false;

    if (config->releaseOverspeedCps < 0.0f ||
        config->engageOverspeedCps <=
            config->releaseOverspeedCps ||
        config->fullDemandOverspeedCps <=
            config->engageOverspeedCps)
    {
        return false;
    }

    /*
     * The brake map used by WheelSpeedController is expected
     * to represent normalized demand in (0, 1].
     */
    float minimumDemand =
        config->map->demandKnots[0];

    float maximumDemand =
        config->map->demandKnots[
            config->map->demandCount - 1U];

    if (minimumDemand <= 0.0f ||
        maximumDemand > 1.0f)
    {
        return false;
    }

    controller->brakeConfig = *config;
    controller->brakeConfigured = true;

    controller->brakeDemand = 0.0f;
    controller->brakePWM = 0.0f;
    controller->activeBrakeEngaged = false;

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


static float WheelSpeedController_GetOverspeedCps(
        const WheelSpeedController *controller)
{
    float target = controller->targetSpeedCps;
    float measured = controller->measuredSpeedCps;

    if (target == 0.0f)
        return 0.0f;

    /*
     * Active braking is only used when the wheel is already
     * moving in the commanded direction.
     *
     * If it is moving opposite the requested direction,
     * normal direction-change control remains responsible.
     */
    bool movingInTargetDirection =
        (target > 0.0f && measured > 0.0f) ||
        (target < 0.0f && measured < 0.0f);

    if (!movingInTargetDirection)
        return 0.0f;

    float overspeed =
        fabsf(measured) - fabsf(target);

    if (overspeed <= 0.0f)
        return 0.0f;

    return overspeed;
}

static float WheelSpeedController_ComputeBrakeDemand(
        const WheelSpeedController *controller,
        float overspeedCps)
{
    const WheelSpeedBrakeConfig *config =
        &controller->brakeConfig;

    const DynamicBrakeMap *map =
        config->map;

    float minimumDemand =
        map->demandKnots[0];

    /*
     * During the hysteresis region, maintain minimum
     * calibrated braking authority.
     */
    if (overspeedCps <= config->engageOverspeedCps)
        return minimumDemand;

    if (overspeedCps >= config->fullDemandOverspeedCps)
        return 1.0f;

    float t =
        (overspeedCps -
         config->engageOverspeedCps) /
        (config->fullDemandOverspeedCps -
         config->engageOverspeedCps);

    return minimumDemand +
        t * (1.0f - minimumDemand);
}

/*
 * Determine from current overspeed whether active braking
 * should be engaged. Implements a hysteresis.
 */
static bool WheelSpeedController_ShouldActivelyBrake(
        const WheelSpeedController *controller,
        float overspeedCps)
{
    if (!controller->brakeConfigured ||
        overspeedCps <= 0.0f)
    {
        return false;
    }

    if (controller->activeBrakeEngaged)
    {
        /*
         * Stay engaged until we cross the lower release
         * threshold.
         */
        return overspeedCps >
            controller->brakeConfig.releaseOverspeedCps;
    }

    /*
     * Enter braking only at the higher threshold.
     */
    return overspeedCps >=
        controller->brakeConfig.engageOverspeedCps;
}

/**
 * Returns whether minimum-PWM deadband compensation should be applied.
 *
 * If the wheel is already moving in the commanded direction faster than
 * the target speed, forcing the output back to the minimum running PWM
 * prevents controlled deceleration. In that case, allow the PI +
 * feedforward result to fall below the calibrated running minimum.
 */
static bool WheelSpeedController_ShouldApplyDeadbandCompensation(
        const WheelSpeedController *controller)
{
    float target = controller->targetSpeedCps;
    float measured = controller->measuredSpeedCps;

    if (target == 0.0f)
        return false;

    bool movingInTargetDirection =
        (target > 0.0f && measured > 0.0f) ||
        (target < 0.0f && measured < 0.0f);

    bool fasterThanTarget =
        fabsf(measured) > fabsf(target);

    return !(movingInTargetDirection && fasterThanTarget);
}

void WheelSpeedController_SetTarget(
    WheelSpeedController *controller,
    float speedCps) {

	if (!controller) return;

	float oldTarget = controller->targetSpeedCps;

	controller->targetSpeedCps = speedCps;

	// Reset PID when command changes direction. This prevents
	// the stale positive integral from fighting the new reverse command.
	if ((oldTarget > 0.0f && speedCps < 0.0f) ||
	    (oldTarget < 0.0f && speedCps > 0.0f))
	{
	    PIDController_Reset(&controller->pid);
	}

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

bool WheelSpeedController_IsStationary(const WheelSpeedController *controller) {
	if (controller == NULL)
		return false;

	return fabsf(controller->measuredSpeedCps) <
	WHEEL_STATIONARY_THRESHOLD_CPS;
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

	/*
	 * Zero target retains the existing semantic:
	 *
	 * command zero speed -> full dynamic brake.
	 */
	if (controller->targetSpeedCps == 0.0f)
	{
	    PIDController_Reset(&controller->pid);

	    controller->outputPWM = 0.0f;

	    controller->brakeDemand = 1.0f;
	    controller->brakePWM = 100.0f;

	    controller->activeBrakeEngaged = false;
	    controller->actuatorMode =
	        WHEEL_SPEED_ACTUATOR_BRAKE;

	    DCMotor_Brake(controller->motor);
	    return;
	}


	/*
	 * Determine whether controlled active braking should take
	 * over from the normal drive controller.
	 */
	float overspeedCps =
	    WheelSpeedController_GetOverspeedCps(controller);

	bool activelyBrake =
	    WheelSpeedController_ShouldActivelyBrake(
	        controller,
	        overspeedCps);

	if (activelyBrake)
	{
	    /*
	     * Reset the propulsion PI when ENTERING active braking.
	     *
	     * While braking, PIDController_Update() is deliberately
	     * not called, so no integral accumulates behind an actuator
	     * mode that is not actually using its output.
	     */
	    if (!controller->activeBrakeEngaged)
	    {
	        PIDController_Reset(&controller->pid);
	    }

	    controller->activeBrakeEngaged = true;

	    controller->brakeDemand =
	        WheelSpeedController_ComputeBrakeDemand(
	            controller,
	            overspeedCps);

	    controller->brakePWM =
	        DynamicBrakeMap_GetPWM(
	            controller->brakeConfig.map,
	            controller->brakeDemand,
	            controller->measuredSpeedCps);

	    controller->outputPWM = 0.0f;

	    controller->actuatorMode =
	        WHEEL_SPEED_ACTUATOR_BRAKE;

	    DCMotor_SetBrakePWM(
	        controller->motor,
	        controller->brakePWM);

	    return;
	}


	/*
	 * Brake hysteresis has released.
	 */
	controller->activeBrakeEngaged = false;
	controller->brakeDemand = 0.0f;
	controller->brakePWM = 0.0f;


	/*
	 * Normal propulsion controller.
	 */
	float pffMagnitude =
	    WheelSpeedController_ComputePFF(controller);

	float ppi =
	    WheelSpeedController_ComputePPI(
	        controller,
	        dt);

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
	 * Apply deadband compensation when driving the wheel toward
	 * its target speed.
	 *
	 * If the wheel is already moving in the commanded direction
	 * faster than the target, allow the controller output to fall
	 * below the minimum running PWM so that the wheel can decelerate.
	 */
	if (WheelSpeedController_ShouldApplyDeadbandCompensation(controller))
	{
	    bool stationary =
	        WheelSpeedController_IsStationary(controller);

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

	/*
	 * When the wheel is already moving in the target direction
	 * faster than requested, propulsion may be reduced all the
	 * way to zero, but it must not reverse polarity.
	 *
	 * Opposite-polarity drive would be plugging/reverse torque,
	 * not controlled dynamic braking.
	 */
	if (controller->brakeConfigured &&
	    overspeedCps > 0.0f)
	{
	    if ((controller->targetSpeedCps > 0.0f &&
	         pwm < 0.0f) ||
	        (controller->targetSpeedCps < 0.0f &&
	         pwm > 0.0f))
	    {
	        pwm = 0.0f;
	    }
	}

	// The controller should also know about output saturation.
	if (pwm > 100.0f)
	    pwm = 100.0f;
	else if (pwm < -100.0f)
	    pwm = -100.0f;

	controller->outputPWM = pwm;

	controller->actuatorMode =
	    pwm == 0.0f ?
	        WHEEL_SPEED_ACTUATOR_COAST :
	        WHEEL_SPEED_ACTUATOR_DRIVE;

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

    controller->brakeDemand = 0.0f;
    controller->brakePWM = 0.0f;

    controller->activeBrakeEngaged = false;

    controller->actuatorMode =
        WHEEL_SPEED_ACTUATOR_COAST;

    PIDController_Reset(&controller->pid);

    DCMotor_SetPWM(controller->motor, 0.0f);

    /*
     * So that when control resumes, the controller does not treat
     * encoder movement while stopped as movement during one control interval.
     */
    controller->previousEncoderCount =
        DCMotor_GetEncoderCount(controller->motor);
}
