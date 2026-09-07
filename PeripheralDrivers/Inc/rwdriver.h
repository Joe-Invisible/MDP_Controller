/*
 * rwdriver.h
 * Low-level DC motor driver interface
 *
 *  Created on: 2026年8月24日
 *      Author: Joe
 */

#ifndef INC_RWDRIVER_H_
#define INC_RWDRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef enum {
    DCMOTOR_MODE_NEUTRAL = 0,
    DCMOTOR_MODE_DRIVE,
    DCMOTOR_MODE_BRAKE
} DCMotorMode;

/**
 * Peripheral Configuration Parameters
 */
typedef struct DCMotorConfg {
	/**
	 * PWM source timer module
	 */
	TIM_HandleTypeDef* pwmHtim;
	/**
	 * PWM source channel 1
	 */
	uint32_t pwmChannel1;
	/**
	 * PWM source channel 1
	 */
	uint32_t pwmChannel2;

	/**
	 * Adjust according to hardware polarity
	 */
	uint32_t flipDirection;

	/**
	 * Encoder timer module
	 */
	TIM_HandleTypeDef* encHtim;

} DCMotorConfg;


/**
 * Application-level runtime states of the Motor.
 */
typedef struct DCMotorState {
	/**
	 * Encoder count.
	 */
	uint16_t encCount;
	/**
	 * Range 0-100
	 * specifies the % duty cycle, e.g., 50 is 50% duty cycle.
	 * This value will be applied to the active channel.
	 *
	 * For BRAKE, this is the duty cycle applied to both
	 * channels.
	 *
	 */
	float activeDutyCycle;

	/**
	 * Torque direction of the motor, valid during DRIVE
	 * state only,
	 * 0: forward (startup NEUTRAL mode default)
	 * 1: reverse
	 * In other modes, this field is not valid.
	 */
	int8_t direction;

	DCMotorMode mode;
} DCMotorState;


typedef struct DCMotor {
	DCMotorConfg config;
	DCMotorState state;
} DCMotor;

/**
 *
 */
bool DCMotor_Init(DCMotor* rm, TIM_HandleTypeDef* pwmHtim, bool flipDirection, TIM_HandleTypeDef* encHtim);

/**
 * Forces both PWM to 0 and stars PWM generation.
 */
bool DCMotor_Enable(DCMotor* rm);

/**
 * Stops PWM generation for both channels. For normal motion
 * commands, use DCMotor_SetPWM with 0% duty cycle instead.
 */
void DCMotor_Disable(DCMotor* rm);

/**
 * Set PWM to drive the motor.
 * dutyCycle:
	 * Range -100~+100
	 * specifies the % duty cycle, e.g., 50 is 50% duty cycle.
	 * This value will be applied to one of the channels depending on its sign.
	 * +: Forward;
	 * -: Reverse
 */
void DCMotor_SetPWM(DCMotor* rm, float dutyCycle);

/**
 * Puts the motor at coast. The wheels can roll freely.
 */
void DCMotor_Neutral(DCMotor* rm);

/**
 * Electrical braking. For the H-bridge driver, this method
 * drives both H-bridge control inputs high during the braking
 * portion, producing an electromagnetic torque that opposes
 * rotation. Prefer this over coasting if there is active need
 * of fast deceleration.
 *
 * Both H-bridge control inputs are driven with the same PWM
 * duty cycle, effectively alternating between BRAKE and COAST.
 *
 * brakePercent:
 * 		Range 0~+100
 * 		Determines the ratio of BRAKE state in one PWM period.
 */
void DCMotor_SetBrakePWM(DCMotor *motor, float brakePercent);

/**
 * Compatibility wrapper for full duty cycle braking.
 * Equivalent to DCMotor_SetBrakePWM with brakePercent=100.
 */
void DCMotor_Brake(DCMotor* rm);

int16_t DCMotor_GetEncoderCount(DCMotor* rm);


#endif /* INC_RWDRIVER_H_ */
