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

/**
 * Quadrature mode encoding resolution of the MG512P30 motor.
 * 13 pulses/mrev * 4 counts/pulse * 30 mrev/orev = 1560
 */
#define MG512P30_QUAD_RESOLUTION ((uint32_t)1560)
#define REAR_WHEEL_DIAMETER

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

	uint32_t flipDirection;

	/**
	 * Encoder timer module
	 */
	TIM_HandleTypeDef* encHtim;

} DCMotorConfg;

/**
 * Mechanical Parameters of the Motor (and mounted wheel)
 */
typedef struct DCMotorIntrn {
	/**
	 * Count per revolution of the encoder, i.e., the resolution
	 */
	uint32_t encCPR;
	/**
	 * Diameter of the wheel mounted to the motor, in millimeters.
	 */
	float wheelDiameter;
} DCMotorIntrn;

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
	 */
	int8_t activeDutyCycle;

	/**
	 * 0: forward
	 * 1: reverse
	 * -1: stopped
	 */
	int8_t direction;

} DCMotorState;


typedef struct DCMotor {
	DCMotorConfg config;
	DCMotorIntrn intrin;
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

void DCMotor_Neutral(DCMotor* rm);

void DCMotor_Brake(DCMotor* rm);

int16_t DCMotor_GetEncoderCount(DCMotor* rm);


#endif /* INC_RWDRIVER_H_ */
