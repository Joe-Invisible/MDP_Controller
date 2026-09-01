/*
 * RobotTestFixture.c
 *
 *  Created on: 2026年8月31日
 *      Author: Joe
 */

#include "tim.h"
#include "i2c.h"

#include "RobotTestFixture.h"

#define MOTORBPWMSRC htim9
#define MOTORCPWMSRC htim1

#define MOTORBENC htim3
#define MOTORCENC htim4

#define SERVOPWMSRC htim8
#define SERVOPWMCH TIM_CHANNEL_1

bool RobotTestFixture_InitIMU(RobotTestFixture *fixture) {
	if (fixture == NULL) return false;
	if (!ICM20948_Init(&fixture->imu, &hi2c2)) return false;

	return true;
}

bool RobotTestFixture_InitRearWheels(RobotTestFixture *fixture) {

	if (fixture == NULL) return false;

	if (!DCMotor_Init(&fixture->leftRearWheel, &MOTORBPWMSRC, false, &MOTORBENC) ||
		!DCMotor_Init(&fixture->rightRearWheel, &MOTORCPWMSRC, true, &MOTORCENC)) {
		return false;
	}

	if (!DCMotor_Enable(&fixture->leftRearWheel)) return false;
	if (!DCMotor_Enable(&fixture->rightRearWheel)) return false;

	return true;
}

bool RobotTestFixture_InitFrontWheels(RobotTestFixture *fixture) {
	if (fixture == NULL) return false;

	if (!Servo_Init(
			&fixture->steeringServo,
			&SERVOPWMSRC, SERVOPWMCH,
			CHASSIS_STEER_MIN_PULSE_US,
			CHASSIS_STEER_CTR_PULSE_US,
			CHASSIS_STEER_MAX_PULSE_US))
		return false;

	if (!Servo_Enable(&fixture->steeringServo))
		return false;

	return true;
}

bool RobotTestFixture_InitWheelControllers(RobotTestFixture *fixture) {
	if (fixture == NULL) return false;

	if (!WheelSpeedController_Init(
			&fixture->leftWheelController,
			&fixture->leftRearWheel,
			WHEELSPEEDCONTROLLER_KP, WHEELSPEEDCONTROLLER_KI,
			WHEELSPEEDCONTROLLER_MIN_FEEDBACK,
			WHEELSPEEDCONTROLLER_MAX_FEEDBACK,
			&leftCalibration)) {
		return false;
	}

	if (!WheelSpeedController_Init(
			&fixture->rightWheelController,
			&fixture->rightRearWheel,
			WHEELSPEEDCONTROLLER_KP, WHEELSPEEDCONTROLLER_KI,
			WHEELSPEEDCONTROLLER_MIN_FEEDBACK,
			WHEELSPEEDCONTROLLER_MAX_FEEDBACK,
			&rightCalibration)) {
		return false;
	}

	return true;
}

bool RobotTestFixture_InitMotionController(
		RobotTestFixture *fixture,
		float headingKp, float headingKi, float headingKd,
		float maxSteeringCorrection) {
	if (fixture == NULL) return false;

	if (!RobotTestFixture_InitIMU(fixture)) return false;

	if (!RobotTestFixture_InitRearWheels(fixture)) return false;

	if (!RobotTestFixture_InitFrontWheels(fixture)) return false;

	if (!RobotTestFixture_InitWheelControllers(fixture)) return false;

	if (!MotionController_Init(
			&fixture->motionController,
			&fixture->leftWheelController,
			&fixture->rightWheelController,
			&fixture->steeringServo,
			&fixture->imu,
			&kinematics, headingKp, headingKi, headingKd, maxSteeringCorrection, -1))
		return false;

	return true;
}
