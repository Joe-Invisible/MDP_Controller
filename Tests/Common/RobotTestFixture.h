/*
 * RobotTestFixture.h
 *
 * Initialization abstraction layer for high-level components testing
 *
 *  Created on: 2026年8月31日
 *      Author: Joe
 */

#ifndef COMMON_ROBOTTESTFIXTURE_H_
#define COMMON_ROBOTTESTFIXTURE_H_

#include <stdbool.h>
#include <stddef.h>


#include "rwdriver.h"
#include "icm20948.h"

#include "WheelSpeedController.h"
#include "WheelSpeedControllerConfig.h"

#include "SteeringController.h"
#include "SteeringControllerConfig.h"

#include "MotionController.h"
#include "MotionControllerConfig.h"

typedef struct RobotTestFixture {
	DCMotor leftRearWheel;
	DCMotor rightRearWheel;

	Servo steeringServo;

	WheelSpeedController leftWheelController;
	WheelSpeedController rightWheelController;

	SteeringController steeringController;

	MotionController motionController;

	ICM20948 imu;
} RobotTestFixture;

bool RobotTestFixture_InitIMU(
	RobotTestFixture *fixture);

bool RobotTestFixture_InitRearWheels(
    RobotTestFixture *fixture);

bool RobotTestFixture_InitFrontWheels(
	RobotTestFixture *fixture);

bool RobotTestFixture_InitWheelControllers(
    RobotTestFixture *fixture);

bool RobotTestFixture_InitSteeringController(
	RobotTestFixture *fixture);

bool RobotTestFixture_InitMotionController(
	RobotTestFixture *fixture,
	float headingKp, float headingKi, float headingKd,
	float maxHeadingSteeringAngleRad,
	float wheelSyncKpCpsPerMm, float maxWheelSyncCorrectionCps);

#endif /* COMMON_ROBOTTESTFIXTURE_H_ */
