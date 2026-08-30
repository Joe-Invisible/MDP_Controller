/*
 * MotionController.h
 *
 * Controls robot motion. Provides primitives that can be
 * composed into complex trajectories.
 *
 * Created on: 2026年8月30日
 * Author: Joe
 */

#ifndef INC_MOTIONCONTROLLER_H_
#define INC_MOTIONCONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

#include "PIDController.h"
#include "WheelSpeedController.h"
#include "MotionControllerConfig.h"
#include "RobotKinematics.h"

#include "fwdriver.h"
#include "icm20948.h"

#define MOTIONCONTROLLER_STOP_STABLE_SAMPLES 3U

typedef enum
{
	/**
	 * No motion command is being executed;
	 * lower-level motion control inactive.
	 */
	MOTIONCONTROLLER_IDLE = 0,
	/**
	 * Executing straight-motion command
	 */
	MOTIONCONTROLLER_STRAIGHT,
	/**
	 * Executing brake command
	 * MotionController will actively command
	 * the robot to attain zero-velocity.
	 */
	MOTIONCONTROLLER_BRAKING,
} MotionControllerMode;

typedef struct
{
	/*
	 * Controlled hardware / lower-level controllers
	 */
	WheelSpeedController *leftWheel;
	WheelSpeedController *rightWheel;

	Servo *steeringServo;
	ICM20948 *imu;

	const RobotKinematics *kinematics;

	/*
	 * Straight-line heading controller
	 */
	PIDController headingPID;

	/*
	 * +1 or -1 depending on physical steering / IMU orientation.
	 */
	int8_t steeringPolarity;

	MotionControllerMode mode;

	/*
	 * Current motion command
	 */
	int8_t motionDirection;
	float targetSpeedCps;
	float targetDistanceMm;

	/*
	 * Relative heading since motion began.
	 */
	float yawDeg;

	/*
	 * Encoder odometry since motion began.
	 */
	int16_t previousLeftEncoderCount;
	int16_t previousRightEncoderCount;

	float leftTravelMm;
	float rightTravelMm;
	float travelledDistanceMm;

	/*
	 * Last steering command, useful for debugging.
	 */
	float steeringCommand;

	/*
	 * Used to reject a single noisy near-zero speed sample
	 * when deciding that the robot has stopped.
	 */
	uint8_t stationarySamples;

} MotionController;


bool MotionController_Init(
	MotionController *controller,
	WheelSpeedController *leftWheel,
	WheelSpeedController *rightWheel,
	Servo *steeringServo,
	ICM20948 *imu,
	const RobotKinematics *kinematics,
	float headingKp,
	float headingKi,
	float headingKd,
	float maxSteeringCorrection,
	int8_t steeringPolarity);

/**
 * Straight line motion command
 * distanceMm - signed displacement relative to current position
 * speedCPS - unsigned magnitude of speed at which to complete this motion
 */
bool MotionController_MoveStraight(
	MotionController *controller,
	float distanceMm,
	float speedCps);

/**
 * Brake command. Unfinished motion will be
 * aborted.
 */
bool MotionController_Brake(
	MotionController *controller);

/**
 * Steps through control laws
 */
bool MotionController_Update(
	MotionController *controller,
	float dt);

/**
 * Stops executing control laws. If called while
 * in motion, this will cause the robot to coast.
 */
void MotionController_Stop(
	MotionController *controller);

/**
 * Returns whether a motion is being executed.
 */
bool MotionController_IsBusy(
	const MotionController *controller);

#endif /* INC_MOTIONCONTROLLER_H_ */
