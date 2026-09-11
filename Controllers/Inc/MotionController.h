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
#include "SteeringController.h"
#include "MotionProfile.h"

#include "MotionControllerConfig.h"
#include "RobotKinematics.h"

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
	 * Executing straight motion command
	 */
	MOTIONCONTROLLER_STRAIGHT,
	/**
	 * Executing curved motion command
	 */
	MOTIONCONTROLLER_ARC,
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

	SteeringController *steering;

	ICM20948 *imu;

	const RobotKinematics *kinematics;

	/*
	 * Straight-line heading controller
	 */
	PIDController headingPID;

	/*
	 * ARC yaw-rate controller.
	 *
	 * Input:  yaw-rate error [rad/s]
	 * Output: raw steering-command correction
	 */
	PIDController arcYawRatePID;

	/*
	 * ARC diagnostics
	 */
	float yawRateDps;		/* raw gyro Z */
	float filteredYawRateDps;  /* LPF output used by ARC controller */
	float arcTargetYawRateRadPerSec;
	float arcYawRateErrorRadPerSec;
	float arcSteeringCorrectionCommand;
	float arcSteeringTargetCommand;

	/*
	 * Rear-wheel synchronization controller.
	 *
	 * Error convention:
	 *
	 *   e_sync =
	 *       (rightTravelMm - leftTravelMm)
	 *       - desiredWheelTravelDifferenceMm
	 *
	 * The controller produces a symmetric speed correction:
	 *
	 *   leftTarget  = nominalTarget + correction
	 *   rightTarget = nominalTarget - correction
	 *
	 * For straight motion, we would want the
	 * difference to be zero, but for curved motions
	 * this difference should be derived from the robot
	 * model.
	 */
	float wheelSyncKpCpsPerMm;
	float maxWheelSyncCorrectionCps;

	float desiredWheelTravelDifferenceMm;
	float wheelSyncErrorMm;
	float wheelSyncCorrectionCps;

	float arcCentreCommand;

	/*
	 * Diagnostics: most recently computed geometric wheel reference.
	 *
	 * These fields are observational only and must not be used
	 * as inputs to the control law.
	 */
	float wheelReferenceCurvaturePerMm;
	float leftBaseTargetCps;
	float rightBaseTargetCps;

	MotionControllerMode mode;

	/*
	 * Current motion command
	 */
	int8_t motionDirection;

	float maxSpeedCps;		/* unsigned requested cruise speed magnitude */
	float targetSpeedCps;	/* signed current profiled reference */

	float targetDistanceMm;

	/*
	 * Geometric path command.
	 *
	 * Straight:
	 *     curvature = 0
	 *
	 * Arc:
	 *     curvature = 1 / radius
	 *
	 * ARC steering is controlled from measured yaw rate
	 * and does not require a physical steering-angle model.
	 */
	float targetCurvaturePerMm;
	float targetSteeringAngleRad;

	MotionProfile motionProfile;

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
	 * Used to reject a single noisy near-zero speed sample
	 * when deciding that the robot has stopped.
	 */
	uint8_t stationarySamples;

} MotionController;


bool MotionController_Init(
    MotionController *controller,
    WheelSpeedController *leftWheel,
    WheelSpeedController *rightWheel,
    SteeringController *steering,
    ICM20948 *imu,
    const RobotKinematics *kinematics,
    float headingKp,
    float headingKi,
    float headingKd,
    float maxHeadingSteeringAngleRad,
	/* We are separating the steering PID with
	 * existing heading PID, might clean up later
	 */
	float arcYawRateKp,
	float arcYawRateKi,
	float arcYawRateKd,
	float maxArcSteeringCommandCorrection,
    float wheelSyncKpCpsPerMm,
    float maxWheelSyncCorrectionCps,
	float motionAccelerationMmps2,
	float motionDecelerationMmps2);

/**
 * Straight line motion.
 *
 * distanceMm:
 * 		Signed rear-axle-centre path length relative to current
 * 		position.
 * speedCPS:
 * 		Unsigned cruising speed. Note that this speed may
 * 		not be attained, pertaining to the configured motion profile
 * 		and specified distance.
 */
bool MotionController_MoveStraight(
	MotionController *controller,
	float distanceMm,
	float speedCps);

/**
 * Constant-curvature arc motion.
 *
 * distanceMm:
 *     Signed rear-axle-centre path length.
 *     Positive = forward, negative = reverse.
 *
 * radiusMm:
 *     Signed radius of curvature.
 *     Positive radius produces positive effective
 *     steering angle.
 *
 * speedCps:
 *     Unsigned centre-speed magnitude.
 */
bool MotionController_MoveArc(
    MotionController *controller,
    float distanceMm,
    float radiusMm,
    float speedCps);

/**
 * Full brake. Unfinished motion will be
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
