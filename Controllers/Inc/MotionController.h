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
/**
 * MotionProfile exact endpoint crossing can deadlock
 * finite-distance motion near the encoder-resolution
 * limit. The motions require a explicit completion
 * tolerance.
 *
 * 0.5 mm is about 4 encoder counts.
 */
#define MOTION_PROFILE_COMPLETION_TOLERANCE_MM 	(0.5f)

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

/**
 * Result of a MotionController operation.
 *
 * MOTIONCONTROLLER_STATUS_OK is the only success value. In particular,
 * a zero-distance move is reported as OK and leaves the controller idle.
 */
typedef enum
{
	MOTIONCONTROLLER_STATUS_OK = 0,
	MOTIONCONTROLLER_STATUS_INVALID_ARGUMENT,
	MOTIONCONTROLLER_STATUS_NOT_INITIALIZED,
	MOTIONCONTROLLER_STATUS_INVALID_CONFIGURATION,
	MOTIONCONTROLLER_STATUS_BUSY,
	MOTIONCONTROLLER_STATUS_INVALID_DISTANCE,
	MOTIONCONTROLLER_STATUS_INVALID_SPEED,
	MOTIONCONTROLLER_STATUS_INVALID_RADIUS,
	MOTIONCONTROLLER_STATUS_UNSUPPORTED_CURVATURE,
	MOTIONCONTROLLER_STATUS_PROFILE_ERROR,
	MOTIONCONTROLLER_STATUS_IMU_ERROR,
	MOTIONCONTROLLER_STATUS_INVALID_STATE,
} MotionControllerStatus;

typedef struct
{
	/* True only after MotionController_Init() completes successfully. */
	bool initialized;

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
	 * ARC outer heading controller.
	 *
	 * Input:  heading error [rad]
	 * Output: yaw-rate correction [rad/s]
	 *
	 * Units of gain: 1/s
	 */
	float arcHeadingKpPerSec;

	/*
	 * ARC diagnostics
	 */
	float yawRateDps;
	float filteredYawRateDps;

	float arcDesiredYawRad;
	float arcHeadingErrorRad;

	float arcFeedforwardYawRateRadPerSec;
	float arcHeadingYawRateCorrectionRadPerSec;
	float arcTargetYawRateRadPerSec;

	float arcCommandedCurvaturePerMm;

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
	 * Absolute raw steering command supplied by curvature
	 * feedforward. Feedback correction is applied around this.
	 */
	float arcSteeringFeedforwardCommand;

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


MotionControllerStatus MotionController_Init(
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

	float arcHeadingKpPerSec,

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
 *
 * A zero distance is a successful no-op. The controller remains idle.
 */
MotionControllerStatus MotionController_MoveStraight(
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
 *
 * A zero distance is a successful no-op. The controller remains idle.
 */
MotionControllerStatus MotionController_MoveArc(
    MotionController *controller,
    float distanceMm,
    float radiusMm,
    float speedCps);

/**
 * Full brake. Unfinished motion will be aborted.
 * Repeated calls while braking are successful no-ops.
 */
MotionControllerStatus MotionController_Brake(
	MotionController *controller);

/**
 * Steps through control laws.
 * Returns MOTIONCONTROLLER_STATUS_IMU_ERROR if yaw feedback fails;
 * the controller coasts and returns to IDLE in that case.
 */
MotionControllerStatus MotionController_Update(
	MotionController *controller,
	float dt);

/**
 * Stops executing control laws. If called while in motion, this will
 * cause the robot to coast and immediately return to IDLE.
 */
MotionControllerStatus MotionController_Stop(
	MotionController *controller);

/**
 * Returns whether a motion is being executed.
 */
bool MotionController_IsBusy(
	const MotionController *controller);

#endif /* INC_MOTIONCONTROLLER_H_ */
