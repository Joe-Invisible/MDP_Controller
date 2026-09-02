/*
 * MotionControllerConfig.c
 *
 *  Created on: 2026年8月30日
 *      Author: Joe
 */

#include "MotionControllerConfig.h"

#define MG512P30_QUAD_RESOLUTION		1560U
#define WHEELBASE_MM					150.0f
#define WHEEL_WIDTH_MM				20.0f
#define ROBOT_REAR_WIDTH_MM			185.0f
#define REAR_TRACK_WIDTH_MM			(ROBOT_REAR_WIDTH_MM - (WHEEL_WIDTH_MM))
/**
 * Effective wheel diameter under load.
 */
#define WHEEL_DIAMETER_MM			65.5f

const RobotKinematics kinematics = {
		.rearEncoderCountsPerRev = MG512P30_QUAD_RESOLUTION,
		.rearWheelDiameterMm = WHEEL_DIAMETER_MM,
};
