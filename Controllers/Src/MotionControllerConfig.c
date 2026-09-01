/*
 * MotionControllerConfig.c
 *
 *  Created on: 2026年8月30日
 *      Author: Joe
 */

#include "MotionControllerConfig.h"

#define MG512P30_QUAD_RESOLUTION		1560U
#define WHEELBASE_MM					150.0f
#define FRONT_TRACK_WIDTH_MM			165.0f
#define REAR_TRACK_WIDTH_MM			164.0f
/**
 * Effective wheel diameter under load.
 */
#define WHEEL_DIAMETER_MM			65.5f

const RobotKinematics kinematics = {
		.rearEncoderCountsPerRev = MG512P30_QUAD_RESOLUTION,
		.rearWheelDiameterMm = WHEEL_DIAMETER_MM,
};
