/*
 * MotionControllerConfig.c
 *
 *  Created on: 2026年8月30日
 *      Author: Joe
 */

#include "MotionControllerConfig.h"

#define MG512P30_QUAD_RESOLUTION		1560U
#define WHEEL_DIAMETER_MM			65.0f
#define WHEELBASE_MM					150.0f
#define FRONT_TRACK_WIDTH_MM			165.0f
#define REAR_TRACK_WIDTH_MM			164.0f
// We need to measure the effective metrics of the wheels

const RobotKinematics kinematics = {
		.rearEncoderCountsPerRev = MG512P30_QUAD_RESOLUTION,
		.rearWheelDiameterMm = WHEEL_DIAMETER_MM,
};
