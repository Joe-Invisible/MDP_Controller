/*
 * RobotKinematics.h
 *
 * Robot geometry, bicycle steering model;
 * An experimentally tuned instance is provided in MotionControllerConfig.h
 *
 *  Created on: 2026年8月30日
 *      Author: Joe
 */

#ifndef INC_ROBOTKINEMATICS_H_
#define INC_ROBOTKINEMATICS_H_

#include <stdint.h>

typedef struct RobotKinematics {
	uint16_t 	rearEncoderCountsPerRev;
	float		rearWheelDiameterMm;
	float		wheelbaseMm;
	float		rearTrackWidthMm;
} RobotKinematics;

/**
 * Bicycle-model curvature from effective steering angle.
 *
 * kappa = tan(delta) / L
 *
 * Units: 1/mm
 */
float RobotKinematics_GetCurvaturePerMm(
    const RobotKinematics *kinematics,
    float steeringAngleRad);


/**
 * Effective steering angle required for a given curvature.
 *
 * delta = atan(L * kappa)
 *
 * Units: radians
 */
float RobotKinematics_GetSteeringAngleRad(
    const RobotKinematics *kinematics,
    float curvaturePerMm);


/**
 * Calculate rear-wheel speed targets for a desired
 * rear-axle-centre speed and path curvature.
 *
 * vL = v * (1 - W*kappa/2)
 * vR = v * (1 + W*kappa/2)
 */
void RobotKinematics_GetRearWheelSpeedTargets(
    const RobotKinematics *kinematics,
    float centreSpeedCps,
    float curvaturePerMm,
    float *leftSpeedCps,
    float *rightSpeedCps);


#endif /* INC_ROBOTKINEMATICS_H_ */
