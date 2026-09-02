/*
 * RobotKinematics.h
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

#endif /* INC_ROBOTKINEMATICS_H_ */
