/*
 * SteeringControllerConfig.h
 *
 *  Created on: 2026年9月3日
 *      Author: Joe
 */

#ifndef INC_STEERINGCONTROLLERCONFIG_H_
#define INC_STEERINGCONTROLLERCONFIG_H_

#include "SteeringController.h"

/*
 * Steering calibration selection.
 *
 * 0:
 *     Original 3 Sep hand-push / focused calibration.
 *
 * 1:
 *     10 Sep self-propelled fixed-point calibration.
 *
 * May also be overridden from the compiler command line with:
 *
 *     -DSTEERING_USE_FIXED_POINT_CALIBRATION=0
 */
#ifndef STEERING_USE_FIXED_POINT_CALIBRATION
#define STEERING_USE_FIXED_POINT_CALIBRATION    (0)
#endif

extern const SteeringControllerCalibration steeringCalibration;


#endif /* INC_STEERINGCONTROLLERCONFIG_H_ */
