/*
 * WheelSpeedControllerConfig.h
 *
 *  Created on: 2026年8月30日
 *      Author: Joe
 */

#ifndef INC_WHEELSPEEDCONTROLLERCONFIG_H_
#define INC_WHEELSPEEDCONTROLLERCONFIG_H_

#include "WheelSpeedController.h"

/**
 * PID Controller parameters
 */
// These constants defines how much at most the PID controller may
// correct the model estimate
#define WHEELSPEEDCONTROLLER_MIN_FEEDBACK 	-30.0f
#define WHEELSPEEDCONTROLLER_MAX_FEEDBACK 	30.0f

#define WHEELSPEEDCONTROLLER_KP				0.02f
#define WHEELSPEEDCONTROLLER_KI				0.0f

/*
 * Calibration obtained from the motor response tests.
 * We provide a pre-calibrated model here, but consumers
 * may provide their own using the exposed struct.
 */
extern const WheelSpeedCalibration leftCalibration;
extern const WheelSpeedCalibration rightCalibration;

#endif /* INC_WHEELSPEEDCONTROLLERCONFIG_H_ */
