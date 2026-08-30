/*
 * WheelSpeedControllerConfig.h
 *
 *  Created on: 2026年8月30日
 *      Author: Joe
 */

#ifndef INC_WHEELSPEEDCONTROLLERCONFIG_H_
#define INC_WHEELSPEEDCONTROLLERCONFIG_H_

#include "WheelSpeedController.h"


/*
 * Calibration obtained from the motor response tests.
 * We provide a pre-calibrated model here, but consumers
 * may provide their own using the exposed struct.
 */
extern const WheelSpeedCalibration leftCalibration;
extern const WheelSpeedCalibration rightCalibration;

#endif /* INC_WHEELSPEEDCONTROLLERCONFIG_H_ */
