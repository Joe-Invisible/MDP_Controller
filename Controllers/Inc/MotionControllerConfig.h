/*
 * MotionControllerConfig.h
 *
 *  Created on: 2026年8月30日
 *      Author: Joe
 */

#ifndef INC_MOTIONCONTROLLERCONFIG_H_
#define INC_MOTIONCONTROLLERCONFIG_H_

#include <stdint.h>

#include "RobotKinematics.h"

typedef struct
{
    float curvaturePerMm;
    float rawSteeringCommand;
} MotionControllerArcFeedforwardPoint;

/**
 * Empirical constant-curvature steering configuration.
 *
 * Negative and positive curvature are deliberately stored as separate
 * branches. Each branch must be ordered by strictly increasing curvature;
 * lookup interpolates only within one branch and never across zero.
 */
typedef struct
{
    const MotionControllerArcFeedforwardPoint *negativePoints;
    uint32_t negativePointCount;

    const MotionControllerArcFeedforwardPoint *positivePoints;
    uint32_t positivePointCount;

    /* Hold time after abrupt raw-command prepositioning. */
    float steeringSettlingTimeSec;
} MotionControllerArcConfig;

/**
 * Immutable tuning and geometry used by one MotionController instance.
 *
 * The referenced kinematics and arc calibration must remain valid for the
 * lifetime of the controller.
 */
typedef struct
{
    const RobotKinematics *kinematics;
    const MotionControllerArcConfig *arcConfig;

    float headingKp;
    float headingKi;
    float headingKd;
    float maxHeadingSteeringAngleRad;

    float arcYawRateKp;
    float arcYawRateKi;
    float arcYawRateKd;
    float maxArcSteeringCommandCorrection;

    float arcHeadingKpPerSec;

    float wheelSyncKpCpsPerMm;
    float maxWheelSyncCorrectionCps;

    float motionAccelerationMmps2;
    float motionDecelerationMmps2;
    float motionCompletionToleranceMm;

    float arcYawRateFilterTauSec;
} MotionControllerConfig;

extern const RobotKinematics kinematics;
extern const MotionControllerArcConfig arcMotionConfig;
extern const MotionControllerConfig motionControllerConfig;

#endif /* INC_MOTIONCONTROLLERCONFIG_H_ */
