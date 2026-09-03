/*
 * SteeringGeometryCalibrationTest.h
 *
 *  Created on: 2026年9月2日
 *      Author: Joe
 */

#ifndef INC_STEERINGGEOMETRYCALIBRATIONTEST_H_
#define INC_STEERINGGEOMETRYCALIBRATIONTEST_H_

#include <stdint.h>


typedef struct SteeringGeometryCalibrationResult
{
    /*
     * Command passed to Servo_SetSteering().
     */
    float steeringCommand;

    /*
     * +1 = ascending sweep
     * -1 = descending sweep
     */
    int8_t sweepDirection;

    /*
     * Timing / acquisition diagnostics.
     */
    uint32_t startTickMs;
    uint32_t durationMs;
    uint32_t sampleCount;

    /*
     * Rear-wheel odometry.
     */
    float leftTravelMm;
    float rightTravelMm;
    float centreTravelMm;

    /*
     * rightTravelMm - leftTravelMm
     */
    float distanceDifferenceMm;

    /*
     * Integrated gyro-Z heading change.
     */
    float yawGyroDeg;

    /*
     * Heading change inferred from rear-wheel odometry:
     *
     *     deltaPsi = (dR - dL) / W
     */
    float yawEncoderDeg;

    /*
     * Estimated rear-axle-centre path curvature.
     */
    float curvatureGyroPerMm;
    float curvatureEncoderPerMm;

    /*
     * Equivalent bicycle-model steering angle:
     *
     *     delta = atan(L * curvature)
     */
    float effectiveAngleGyroRad;
    float effectiveAngleEncoderRad;

    /*
     * Diagnostics.
     */
    uint8_t timedOut;
    uint8_t aborted;
    uint8_t imuReadFailed;
    uint8_t valid;

} SteeringGeometryCalibrationResult;


#define STEERING_GEOMETRY_CAL_RESULT_COUNT 26U


extern volatile SteeringGeometryCalibrationResult
    g_steeringGeometryCalibrationResults[
        STEERING_GEOMETRY_CAL_RESULT_COUNT];

extern volatile uint32_t
    g_steeringGeometryCalibrationResultCount;


void SteeringGeometryCalibrationTestRun(void);


#endif /* INC_STEERINGGEOMETRYCALIBRATIONTEST_H_ */
