/*
 * SteeringGeometryCalibrationTest.h
 *
 * Focused steering-geometry calibration for the positive-curvature
 * operating region used by the constant-radius arc test.
 *
 * Created on: 2026年9月2日
 * Updated on: 2026年9月9日
 * Author: Joe
 */

#ifndef INC_STEERINGGEOMETRYCALIBRATIONTEST_H_
#define INC_STEERINGGEOMETRYCALIBRATIONTEST_H_

#include <stdint.h>

typedef struct SteeringGeometryCalibrationResult
{
    /*
     * Raw command passed to SteeringController_SetCommand().
     */
    float steeringCommand;

    /*
     * Raw-command approach direction.
     *
     * +1 = increasing command
     * -1 = decreasing command
     *
     * The focused arc calibration deliberately uses -1 for
     * every run to reproduce the branch used after deterministic
     * centering when commanding positive curvature.
     */
    int8_t sweepDirection;

    /*
     * Explicit focused-test indexing.
     */
    uint8_t commandIndex;
    uint8_t repeatIndex;

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


/*
 * Five focused commands, three repeats each.
 */
#define STEERING_GEOMETRY_CAL_RESULT_COUNT 15U


extern volatile SteeringGeometryCalibrationResult
    g_steeringGeometryCalibrationResults[
        STEERING_GEOMETRY_CAL_RESULT_COUNT];

extern volatile uint32_t
    g_steeringGeometryCalibrationResultCount;


void SteeringGeometryCalibrationTestRun(void);


#endif /* INC_STEERINGGEOMETRYCALIBRATIONTEST_H_ */
