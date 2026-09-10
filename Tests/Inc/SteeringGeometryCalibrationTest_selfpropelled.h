/*
 * SteeringGeometryCalibrationTest.h
 *
 * Full-range self-propelled steering-geometry calibration.
 *
 * The calibration measures:
 *
 *     raw servo command
 *         ->
 *     physical trajectory curvature
 *         ->
 *     effective bicycle-model steering angle
 *
 * It deliberately does NOT use the existing steering angle-to-command
 * calibration table to generate the steering command under test.
 *
 * Created on: 2026年9月2日
 * Redesigned on: 2026年9月9日
 * Author: Joe
 */

#ifndef INC_STEERINGGEOMETRYCALIBRATIONTEST_H_
#define INC_STEERINGGEOMETRYCALIBRATIONTEST_H_

#include <stdint.h>


#define STEERING_GEOMETRY_CAL_POINT_COUNT      (13U)
#define STEERING_GEOMETRY_CAL_SWEEP_COUNT      (2U)
#define STEERING_GEOMETRY_CAL_PHASE_COUNT      (2U)

#define STEERING_GEOMETRY_CAL_RESULT_COUNT \
    (STEERING_GEOMETRY_CAL_POINT_COUNT * \
     STEERING_GEOMETRY_CAL_SWEEP_COUNT * \
     STEERING_GEOMETRY_CAL_PHASE_COUNT)


typedef enum
{
    STEERING_GEOMETRY_CAL_SCOUT = 0,
    STEERING_GEOMETRY_CAL_REFINED = 1

} SteeringGeometryCalibrationPhase;


typedef enum
{
    STEERING_GEOMETRY_CAL_INCREASING = 1,
    STEERING_GEOMETRY_CAL_DECREASING = -1

} SteeringGeometryCalibrationSweepDirection;


typedef struct
{
    /*
     * Experiment identity.
     */
    float steeringCommand;

    int8_t sweepDirection;
    uint8_t phase;
    uint8_t commandIndex;
    uint8_t valid;


    /*
     * Rear-wheel curvature reference used for propulsion.
     *
     * SCOUT:
     *     0 /mm
     *     -> equal rear-wheel path relationship.
     *
     * REFINED:
     *     curvature measured by the corresponding SCOUT run.
     */
    float propulsionCurvaturePerMm;


    /*
     * Timing.
     */
    uint32_t startTickMs;
    uint32_t durationMs;
    uint32_t sampleCount;


    /*
     * State captured at the end of the measurement phase,
     * before the stopping/braking tail.
     */
    float travelledDistanceMm;

    float leftTravelMm;
    float rightTravelMm;

    float wheelTravelDifferenceMm;

    float desiredWheelTravelDifferenceMm;
    float wheelSyncErrorMm;
    float maxAbsWheelSyncErrorMm;


    /*
     * Gyro trajectory measurement.
     */
    float yawGyroDeg;

    float curvatureGyroPerMm;
    float effectiveAngleGyroRad;


    /*
     * Rear-wheel differential diagnostic.
     *
     * This is NOT the primary steering calibration quantity.
     */
    float yawEncoderDeg;

    float curvatureEncoderPerMm;
    float effectiveAngleEncoderRad;


    /*
     * Final stationary state after braking.
     *
     * Calibration uses the pre-braking values above.
     */
    float finalDistanceMm;
    float finalYawDeg;


    /*
     * Diagnostics.
     */
    uint8_t timedOut;
    uint8_t aborted;
    uint8_t imuReadFailed;
    uint8_t profileFailed;

} SteeringGeometryCalibrationResult;


/*
 * Results in chronological run order.
 *
 * Expected count after a complete experiment:
 *
 *     13 points
 *   x 2 sweep directions
 *   x 2 phases
 *   = 52 runs
 */
extern volatile SteeringGeometryCalibrationResult
    g_steeringGeometryCalibrationResults[
        STEERING_GEOMETRY_CAL_RESULT_COUNT];

extern volatile uint32_t
    g_steeringGeometryCalibrationResultCount;


/*
 * Convenience tables indexed as:
 *
 *     [0] = increasing-command sweep
 *     [1] = decreasing-command sweep
 *
 * and then by command index:
 *
 *     0  -> -12
 *     1  -> -10
 *     ...
 *     12 -> +12
 *
 * Scout curvature is used only to generate the refined rear-wheel
 * relationship.  Refined effective angle is the candidate data for
 * the replacement SteeringController calibration table.
 */
extern volatile float
    g_steeringGeometryCalibrationScoutCurvaturePerMm[
        STEERING_GEOMETRY_CAL_SWEEP_COUNT]
        [STEERING_GEOMETRY_CAL_POINT_COUNT];

extern volatile float
    g_steeringGeometryCalibrationRefinedAngleRad[
        STEERING_GEOMETRY_CAL_SWEEP_COUNT]
        [STEERING_GEOMETRY_CAL_POINT_COUNT];


/*
 * Human-readable description retained for debugger inspection.
 */
extern volatile char
    g_steeringGeometryCalibrationInfo[];


void SteeringGeometryCalibrationTestRun(void);


#endif /* INC_STEERINGGEOMETRYCALIBRATIONTEST_H_ */
