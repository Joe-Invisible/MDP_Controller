/*
 * SteeringGeometryCalibrationIterationTest.h
 *
 * Targeted iterative continuation of the full-range self-propelled
 * steering-geometry calibration.
 *
 * This version resumes from the completed Round-2 continuation state.
 * Twenty-one points are already accepted; only five remain unresolved.
 *
 * For each unresolved raw steering command, the test solves the fixed-point
 * condition
 *
 *     measuredCurvature(rawCommand, wheelCurvatureReference)
 *         =
 *     wheelCurvatureReference
 *
 * using a persistent bracketed root solver whenever a sign-changing
 * bracket is known, with cautious unbracketed secant exploration otherwise.
 *
 * Created on: 2026年9月10日
 * Author: Joe
 */

#ifndef INC_STEERINGGEOMETRYCALIBRATIONITERATIONTEST_H_
#define INC_STEERINGGEOMETRYCALIBRATIONITERATIONTEST_H_

#include <stdint.h>


#define STEERING_GEOMETRY_ITER_POINT_COUNT          (13U)
#define STEERING_GEOMETRY_ITER_SWEEP_COUNT          (2U)
#define STEERING_GEOMETRY_ITER_MAX_ROUNDS           (3U)

/*
 * Five points are unresolved in the Round-2 seed dataset.
 * At most five runs can occur per continuation round.
 */
#define STEERING_GEOMETRY_ITER_MAX_RESULT_COUNT     (15U)


typedef enum
{
    STEERING_GEOMETRY_ITER_INCREASING = 1,
    STEERING_GEOMETRY_ITER_DECREASING = -1

} SteeringGeometryIterationSweepDirection;


typedef enum
{
    STEERING_GEOMETRY_ITER_UPDATE_NONE = 0,

    /* Root is bracketed. */
    STEERING_GEOMETRY_ITER_UPDATE_BRACKET_SECANT = 1,
    STEERING_GEOMETRY_ITER_UPDATE_BISECTION = 2,

    /* No bracket exists yet. */
    STEERING_GEOMETRY_ITER_UPDATE_UNBRACKETED_SECANT = 3,
    STEERING_GEOMETRY_ITER_UPDATE_UNBRACKETED_SECANT_CAPPED = 4,
    STEERING_GEOMETRY_ITER_UPDATE_UNBRACKETED_FIXED_POINT = 5,
    STEERING_GEOMETRY_ITER_UPDATE_UNBRACKETED_FIXED_POINT_CAPPED = 6

} SteeringGeometryIterationUpdateMethod;


typedef struct
{
    /*
     * Point identity.
     */
    float steeringCommand;
    int8_t sweepDirection;
    uint8_t commandIndex;


    /*
     * Two most recent fixed-point observations.
     *
     * Error:
     *
     *     g(kappaRef)
     *         =
     *     measuredCurvature - referenceCurvature
     */
    float previousReferenceCurvaturePerMm;
    float previousErrorCurvaturePerMm;

    float currentReferenceCurvaturePerMm;
    float currentMeasuredCurvaturePerMm;
    float currentErrorCurvaturePerMm;


    /*
     * Persistent root bracket.
     *
     * When hasBracket != 0:
     *
     *     bracketLowReferencePerMm
     *         <=
     *     bracketHighReferencePerMm
     *
     * and the endpoint errors have opposite signs.
     *
     * Once established, subsequent solver steps remain inside
     * this bracket.
     */
    float bracketLowReferencePerMm;
    float bracketLowErrorPerMm;

    float bracketHighReferencePerMm;
    float bracketHighErrorPerMm;

    uint8_t hasBracket;


    /*
     * Reference curvature to use for the next autonomous run.
     */
    float nextReferenceCurvaturePerMm;


    /*
     * Candidate final calibration quantity.
     *
     * Only trust this value as production calibration when
     * converged != 0.
     */
    float finalCurvaturePerMm;
    float finalEffectiveAngleRad;


    float tolerancePerMm;

    uint8_t converged;
    uint8_t continuationIterationCount;
    uint8_t lastUpdateMethod;
    uint8_t unbracketedAttemptCount;

} SteeringGeometryIterationPointState;


typedef struct
{
    /*
     * Experiment identity.
     */
    float steeringCommand;
    int8_t sweepDirection;
    uint8_t commandIndex;
    uint8_t roundIndex;
    uint8_t valid;


    /*
     * Fixed-point reference used for this autonomous run.
     */
    float referenceCurvaturePerMm;


    /*
     * Autonomous motion result captured before the braking tail.
     */
    float travelledDistanceMm;

    float leftTravelMm;
    float rightTravelMm;

    float wheelTravelDifferenceMm;

    float desiredWheelTravelDifferenceMm;
    float wheelSyncErrorMm;
    float maxAbsWheelSyncErrorMm;


    /*
     * Primary gyro geometry.
     */
    float yawGyroDeg;
    float measuredCurvaturePerMm;
    float effectiveAngleGyroRad;


    /*
     * Rear-wheel differential diagnostic.
     */
    float yawEncoderDeg;
    float curvatureEncoderPerMm;
    float effectiveAngleEncoderRad;


    /*
     * Fixed-point convergence information.
     */
    float curvatureErrorPerMm;
    float tolerancePerMm;

    float nextReferenceCurvaturePerMm;

    uint8_t converged;
    uint8_t updateMethod;


    /*
     * Final stationary state after braking.
     */
    float finalDistanceMm;
    float finalYawDeg;


    /*
     * Timing and failure diagnostics.
     */
    uint32_t startTickMs;
    uint32_t durationMs;
    uint32_t sampleCount;

    uint8_t timedOut;
    uint8_t aborted;
    uint8_t imuReadFailed;
    uint8_t profileFailed;

} SteeringGeometryIterationResult;


/*
 * [0] = increasing-command branch
 * [1] = decreasing-command branch
 *
 * commandIndex:
 *
 *     0  -> -12
 *     1  -> -10
 *     ...
 *     12 -> +12
 */
extern volatile SteeringGeometryIterationPointState
    g_steeringGeometryIterationPointState[
        STEERING_GEOMETRY_ITER_SWEEP_COUNT]
        [STEERING_GEOMETRY_ITER_POINT_COUNT];


/*
 * Candidate replacement calibration tables.
 *
 * A table entry is production-usable only if the corresponding
 * g_steeringGeometryIterationConverged[][] entry is nonzero.
 */
extern volatile float
    g_steeringGeometryIterationCurvaturePerMm[
        STEERING_GEOMETRY_ITER_SWEEP_COUNT]
        [STEERING_GEOMETRY_ITER_POINT_COUNT];

extern volatile float
    g_steeringGeometryIterationAngleRad[
        STEERING_GEOMETRY_ITER_SWEEP_COUNT]
        [STEERING_GEOMETRY_ITER_POINT_COUNT];

extern volatile uint8_t
    g_steeringGeometryIterationConverged[
        STEERING_GEOMETRY_ITER_SWEEP_COUNT]
        [STEERING_GEOMETRY_ITER_POINT_COUNT];

extern volatile uint8_t
    g_steeringGeometryIterationCount[
        STEERING_GEOMETRY_ITER_SWEEP_COUNT]
        [STEERING_GEOMETRY_ITER_POINT_COUNT];


/*
 * Chronological continuation-run records.
 */
extern volatile SteeringGeometryIterationResult
    g_steeringGeometryIterationResults[
        STEERING_GEOMETRY_ITER_MAX_RESULT_COUNT];

extern volatile uint32_t
    g_steeringGeometryIterationResultCount;

extern volatile uint32_t
    g_steeringGeometryIterationUnresolvedCount;

extern volatile uint32_t
    g_steeringGeometryIterationCompletedRounds;


/*
 * Human-readable configuration string for debugger export.
 */
extern volatile char
    g_steeringGeometryIterationInfo[];


void SteeringGeometryCalibrationIterationTestRun(void);


#endif /* INC_STEERINGGEOMETRYCALIBRATIONITERATIONTEST_H_ */
