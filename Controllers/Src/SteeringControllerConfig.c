/*
 * SteeringControllerConfig.c
 *
 * Effective bicycle steering-angle calibration.
 *
 * Two calibration sets are retained:
 *
 * STEERING_USE_FIXED_POINT_CALIBRATION == 0
 *     Original focused steering calibration, 3 Sep 2026.
 *
 * STEERING_USE_FIXED_POINT_CALIBRATION == 1
 *     Self-propelled fixed-point steering calibration,
 *     completed 10 Sep 2026.
 *
 * Author: Joe
 */

#include "SteeringControllerConfig.h"


/*
 * --------------------------------------------------------------------------
 * Raw servo-command envelope
 * --------------------------------------------------------------------------
 *
 * These are NOT the interpolation limits of either hysteresis branch.
 *
 * They define the raw command range that SteeringController is permitted
 * to send to Servo_SetSteering().
 *
 * This wider range is retained because commands outside a branch's
 * precision interpolation table may still be used for deterministic
 * hysteresis preconditioning.
 */
#define STEERING_CAL_MIN_COMMAND			(-12.0f)
#define STEERING_CAL_MAX_COMMAND			(+12.0f)


/*
 * Minimum effective-angle reversal required before switching to the
 * opposite major hysteresis branch.
 */
#define REVERSAL_DEADBAND_RAD           (0.0005f)


/*
 * Maximum raw Servo_SetSteering() command slew rate.
 *
 * Units: raw command units / second.
 */
#define MAX_COMMAND_RATE                (60.0f)



#if STEERING_USE_FIXED_POINT_CALIBRATION


/*
 * ==========================================================================
 * NEW SELF-PROPELLED FIXED-POINT CALIBRATION
 * ==========================================================================
 *
 * Calibration campaign completed 10 Sep 2026.
 *
 * Raw command -> effective bicycle steering angle.
 *
 * Only monotonic precision segments are installed in the inverse model.
 *
 * Excluded points are not necessarily mechanically unusable; they are
 * excluded because they do not belong to the current one-to-one precision
 * interpolation segment.
 */


/*
 * Increasing-command branch.
 *
 * Raw -12 was excluded because the endpoint was non-monotonic:
 *
 *     raw -12 -> +0.0165024772 rad
 *     raw -10 -> +0.0175637864 rad
 *
 * Therefore raw -10 is currently the positive-angle endpoint of the
 * monotonic increasing-command precision segment.
 */
static const SteeringCalibrationPoint
increasingCalibration[] =
{
    { -10.0f, +0.0175637864f },
    {  -8.0f, +0.0169775914f },
    {  -6.0f, +0.0146539966f },
    {  -4.0f, +0.0137811946f },
    {  -2.0f, +0.0133641148f },
    {   0.0f, +0.0111903967f },
    {  +2.0f, +0.0087571144f },
    {  +4.0f, +0.0058131493f },
    {  +6.0f, -0.0075571588f },
    {  +8.0f, -0.0199762844f },
    { +10.0f, -0.0320170484f },
    { +12.0f, -0.0367909856f },
};


/*
 * Decreasing-command branch.
 *
 * The ordinary monotonic precision segment ends at raw +2.
 *
 * Raw -8 remained physically variable during fixed-point continuation.
 * The installed value is therefore the midpoint of the final tightly
 * localized fixed-point curvature bracket:
 *
 *     kappa in
 *       [-6.10171919e-5, -5.87630821e-5] /mm
 *
 * midpoint:
 *
 *     kappa = -5.98901370e-5 /mm
 *
 * with wheelbase L = 145 mm:
 *
 *     delta = atan(L * kappa)
 *           = -0.0086838516 rad
 *
 * Treat this point as lower-confidence than neighboring anchors.
 *
 * Commands +4 ... +12 are deliberately not part of this precision
 * inverse table because that region is non-monotonic. They remain inside
 * the raw servo envelope and can later form separately calibrated
 * extreme-steering segments.
 */
static const SteeringCalibrationPoint
decreasingCalibration[] =
{
    { -12.0f, +0.0184819587f },
    { -10.0f, +0.0019744604f },
    {  -8.0f, -0.0086838516f },
    {  -6.0f, -0.0154147632f },
    {  -4.0f, -0.0235550515f },
    {  -2.0f, -0.0293643177f },
    {   0.0f, -0.0334028713f },
    {  +2.0f, -0.0386285931f },
};


#else


/*
 * ==========================================================================
 * ORIGINAL 3 SEP CALIBRATION
 * ==========================================================================
 *
 * Retained as a compile-time rollback/reference calibration.
 */

static const SteeringCalibrationPoint
increasingCalibration[] =
{
    { -12.0f,  0.05356644f },
    { -10.0f,  0.04966983f },
    {  -8.0f,  0.04521477f },
    {  -6.0f,  0.03532518f },
    {  -4.0f,  0.03532518f },
    {  -2.0f,  0.02751669f },
    {   0.0f,  0.02261428f },
    {   2.0f,  0.01048893f },
    {   4.0f, -0.00086626f },
    {   6.0f, -0.02022749f },
    {   8.0f, -0.03733554f },
    {  10.0f, -0.04726548f },
    {  12.0f, -0.05163438f },
};


static const SteeringCalibrationPoint
decreasingCalibration[] =
{
    { -12.0f,  0.01613119f },
    { -10.0f,  0.00331176f },
    {  -8.0f, -0.01063315f },
    {  -6.0f, -0.02289677f },
    {  -4.0f, -0.03352704f },
    {  -2.0f, -0.04343720f },
    {   0.0f, -0.06432911f },
    {   2.0f, -0.07003503f },
    {   4.0f, -0.08549435f },
    {   6.0f, -0.09573198f },
    {   8.0f, -0.09997798f },
    {  10.0f, -0.10111032f },
    {  12.0f, -0.10240267f },
};


#endif


#define INCREASING_CAL_COUNT \
    ((uint32_t)(sizeof(increasingCalibration) / \
                sizeof(increasingCalibration[0])))

#define DECREASING_CAL_COUNT \
    ((uint32_t)(sizeof(decreasingCalibration) / \
                sizeof(decreasingCalibration[0])))


const SteeringControllerCalibration
steeringCalibration =
{
    .increasingPoints =
        increasingCalibration,

    .increasingPointCount =
        INCREASING_CAL_COUNT,

    .decreasingPoints =
        decreasingCalibration,

    .decreasingPointCount =
        DECREASING_CAL_COUNT,

    /*
     * Raw command envelope, NOT per-branch inverse-table extent.
     */
    .minCommand =
        STEERING_CAL_MIN_COMMAND,

    .maxCommand =
        STEERING_CAL_MAX_COMMAND,

    .reversalDeadbandRad =
        REVERSAL_DEADBAND_RAD,

    .maxCommandRatePerSec =
        MAX_COMMAND_RATE,
};
