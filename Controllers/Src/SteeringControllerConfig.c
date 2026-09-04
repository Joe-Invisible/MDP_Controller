/*
 * SteeringControllerConfig.c
 *
 * Effective bicycle steering-angle calibration.
 *
 * Calibration source:
 * SteeringGeometryTest_focused_11.36V
 * 2026-09-03.
 */

#include "SteeringControllerConfig.h"


/*
 * Focused calibration range.
 *
 * The straight-line heading controller should remain within
 * this range until a wider steering range has been calibrated
 * for the hysteresis model.
 */
#define STEERING_CAL_MIN_COMMAND      (-12.0f)
#define STEERING_CAL_MAX_COMMAND      ( 12.0f)


#define REVERSAL_DEADBAN_RAD			(0.0005f)

/*
 * Increasing-command branch:
 *
 *     -12 -> -10 -> ... -> +12
 *
 * Effective angles are gyro-derived.
 *
 * The measured -6 and -4 samples contained a small
 * non-monotonic inversion attributed to experimental noise:
 *
 *     -6 : 0.0328699648 rad
 *     -4 : 0.0377804004 rad
 *
 * Those two points are pooled to their mean so that the
 * physical model remains monotonic.
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


/*
 * Decreasing-command branch:
 *
 *     +12 -> +10 -> ... -> -12
 *
 * Stored in increasing command order for interpolation.
 *
 * The measured gyro-derived branch was already monotonic.
 */
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

    .minCommand =
        STEERING_CAL_MIN_COMMAND,

    .maxCommand =
        STEERING_CAL_MAX_COMMAND,

	.reversalDeadbandRad =
		REVERSAL_DEADBAN_RAD,
};
