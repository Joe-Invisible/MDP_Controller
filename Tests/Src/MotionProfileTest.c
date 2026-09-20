/*
 * MotionProfileTest.c
 *
 * Created on: 2026年9月7日
 * Author: Joe
 */

#include "MotionProfileTest.h"

#include "MotionProfile.h"
#include "MotionController.h" // for tolerance definiton

#include <math.h>
#include <stdbool.h>
#include <stdint.h>


#define TEST_DT_SEC                 0.01f

#define TEST_ACCELERATION_MMPS2     500.0f
#define TEST_DECELERATION_MMPS2     500.0f
#define TEST_MAX_SPEED_MMPS         300.0f

#define TRAPEZOID_DISTANCE_MM       1000.0f
#define TRIANGLE_DISTANCE_MM        100.0f

#define TRAPEZOID_MAX_SAMPLES       512U
#define TRIANGLE_MAX_SAMPLES        256U

#define SPEED_EPSILON_MMPS          0.01f
#define DISTANCE_TOLERANCE_MM       0.5f

#define TEST_COMPLETION_TOLERANCE_MM (0.5)

typedef struct {
    float timeSec;
    float travelledDistanceMm;
    float remainingDistanceMm;
    float targetSpeedMmps;

} MotionProfileDebugSample;


typedef struct
{
    bool initPassed;
    bool startPassed;

    bool completed;
    bool speedLimitPassed;
    bool expectedShapePassed;
    bool finalDistancePassed;

    bool passed;

    uint32_t sampleCount;
    uint32_t maxSpeedSampleCount;

    float peakSpeedMmps;
    float finalDistanceMm;

} MotionProfileTestResult;


/*
 * Kept as static arrays so they can be inspected/exported
 * using the debugger, similarly to the other controller tests.
 */
static MotionProfileDebugSample
    trapezoidSamples[TRAPEZOID_MAX_SAMPLES];

static MotionProfileDebugSample
    triangleSamples[TRIANGLE_MAX_SAMPLES];

static MotionProfileTestResult trapezoidResult;
static MotionProfileTestResult triangleResult;


static void MotionProfileTest_RunCase(
    float targetDistanceMm,
    bool expectCruise,
    MotionProfileDebugSample *samples,
    uint32_t sampleCapacity,
    MotionProfileTestResult *result)
{
    MotionProfile profile = {0};

    float travelledDistanceMm = 0.0f;
    float timeSec = 0.0f;

    *result = (MotionProfileTestResult){0};

    result->speedLimitPassed = true;


    result->initPassed = MotionProfile_Init(
        &profile,
        TEST_ACCELERATION_MMPS2,
        TEST_DECELERATION_MMPS2,
		TEST_COMPLETION_TOLERANCE_MM);

    if (!result->initPassed)
        return;


    result->startPassed = MotionProfile_Start(
        &profile,
        targetDistanceMm,
        TEST_MAX_SPEED_MMPS);

    if (!result->startPassed)
        return;


    for (uint32_t i = 0; i < sampleCapacity; i++)
    {
        float targetSpeedMmps =
            MotionProfile_Update(
                &profile,
                travelledDistanceMm,
                TEST_DT_SEC);


        /*
         * Save the current state before advancing the idealized
         * simulated vehicle.
         */
        samples[i].timeSec = timeSec;

        samples[i].travelledDistanceMm =
            travelledDistanceMm;

        samples[i].remainingDistanceMm =
            targetDistanceMm - travelledDistanceMm;

        samples[i].targetSpeedMmps =
            targetSpeedMmps;


        result->sampleCount = i + 1U;


        if (targetSpeedMmps > result->peakSpeedMmps)
        {
            result->peakSpeedMmps =
                targetSpeedMmps;
        }


        if (fabsf(
                targetSpeedMmps -
                TEST_MAX_SPEED_MMPS)
            <= SPEED_EPSILON_MMPS)
        {
            result->maxSpeedSampleCount++;
        }


        if (targetSpeedMmps >
            TEST_MAX_SPEED_MMPS +
            SPEED_EPSILON_MMPS)
        {
            result->speedLimitPassed = false;
        }


        if (!MotionProfile_IsActive(&profile))
        {
            result->completed = true;
            break;
        }


        /*
         * Idealized plant:
         *
         * Assume the vehicle follows the generated reference
         * perfectly over this control interval.
         */
        travelledDistanceMm +=
            targetSpeedMmps *
            TEST_DT_SEC;

        timeSec += TEST_DT_SEC;
    }


    result->finalDistanceMm =
        travelledDistanceMm;


    /*
     * A trapezoidal profile must spend at least two samples
     * at the configured maximum speed.
     *
     * A triangular profile must never reach that speed.
     */
    if (expectCruise)
    {
        result->expectedShapePassed =
            result->maxSpeedSampleCount >= 2U;
    }
    else
    {
        result->expectedShapePassed =
            result->maxSpeedSampleCount == 0U &&
            result->peakSpeedMmps <
                TEST_MAX_SPEED_MMPS -
                SPEED_EPSILON_MMPS;
    }


    result->finalDistancePassed =
        fabsf(
            result->finalDistanceMm -
            targetDistanceMm)
        <= DISTANCE_TOLERANCE_MM;


    result->passed =
        result->initPassed &&
        result->startPassed &&
        result->completed &&
        result->speedLimitPassed &&
        result->expectedShapePassed &&
        result->finalDistancePassed;
}


void MotionProfileTestRun(void)
{
    MotionProfileTest_RunCase(
        TRAPEZOID_DISTANCE_MM,
        true,
        trapezoidSamples,
        TRAPEZOID_MAX_SAMPLES,
        &trapezoidResult);


    MotionProfileTest_RunCase(
        TRIANGLE_DISTANCE_MM,
        false,
        triangleSamples,
        TRIANGLE_MAX_SAMPLES,
        &triangleResult);
}
