/*
 * SteeringGeometryCalibrationIterationTest.c
 *
 * Round-3 continuation of the self-propelled steering-geometry calibration.
 *
 * --------------------------------------------------------------------------
 * PURPOSE
 * --------------------------------------------------------------------------
 *
 * We seek, for each raw steering command u, a self-consistent curvature
 * kappa* satisfying
 *
 *     measuredCurvature(u, kappa*) = kappa*
 *
 * where the reference curvature is also used to generate the compatible
 * rear-wheel speed/path relationship.
 *
 * Define the fixed-point error
 *
 *     g(kappaRef)
 *         =
 *     measuredCurvature - kappaRef
 *
 * and solve
 *
 *     g(kappa*) = 0.
 *
 *
 * --------------------------------------------------------------------------
 * ROUND-2 RESUME STATE
 * --------------------------------------------------------------------------
 *
 * This file is seeded directly from the completed Round-2 export:
 *
 *     21 / 26 points converged
 *     5 / 26 points unresolved
 *
 * Unresolved:
 *
 *     increasing: +6, +8
 *     decreasing: -12, -10, -8
 *
 * Four of these five already have a persistent sign-changing root bracket:
 *
 *     INC +6
 *     INC +8
 *     DEC -10
 *     DEC -8
 *
 * DEC -12 remains unbracketed and is explored cautiously.
 *
 *
 * --------------------------------------------------------------------------
 * IMPROVED SOLVER
 * --------------------------------------------------------------------------
 *
 * Once a sign-changing bracket [a,b] is known:
 *
 *     g(a) * g(b) < 0
 *
 * the solver NEVER deliberately leaves that interval again.
 *
 * It first attempts a regula-falsi / bracketed-secant estimate. If that
 * estimate lies too close to either bracket endpoint, it falls back to
 * bisection to guarantee useful bracket contraction.
 *
 * Before a bracket exists, the solver uses a step-limited secant estimate
 * from the two most recent observations, falling back to ordinary
 * fixed-point iteration if necessary.
 *
 *
 * --------------------------------------------------------------------------
 * CONVERGENCE TEST
 * --------------------------------------------------------------------------
 *
 * A point is accepted when
 *
 *     |kappaMeasured - kappaReference|
 *
 * is less than
 *
 *     max(
 *         1.0e-5 /mm,
 *         10% * |kappaReference|
 *     ).
 *
 *
 * --------------------------------------------------------------------------
 * BRANCH / BACKLASH HANDLING
 * --------------------------------------------------------------------------
 *
 * Increasing and decreasing raw-command branches remain separate.
 * Every continuation round performs:
 *
 *     increasing unresolved points in increasing command order
 *     decreasing unresolved points in decreasing command order
 *
 * with the same branch preconditioning used in the original calibration.
 *
 * Between autonomous runs:
 *
 *     LIFT + REPOSITION the robot.
 *     DO NOT push or roll it back.
 *
 *
 * --------------------------------------------------------------------------
 * EXPERIMENTAL RECORD
 * --------------------------------------------------------------------------
 *
 * This source resumes after completed Rounds 1 and 2. The next local round
 * is therefore exported/displayed as Round 3.
 *
 * Record battery voltage and actual acquisition time separately with the
 * exported dataset.
 *
 * Created on: 2026年9月10日
 * Author: Joe
 */

#include "SteeringGeometryCalibrationIterationTest.h"

#include "RobotTestFixture.h"
#include "RobotKinematics.h"
#include "MotionProfile.h"

#include "userbutton.h"
#include "oled.h"
#include "oledutils.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>


/* -------------------------------------------------------------------------- */
/* Constants                                                                  */
/* -------------------------------------------------------------------------- */

#define ITER_PI_F                          (3.14159265358979323846f)
#define ITER_DEG_TO_RAD_F                  (ITER_PI_F / 180.0f)
#define ITER_RAD_TO_DEG_F                  (180.0f / ITER_PI_F)

#define ITER_CONTROL_PERIOD_MS             (10U)
#define ITER_CONTROL_PERIOD_S              (0.010f)

#define ITER_COMMAND_RATE_PER_SEC          (60.0f)

#define ITER_COMMAND_SETTLE_MS             (500U)
#define ITER_EXTREME_PRELOAD_HOLD_MS       (500U)
#define ITER_HANDS_OFF_MS                  (400U)

#define ITER_TARGET_TRAVEL_MM              (1000.0f)
#define ITER_MAX_CENTRE_SPEED_CPS          (2000.0f)

#define ITER_ACCELERATION_MMPS2            (500.0f)
#define ITER_DECELERATION_MMPS2            (250.0f)

#define ITER_SYNC_KP_CPS_PER_MM            (10.0f)
#define ITER_SYNC_MAX_CORRECTION_CPS       (100.0f)

#define ITER_RUN_TIMEOUT_MS                (12000U)
#define ITER_BRAKE_TIMEOUT_MS              (3000U)
#define ITER_STOP_STABLE_SAMPLES           (3U)

#define ITER_MIN_VALID_TRAVEL_MM           (900.0f)


/*
 * Fixed-point convergence.
 */
#define ITER_CURVATURE_ABS_TOL_PER_MM      (1.0e-5f)
#define ITER_CURVATURE_REL_TOL             (0.10f)


/*
 * Bracketed solver.
 */
#define ITER_BRACKET_EDGE_FRACTION         (0.10f)
#define ITER_SECANT_DENOM_EPS              (1.0e-9f)


/*
 * Unbracketed exploration.
 */
#define ITER_UNBRACKETED_MAX_STEP_PER_MM   (7.5e-5f)

#define ITER_MAX_ABS_CURVATURE_PER_MM      (3.0e-4f)


/*
 * Rounds 1 and 2 have already been completed.
 *
 * localRoundIndex = 0 therefore corresponds to experimental Round 3.
 */
#define ITER_BASE_ROUND_INDEX              (2U)


static const float iterCommands[
    STEERING_GEOMETRY_ITER_POINT_COUNT] =
{
    -12.0f,
    -10.0f,
     -8.0f,
     -6.0f,
     -4.0f,
     -2.0f,
      0.0f,
      2.0f,
      4.0f,
      6.0f,
      8.0f,
     10.0f,
     12.0f
};


/*
 * State after completed Round 2.
 *
 * [0] = increasing branch
 * [1] = decreasing branch
 *
 * command index:
 *
 *     0  -> -12
 *     1  -> -10
 *     ...
 *     12 -> +12
 *
 * For converged points this is the accepted candidate curvature.
 * For the five unresolved points this is simply the latest measured
 * curvature and remains provisional until convergence.
 */
static const float iterSeedFinalCurvaturePerMm[
    STEERING_GEOMETRY_ITER_SWEEP_COUNT]
    [STEERING_GEOMETRY_ITER_POINT_COUNT] =
{
    {
         1.1382051200e-04f,
         1.2114202400e-04f,
         1.1709809000e-04f,
         1.0106927900e-04f,
         9.5048737400e-05f,
         9.2171794700e-05f,
         7.7178374300e-05f,
         6.0395435900e-05f,
         4.0091137600e-05f,
        -5.6800981800e-05f,
        -1.5815632700e-04f,
        -2.2088270600e-04f,
        -2.5384547200e-04f
    },

    {
         1.0937401400e-04f,
         3.1824984000e-05f,
        -4.3640830000e-05f,
        -1.0631712900e-04f,
        -1.6247868200e-04f,
        -2.0257076500e-04f,
        -2.3045034300e-04f,
        -2.6653669200e-04f,
        -2.6470673000e-04f,
        -2.6124599400e-04f,
        -2.8161623000e-04f,
        -3.0828072300e-04f,
        -3.0603102600e-04f
    }
};


static const uint8_t iterSeedConverged[
    STEERING_GEOMETRY_ITER_SWEEP_COUNT]
    [STEERING_GEOMETRY_ITER_POINT_COUNT] =
{
    {
        1U, 1U, 1U, 1U, 1U, 1U, 1U,
        1U, 1U,
        0U, 0U,
        1U, 1U
    },

    {
        0U, 0U, 0U,
        1U, 1U, 1U, 1U, 1U, 1U,
        1U, 1U, 1U, 1U
    }
};


static const uint8_t iterSeedIterationCount[
    STEERING_GEOMETRY_ITER_SWEEP_COUNT]
    [STEERING_GEOMETRY_ITER_POINT_COUNT] =
{
    {
        0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U,
        2U, 2U,
        1U, 1U
    },

    {
        2U, 2U, 2U,
        1U, 1U, 1U, 1U,
        2U, 2U,
        1U, 1U,
        2U, 2U
    }
};


/* -------------------------------------------------------------------------- */
/* Debugger-visible exports                                                   */
/* -------------------------------------------------------------------------- */

volatile SteeringGeometryIterationPointState
    g_steeringGeometryIterationPointState[
        STEERING_GEOMETRY_ITER_SWEEP_COUNT]
        [STEERING_GEOMETRY_ITER_POINT_COUNT];


volatile float
    g_steeringGeometryIterationCurvaturePerMm[
        STEERING_GEOMETRY_ITER_SWEEP_COUNT]
        [STEERING_GEOMETRY_ITER_POINT_COUNT];


volatile float
    g_steeringGeometryIterationAngleRad[
        STEERING_GEOMETRY_ITER_SWEEP_COUNT]
        [STEERING_GEOMETRY_ITER_POINT_COUNT];


volatile uint8_t
    g_steeringGeometryIterationConverged[
        STEERING_GEOMETRY_ITER_SWEEP_COUNT]
        [STEERING_GEOMETRY_ITER_POINT_COUNT];


volatile uint8_t
    g_steeringGeometryIterationCount[
        STEERING_GEOMETRY_ITER_SWEEP_COUNT]
        [STEERING_GEOMETRY_ITER_POINT_COUNT];


volatile SteeringGeometryIterationResult
    g_steeringGeometryIterationResults[
        STEERING_GEOMETRY_ITER_MAX_RESULT_COUNT];


volatile uint32_t
    g_steeringGeometryIterationResultCount = 0U;


volatile uint32_t
    g_steeringGeometryIterationUnresolvedCount = 0U;


volatile uint32_t
    g_steeringGeometryIterationCompletedRounds = 0U;


volatile char g_steeringGeometryIterationInfo[] =
    "Round-3 steering fixed-point continuation; "
    "seeded from completed Round-2 state (21/26 converged, 5 unresolved); "
    "1000mm @ max 2000CPS; accel=500, decel=250, Ksync=10; "
    "convergence=max(1e-5/mm,10% reference); "
    "persistent bracketed secant/bisection once sign change exists; "
    "cautious capped secant exploration while unbracketed";


/* -------------------------------------------------------------------------- */
/* Generic helpers                                                            */
/* -------------------------------------------------------------------------- */

static float SteeringIter_Clamp(
    float value,
    float minValue,
    float maxValue)
{
    if (value > maxValue)
    {
        return maxValue;
    }

    if (value < minValue)
    {
        return minValue;
    }

    return value;
}


static uint8_t SteeringIter_GetSweepSlot(
    int8_t sweepDirection)
{
    return
        (sweepDirection ==
         STEERING_GEOMETRY_ITER_INCREASING)
            ? 0U
            : 1U;
}


static const char *SteeringIter_GetSweepName(
    int8_t sweepDirection)
{
    return
        (sweepDirection ==
         STEERING_GEOMETRY_ITER_INCREASING)
            ? "INC"
            : "DEC";
}


static float SteeringIter_GetMmPerCount(
    const RobotTestFixture *fixture)
{
    const RobotKinematics *kinematics =
        fixture->motionController.kinematics;

    return
        ITER_PI_F *
        kinematics->rearWheelDiameterMm /
        (float)kinematics->rearEncoderCountsPerRev;
}


/* -------------------------------------------------------------------------- */
/* Fixed-point convergence / update                                           */
/* -------------------------------------------------------------------------- */

static float SteeringIter_GetTolerance(
    float referenceCurvaturePerMm)
{
    float relativeTolerance =
        ITER_CURVATURE_REL_TOL *
        fabsf(referenceCurvaturePerMm);

    return
        fmaxf(
            ITER_CURVATURE_ABS_TOL_PER_MM,
            relativeTolerance);
}


static bool SteeringIter_IsConverged(
    float referenceCurvaturePerMm,
    float measuredCurvaturePerMm)
{
    float error =
        measuredCurvaturePerMm -
        referenceCurvaturePerMm;

    return
        fabsf(error) <=
        SteeringIter_GetTolerance(
            referenceCurvaturePerMm);
}


static bool SteeringIter_HaveOppositeSigns(
    float a,
    float b)
{
    return
        ((a < 0.0f) && (b > 0.0f)) ||
        ((a > 0.0f) && (b < 0.0f));
}


static void SteeringIter_SetBracket(
    SteeringGeometryIterationPointState *state,
    float referenceA,
    float errorA,
    float referenceB,
    float errorB)
{
    if (referenceA <= referenceB)
    {
        state->bracketLowReferencePerMm =
            referenceA;

        state->bracketLowErrorPerMm =
            errorA;

        state->bracketHighReferencePerMm =
            referenceB;

        state->bracketHighErrorPerMm =
            errorB;
    }
    else
    {
        state->bracketLowReferencePerMm =
            referenceB;

        state->bracketLowErrorPerMm =
            errorB;

        state->bracketHighReferencePerMm =
            referenceA;

        state->bracketHighErrorPerMm =
            errorA;
    }


    state->hasBracket =
        1U;
}


/*
 * Tighten an existing sign-changing bracket using a newly sampled
 * reference/error pair.
 *
 * All bracketed solver references are generated inside the current
 * bracket, so the new point should replace whichever endpoint has the
 * same error sign.
 */
static void SteeringIter_UpdateBracket(
    SteeringGeometryIterationPointState *state,
    float reference,
    float error)
{
    if (!state->hasBracket)
    {
        return;
    }


    if (error == 0.0f)
    {
        state->bracketLowReferencePerMm =
            reference;

        state->bracketLowErrorPerMm =
            0.0f;

        state->bracketHighReferencePerMm =
            reference;

        state->bracketHighErrorPerMm =
            0.0f;

        return;
    }


    if (((error > 0.0f) &&
         (state->bracketLowErrorPerMm > 0.0f)) ||
        ((error < 0.0f) &&
         (state->bracketLowErrorPerMm < 0.0f)))
    {
        state->bracketLowReferencePerMm =
            reference;

        state->bracketLowErrorPerMm =
            error;

        return;
    }


    state->bracketHighReferencePerMm =
        reference;

    state->bracketHighErrorPerMm =
        error;
}


/*
 * Compute the next reference while strictly remaining inside an
 * established sign-changing bracket.
 *
 * First attempt a regula-falsi / bracketed-secant estimate. If that
 * estimate is invalid or too near either endpoint, use the midpoint.
 */
static float SteeringIter_ComputeBracketedReference(
    const SteeringGeometryIterationPointState *state,
    uint8_t *updateMethod)
{
    float lowReference =
        state->bracketLowReferencePerMm;

    float lowError =
        state->bracketLowErrorPerMm;

    float highReference =
        state->bracketHighReferencePerMm;

    float highError =
        state->bracketHighErrorPerMm;


    float midpoint =
        0.5f *
        (lowReference +
         highReference);


    float denominator =
        highError -
        lowError;


    if (fabsf(denominator) <=
        ITER_SECANT_DENOM_EPS)
    {
        *updateMethod =
            STEERING_GEOMETRY_ITER_UPDATE_BISECTION;

        return midpoint;
    }


    float candidate =
        highReference -
        highError *
        (highReference -
         lowReference) /
        denominator;


    float width =
        highReference -
        lowReference;


    float innerLow =
        lowReference +
        ITER_BRACKET_EDGE_FRACTION *
        width;


    float innerHigh =
        highReference -
        ITER_BRACKET_EDGE_FRACTION *
        width;


    if (!isfinite(candidate) ||
        candidate <= innerLow ||
        candidate >= innerHigh)
    {
        *updateMethod =
            STEERING_GEOMETRY_ITER_UPDATE_BISECTION;

        return midpoint;
    }


    *updateMethod =
        STEERING_GEOMETRY_ITER_UPDATE_BRACKET_SECANT;


    return candidate;
}


/*
 * Unbracketed exploration.
 *
 * Try secant extrapolation from the two most recent observations.
 * If that is unavailable, use ordinary fixed-point iteration:
 *
 *     kappaNext = kappaMeasured.
 *
 * Every unbracketed step is limited.
 */
static float SteeringIter_ComputeUnbracketedReference(
    float previousReferenceCurvaturePerMm,
    float previousErrorCurvaturePerMm,
    float currentReferenceCurvaturePerMm,
    float currentErrorCurvaturePerMm,
    uint8_t *updateMethod)
{
    float candidate =
        currentReferenceCurvaturePerMm +
        currentErrorCurvaturePerMm;


    uint8_t method =
        STEERING_GEOMETRY_ITER_UPDATE_UNBRACKETED_FIXED_POINT;


    float denominator =
        currentErrorCurvaturePerMm -
        previousErrorCurvaturePerMm;


    if ((fabsf(denominator) >
         ITER_SECANT_DENOM_EPS) &&
        (fabsf(
            currentReferenceCurvaturePerMm -
            previousReferenceCurvaturePerMm) >
         1.0e-12f))
    {
        float secantCandidate =
            currentReferenceCurvaturePerMm -
            currentErrorCurvaturePerMm *
            (currentReferenceCurvaturePerMm -
             previousReferenceCurvaturePerMm) /
            denominator;


        if (isfinite(secantCandidate))
        {
            candidate =
                secantCandidate;

            method =
                STEERING_GEOMETRY_ITER_UPDATE_UNBRACKETED_SECANT;
        }
    }


    float requestedStep =
        candidate -
        currentReferenceCurvaturePerMm;


    if (fabsf(requestedStep) >
        ITER_UNBRACKETED_MAX_STEP_PER_MM)
    {
        candidate =
            currentReferenceCurvaturePerMm +
            copysignf(
                ITER_UNBRACKETED_MAX_STEP_PER_MM,
                requestedStep);


        if (method ==
            STEERING_GEOMETRY_ITER_UPDATE_UNBRACKETED_SECANT)
        {
            method =
                STEERING_GEOMETRY_ITER_UPDATE_UNBRACKETED_SECANT_CAPPED;
        }
        else
        {
            method =
                STEERING_GEOMETRY_ITER_UPDATE_UNBRACKETED_FIXED_POINT_CAPPED;
        }
    }


    candidate =
        SteeringIter_Clamp(
            candidate,
            -ITER_MAX_ABS_CURVATURE_PER_MM,
            +ITER_MAX_ABS_CURVATURE_PER_MM);


    *updateMethod =
        method;


    return candidate;
}


static float SteeringIter_ComputeNextReference(
    const SteeringGeometryIterationPointState *state,
    uint8_t *updateMethod)
{
    if (state->hasBracket)
    {
        return
            SteeringIter_ComputeBracketedReference(
                state,
                updateMethod);
    }


    return
        SteeringIter_ComputeUnbracketedReference(
            state->previousReferenceCurvaturePerMm,
            state->previousErrorCurvaturePerMm,
            state->currentReferenceCurvaturePerMm,
            state->currentErrorCurvaturePerMm,
            updateMethod);
}


/* -------------------------------------------------------------------------- */
/* Export-state initialisation                                                */
/* -------------------------------------------------------------------------- */

static void SteeringIter_SeedBracketedPoint(
    uint8_t sweep,
    uint8_t point,

    float previousReference,
    float previousError,

    float currentReference,
    float currentMeasured,

    float bracketReferenceA,
    float bracketErrorA,

    float bracketReferenceB,
    float bracketErrorB)
{
    SteeringGeometryIterationPointState state =
        g_steeringGeometryIterationPointState[
            sweep][point];


    state.previousReferenceCurvaturePerMm =
        previousReference;

    state.previousErrorCurvaturePerMm =
        previousError;


    state.currentReferenceCurvaturePerMm =
        currentReference;

    state.currentMeasuredCurvaturePerMm =
        currentMeasured;

    state.currentErrorCurvaturePerMm =
        currentMeasured -
        currentReference;


    SteeringIter_SetBracket(
        &state,
        bracketReferenceA,
        bracketErrorA,
        bracketReferenceB,
        bracketErrorB);


    state.tolerancePerMm =
        SteeringIter_GetTolerance(
            currentReference);


    state.converged =
        0U;


    state.unbracketedAttemptCount =
        0U;


    uint8_t method =
        STEERING_GEOMETRY_ITER_UPDATE_NONE;


    state.nextReferenceCurvaturePerMm =
        SteeringIter_ComputeNextReference(
            &state,
            &method);


    state.lastUpdateMethod =
        method;


    g_steeringGeometryIterationPointState[
        sweep][point] =
            state;
}


static void SteeringIter_SeedUnbracketedPoint(
    uint8_t sweep,
    uint8_t point,

    float previousReference,
    float previousError,

    float currentReference,
    float currentMeasured)
{
    SteeringGeometryIterationPointState state =
        g_steeringGeometryIterationPointState[
            sweep][point];


    state.previousReferenceCurvaturePerMm =
        previousReference;

    state.previousErrorCurvaturePerMm =
        previousError;


    state.currentReferenceCurvaturePerMm =
        currentReference;

    state.currentMeasuredCurvaturePerMm =
        currentMeasured;

    state.currentErrorCurvaturePerMm =
        currentMeasured -
        currentReference;


    state.hasBracket =
        0U;


    state.tolerancePerMm =
        SteeringIter_GetTolerance(
            currentReference);


    state.converged =
        0U;


    /*
     * Two continuation probes have already been performed at this
     * endpoint in Rounds 1 and 2.
     */
    state.unbracketedAttemptCount =
        2U;


    uint8_t method =
        STEERING_GEOMETRY_ITER_UPDATE_NONE;


    state.nextReferenceCurvaturePerMm =
        SteeringIter_ComputeNextReference(
            &state,
            &method);


    state.lastUpdateMethod =
        method;


    g_steeringGeometryIterationPointState[
        sweep][point] =
            state;
}


static void SteeringIter_ResetExports(void)
{
    g_steeringGeometryIterationResultCount =
        0U;


    /*
     * Experimental Rounds 1 and 2 are already complete.
     */
    g_steeringGeometryIterationCompletedRounds =
        2U;


    g_steeringGeometryIterationUnresolvedCount =
        0U;


    for (uint32_t i = 0U;
         i <
            STEERING_GEOMETRY_ITER_MAX_RESULT_COUNT;
         i++)
    {
        g_steeringGeometryIterationResults[i] =
            (SteeringGeometryIterationResult){0};
    }


    /*
     * Baseline state for all 26 points from the completed Round-2
     * candidate table.
     */
    for (uint32_t sweep = 0U;
         sweep <
            STEERING_GEOMETRY_ITER_SWEEP_COUNT;
         sweep++)
    {
        int8_t direction =
            (sweep == 0U)
                ? STEERING_GEOMETRY_ITER_INCREASING
                : STEERING_GEOMETRY_ITER_DECREASING;


        for (uint32_t point = 0U;
             point <
                STEERING_GEOMETRY_ITER_POINT_COUNT;
             point++)
        {
            SteeringGeometryIterationPointState state =
                {0};


            float finalCurvature =
                iterSeedFinalCurvaturePerMm[
                    sweep][point];


            state.steeringCommand =
                iterCommands[point];


            state.sweepDirection =
                direction;


            state.commandIndex =
                (uint8_t)point;


            state.finalCurvaturePerMm =
                finalCurvature;


            /*
             * Updated after fixture initialisation, when wheelbase is
             * available from the current RobotKinematics object.
             */
            state.finalEffectiveAngleRad =
                0.0f;


            state.converged =
                iterSeedConverged[
                    sweep][point];


            state.continuationIterationCount =
                iterSeedIterationCount[
                    sweep][point];


            state.previousReferenceCurvaturePerMm =
                finalCurvature;

            state.previousErrorCurvaturePerMm =
                0.0f;


            state.currentReferenceCurvaturePerMm =
                finalCurvature;

            state.currentMeasuredCurvaturePerMm =
                finalCurvature;

            state.currentErrorCurvaturePerMm =
                0.0f;


            state.nextReferenceCurvaturePerMm =
                finalCurvature;


            state.tolerancePerMm =
                SteeringIter_GetTolerance(
                    finalCurvature);


            state.lastUpdateMethod =
                STEERING_GEOMETRY_ITER_UPDATE_NONE;


            g_steeringGeometryIterationPointState[
                sweep][point] =
                    state;


            if (!state.converged)
            {
                g_steeringGeometryIterationUnresolvedCount++;
            }
        }
    }


    /*
     * --------------------------------------------------------------
     * INC +6
     *
     * g(-5.95317142e-5) = +2.51623751e-5
     * g(-3.80802339e-5) = -1.87207479e-5
     * --------------------------------------------------------------
     */
    SteeringIter_SeedBracketedPoint(
        0U,
        9U,

        -5.95317142e-5f,
        +2.51623751e-5f,

        -3.80802339e-5f,
        -5.68009818e-5f,

        -5.95317142e-5f,
        +2.51623751e-5f,

        -3.80802339e-5f,
        -1.87207479e-5f);


    /*
     * --------------------------------------------------------------
     * INC +8
     *
     * g(-1.34227972e-4) = +1.65043457e-5
     * g(-1.11741152e-4) = -4.64151744e-5
     * --------------------------------------------------------------
     */
    SteeringIter_SeedBracketedPoint(
        0U,
        10U,

        -1.34227972e-4f,
        +1.65043457e-5f,

        -1.11741152e-4f,
        -1.58156327e-4f,

        -1.34227972e-4f,
        +1.65043457e-5f,

        -1.11741152e-4f,
        -4.64151744e-5f);


    /*
     * --------------------------------------------------------------
     * DEC -10
     *
     * Older refined observation:
     *
     *     ref = +1.23456248e-5
     *     g   = -1.47297069e-5
     *
     * Round 2:
     *
     *     ref = +8.50182005e-6
     *     g   = +2.33231640e-5
     * --------------------------------------------------------------
     */
    SteeringIter_SeedBracketedPoint(
        1U,
        1U,

        +1.23456248e-5f,
        -1.47297069e-5f,

        +8.50182005e-6f,
        +3.18249840e-5f,

        +8.50182005e-6f,
        +2.33231640e-5f,

        +1.23456248e-5f,
        -1.47297069e-5f);


    /*
     * --------------------------------------------------------------
     * DEC -8
     *
     * Older refined observation:
     *
     *     ref = -4.26169972e-5
     *     g   = -2.16327790e-5
     *
     * Round 2:
     *
     *     ref = -6.10171919e-5
     *     g   = +1.73763619e-5
     * --------------------------------------------------------------
     */
    SteeringIter_SeedBracketedPoint(
        1U,
        2U,

        -4.26169972e-5f,
        -2.16327790e-5f,

        -6.10171919e-5f,
        -4.36408300e-5f,

        -6.10171919e-5f,
        +1.73763619e-5f,

        -4.26169972e-5f,
        -2.16327790e-5f);


    /*
     * --------------------------------------------------------------
     * DEC -12
     *
     * Still unbracketed after Round 2.
     *
     * Round 1:
     *
     *     ref = +1.06706095e-4
     *     g   = +2.20616057e-5
     *
     * Round 2:
     *
     *     ref = +6.25828834e-5
     *     g   = +4.67911304e-5
     *
     * Both errors are positive.
     * --------------------------------------------------------------
     */
    SteeringIter_SeedUnbracketedPoint(
        1U,
        0U,

        +1.06706095e-4f,
        +2.20616057e-5f,

        +6.25828834e-5f,
        +1.09374014e-4f);
}


/*
 * Must be called after fixture initialisation because effective steering
 * angle uses the current physical wheelbase.
 */
static void SteeringIter_RefreshFinalTables(
    const RobotTestFixture *fixture)
{
    const RobotKinematics *kinematics =
        fixture->motionController.kinematics;


    for (uint32_t sweep = 0U;
         sweep <
            STEERING_GEOMETRY_ITER_SWEEP_COUNT;
         sweep++)
    {
        for (uint32_t point = 0U;
             point <
                STEERING_GEOMETRY_ITER_POINT_COUNT;
             point++)
        {
            SteeringGeometryIterationPointState state =
                g_steeringGeometryIterationPointState[
                    sweep][point];


            state.finalEffectiveAngleRad =
                atanf(
                    kinematics->wheelbaseMm *
                    state.finalCurvaturePerMm);


            g_steeringGeometryIterationPointState[
                sweep][point] =
                    state;


            g_steeringGeometryIterationCurvaturePerMm[
                sweep][point] =
                    state.finalCurvaturePerMm;


            g_steeringGeometryIterationAngleRad[
                sweep][point] =
                    state.finalEffectiveAngleRad;


            g_steeringGeometryIterationConverged[
                sweep][point] =
                    state.converged;


            g_steeringGeometryIterationCount[
                sweep][point] =
                    state.continuationIterationCount;
        }
    }
}


/* -------------------------------------------------------------------------- */
/* OLED                                                                       */
/* -------------------------------------------------------------------------- */

static void SteeringIter_ShowMessage(
    const char *line1,
    const char *line2)
{
    OLED_Clear();

    OLED_Printf(
        0, 0,
        "STEER ITER CAL");

    if (line1 != NULL)
    {
        OLED_Printf(
            0, 2,
            "%s",
            line1);
    }

    if (line2 != NULL)
    {
        OLED_Printf(
            0, 3,
            "%s",
            line2);
    }

    OLED_Refresh_Gram();
}


static void SteeringIter_ShowReady(
    uint32_t roundIndex,
    int8_t sweepDirection,
    uint32_t commandIndex)
{
    OLED_Clear();

    OLED_Printf(
        0, 0,
        "ITER R%lu %s",
        (unsigned long)(roundIndex + 1U),
        SteeringIter_GetSweepName(
            sweepDirection));

    OLED_Printf(
        0, 1,
        "CMD %+.1f",
        iterCommands[commandIndex]);

    OLED_Printf(
        0, 2,
        "LEFT:%lu",
        (unsigned long)
            g_steeringGeometryIterationUnresolvedCount);

    OLED_Printf(
        0, 3,
        "LIFT+REPOSITION");

    OLED_Printf(
        0, 4,
        "DO NOT PUSH BACK");

    OLED_Printf(
        0, 5,
        "SW1 = RUN");

    OLED_Refresh_Gram();
}


static void SteeringIter_ShowResult(
    const SteeringGeometryIterationResult *result)
{
    OLED_Clear();

    OLED_Printf(
        0, 0,
        "R%u %s U%+.0f",
        (unsigned int)(result->roundIndex + 1U),
        SteeringIter_GetSweepName(
            result->sweepDirection),
        result->steeringCommand);

    OLED_Printf(
        0, 1,
        "ref:%+.2e",
        result->referenceCurvaturePerMm);

    OLED_Printf(
        0, 2,
        "act:%+.2e",
        result->measuredCurvaturePerMm);

    OLED_Printf(
        0, 3,
        "err:%+.2e",
        result->curvatureErrorPerMm);

    OLED_Printf(
        0, 4,
        "yaw:%+.2f",
        result->yawGyroDeg);

    OLED_Printf(
        0, 5,
        "%s",
        result->converged ?
            "CONVERGED" :
            "CONTINUE");

    OLED_Refresh_Gram();
}


/* -------------------------------------------------------------------------- */
/* Steering branch preparation                                               */
/* -------------------------------------------------------------------------- */

static void SteeringIter_SlewRawCommand(
    RobotTestFixture *fixture,
    float targetCommand)
{
    float currentCommand =
        SteeringController_GetCommand(
            &fixture->steeringController);


    const float maxDelta =
        ITER_COMMAND_RATE_PER_SEC *
        ITER_CONTROL_PERIOD_S;


    while (fabsf(
        targetCommand -
        currentCommand) > 0.0001f)
    {
        float delta =
            targetCommand -
            currentCommand;


        delta =
            SteeringIter_Clamp(
                delta,
                -maxDelta,
                +maxDelta);


        currentCommand +=
            delta;


        SteeringController_SetCommand(
            &fixture->steeringController,
            currentCommand);


        HAL_Delay(
            ITER_CONTROL_PERIOD_MS);
    }


    SteeringController_SetCommand(
        &fixture->steeringController,
        targetCommand);
}


static void SteeringIter_PreconditionSweep(
    RobotTestFixture *fixture,
    int8_t sweepDirection)
{
    if (sweepDirection ==
        STEERING_GEOMETRY_ITER_INCREASING)
    {
        SteeringIter_SlewRawCommand(
            fixture,
            +12.0f);

        HAL_Delay(
            ITER_EXTREME_PRELOAD_HOLD_MS);


        SteeringIter_SlewRawCommand(
            fixture,
            -12.0f);

        HAL_Delay(
            ITER_EXTREME_PRELOAD_HOLD_MS);
    }
    else
    {
        SteeringIter_SlewRawCommand(
            fixture,
            -12.0f);

        HAL_Delay(
            ITER_EXTREME_PRELOAD_HOLD_MS);


        SteeringIter_SlewRawCommand(
            fixture,
            +12.0f);

        HAL_Delay(
            ITER_EXTREME_PRELOAD_HOLD_MS);
    }
}


static void SteeringIter_ApproachAndSettle(
    RobotTestFixture *fixture,
    float targetCommand)
{
    SteeringIter_SlewRawCommand(
        fixture,
        targetCommand);

    HAL_Delay(
        ITER_COMMAND_SETTLE_MS);
}


/* -------------------------------------------------------------------------- */
/* Odometry + IMU                                                             */
/* -------------------------------------------------------------------------- */

typedef struct
{
    int16_t previousLeftEncoder;
    int16_t previousRightEncoder;

    float leftTravelMm;
    float rightTravelMm;

    float centreTravelMm;
    float yawDeg;

} SteeringIterMeasurementState;


static void SteeringIter_ResetMeasurement(
    RobotTestFixture *fixture,
    SteeringIterMeasurementState *state)
{
    *state =
        (SteeringIterMeasurementState){0};


    state->previousLeftEncoder =
        DCMotor_GetEncoderCount(
            fixture->motionController.leftWheel->motor);


    state->previousRightEncoder =
        DCMotor_GetEncoderCount(
            fixture->motionController.rightWheel->motor);
}


static void SteeringIter_UpdateOdometry(
    RobotTestFixture *fixture,
    SteeringIterMeasurementState *state,
    float mmPerCount)
{
    int16_t currentLeft =
        DCMotor_GetEncoderCount(
            fixture->motionController.leftWheel->motor);


    int16_t currentRight =
        DCMotor_GetEncoderCount(
            fixture->motionController.rightWheel->motor);


    int16_t deltaLeft =
        (int16_t)(
            (uint16_t)currentLeft -
            (uint16_t)state->previousLeftEncoder);


    int16_t deltaRight =
        (int16_t)(
            (uint16_t)currentRight -
            (uint16_t)state->previousRightEncoder);


    state->previousLeftEncoder =
        currentLeft;


    state->previousRightEncoder =
        currentRight;


    state->leftTravelMm +=
        (float)deltaLeft *
        mmPerCount;


    state->rightTravelMm +=
        (float)deltaRight *
        mmPerCount;


    state->centreTravelMm =
        0.5f *
        (state->leftTravelMm +
         state->rightTravelMm);
}


static bool SteeringIter_UpdateYaw(
    RobotTestFixture *fixture,
    SteeringIterMeasurementState *state,
    float dt)
{
    ICM20948Measurement measurement = {0};


    if (!ICM20948_ReadMeasurement(
            fixture->motionController.imu,
            &measurement))
    {
        return false;
    }


    state->yawDeg +=
        measurement.gyroDps.z *
        dt;


    return true;
}


/* -------------------------------------------------------------------------- */
/* Local rear-wheel relationship controller                                   */
/* -------------------------------------------------------------------------- */

static void SteeringIter_SetRearWheelTargets(
    RobotTestFixture *fixture,
    float centreSpeedCps,
    float referenceCurvaturePerMm,
    float desiredWheelTravelDifferenceMm,
    float actualWheelTravelDifferenceMm,
    float *wheelSyncErrorMm)
{
    const RobotKinematics *kinematics =
        fixture->motionController.kinematics;


    WheelSpeedController *leftWheel =
        fixture->motionController.leftWheel;


    WheelSpeedController *rightWheel =
        fixture->motionController.rightWheel;


    float leftBaseTargetCps;
    float rightBaseTargetCps;


    RobotKinematics_GetRearWheelSpeedTargets(
        kinematics,
        centreSpeedCps,
        referenceCurvaturePerMm,
        &leftBaseTargetCps,
        &rightBaseTargetCps);


    float errorMm =
        actualWheelTravelDifferenceMm -
        desiredWheelTravelDifferenceMm;


    float correctionCps =
        ITER_SYNC_KP_CPS_PER_MM *
        errorMm;


    float smallestBaseMagnitudeCps =
        fminf(
            fabsf(leftBaseTargetCps),
            fabsf(rightBaseTargetCps));


    float correctionLimitCps =
        fminf(
            ITER_SYNC_MAX_CORRECTION_CPS,
            smallestBaseMagnitudeCps);


    correctionCps =
        SteeringIter_Clamp(
            correctionCps,
            -correctionLimitCps,
            +correctionLimitCps);


    WheelSpeedController_SetTarget(
        leftWheel,
        leftBaseTargetCps +
        correctionCps);


    WheelSpeedController_SetTarget(
        rightWheel,
        rightBaseTargetCps -
        correctionCps);


    *wheelSyncErrorMm =
        errorMm;
}


/* -------------------------------------------------------------------------- */
/* Geometry result                                                            */
/* -------------------------------------------------------------------------- */

static void SteeringIter_ComputeGeometry(
    RobotTestFixture *fixture,
    SteeringGeometryIterationResult *result)
{
    const RobotKinematics *kinematics =
        fixture->motionController.kinematics;


    result->wheelTravelDifferenceMm =
        result->rightTravelMm -
        result->leftTravelMm;


    if (fabsf(
        result->travelledDistanceMm) <
        ITER_MIN_VALID_TRAVEL_MM)
    {
        return;
    }


    float yawGyroRad =
        result->yawGyroDeg *
        ITER_DEG_TO_RAD_F;


    result->measuredCurvaturePerMm =
        yawGyroRad /
        result->travelledDistanceMm;


    result->effectiveAngleGyroRad =
        atanf(
            kinematics->wheelbaseMm *
            result->measuredCurvaturePerMm);


    float yawEncoderRad =
        result->wheelTravelDifferenceMm /
        kinematics->rearTrackWidthMm;


    result->yawEncoderDeg =
        yawEncoderRad *
        ITER_RAD_TO_DEG_F;


    result->curvatureEncoderPerMm =
        yawEncoderRad /
        result->travelledDistanceMm;


    result->effectiveAngleEncoderRad =
        atanf(
            kinematics->wheelbaseMm *
            result->curvatureEncoderPerMm);
}


/* -------------------------------------------------------------------------- */
/* One autonomous continuation run                                            */
/* -------------------------------------------------------------------------- */

static void SteeringIter_RunOne(
    RobotTestFixture *fixture,
    uint32_t roundIndex,
    int8_t sweepDirection,
    uint32_t commandIndex,
    float referenceCurvaturePerMm,
    SteeringGeometryIterationResult *result)
{
    *result =
        (SteeringGeometryIterationResult){0};


    result->steeringCommand =
        iterCommands[commandIndex];


    result->sweepDirection =
        sweepDirection;


    result->commandIndex =
        (uint8_t)commandIndex;


    result->roundIndex =
        (uint8_t)roundIndex;


    result->referenceCurvaturePerMm =
        referenceCurvaturePerMm;


    SteeringIter_ApproachAndSettle(
        fixture,
        result->steeringCommand);


    WheelSpeedController_Stop(
        fixture->motionController.leftWheel);


    WheelSpeedController_Stop(
        fixture->motionController.rightWheel);


    HAL_Delay(
        ITER_HANDS_OFF_MS);


    SteeringIterMeasurementState measurement =
        {0};


    SteeringIter_ResetMeasurement(
        fixture,
        &measurement);


    /*
     * Prime the IMU transaction before t = 0.
     */
    ICM20948Measurement prime =
        {0};


    if (!ICM20948_ReadMeasurement(
            fixture->motionController.imu,
            &prime))
    {
        result->imuReadFailed =
            1U;

        return;
    }


    MotionProfile profile =
        {0};


    if (!MotionProfile_Init(
            &profile,
            ITER_ACCELERATION_MMPS2,
            ITER_DECELERATION_MMPS2))
    {
        result->profileFailed =
            1U;

        return;
    }


    const float mmPerCount =
        SteeringIter_GetMmPerCount(
            fixture);


    const float maxSpeedMmps =
        ITER_MAX_CENTRE_SPEED_CPS *
        mmPerCount;


    if (!MotionProfile_Start(
            &profile,
            ITER_TARGET_TRAVEL_MM,
            maxSpeedMmps))
    {
        result->profileFailed =
            1U;

        return;
    }


    uint32_t startTick =
        HAL_GetTick();


    uint32_t lastControlTick =
        startTick;


    result->startTickMs =
        startTick;


    float desiredWheelTravelDifferenceMm =
        0.0f;


    float wheelSyncErrorMm =
        0.0f;


    float maxAbsWheelSyncErrorMm =
        0.0f;


    bool measurementCaptured =
        false;


    bool braking =
        false;


    uint32_t stationarySamples =
        0U;


    uint32_t brakeStartTick =
        0U;


    while (true)
    {
        uint32_t now =
            HAL_GetTick();


        if (!braking &&
            ((now - startTick) >=
             ITER_RUN_TIMEOUT_MS))
        {
            result->timedOut =
                1U;


            braking =
                true;


            brakeStartTick =
                now;


            WheelSpeedController_SetTarget(
                fixture->motionController.leftWheel,
                0.0f);


            WheelSpeedController_SetTarget(
                fixture->motionController.rightWheel,
                0.0f);
        }


        if (braking &&
            ((now - brakeStartTick) >=
             ITER_BRAKE_TIMEOUT_MS))
        {
            break;
        }


        if (!braking &&
            SW1_ReadState() ==
                SW1_Enabled)
        {
            result->aborted =
                1U;


            braking =
                true;


            brakeStartTick =
                now;


            WheelSpeedController_SetTarget(
                fixture->motionController.leftWheel,
                0.0f);


            WheelSpeedController_SetTarget(
                fixture->motionController.rightWheel,
                0.0f);
        }


        if ((now - lastControlTick) <
            ITER_CONTROL_PERIOD_MS)
        {
            continue;
        }


        lastControlTick +=
            ITER_CONTROL_PERIOD_MS;


        float previousCentreTravelMm =
            measurement.centreTravelMm;


        SteeringIter_UpdateOdometry(
            fixture,
            &measurement,
            mmPerCount);


        float deltaCentreMm =
            measurement.centreTravelMm -
            previousCentreTravelMm;


        if (!SteeringIter_UpdateYaw(
                fixture,
                &measurement,
                ITER_CONTROL_PERIOD_S))
        {
            result->imuReadFailed =
                1U;


            if (!braking)
            {
                braking =
                    true;


                brakeStartTick =
                    now;


                WheelSpeedController_SetTarget(
                    fixture->motionController.leftWheel,
                    0.0f);


                WheelSpeedController_SetTarget(
                    fixture->motionController.rightWheel,
                    0.0f);
            }
        }


        result->sampleCount++;


        desiredWheelTravelDifferenceMm +=
            fixture->motionController.kinematics->
                rearTrackWidthMm *
            referenceCurvaturePerMm *
            deltaCentreMm;


        float actualWheelTravelDifferenceMm =
            measurement.rightTravelMm -
            measurement.leftTravelMm;


        if (!braking)
        {
            float targetSpeedMmps =
                MotionProfile_Update(
                    &profile,
                    fabsf(
                        measurement.centreTravelMm),
                    ITER_CONTROL_PERIOD_S);


            if (!MotionProfile_IsActive(
                    &profile))
            {
                result->travelledDistanceMm =
                    measurement.centreTravelMm;


                result->leftTravelMm =
                    measurement.leftTravelMm;


                result->rightTravelMm =
                    measurement.rightTravelMm;


                result->yawGyroDeg =
                    measurement.yawDeg;


                result->desiredWheelTravelDifferenceMm =
                    desiredWheelTravelDifferenceMm;


                result->wheelSyncErrorMm =
                    actualWheelTravelDifferenceMm -
                    desiredWheelTravelDifferenceMm;


                result->maxAbsWheelSyncErrorMm =
                    maxAbsWheelSyncErrorMm;


                SteeringIter_ComputeGeometry(
                    fixture,
                    result);


                measurementCaptured =
                    true;


                braking =
                    true;


                brakeStartTick =
                    now;


                WheelSpeedController_SetTarget(
                    fixture->motionController.leftWheel,
                    0.0f);


                WheelSpeedController_SetTarget(
                    fixture->motionController.rightWheel,
                    0.0f);
            }
            else
            {
                float centreSpeedCps =
                    targetSpeedMmps /
                    mmPerCount;


                SteeringIter_SetRearWheelTargets(
                    fixture,
                    centreSpeedCps,
                    referenceCurvaturePerMm,
                    desiredWheelTravelDifferenceMm,
                    actualWheelTravelDifferenceMm,
                    &wheelSyncErrorMm);


                float absSyncErrorMm =
                    fabsf(
                        wheelSyncErrorMm);


                if (absSyncErrorMm >
                    maxAbsWheelSyncErrorMm)
                {
                    maxAbsWheelSyncErrorMm =
                        absSyncErrorMm;
                }
            }
        }


        WheelSpeedController_Update(
            fixture->motionController.leftWheel,
            ITER_CONTROL_PERIOD_S);


        WheelSpeedController_Update(
            fixture->motionController.rightWheel,
            ITER_CONTROL_PERIOD_S);


        if (braking)
        {
            bool leftStationary =
                WheelSpeedController_IsStationary(
                    fixture->motionController.leftWheel);


            bool rightStationary =
                WheelSpeedController_IsStationary(
                    fixture->motionController.rightWheel);


            if (leftStationary &&
                rightStationary)
            {
                stationarySamples++;


                if (stationarySamples >=
                    ITER_STOP_STABLE_SAMPLES)
                {
                    break;
                }
            }
            else
            {
                stationarySamples =
                    0U;
            }
        }
    }


    WheelSpeedController_Stop(
        fixture->motionController.leftWheel);


    WheelSpeedController_Stop(
        fixture->motionController.rightWheel);


    result->durationMs =
        HAL_GetTick() -
        startTick;


    result->finalDistanceMm =
        measurement.centreTravelMm;


    result->finalYawDeg =
        measurement.yawDeg;


    if (!measurementCaptured)
    {
        result->travelledDistanceMm =
            measurement.centreTravelMm;


        result->leftTravelMm =
            measurement.leftTravelMm;


        result->rightTravelMm =
            measurement.rightTravelMm;


        result->yawGyroDeg =
            measurement.yawDeg;


        result->desiredWheelTravelDifferenceMm =
            desiredWheelTravelDifferenceMm;


        result->wheelSyncErrorMm =
            (measurement.rightTravelMm -
             measurement.leftTravelMm) -
            desiredWheelTravelDifferenceMm;


        result->maxAbsWheelSyncErrorMm =
            maxAbsWheelSyncErrorMm;


        SteeringIter_ComputeGeometry(
            fixture,
            result);
    }


    if (measurementCaptured &&
        !result->timedOut &&
        !result->aborted &&
        !result->imuReadFailed &&
        !result->profileFailed &&
        (fabsf(
            result->travelledDistanceMm) >=
         ITER_MIN_VALID_TRAVEL_MM))
    {
        result->valid =
            1U;
    }
}


/* -------------------------------------------------------------------------- */
/* Apply one run to its point-state fixed-point solver                        */
/* -------------------------------------------------------------------------- */

static void SteeringIter_UpdatePointState(
    RobotTestFixture *fixture,
    SteeringGeometryIterationResult *result)
{
    uint8_t sweepSlot =
        SteeringIter_GetSweepSlot(
            result->sweepDirection);


    uint32_t point =
        result->commandIndex;


    SteeringGeometryIterationPointState state =
        g_steeringGeometryIterationPointState[
            sweepSlot][point];


    /*
     * Preserve the current observation before installing the new one.
     */
    float oldReference =
        state.currentReferenceCurvaturePerMm;


    float oldError =
        state.currentErrorCurvaturePerMm;


    /*
     * Shift history.
     */
    state.previousReferenceCurvaturePerMm =
        oldReference;


    state.previousErrorCurvaturePerMm =
        oldError;


    /*
     * Install the new physical observation.
     */
    state.currentReferenceCurvaturePerMm =
        result->referenceCurvaturePerMm;


    state.currentMeasuredCurvaturePerMm =
        result->measuredCurvaturePerMm;


    state.currentErrorCurvaturePerMm =
        result->measuredCurvaturePerMm -
        result->referenceCurvaturePerMm;


    state.tolerancePerMm =
        SteeringIter_GetTolerance(
            result->referenceCurvaturePerMm);


    state.continuationIterationCount++;


    result->curvatureErrorPerMm =
        state.currentErrorCurvaturePerMm;


    result->tolerancePerMm =
        state.tolerancePerMm;


    bool converged =
        SteeringIter_IsConverged(
            result->referenceCurvaturePerMm,
            result->measuredCurvaturePerMm);


    if (converged)
    {
        state.converged =
            1U;


        state.finalCurvaturePerMm =
            result->measuredCurvaturePerMm;


        state.finalEffectiveAngleRad =
            atanf(
                fixture->motionController.kinematics->
                    wheelbaseMm *
                state.finalCurvaturePerMm);


        state.nextReferenceCurvaturePerMm =
            state.finalCurvaturePerMm;


        state.lastUpdateMethod =
            STEERING_GEOMETRY_ITER_UPDATE_NONE;


        result->converged =
            1U;


        result->nextReferenceCurvaturePerMm =
            state.nextReferenceCurvaturePerMm;


        result->updateMethod =
            STEERING_GEOMETRY_ITER_UPDATE_NONE;


        if (g_steeringGeometryIterationUnresolvedCount >
            0U)
        {
            g_steeringGeometryIterationUnresolvedCount--;
        }
    }
    else
    {
        /*
         * Maintain or establish a persistent root bracket.
         */
        if (state.hasBracket)
        {
            SteeringIter_UpdateBracket(
                &state,
                state.currentReferenceCurvaturePerMm,
                state.currentErrorCurvaturePerMm);
        }
        else
        {
            if (SteeringIter_HaveOppositeSigns(
                    oldError,
                    state.currentErrorCurvaturePerMm))
            {
                SteeringIter_SetBracket(
                    &state,

                    oldReference,
                    oldError,

                    state.currentReferenceCurvaturePerMm,
                    state.currentErrorCurvaturePerMm);
            }
            else
            {
                state.unbracketedAttemptCount++;
            }
        }


        /*
         * Latest measurement remains provisional until convergence.
         */
        state.finalCurvaturePerMm =
            result->measuredCurvaturePerMm;


        state.finalEffectiveAngleRad =
            atanf(
                fixture->motionController.kinematics->
                    wheelbaseMm *
                state.finalCurvaturePerMm);


        uint8_t method =
            STEERING_GEOMETRY_ITER_UPDATE_NONE;


        state.nextReferenceCurvaturePerMm =
            SteeringIter_ComputeNextReference(
                &state,
                &method);


        state.lastUpdateMethod =
            method;


        result->converged =
            0U;


        result->nextReferenceCurvaturePerMm =
            state.nextReferenceCurvaturePerMm;


        result->updateMethod =
            method;
    }


    g_steeringGeometryIterationPointState[
        sweepSlot][point] =
            state;


    SteeringIter_RefreshFinalTables(
        fixture);
}


/* -------------------------------------------------------------------------- */
/* Branch / round execution                                                   */
/* -------------------------------------------------------------------------- */

static bool SteeringIter_BranchHasUnresolved(
    int8_t sweepDirection)
{
    uint8_t sweepSlot =
        SteeringIter_GetSweepSlot(
            sweepDirection);


    for (uint32_t point = 0U;
         point <
            STEERING_GEOMETRY_ITER_POINT_COUNT;
         point++)
    {
        if (!g_steeringGeometryIterationPointState[
                sweepSlot][point].converged)
        {
            return true;
        }
    }


    return false;
}


static bool SteeringIter_RunBranch(
    RobotTestFixture *fixture,
    uint32_t roundIndex,
    int8_t sweepDirection)
{
    if (!SteeringIter_BranchHasUnresolved(
            sweepDirection))
    {
        return true;
    }


    SteeringIter_ShowMessage(
        SteeringIter_GetSweepName(
            sweepDirection),
        "PRECONDITION");


    SteeringIter_PreconditionSweep(
        fixture,
        sweepDirection);


    for (uint32_t ordinal = 0U;
         ordinal <
            STEERING_GEOMETRY_ITER_POINT_COUNT;
         ordinal++)
    {
        uint32_t commandIndex;


        if (sweepDirection ==
            STEERING_GEOMETRY_ITER_INCREASING)
        {
            commandIndex =
                ordinal;
        }
        else
        {
            commandIndex =
                (STEERING_GEOMETRY_ITER_POINT_COUNT -
                 1U) -
                ordinal;
        }


        uint8_t sweepSlot =
            SteeringIter_GetSweepSlot(
                sweepDirection);


        SteeringGeometryIterationPointState state =
            g_steeringGeometryIterationPointState[
                sweepSlot][commandIndex];


        if (state.converged)
        {
            continue;
        }


        SteeringIter_ShowReady(
            roundIndex,
            sweepDirection,
            commandIndex);


        SW1_WaitForPressAndRelease();


        if (g_steeringGeometryIterationResultCount >=
            STEERING_GEOMETRY_ITER_MAX_RESULT_COUNT)
        {
            return false;
        }


        SteeringGeometryIterationResult result =
            {0};


        SteeringIter_RunOne(
            fixture,
            roundIndex,
            sweepDirection,
            commandIndex,
            state.nextReferenceCurvaturePerMm,
            &result);


        if (!result.valid)
        {
            g_steeringGeometryIterationResults[
                g_steeringGeometryIterationResultCount] =
                    result;


            g_steeringGeometryIterationResultCount++;


            SteeringIter_ShowResult(
                &result);


            return false;
        }


        SteeringIter_UpdatePointState(
            fixture,
            &result);


        g_steeringGeometryIterationResults[
            g_steeringGeometryIterationResultCount] =
                result;


        g_steeringGeometryIterationResultCount++;


        SteeringIter_ShowResult(
            &result);


        HAL_Delay(
            500U);
    }


    return true;
}


/* -------------------------------------------------------------------------- */
/* Hardware / fixture init                                                    */
/* -------------------------------------------------------------------------- */

static bool SteeringIter_Init(
    RobotTestFixture *fixture)
{
    OLED_Init();

    OLED_Clear();
    OLED_Refresh_Gram();


    /*
     * Heading controller is not used.
     *
     * MotionController itself does not execute the calibration motion;
     * the fixture is reused for the tested peripheral/controller objects.
     */
    if (!RobotTestFixture_InitMotionController(
            fixture,

            0.0f,
            0.0f,
            0.0f,
            0.010f,

            ITER_SYNC_KP_CPS_PER_MM,
            ITER_SYNC_MAX_CORRECTION_CPS,

            ITER_ACCELERATION_MMPS2,
            ITER_DECELERATION_MMPS2))
    {
        return false;
    }


    WheelSpeedController_Stop(
        fixture->motionController.leftWheel);


    WheelSpeedController_Stop(
        fixture->motionController.rightWheel);


    return true;
}


/* -------------------------------------------------------------------------- */
/* Public test                                                                */
/* -------------------------------------------------------------------------- */

void SteeringGeometryCalibrationIterationTestRun(void)
{
    RobotTestFixture fixture =
        {0};


    volatile char infoKeepAlive =
        g_steeringGeometryIterationInfo[0];


    (void)infoKeepAlive;


    SteeringIter_ResetExports();


    SteeringIter_ShowMessage(
        "KEEP ROBOT STILL",
        "INITIALISING");


    HAL_Delay(
        1000U);


    if (!SteeringIter_Init(
            &fixture))
    {
        SteeringIter_ShowMessage(
            "INIT FAILED",
            NULL);


        SW1_WhileNotPressed();

        return;
    }


    SteeringIter_RefreshFinalTables(
        &fixture);


    OLED_Clear();

    OLED_Printf(
        0, 0,
        "ITER CAL READY");

    OLED_Printf(
        0, 1,
        "UNRES:%lu",
        (unsigned long)
            g_steeringGeometryIterationUnresolvedCount);

    OLED_Printf(
        0, 3,
        "MAX ROUNDS:%u",
        (unsigned int)
            STEERING_GEOMETRY_ITER_MAX_ROUNDS);

    OLED_Printf(
        0, 5,
        "SW1 = BEGIN");

    OLED_Refresh_Gram();


    SW1_WaitForPressAndRelease();


    for (uint32_t localRoundIndex = 0U;
         localRoundIndex <
            STEERING_GEOMETRY_ITER_MAX_ROUNDS;
         localRoundIndex++)
    {
        uint32_t roundIndex =
            ITER_BASE_ROUND_INDEX +
            localRoundIndex;
        if (g_steeringGeometryIterationUnresolvedCount ==
            0U)
        {
            break;
        }


        if (!SteeringIter_RunBranch(
                &fixture,
                roundIndex,
                STEERING_GEOMETRY_ITER_INCREASING))
        {
            goto iteration_failed;
        }


        if (!SteeringIter_RunBranch(
                &fixture,
                roundIndex,
                STEERING_GEOMETRY_ITER_DECREASING))
        {
            goto iteration_failed;
        }


        g_steeringGeometryIterationCompletedRounds =
            roundIndex +
            1U;


        if (g_steeringGeometryIterationUnresolvedCount ==
            0U)
        {
            break;
        }


        OLED_Clear();

        OLED_Printf(
            0, 0,
            "ROUND %lu DONE",
            (unsigned long)(roundIndex + 1U));

        OLED_Printf(
            0, 1,
            "UNRES:%lu",
            (unsigned long)
                g_steeringGeometryIterationUnresolvedCount);

        OLED_Printf(
            0, 3,
            "EXPORT IF WANTED");

        OLED_Printf(
            0, 5,
            "SW1 NEXT ROUND");

        OLED_Refresh_Gram();


        SW1_WaitForPressAndRelease();
    }


    WheelSpeedController_Stop(
        fixture.motionController.leftWheel);


    WheelSpeedController_Stop(
        fixture.motionController.rightWheel);


    SteeringController_Centre(
        &fixture.steeringController);


    SteeringIter_RefreshFinalTables(
        &fixture);


    OLED_Clear();

    OLED_Printf(
        0, 0,
        "ITER CAL DONE");

    OLED_Printf(
        0, 1,
        "RUNS:%lu",
        (unsigned long)
            g_steeringGeometryIterationResultCount);

    OLED_Printf(
        0, 2,
        "UNRES:%lu",
        (unsigned long)
            g_steeringGeometryIterationUnresolvedCount);

    OLED_Printf(
        0, 4,
        "EXPORT RESULTS");

    OLED_Printf(
        0, 5,
        "CHECK CONVERGED");

    OLED_Refresh_Gram();


    SW1_WhileNotPressed();

    return;


iteration_failed:

    WheelSpeedController_Stop(
        fixture.motionController.leftWheel);


    WheelSpeedController_Stop(
        fixture.motionController.rightWheel);


    SteeringController_Centre(
        &fixture.steeringController);


    SteeringIter_RefreshFinalTables(
        &fixture);


    OLED_Clear();

    OLED_Printf(
        0, 0,
        "ITER CAL STOP");

    OLED_Printf(
        0, 1,
        "RUNS:%lu",
        (unsigned long)
            g_steeringGeometryIterationResultCount);

    OLED_Printf(
        0, 2,
        "UNRES:%lu",
        (unsigned long)
            g_steeringGeometryIterationUnresolvedCount);

    OLED_Printf(
        0, 4,
        "EXPORT PARTIAL");

    OLED_Refresh_Gram();


    SW1_WhileNotPressed();
}
