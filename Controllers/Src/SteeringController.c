/*
 * SteeringController.c
 *
 *  Created on: 2026年9月3日
 *      Author: Joe
 */

#include "SteeringController.h"

#include <math.h>
#include <stddef.h>

#define STEERING_ANGLE_MODEL_EPSILON_RAD 	(1.0e-6f)
#define STEERING_COMMAND_MODEL_EPSILON 		(1.0e-4f)

/* ------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------ */

static float SteeringController_Clamp(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}


static bool SteeringController_ValidateTable(
    const SteeringCalibrationPoint *points,
    uint32_t count)
{
    if (points == NULL ||
        count < 2U)
    {
        return false;
    }

    for (uint32_t i = 1U;
         i < count;
         i++)
    {
        /*
         * Commands must be strictly increasing.
         */
        if (points[i].command <=
            points[i - 1U].command)
        {
            return false;
        }

        /*
         * For this chassis, effective steering angle
         * decreases as steering command increases.
         *
         * Equal values are allowed because noisy calibration
         * points may have been pooled during smoothing.
         */
        if (points[i].effectiveAngleRad >
            points[i - 1U].effectiveAngleRad)
        {
            return false;
        }
    }

    return true;
}


static float SteeringController_Interpolate(
    const SteeringCalibrationPoint *points,
    uint32_t count,
    float command)
{
    if (command <= points[0].command)
    {
        return points[0].effectiveAngleRad;
    }

    if (command >= points[count - 1U].command)
    {
        return points[count - 1U].effectiveAngleRad;
    }

    for (uint32_t i = 0U;
         i < count - 1U;
         i++)
    {
        const SteeringCalibrationPoint *lower =
            &points[i];

        const SteeringCalibrationPoint *upper =
            &points[i + 1U];

        if (command <= upper->command)
        {
            float fraction =
                (command - lower->command) /
                (upper->command - lower->command);

            return
                lower->effectiveAngleRad +
                fraction *
                (upper->effectiveAngleRad -
                 lower->effectiveAngleRad);
        }
    }

    /*
     * Boundary checks above make this unreachable.
     */
    return points[count - 1U].effectiveAngleRad;
}


/*
 * Inverse of SteeringController_Interpolate().
 *
 * Calibration points are stored in increasing command order,
 * while effective angle is monotonically non-increasing.
 */
static float SteeringController_InterpolateCommand(
    const SteeringCalibrationPoint *points,
    uint32_t count,
    float effectiveAngleRad)
{
    /*
     * Highest effective angle occurs at the lowest command.
     */
    if (effectiveAngleRad >= points[0].effectiveAngleRad)
    {
        return points[0].command;
    }

    /*
     * Lowest effective angle occurs at the highest command.
     */
    if (effectiveAngleRad <=
        points[count - 1U].effectiveAngleRad)
    {
        return points[count - 1U].command;
    }

    for (uint32_t i = 0U;
         i < count - 1U;
         i++)
    {
        const SteeringCalibrationPoint *lowerCommand =
            &points[i];

        const SteeringCalibrationPoint *upperCommand =
            &points[i + 1U];

        /*
         * Angle decreases as command increases.
         *
         * The equality here deliberately chooses the first
         * command at which a flat calibration section reaches
         * the requested angle.
         */
        if (effectiveAngleRad >=
            upperCommand->effectiveAngleRad)
        {
            float angleSpan =
                upperCommand->effectiveAngleRad -
                lowerCommand->effectiveAngleRad;

            /*
             * A flat section can occur after smoothing.
             * Equality would normally have been caught by the
             * preceding segment; retain a safe fallback.
             */
            if (angleSpan == 0.0f)
            {
                return lowerCommand->command;
            }

            float fraction =
                (effectiveAngleRad -
                 lowerCommand->effectiveAngleRad) /
                angleSpan;

            return
                lowerCommand->command +
                fraction *
                (upperCommand->command -
                 lowerCommand->command);
        }
    }

    return points[count - 1U].command;
}


static const SteeringCalibrationPoint *
SteeringController_GetBranch(
    const SteeringController *controller,
    int8_t direction,
    uint32_t *pointCount)
{
    if (direction > 0)
    {
        *pointCount =
            controller->calibration->
                increasingPointCount;

        return
            controller->calibration->
                increasingPoints;
    }

    *pointCount =
        controller->calibration->
            decreasingPointCount;

    return
        controller->calibration->
            decreasingPoints;
}


/*
 * Update the effective steering-angle estimate.
 *
 * The two experimentally measured calibration branches form
 * a hysteresis loop.
 *
 * On reversal, the physical wheel angle is held until the
 * newly selected calibration branch reaches the previous
 * wheel angle. This models mechanical backlash without
 * producing an artificial instantaneous angle jump.
 */
static void SteeringController_UpdateModel(
    SteeringController *controller,
    float newCommand)
{
    float deltaCommand =
        newCommand -
        controller->command;

    int8_t newDirection;

    if (deltaCommand > 0.0f)
    {
        newDirection = +1;
    }
    else if (deltaCommand < 0.0f)
    {
        newDirection = -1;
    }
    else
    {
        /*
         * No actuator movement, therefore no change in
         * hysteresis state.
         */
        return;
    }

    /*
     * Initial movement or a command-direction reversal begins
     * backlash take-up.
     */
    if (controller->movementDirection == 0 ||
        newDirection != controller->movementDirection)
    {
        controller->movementDirection =
            newDirection;

        controller->backlashActive =
            true;

        controller->backlashHoldAngleRad =
            controller->effectiveAngleRad;
    }

    uint32_t pointCount = 0U;

    const SteeringCalibrationPoint *branch =
        SteeringController_GetBranch(
            controller,
            controller->movementDirection,
            &pointCount);

    float branchAngleRad =
        SteeringController_Interpolate(
            branch,
            pointCount,
            newCommand);

    if (controller->backlashActive)
    {
        bool backlashTakenUp;

        /*
         * Effective steering angle decreases as command
         * increases.
         *
         * Increasing command:
         *   wait for the increasing branch to fall to the
         *   held wheel angle.
         *
         * Decreasing command:
         *   wait for the decreasing branch to rise to the
         *   held wheel angle.
         */
        if (controller->movementDirection > 0)
        {
            backlashTakenUp =
                branchAngleRad <=
                controller->backlashHoldAngleRad +
				STEERING_ANGLE_MODEL_EPSILON_RAD;
        }
        else
        {
            backlashTakenUp =
                branchAngleRad >=
                controller->backlashHoldAngleRad +
				STEERING_ANGLE_MODEL_EPSILON_RAD;
        }

        if (!backlashTakenUp)
        {
            controller->effectiveAngleRad =
                controller->backlashHoldAngleRad;

            return;
        }

        controller->backlashActive =
            false;
    }

    controller->effectiveAngleRad =
        branchAngleRad;
}


/* ------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------ */

bool SteeringController_Init(
    SteeringController *controller,
    Servo *servo,
    const SteeringControllerCalibration *calibration)
{
    if (controller == NULL ||
        servo == NULL ||
        calibration == NULL)
    {
        return false;
    }

    if (calibration->minCommand >=
        calibration->maxCommand)
    {
        return false;
    }

    if (!SteeringController_ValidateTable(
            calibration->increasingPoints,
            calibration->increasingPointCount))
    {
        return false;
    }

    if (!SteeringController_ValidateTable(
            calibration->decreasingPoints,
            calibration->decreasingPointCount))
    {
        return false;
    }

    /*
     * The configured min/max command now define the overall raw actuator
     * envelope.
     *
     * Individual hysteresis branches are allowed to cover smaller,
     * asymmetric monotonic interpolation ranges.
     *
     * Every calibration point must, however, remain inside the permitted
     * raw servo-command envelope.
     */
    if (calibration->increasingPoints[0].command <
            calibration->minCommand ||
        calibration->increasingPoints[
            calibration->increasingPointCount - 1U
        ].command >
            calibration->maxCommand)
    {
        return false;
    }

    if (calibration->decreasingPoints[0].command <
            calibration->minCommand ||
        calibration->decreasingPoints[
            calibration->decreasingPointCount - 1U
        ].command >
            calibration->maxCommand)
    {
        return false;
    }

    if (calibration->reversalDeadbandRad < 0.0f ||
        calibration->maxCommandRatePerSec <= 0.0f)
    {
        return false;
    }

    float increasingMinAngleRad =
        calibration->increasingPoints[
            calibration->increasingPointCount - 1U
        ].effectiveAngleRad;

    float increasingMaxAngleRad =
        calibration->increasingPoints[0].
            effectiveAngleRad;

    float decreasingMinAngleRad =
        calibration->decreasingPoints[
            calibration->decreasingPointCount - 1U
        ].effectiveAngleRad;

    float decreasingMaxAngleRad =
        calibration->decreasingPoints[0].
            effectiveAngleRad;

    float minEffectiveAngleRad =
        fmaxf(
            increasingMinAngleRad,
            decreasingMinAngleRad);

    float maxEffectiveAngleRad =
        fminf(
            increasingMaxAngleRad,
            decreasingMaxAngleRad);

    if (minEffectiveAngleRad >
        maxEffectiveAngleRad)
    {
        /*
         * No angle exists that is controllable on both branches.
         */
        return false;
    }

    *controller =
        (SteeringController){0};

    controller->servo =
        servo;

    controller->calibration =
        calibration;

    controller->minEffectiveAngleRad =
        minEffectiveAngleRad;

    controller->maxEffectiveAngleRad =
        maxEffectiveAngleRad;

    controller->effectiveAngleRad = 0.0f;
    controller->targetEffectiveAngleRad = 0.0f;

    controller->reversalPending = false;

    controller->centreState = STEERING_CENTRE_IDLE;
    controller->centreCommand = 0.0f;

    /*
     * Establish the initial model reference.
     *
     * At system initialisation there is no useful linkage
     * history available, so nominal servo centre is defined as
     * zero effective bicycle steering angle.
     */
    Servo_Centre(
        controller->servo);

    controller->command =
        0.0f;

    controller->effectiveAngleRad =
        0.0f;

    controller->movementDirection =
        0;

    controller->backlashActive =
        false;

    controller->backlashHoldAngleRad =
        0.0f;

    return true;
}


static void SteeringController_ApplyCommand(
    SteeringController *controller,
    float command)
{
    /*
     * For now we deliberately keep the actuator inside the
     * experimentally calibrated model range.
     */
    float clampedCommand =
        SteeringController_Clamp(
            command,
            controller->calibration->minCommand,
            controller->calibration->maxCommand);

    /*
     * Update the linkage model before replacing the old
     * command, because UpdateModel() needs the command delta.
     */
    SteeringController_UpdateModel(
        controller,
        clampedCommand);

    controller->command =
        clampedCommand;

    Servo_SetSteering(
        controller->servo,
        clampedCommand);
}

static void SteeringController_ApplyCommandRateLimited(
    SteeringController *controller,
    float desiredCommand,
    float dt)
{
    if (controller == NULL ||
        controller->calibration == NULL ||
        dt <= 0.0f)
    {
        return;
    }

    desiredCommand =
        SteeringController_Clamp(
            desiredCommand,
            controller->calibration->minCommand,
            controller->calibration->maxCommand);

    float maxDeltaCommand =
        controller->calibration->maxCommandRatePerSec * dt;

    float deltaCommand =
        desiredCommand - controller->command;

    if (deltaCommand > maxDeltaCommand)
    {
        deltaCommand = maxDeltaCommand;
    }
    else if (deltaCommand < -maxDeltaCommand)
    {
        deltaCommand = -maxDeltaCommand;
    }

    SteeringController_ApplyCommand(
        controller,
        controller->command + deltaCommand);
}


void SteeringController_SetCommand(
    SteeringController *controller,
    float command)
{
    if (controller == NULL ||
        controller->servo == NULL ||
        controller->calibration == NULL)
    {
        return;
    }

    SteeringController_ApplyCommand(
        controller,
        command);

    controller->targetEffectiveAngleRad =
        controller->effectiveAngleRad;

    controller->reversalPending = false;
}

static void SteeringController_SetEffectiveAngleRadInternal(
    SteeringController *controller,
    float targetAngleRad,
    float dt,
    bool forceReversal,
    bool rateLimited)
{
    if (controller == NULL ||
        controller->servo == NULL ||
        controller->calibration == NULL)
    {
        return;
    }

    if (controller->centreState != STEERING_CENTRE_IDLE)
    {
        return;
    }


    float clampedTargetAngleRad =
        SteeringController_Clamp(
            targetAngleRad,
            controller->minEffectiveAngleRad,
            controller->maxEffectiveAngleRad);

    controller->targetEffectiveAngleRad =
        clampedTargetAngleRad;

    float angleErrorRad =
        clampedTargetAngleRad -
        controller->effectiveAngleRad;

    int8_t requiredCommandDirection;

    if (angleErrorRad < 0.0f)
    {
        /*
         * Effective steering angle decreases as raw command
         * increases.
         */
        requiredCommandDirection = +1;
    }
    else if (angleErrorRad > 0.0f)
    {
        requiredCommandDirection = -1;
    }
    else
    {
        controller->reversalPending = false;

        if (controller->movementDirection == 0)
        {
            /*
             * Deterministically establish the decreasing-command
             * zero-angle branch after initialization.
             */
            requiredCommandDirection = -1;
        }
        else
        {
            return;
        }
    }

    /*
     * Centre() forced operations retain the old instantaneous
     * behavior for now. This deliberately keeps the first patch
     * local to normal heading control.
     */
    if (!rateLimited)
    {
        uint32_t pointCount = 0U;

        const SteeringCalibrationPoint *branch =
            SteeringController_GetBranch(
                controller,
                requiredCommandDirection,
                &pointCount);

        float requiredCommand =
            SteeringController_InterpolateCommand(
                branch,
                pointCount,
                clampedTargetAngleRad);

        controller->reversalPending = false;

        SteeringController_ApplyCommand(
            controller,
            requiredCommand);

        return;
    }

    /*
     * If we are already traversing backlash, first finish moving
     * toward the point on the newly selected branch corresponding
     * to the held effective wheel angle.
     *
     * Do NOT chase the final requested angle through the backlash
     * region.
     */
    if (controller->backlashActive)
    {
        uint32_t pointCount = 0U;

        const SteeringCalibrationPoint *branch =
            SteeringController_GetBranch(
                controller,
                controller->movementDirection,
                &pointCount);

        float takeupCommand =
            SteeringController_InterpolateCommand(
                branch,
                pointCount,
                controller->backlashHoldAngleRad);

        SteeringController_ApplyCommandRateLimited(
            controller,
            takeupCommand,
            dt);

        /*
         * Reaching the raw command corresponding to the held wheel
         * angle means the opposite side of the backlash has been
         * engaged.
         */
        if (fabsf(controller->command - takeupCommand) <
            STEERING_COMMAND_MODEL_EPSILON)
        {
            controller->backlashActive = false;
            controller->effectiveAngleRad =
                controller->backlashHoldAngleRad;
        }

        return;
    }
    bool directionReversal =
        controller->movementDirection != 0 &&
        requiredCommandDirection !=
            controller->movementDirection;

    /*
     * Suppress insignificant target reversals.
     */
    if (!forceReversal &&
        directionReversal &&
        fabsf(angleErrorRad) <
            controller->calibration->reversalDeadbandRad)
    {
        controller->reversalPending = true;
        return;
    }

    controller->reversalPending = false;

    uint32_t pointCount = 0U;

    const SteeringCalibrationPoint *branch =
        SteeringController_GetBranch(
            controller,
            requiredCommandDirection,
            &pointCount);

    float desiredCommand;

    if (directionReversal)
    {
        /*
         * A genuine major-branch reversal has been accepted.
         *
         * First move through backlash toward the raw command on
         * the opposite branch that corresponds to the CURRENT
         * effective wheel angle.
         *
         * SteeringController_UpdateModel() will see the raw
         * command reversal, assert backlashActive and hold
         * effectiveAngleRad while we traverse the play.
         */
        desiredCommand =
            SteeringController_InterpolateCommand(
                branch,
                pointCount,
                controller->effectiveAngleRad);
    }
    else
    {
        /*
         * Already travelling on the appropriate branch:
         * proceed toward the requested effective angle.
         */
        desiredCommand =
            SteeringController_InterpolateCommand(
                branch,
                pointCount,
                clampedTargetAngleRad);
    }

    SteeringController_ApplyCommandRateLimited(
        controller,
        desiredCommand,
        dt);
}

void SteeringController_SetEffectiveAngleRad(
    SteeringController *controller,
    float targetAngleRad,
	float dt)
{
	SteeringController_SetEffectiveAngleRadInternal(
		controller,
		targetAngleRad,
		dt,
		false,
		true);
}


void SteeringController_Centre(
    SteeringController *controller)
{
    if (controller == NULL)
    {
        return;
    }

    /*
     * Do not reset the hysteresis model here.
     *
     * Moving to command zero does not physically erase
     * backlash in the linkage.
     *
     * Forces centering even if it is within reversal
     * deadband
     */
    SteeringController_SetEffectiveAngleRadInternal(
        controller,
		0.0f,
        0.0f,
        true,
		false);

    /*
     * Leave the model in an established, non-backlash
     * state after the immediate centering operation
     */
    controller->targetEffectiveAngleRad = 0.0f;
    controller->effectiveAngleRad = 0.0f;
    controller->movementDirection = -1;
    controller->backlashActive = false;
    controller->backlashHoldAngleRad = 0.0f;
    controller->reversalPending = false;
}


float SteeringController_GetCommand(
    const SteeringController *controller)
{
    if (controller == NULL)
    {
        return 0.0f;
    }

    return controller->command;
}


float SteeringController_GetEffectiveAngleRad(
    const SteeringController *controller)
{
    if (controller == NULL)
    {
        return 0.0f;
    }

    return controller->effectiveAngleRad;
}


bool SteeringController_IsBacklashActive(
    const SteeringController *controller)
{
    if (controller == NULL)
    {
        return false;
    }

    return controller->backlashActive;
}


int8_t SteeringController_GetMovementDirection(
    const SteeringController *controller)
{
    if (controller == NULL)
    {
        return 0;
    }

    return controller->movementDirection;
}

float SteeringController_GetTargetEffectiveAngleRad(
    const SteeringController *controller)
{
    if (controller == NULL)
    {
        return 0.0f;
    }

    return controller->targetEffectiveAngleRad;
}


float SteeringController_GetMinEffectiveAngleRad(
    const SteeringController *controller)
{
    if (controller == NULL)
    {
        return 0.0f;
    }

    return controller->minEffectiveAngleRad;
}


float SteeringController_GetMaxEffectiveAngleRad(
    const SteeringController *controller)
{
    if (controller == NULL)
    {
        return 0.0f;
    }

    return controller->maxEffectiveAngleRad;
}

bool SteeringController_IsReversalPending(
    const SteeringController *controller)
{
    if (controller == NULL)
    {
        return false;
    }

    return controller->reversalPending;
}

void SteeringController_StartCentre(SteeringController *controller)
{
    if (controller == NULL ||
        controller->calibration == NULL)
    {
        return;
    }

    /*
     * Centre is deliberately established on the decreasing-command
     * calibration branch.
     *
     * First move to maxCommand, then approach the calibrated zero
     * from above.
     */
    controller->centreCommand =
        SteeringController_InterpolateCommand(
            controller->calibration->decreasingPoints,
            controller->calibration->decreasingPointCount,
            0.0f);

    controller->targetEffectiveAngleRad = 0.0f;
    controller->reversalPending = false;

    controller->centreState = STEERING_CENTRE_PRECONDITION;
}

bool SteeringController_UpdateCentre(SteeringController *controller,
                                     float dt)
{
    if (controller == NULL ||
        controller->calibration == NULL)
    {
        return false;
    }

    switch (controller->centreState)
    {
    case STEERING_CENTRE_IDLE:
        return true;

    case STEERING_CENTRE_PRECONDITION:
    {
        const float preconditionCommand =
            controller->calibration->maxCommand;

        SteeringController_ApplyCommandRateLimited(
            controller,
            preconditionCommand,
            dt);

        if (fabsf(controller->command - preconditionCommand) <=
            STEERING_COMMAND_MODEL_EPSILON)
        {
            /*
             * We are now definitely on the high-command side.
             * The next phase approaches centre in decreasing-command
             * direction.
             */
            controller->centreState = STEERING_CENTRE_APPROACH;
        }

        return false;
    }

    case STEERING_CENTRE_APPROACH:
    {
        SteeringController_ApplyCommandRateLimited(
            controller,
            controller->centreCommand,
            dt);

        if (fabsf(controller->command -
                  controller->centreCommand) <=
            STEERING_COMMAND_MODEL_EPSILON)
        {
            /*
             * The physical linkage has now approached calibrated
             * centre from the decreasing-command direction.
             *
             * Synchronise the software model with that known
             * mechanical branch.
             */
            controller->command = controller->centreCommand;

            controller->targetEffectiveAngleRad = 0.0f;
            controller->effectiveAngleRad = 0.0f;

            controller->movementDirection = -1;

            controller->backlashActive = false;
            controller->backlashHoldAngleRad = 0.0f;

            controller->reversalPending = false;

            controller->centreState = STEERING_CENTRE_IDLE;

            return true;
        }

        return false;
    }

    default:
        controller->centreState = STEERING_CENTRE_IDLE;
        return false;
    }
}

bool SteeringController_IsCentring(
    const SteeringController *controller)
{
    if (controller == NULL)
    {
        return false;
    }

    return controller->centreState != STEERING_CENTRE_IDLE;
}
