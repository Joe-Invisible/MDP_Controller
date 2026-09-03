/*
 * SteeringController.c
 *
 *  Created on: 2026年9月3日
 *      Author: Joe
 */

#include "SteeringController.h"

#include <math.h>
#include <stddef.h>


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
                controller->backlashHoldAngleRad;
        }
        else
        {
            backlashTakenUp =
                branchAngleRad >=
                controller->backlashHoldAngleRad;
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
     * Both tables must cover the configured operating range.
     */
    if (calibration->increasingPoints[0].command >
            calibration->minCommand ||
        calibration->increasingPoints[
            calibration->increasingPointCount - 1U
        ].command <
            calibration->maxCommand)
    {
        return false;
    }

    if (calibration->decreasingPoints[0].command >
            calibration->minCommand ||
        calibration->decreasingPoints[
            calibration->decreasingPointCount - 1U
        ].command <
            calibration->maxCommand)
    {
        return false;
    }

    *controller =
        (SteeringController){0};

    controller->servo =
        servo;

    controller->calibration =
        calibration;

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
     */
    SteeringController_SetCommand(
        controller,
        0.0f);
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
