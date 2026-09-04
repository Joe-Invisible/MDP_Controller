/*
 * SteeringController.h
 *
 * Models and controls the front steering mechanism.
 *
 * Converts normalized Servo_SetSteering() commands into an
 * estimated effective bicycle-model steering angle while
 * accounting for measured linkage hysteresis.
 *
 *  Created on: 2026年9月3日
 *      Author: Joe
 */

#ifndef INC_STEERINGCONTROLLER_H_
#define INC_STEERINGCONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

#include "fwdriver.h"

typedef struct
{
    float command;
    float effectiveAngleRad;

} SteeringCalibrationPoint;


typedef struct
{
    /*
     * Calibration branch followed when steering command
     * is increasing.
     */
    const SteeringCalibrationPoint *increasingPoints;
    uint32_t increasingPointCount;

    /*
     * Calibration branch followed when steering command
     * is decreasing.
     */
    const SteeringCalibrationPoint *decreasingPoints;
    uint32_t decreasingPointCount;

    /*
     * Command range over which the effective-angle model
     * has been experimentally calibrated.
     */
    float minCommand;
    float maxCommand;


    /*
     * Minimum effective-angle reversal required before
     * switching to the opposite major hysteresis branch.
     *
     * Small target reversals inside this band are held on
     * the current branch.
     */
    float reversalDeadbandRad;

} SteeringControllerCalibration;


typedef struct
{
    Servo *servo;

    const SteeringControllerCalibration *calibration;

    /*
     * Actual command currently sent to Servo_SetSteering().
     */
    float command;

    /*
     * Estimated effective bicycle-model steering angle.
     *
     * This is a model estimate, not a direct measurement.
     */
    float effectiveAngleRad;

    /*
     * Requested effective bicycle-model steering angle.
     *
     * This may differ temporarily from effectiveAngleRad when
     * the mechanism cannot immediately reach the requested
     * position.
     */
    float targetEffectiveAngleRad;

    /*
     * Effective-angle range that can be reached on either
     * calibrated hysteresis branch.
     */
    float minEffectiveAngleRad;
    float maxEffectiveAngleRad;

    /*
     * Direction of the most recent steering-command motion:
     *
     *   +1 = increasing command
     *   -1 = decreasing command
     *    0 = no direction established since initialization
     */
    int8_t movementDirection;

    /*
     * When the command reverses direction, the linkage must
     * traverse backlash before the wheels begin to move.
     *
     * While backlashActive is true, effectiveAngleRad remains
     * at backlashHoldAngleRad.
     */
    bool backlashActive;
    float backlashHoldAngleRad;

    bool reversalPending;

} SteeringController;


/**
 * @brief Initialise the steering controller.
 *
 * The servo is centred and the effective-angle model is
 * initially referenced to zero.
 *
 * The supplied calibration must contain monotonically
 * increasing command values and monotonically non-increasing
 * effective angles.
 */
bool SteeringController_Init(
    SteeringController *controller,
    Servo *servo,
    const SteeringControllerCalibration *calibration);


/**
 * @brief Command a steering position.
 *
 * The command is clamped to the experimentally calibrated
 * range before being sent to the servo.
 *
 * The effective steering-angle estimate and hysteresis state
 * are updated automatically.
 */
void SteeringController_SetCommand(
    SteeringController *controller,
    float command);

/**
 * @brief Request an effective bicycle-model steering angle.
 *
 * The requested angle is clamped to the range that is
 * reachable on both calibrated hysteresis branches.
 *
 * The controller selects the appropriate calibration branch
 * from the required direction of physical steering motion,
 * performs the inverse calibration, and commands the servo.
 */
void SteeringController_SetEffectiveAngleRad(
    SteeringController *controller,
    float targetAngleRad);

/**
 * @brief Return the currently requested effective steering
 *        angle, in radians.
 */
float SteeringController_GetTargetEffectiveAngleRad(
    const SteeringController *controller);

/**
 * @brief Return the minimum bidirectionally controllable
 *        effective steering angle.
 */
float SteeringController_GetMinEffectiveAngleRad(
    const SteeringController *controller);

/**
 * @brief Return the maximum bidirectionally controllable
 *        effective steering angle.
 */
float SteeringController_GetMaxEffectiveAngleRad(
    const SteeringController *controller);

/**
 * @brief Command the nominal servo centre.
 *
 * This does NOT erase hysteresis state. It is equivalent to
 * SteeringController_SetCommand(controller, 0.0f).
 */
void SteeringController_Centre(
    SteeringController *controller);


/**
 * @brief Return the actual clamped steering command.
 */
float SteeringController_GetCommand(
    const SteeringController *controller);


/**
 * @brief Return the estimated effective bicycle steering
 *        angle, in radians.
 */
float SteeringController_GetEffectiveAngleRad(
    const SteeringController *controller);


/**
 * @brief Return whether the linkage model is currently
 *        traversing backlash.
 */
bool SteeringController_IsBacklashActive(
    const SteeringController *controller);


/**
 * @brief Return current steering-command movement direction.
 *
 * +1 = increasing
 * -1 = decreasing
 *  0 = not yet established
 */
int8_t SteeringController_GetMovementDirection(
    const SteeringController *controller);

/**
 * @brief Return whether the requested angle is pending
 * 		due to the configured reversal deadband.
 */
bool SteeringController_IsReversalPending(
    const SteeringController *controller);

#endif /* INC_STEERINGCONTROLLER_H_ */
