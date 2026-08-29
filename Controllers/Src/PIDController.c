/*
 * PIDController.c
 *
 *  Created on: 2026年8月29日
 *      Author: Joe
 */

#include "PIDController.h"
#include <stdlib.h>


static float PIDController_Clamp(
        float value,
        float min,
        float max)
{
    if (value > max)
        return max;

    if (value < min)
        return min;

    return value;
}


bool PIDController_Init(
        PIDController *controller,
        float kp,
        float ki,
        float kd,
        float outputMin,
        float outputMax)
{
    if (controller == NULL ||
        outputMin >= outputMax)
    {
        return false;
    }

    *controller = (PIDController){0};

    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;

    controller->outputMin = outputMin;
    controller->outputMax = outputMax;

    controller->integral = 0.0f;
    controller->previousError = 0.0f;
    controller->output = 0.0f;

    controller->hasPreviousError = false;

    return true;
}


float PIDController_Update(
        PIDController *controller,
        float error,
        float dt)
{
    if (controller == NULL || dt <= 0.0f)
        return 0.0f;

    /*
     * Proportional term.
     */
    float proportional = controller->kp * error;


    /*
     * Derivative term.
     *
     * Do not calculate a derivative on the first update because
     * there is no previous error sample yet.
     */
    float derivative = 0.0f;

    if (controller->hasPreviousError)
    {
        derivative =
                controller->kd *
                (error - controller->previousError) /
                dt;
    }


    /*
     * Calculate the candidate integral first.
     * It is only committed below if doing so will not cause
     * additional integral windup.
     */
    float candidateIntegral =
            controller->integral +
            error * dt;

    float candidateOutput =
            proportional +
            controller->ki * candidateIntegral +
            derivative;


    /*
     * Simple conditional-integration anti-windup.
     *
     * If the controller is saturated and the current error would
     * drive the output even further into saturation, do not
     * accumulate the integral term.
     *
     * If the error acts in the opposite direction, integration is
     * allowed so that the controller can recover from saturation.
     */
    bool saturatedHigh =
            candidateOutput > controller->outputMax;

    bool saturatedLow =
            candidateOutput < controller->outputMin;

    if (!((saturatedHigh && error > 0.0f) ||
          (saturatedLow  && error < 0.0f)))
    {
        controller->integral = candidateIntegral;
    }


    /*
     * Recalculate using the accepted integral state.
     */
    float output =
            proportional +
            controller->ki * controller->integral +
            derivative;

    output = PIDController_Clamp(
            output,
            controller->outputMin,
            controller->outputMax);


    controller->previousError = error;
    controller->hasPreviousError = true;
    controller->output = output;

    return output;
}


void PIDController_Reset(PIDController *controller)
{
    if (controller == NULL)
        return;

    controller->integral = 0.0f;
    controller->previousError = 0.0f;
    controller->output = 0.0f;

    controller->hasPreviousError = false;
}
