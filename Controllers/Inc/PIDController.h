/*
 * PIDController.h
 *
 *  Created on: 2026年8月29日
 *      Author: Joe
 */

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include <stdbool.h>

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float previousError;

    float outputMin;
    float outputMax;

    float output;

    bool hasPreviousError;

} PIDController;


/**
 * @brief Initialise a PID controller.
 *
 * @param controller Pointer to PID controller.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 * @param outputMin Minimum controller output.
 * @param outputMax Maximum controller output.
 *
 * @return true on success, false if parameters are invalid.
 */
bool PIDController_Init(
        PIDController *controller,
        float kp,
        float ki,
        float kd,
        float outputMin,
        float outputMax);


/**
 * @brief Update the PID controller.
 *
 * @param controller Pointer to PID controller.
 * @param error Current control error.
 * @param dt Time since the previous update, in seconds.
 *
 * @return Controller output.
 */
float PIDController_Update(
        PIDController *controller,
        float error,
        float dt);


/**
 * @brief Reset accumulated controller state.
 *
 * Gains and output limits are preserved.
 */
void PIDController_Reset(PIDController *controller);

#endif /* PIDCONTROLLER_H_ */
