/*
 * SteeringControllerTest.h
 */

#ifndef INC_STEERINGCONTROLLERTEST_H_
#define INC_STEERINGCONTROLLERTEST_H_

#include "SteeringController.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint32_t step;

    float requestedAngleRad;
    float targetAngleRad;
    float effectiveAngleRad;

    float command;

    bool backlashActive;
    int8_t movementDirection;

} SteeringControllerTestLogSample;

void SteeringControllerTestRun(void);

#endif /* INC_STEERINGCONTROLLERTEST_H_ */
