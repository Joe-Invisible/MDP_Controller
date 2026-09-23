#ifndef INC_COMMANDMOTION_H_
#define INC_COMMANDMOTION_H_

#include "CommandParser.h"

/* UART runtime radius: matches Pi TURN_RADIUS_MM and the existing 275 mm
 * left/right feedforward calibration points. Floor validation is required
 * after a radius change; calibration experiments keep their own settings.
 */
#define COMMANDMOTION_TURN_RADIUS_MM 275.0f
#define COMMANDMOTION_TURN_SPEED_CPS 2000.0f
#define COMMANDMOTION_STRAIGHT_SPEED_CPS 5000.0f
#define COMMANDMOTION_MAX_TURN_DEG 360.0f

typedef struct {
    float distanceMm;
    float radiusMm; /* Zero means straight. Positive means left. */
    float speedCps;
} CommandMotion;

/* Pure conversion, also used to validate a whole batch before acceptance. */
bool CommandMotion_Resolve(const Command *command, CommandMotion *motion);

#endif
