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

/* UART straight moves use a 1 mm endpoint tolerance: observed stalls were
 * 0.54–0.94 mm short with drive still active. This is an acceptance policy,
 * not a motor recalibration. Turns retain the shared controller tolerance.
 * Straight requests <= this tolerance can complete without wheel movement.
 */
#define COMMANDMOTION_STRAIGHT_COMPLETION_TOLERANCE_MM 1.0f

typedef struct {
    float distanceMm;
    float radiusMm; /* Zero means straight. Positive means left. */
    float speedCps;
    float speedMmps; /* Nonzero for U; converted using runtime geometry. */
} CommandMotion;

/* Pure conversion, also used to validate a whole batch before acceptance. */
bool CommandMotion_Resolve(const Command *command, CommandMotion *motion);

#endif
