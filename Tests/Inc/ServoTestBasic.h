#ifndef INC_SERVOTESTBASIC_H_
#define INC_SERVOTESTBASIC_H_

#include "fwdriver.h"
#include "oled.h"
#include "tim.h"

/**
 * Sweeps the steering across the calibrated range and back to centre.
 *
 * Stays inside CHASSIS_STEER_MIN/MAX, so nothing is driven against a
 * mechanical stop. Run this once the limits are known.
 */
void ServoTestRun();

/**
 * Finds the real steering limits of this chassis.
 *
 * Steps the servo outward from centre in CAL_STEP_US increments, displaying
 * the pulse width at every step. Watch the steering and note the pulse width
 * at which it stops moving: that value, less a small safety margin, is what
 * belongs in Servo_Init.
 *
 * This deliberately probes beyond the limits currently believed correct, so
 * the servo will end each sweep pushing against a stop. The servo reports
 * nothing back, so it will keep pushing at up to 2.5A until the next step.
 * Keep the sweep bounds tight and stay ready to cut power.
 */
void ServoCalibrationRun();

/**
 * Finds the pulse width at which the wheels point straight ahead.
 *
 * Crawls through a narrow band around the nominal 1500us centre with a long
 * dwell per step, so the steering can be judged while effectively stationary
 * rather than mid-slew. Sweeps in both directions, because a servo does not
 * settle at quite the same place approaching a position from either side: if
 * the two readings differ, take the midpoint.
 *
 * Stays well inside the mechanical stops, so nothing is driven against a limit
 * here.
 */
void ServoCentreFindRun();

#endif /* INC_SERVOTESTBASIC_H_ */
