#include "ServoTestBasic.h"
#include "oled.h"
#include "oledutils.h"

#define SERVOPWMSRC htim8
#define SERVOPWMCH TIM_CHANNEL_1

/**
 * Limit-finding sweep bounds.
 *
 * Set to the servo's full absolute range, so that a linkage which does not
 * bind before the servo's own end stops still yields a result.
 *
 * Steps are coarse and the dwell short: every step taken past a mechanical
 * stop is time spent stalled, and covering this range finely would mean
 * minutes of it. 20us is about 1.8 degrees of servo travel, which the servo
 * completes in well under the dwell. Refine both once the stops are roughly
 * located.
 */
#define CAL_MIN_PULSE_US SERVO_ABS_MIN_PULSE_US
#define CAL_MAX_PULSE_US SERVO_ABS_MAX_PULSE_US
#define CAL_STEP_US ((uint16_t)20)
#define CAL_STEP_DELAY_MS ((uint32_t)150)

/**
 * Centre-finding sweep bounds.
 *
 * Straight-ahead is rarely at exactly 1500us, because the servo horn is
 * splined and only mounts at discrete angles. This sweep crawls through a
 * narrow band around the nominal centre with a long dwell, so the wheels can
 * be judged straight while effectively stationary rather than mid-slew.
 */
#define CTR_MIN_PULSE_US 	((uint16_t)1400)
#define CTR_MAX_PULSE_US 	((uint16_t)1700)
#define CTR_STEP_US 			((uint16_t)10)
#define CTR_STEP_DELAY_MS 	((uint32_t)600)

/**
 * Display rows. The font is 8x12, so rows sit 16 apart and a line holds at
 * most 16 characters. Every string is padded to a fixed width, because
 * OLED_ShowChar paints the character background and so a shorter string would
 * leave the tail of the previous one on screen.
 */
#define ROW_TITLE ((uint8_t)0)
#define ROW_PHASE ((uint8_t)1)
#define ROW_PULSE ((uint8_t)2)
#define ROW_STEER ((uint8_t)3)

static void ServoShowPulse(Servo *sm) {
  OLED_Printf(0, ROW_PULSE, "PULSE %4u us ", Servo_GetPulseUs(sm));
  OLED_Refresh_Gram();
}

void ServoTestRun() {
  Servo servo = {0};

  if (!Servo_Init(&servo, &SERVOPWMSRC, SERVOPWMCH, CHASSIS_STEER_MIN_PULSE_US,
                  CHASSIS_STEER_CTR_PULSE_US, CHASSIS_STEER_MAX_PULSE_US))
    return;

  OLED_Clear();
  OLED_Printf(0, ROW_TITLE, "SERVO TEST    ");

  if (!Servo_Enable(&servo)) {
    OLED_Printf(0, ROW_PHASE, "ENABLE FAILED ");
    OLED_Refresh_Gram();
    return;
  }

  // Let the servo actually reach centre before asking it to go elsewhere.
  HAL_Delay(1000);

  static const int8_t steps[] = {0, -50, -100, -50, 0, 50, 100, 50, 0};

  for (unsigned i = 0; i < sizeof steps / sizeof steps[0]; i++) {
    Servo_SetSteering(&servo, steps[i]);

    OLED_Printf(0, ROW_STEER, "STEER %+4d    ", steps[i]);
    ServoShowPulse(&servo);

    HAL_Delay(1000);
  }

  // Release holding torque rather than leaving the servo pushing.
  Servo_Disable(&servo);
  OLED_Printf(0, ROW_PHASE, "DONE          ");
  OLED_Refresh_Gram();
}

/**
 * Steps the servo from one pulse width to another in stepUs increments,
 * holding each for dwellMs and displaying it, so the operator can read off
 * the value where the steering stops moving or looks straight.
 */
static void ServoSweep(Servo *sm, uint16_t from, uint16_t to, uint16_t stepUs,
                       uint32_t dwellMs) {
  int32_t step = (to >= from) ? (int32_t)stepUs : -(int32_t)stepUs;
  int32_t p = (int32_t)from;

  for (;;) {
    Servo_SetPulseUs(sm, (uint16_t)p);
    ServoShowPulse(sm);
    HAL_Delay(dwellMs);

    if (p == (int32_t)to)
      break;

    p += step;

    // Land exactly on 'to' even when the span is not a whole number of steps.
    if ((step > 0 && p > (int32_t)to) || (step < 0 && p < (int32_t)to))
      p = (int32_t)to;
  }
}

void ServoCalibrationRun() {
  Servo servo = {0};

  // Initialised with the exploration bounds rather than the believed steering
  // limits: the whole point of this test is to probe past them.
  if (!Servo_Init(&servo, &SERVOPWMSRC, SERVOPWMCH, CAL_MIN_PULSE_US,
                  CHASSIS_STEER_CTR_PULSE_US, CAL_MAX_PULSE_US))
    return;

  OLED_Clear();
  OLED_Printf(0, ROW_TITLE, "SERVO CAL     ");

  if (!Servo_Enable(&servo)) {
    OLED_Printf(0, ROW_PHASE, "ENABLE FAILED ");
    OLED_Refresh_Gram();
    return;
  }

  HAL_Delay(1000);

  OLED_Printf(0, ROW_PHASE, "SWEEP MIN <<< ");
  ServoSweep(&servo, CHASSIS_STEER_CTR_PULSE_US, CAL_MIN_PULSE_US,
             CAL_STEP_US, CAL_STEP_DELAY_MS);

  OLED_Printf(0, ROW_PHASE, "TO CENTRE     ");
  ServoSweep(&servo, CAL_MIN_PULSE_US, CHASSIS_STEER_CTR_PULSE_US,
             CAL_STEP_US, CAL_STEP_DELAY_MS);
  HAL_Delay(1000);

  OLED_Printf(0, ROW_PHASE, "SWEEP MAX >>> ");
  ServoSweep(&servo, CHASSIS_STEER_CTR_PULSE_US, CAL_MAX_PULSE_US,
             CAL_STEP_US, CAL_STEP_DELAY_MS);

  OLED_Printf(0, ROW_PHASE, "TO CENTRE     ");
  ServoSweep(&servo, CAL_MAX_PULSE_US, CHASSIS_STEER_CTR_PULSE_US,
             CAL_STEP_US, CAL_STEP_DELAY_MS);

  // Nothing should be left pushing once the sweep is over.
  Servo_Disable(&servo);
  OLED_Printf(0, ROW_PHASE, "CAL DONE      ");
  OLED_Refresh_Gram();
}

void ServoCentreFindRun() {
  Servo servo = {0};

  if (!Servo_Init(&servo, &SERVOPWMSRC, SERVOPWMCH, CTR_MIN_PULSE_US,
                  CHASSIS_STEER_CTR_PULSE_US, CTR_MAX_PULSE_US))
    return;

  OLED_Clear();
  OLED_Printf(0, ROW_TITLE, "FIND CENTRE   ");

  if (!Servo_Enable(&servo)) {
    OLED_Printf(0, ROW_PHASE, "ENABLE FAILED ");
    OLED_Refresh_Gram();
    return;
  }

  HAL_Delay(1000);

  // Swept in both directions because a servo does not settle at quite the same
  // place approaching a position from either side. Straight-ahead is the value
  // that looks right on both passes; if the two readings differ, take the
  // midpoint.
  OLED_Printf(0, ROW_PHASE, "PASS 1  >>>   ");
  ServoSweep(&servo, CTR_MIN_PULSE_US, CTR_MAX_PULSE_US, CTR_STEP_US,
             CTR_STEP_DELAY_MS);

  OLED_Printf(0, ROW_PHASE, "PASS 2  <<<   ");
  ServoSweep(&servo, CTR_MAX_PULSE_US, CTR_MIN_PULSE_US, CTR_STEP_US,
             CTR_STEP_DELAY_MS);

  Servo_SetPulseUs(&servo, CHASSIS_STEER_CTR_PULSE_US);
  ServoShowPulse(&servo);
  HAL_Delay(1000);

  Servo_Disable(&servo);
  OLED_Printf(0, ROW_PHASE, "CENTRE DONE   ");
  OLED_Refresh_Gram();
}
