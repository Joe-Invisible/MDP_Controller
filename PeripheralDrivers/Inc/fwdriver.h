#ifndef INC_FWDRIVER_H_
#define INC_FWDRIVER_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * PWM frame period expected by the servo, in microseconds.
 * 20000us = 50Hz. Only the width of the high pulse carries the command;
 * the frame period is just how often the command is repeated.
 */
#define SERVO_FRAME_PERIOD_US ((uint32_t)20000)

/**
 * Absolute pulse width limits of the TD-8120MG, in microseconds.
 *
 * These describe the servo itself and are identical for every unit. They are
 * a last-resort guard against bad input reaching the hardware. Never widen
 * them.
 */
#define SERVO_ABS_MIN_PULSE_US ((uint16_t)500)
#define SERVO_ABS_MAX_PULSE_US ((uint16_t)2500)

/**
 * Usable pulse widths for this chassis, measured with ServoCalibrationRun and
 * ServoCentreFindRun.
 *
 * Min and max sit short of the mechanical stops, so a full-deflection steering
 * command parks just clear of one rather than stalling against it. The margin
 * is larger on the right: the servo begins to strain well before the stop that
 * a coarse sweep appears to show on that side, and a stalled TD-8120MG draws
 * up to 2.5A with nothing to report it.
 *
 * Centre is the midpoint of two hysteresis readings: the wheels look straight
 * at 1550us when approached from below and 1480us from above. That 70us gap is
 * backlash in the linkage, which sits outside the servo's own feedback loop and
 * so is never corrected. No single value is right for both directions; 1515us
 * splits the error rather than putting all of it on one. This leaves the travel
 * spans slightly unequal, 665us and 735us, which Servo_SetSteering already
 * scales independently.
 *
 * Do not take these from the TD-8120MG datasheet, which quotes a working range
 * of "roughly 65 to 85" ticks of 20us (1300-1700us). That describes a
 * different linkage and understates this one's travel about fourfold. The
 * stops depend on the chassis, not the servo, and must be measured per robot.
 */
#define CHASSIS_STEER_MIN_PULSE_US ((uint16_t)900)
#define CHASSIS_STEER_CTR_PULSE_US ((uint16_t)1515)
#define CHASSIS_STEER_MAX_PULSE_US ((uint16_t)2200)

/**
 * Steering command range, by convention matching DCMotor_SetPWM.
 */
#define SERVO_STEER_MIN ((int8_t)-100)
#define SERVO_STEER_MAX ((int8_t)100)

/**
 * Peripheral Configuration Parameters
 *
 * There is no encoder counterpart to DCMotorConfg here: the servo exposes no
 * feedback to the MCU. Its shaft potentiometer is wired only to its own
 * internal control loop.
 */
typedef struct ServoConfg {
  /**
   * PWM source timer module
   */
  TIM_HandleTypeDef *pwmHtim;
  /**
   * PWM source channel
   */
  uint32_t pwmChannel;

} ServoConfg;

/**
 * Mechanical Parameters of the servo as mounted on the chassis.
 *
 * Because nothing reports back from the servo, these limits are the only
 * protection the steering linkage has. Commanding past them drives the servo
 * against a mechanical stop, which it will keep pushing against at up to
 * 2.5A for as long as the command stands.
 */
typedef struct ServoIntrn {
  /**
   * Pulse width at full deflection in the negative steering direction.
   */
  uint16_t minPulseUs;
  /**
   * Pulse width with the wheels pointing straight ahead.
   */
  uint16_t centrePulseUs;
  /**
   * Pulse width at full deflection in the positive steering direction.
   */
  uint16_t maxPulseUs;

} ServoIntrn;

/**
 * Application-level runtime states of the servo.
 *
 * These record what was last commanded, NOT what the servo actually did.
 * Unlike DCMotorState.encCount, nothing here is a measurement.
 */
typedef struct ServoState {
  /**
   * Last commanded pulse width, in microseconds.
   */
  uint16_t pulseUs;

  /**
   * Range -100~+100
   * Last commanded steering position.
   * -: one direction
   * +: the other
   * 0: centred
   */
  int8_t steer;

} ServoState;

typedef struct Servo {
  ServoConfg config;
  ServoIntrn intrin;
  ServoState state;
} Servo;

/**
 * Initialises the driver struct. Does not touch the hardware.
 *
 * The calibrated limits must satisfy
 *   SERVO_ABS_MIN_PULSE_US <= minPulseUs
 *                          <  centrePulseUs
 *                          <  maxPulseUs
 *                          <= SERVO_ABS_MAX_PULSE_US
 */
bool Servo_Init(Servo *sm, TIM_HandleTypeDef *pwmHtim, uint32_t pwmChannel,
                uint16_t minPulseUs, uint16_t centrePulseUs,
                uint16_t maxPulseUs);

/**
 * Centres the servo, then starts PWM generation. Centring before the output
 * is enabled avoids slewing to an unknown position on start-up.
 */
bool Servo_Enable(Servo *sm);

/**
 * Stops PWM generation. The servo loses holding torque and the steering turns
 * freely. Prefer this over parking at a limit whenever the robot is left
 * powered but idle, and before halting at a debugger breakpoint.
 */
void Servo_Disable(Servo *sm);

/**
 * Steer to a position.
 * steer:
 * Range -100~+100, mapped linearly onto the calibrated limits.
 * -100: minPulseUs
 *    0: centrePulseUs
 * +100: maxPulseUs
 * Values outside the range are clamped.
 */
void Servo_SetSteering(Servo *sm, int8_t steer);

/**
 * Sets the pulse width directly, in microseconds, clamped to the ABSOLUTE
 * limits only. This deliberately bypasses the calibrated limits.
 *
 * It exists for calibration, where the whole point is to probe beyond the
 * limits currently believed correct. Normal motion commands must use
 * Servo_SetSteering instead.
 */
void Servo_SetPulseUs(Servo *sm, uint16_t pulseUs);

void Servo_Centre(Servo *sm);

/**
 * Returns the last commanded pulse width. This is intent, not measurement.
 */
uint16_t Servo_GetPulseUs(Servo *sm);

#endif /* INC_FWDRIVER_H_ */
