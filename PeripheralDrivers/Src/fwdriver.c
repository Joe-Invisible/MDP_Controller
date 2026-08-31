#include "fwdriver.h"
#include <math.h>

/**
 * Converts a pulse width in microseconds to a timer compare value.
 *
 * The scale is derived from the timer's own auto-reload value rather than
 * hard-coded, so that a CubeMX regeneration which changes the prescaler
 * cannot silently leave the steering pointing somewhere else.
 *
 * With the current TIM8 configuration (72MHz / 72 = 1MHz, ARR = 19999) one
 * tick is 1us and this returns pulseUs unchanged.
 */
static uint32_t ServoPulseToCompare(TIM_HandleTypeDef *htim, float pulseUs) {
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
  return (uint32_t)lroundf((pulseUs * period) / SERVO_FRAME_PERIOD_US);
}

bool Servo_Init(Servo *sm, TIM_HandleTypeDef *pwmHtim, uint32_t pwmChannel,
                uint16_t minPulseUs, uint16_t centrePulseUs,
                uint16_t maxPulseUs) {

  if (!sm || !pwmHtim)
    return false;

  // Strict ordering also rules out a zero-width span on either side, which
  // would make the steering scale meaningless.
  if (minPulseUs >= centrePulseUs || centrePulseUs >= maxPulseUs)
    return false;
  if (minPulseUs < SERVO_ABS_MIN_PULSE_US)
    return false;
  if (maxPulseUs > SERVO_ABS_MAX_PULSE_US)
    return false;

  sm->config.pwmHtim = pwmHtim;
  sm->config.pwmChannel = pwmChannel;

  sm->intrin.minPulseUs = minPulseUs;
  sm->intrin.centrePulseUs = centrePulseUs;
  sm->intrin.maxPulseUs = maxPulseUs;

  sm->state.pulseUs = centrePulseUs;
  sm->state.steer = 0;

  return true;
}

bool Servo_Enable(Servo *sm) {
  if (!sm)
    return false;

  // Load the centre position before the output is enabled, so the servo
  // does not slew to whatever happened to be in the compare register.
  Servo_Centre(sm);

  if (HAL_TIM_PWM_Start(sm->config.pwmHtim, sm->config.pwmChannel) != HAL_OK)
    return false;

  return true;
}

void Servo_Disable(Servo *sm) {
  if (!sm)
    return;

  HAL_TIM_PWM_Stop(sm->config.pwmHtim, sm->config.pwmChannel);
}

void Servo_SetPulseUs(Servo *sm, float pulseUs) {
  if (!sm)
    return;

  // Absolute clamp only: this function exists so that calibration can probe
  // past the limits currently believed correct. Callers that want the
  // calibrated limits enforced must use Servo_SetSteering.
  if (pulseUs < SERVO_ABS_MIN_PULSE_US)
    pulseUs = SERVO_ABS_MIN_PULSE_US;
  if (pulseUs > SERVO_ABS_MAX_PULSE_US)
    pulseUs = SERVO_ABS_MAX_PULSE_US;

  sm->state.pulseUs = pulseUs;

  __HAL_TIM_SET_COMPARE(sm->config.pwmHtim, sm->config.pwmChannel,
                        ServoPulseToCompare(sm->config.pwmHtim, pulseUs));
}

void Servo_SetSteering(Servo *sm, float steer) {
  if (!sm)
    return;

  if (steer > SERVO_STEER_MAX)
    steer = SERVO_STEER_MAX;
  if (steer < SERVO_STEER_MIN)
    steer = SERVO_STEER_MIN;

  // Each half is scaled against its own span. The linkage is rarely
  // symmetric about the centre, so a single shared scale would either
  // overshoot one mechanical stop or fail to reach the other.
  //
  // Arithmetic is done in int32_t: steer is int8_t and the spans are
  // uint16_t, so the intermediate product overflows the narrower type and
  // the mixed signedness would otherwise convert badly.
  int32_t centre = (int32_t)sm->intrin.centrePulseUs;
  float pulseUs;

  if (steer >= 0) {
    float span = (int32_t)sm->intrin.maxPulseUs - centre;
    pulseUs = (float)centre + (steer * span) / SERVO_STEER_MAX;
  } else {
    float span = centre - (int32_t)sm->intrin.minPulseUs;
    pulseUs = (float)centre - (-steer * span) / SERVO_STEER_MAX;
  }

  Servo_SetPulseUs(sm, pulseUs);

  // Recorded after the call, which has no way to know which steering
  // command it came from.
  sm->state.steer = steer;
}

void Servo_Centre(Servo *sm) { Servo_SetSteering(sm, 0); }

uint16_t Servo_GetPulseUs(Servo *sm) {
  if (!sm)
    return 0;
  return sm->state.pulseUs;
}
