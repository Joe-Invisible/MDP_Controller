#ifndef ULTRASONIC_SENSOR_CONFIG_H
#define ULTRASONIC_SENSOR_CONFIG_H
#include "hcsr04.h"
/* One physical timer/sensor. Runtime and standalone tests are mutually exclusive. */
extern HCSR04_HandleTypeDef hcsr04;
#endif
