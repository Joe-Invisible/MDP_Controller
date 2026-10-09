#ifndef RWDRIVER_H
#define RWDRIVER_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef enum { DCMOTOR_MODE_NEUTRAL=0, DCMOTOR_MODE_DRIVE, DCMOTOR_MODE_BRAKE } DCMotorMode;
typedef struct { void *pwmHtim; } DCMotorConfg;
typedef struct { float activeDutyCycle; DCMotorMode mode; } DCMotorState;
typedef struct { DCMotorConfg config; DCMotorState state; double position; float speed; float command; int wheel; } DCMotor;
void DCMotor_Brake(DCMotor *m);
void DCMotor_Neutral(DCMotor *m);
void DCMotor_SetPWM(DCMotor *m, float pwm);
void DCMotor_SetBrakePWM(DCMotor *m, float pwm);
int16_t DCMotor_GetEncoderCount(DCMotor *m);
#endif
