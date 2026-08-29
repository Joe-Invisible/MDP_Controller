/*
 * DCMotorTestBasic.c
 *
 *  Created on: 2026年8月24日
 *      Author: Joe
 */


#include "DCMotorTestBasic.h"
#include "oledutils.h"
#include "userbutton.h"
#include <stdlib.h>

#define MOTORBPWMSRC htim9
#define MOTORCPWMSRC htim1

#define MOTORBENC htim3
#define MOTORCENC htim4

void DCMotorTestRun() {
	OLED_Clear();
	OLED_Printf(0, 0, "Motor Test");
	OLED_Printf(0, 1, "Press SW1 to start");
	OLED_Refresh_Gram();
	SW1_WhileNotPressed();

	DCMotor lmotor = { 0 };
	DCMotor rmotor = { 0 };
	DCMotor_Init(&lmotor, &MOTORBPWMSRC, false, &MOTORBENC);
	DCMotor_Init(&rmotor, &MOTORCPWMSRC, true, &MOTORCENC);

	DCMotor_Enable(&lmotor);
	DCMotor_Enable(&rmotor);

	// 3 different speeds forward
	DCMotor_SetPWM(&lmotor, 60);
	DCMotor_SetPWM(&rmotor, 60);
	HAL_Delay(2000);

	DCMotor_SetPWM(&lmotor, 90);
	DCMotor_SetPWM(&rmotor, 90);
	HAL_Delay(2000);

	DCMotor_SetPWM(&lmotor, 100);
	DCMotor_SetPWM(&rmotor, 100);
	HAL_Delay(2000);

	// Brake
	DCMotor_Brake(&lmotor);
	DCMotor_Brake(&rmotor);
	HAL_Delay(2000);

	// 3 different speeds reverse
	DCMotor_SetPWM(&lmotor, -60);
	DCMotor_SetPWM(&rmotor, -60);
	HAL_Delay(2000);

	DCMotor_SetPWM(&lmotor, -90);
	DCMotor_SetPWM(&rmotor, -90);
	HAL_Delay(2000);

	DCMotor_SetPWM(&lmotor, -100);
	DCMotor_SetPWM(&rmotor, -100);
	HAL_Delay(2000);

	// Brake
	DCMotor_Brake(&lmotor);
	DCMotor_Brake(&rmotor);
	HAL_Delay(500);

	DCMotor_SetPWM(&lmotor, 0);
	DCMotor_SetPWM(&rmotor, 0);

	int16_t lcount = DCMotor_GetEncoderCount(&lmotor);
	int16_t rcount = DCMotor_GetEncoderCount(&rmotor);

	OLED_Clear();
	OLED_Printf(0, 0, "L:%d", lcount);
	OLED_Printf(0, 1, "R:%d", rcount);
	OLED_Printf(0, 2, "D:%d", (int)lcount - (int)rcount);
	OLED_Refresh_Gram();

}

// Minimum encoder counts to consider the motor moving
#define MIN_TEST_COUNTS		5
#define MIN_MIN_PWM			50
#define MIN_MAX_PWM			65
#define MIN_PWM_TEST_STEP	1

#define PWM_TEST_STEP         1
#define RUN_START_PWM         65

#define RUN_SETTLE_TIME_MS    500U
#define RUN_SAMPLE_TIME_MS    1000U

#define RESPONSE_MIN_PWM      40
#define RESPONSE_MAX_PWM      100
#define RESPONSE_STEP         5
#define RESPONSE_SETTLE_MS    500U
#define RESPONSE_SAMPLE_MS    1000U

#define RESPONSE_POINT_COUNT \
    (((RESPONSE_MAX_PWM - RESPONSE_MIN_PWM) / RESPONSE_STEP) + 1)


typedef struct {
    int8_t pwm;
    int16_t deltaCounts;
} DCMotorResponsePoint;


/*
 * Stored globally so they can easily be inspected in the debugger
 * after the test finishes.
 */
volatile DCMotorResponsePoint responseLeftForward[RESPONSE_POINT_COUNT];
volatile DCMotorResponsePoint responseLeftReverse[RESPONSE_POINT_COUNT];
volatile DCMotorResponsePoint responseRightForward[RESPONSE_POINT_COUNT];
volatile DCMotorResponsePoint responseRightReverse[RESPONSE_POINT_COUNT];

static void DCMotorTest_FindMinimumStartingPWM(DCMotor* rm, int dispRow, int direction) {
	int16_t countOld = 0;

	int16_t count = DCMotor_GetEncoderCount(rm);

	int16_t delta = 0;
	int8_t minPWMVal = 0;

	for (int8_t i = direction * MIN_MIN_PWM; abs(i) <= MIN_MAX_PWM; i += direction * MIN_PWM_TEST_STEP) {
		DCMotor_SetPWM(rm, 0);
		// Wait for motor to become stationary
		do {
			countOld = count;
			HAL_Delay(100U);
			count = DCMotor_GetEncoderCount(rm);
		} while (countOld != count);

		int16_t startCount = DCMotor_GetEncoderCount(rm);
		// test target
		DCMotor_SetPWM(rm, i);
		HAL_Delay(1000U);

		int16_t endCount = DCMotor_GetEncoderCount(rm);

		delta = endCount - startCount;

		OLED_Printf(0, dispRow, "PWM %d%% d %d", i, delta);
		OLED_Refresh_Gram();
		if (abs(delta) > MIN_TEST_COUNTS) {
			minPWMVal = i;
			break;
		}
	}

	// Wait for motor to become stationary
	DCMotor_SetPWM(rm, 0);
	do {
		countOld = count;
		HAL_Delay(100U);
		count = DCMotor_GetEncoderCount(rm);
	} while (countOld != count);

	OLED_Printf(0, dispRow, "PWM %d%% d %d", minPWMVal, delta);

	OLED_Refresh_Gram();
}

void DCMotorTestMinimumPWM() {
	OLED_Clear();
	OLED_Printf(0, 0, "Min Starting PWM");
	OLED_Printf(0, 1, "Press SW1 to start");
	OLED_Refresh_Gram();
	SW1_WhileNotPressed();

	OLED_Printf(0, 1, "Testing forward turn...");
	OLED_Refresh_Gram();

	DCMotor lmotor = { 0 };
	DCMotor rmotor = { 0 };
	DCMotor_Init(&lmotor, &MOTORBPWMSRC, false, &MOTORBENC);
	DCMotor_Init(&rmotor, &MOTORCPWMSRC, true, &MOTORCENC);

	DCMotor_Enable(&lmotor);
	DCMotor_Enable(&rmotor);

	DCMotorTest_FindMinimumStartingPWM(&lmotor, 2, 1);
	DCMotorTest_FindMinimumStartingPWM(&rmotor, 3, 1);

	OLED_Printf(0, 4, "SW1: Reverse");
	OLED_Refresh_Gram();
	SW1_WhileNotPressed();


	OLED_Clear();
	OLED_Printf(0, 0, "Min Starting PWM");
	OLED_Printf(0, 1, "Testing reverse turn...");
	OLED_Refresh_Gram();

	DCMotorTest_FindMinimumStartingPWM(&lmotor, 2, -1);
	DCMotorTest_FindMinimumStartingPWM(&rmotor, 3, -1);

}


static int16_t DCMotorTest_GetDelta(
        DCMotor *motor,
        uint32_t sampleTimeMs)
{
    int16_t startCount = DCMotor_GetEncoderCount(motor);

    HAL_Delay(sampleTimeMs);

    int16_t endCount = DCMotor_GetEncoderCount(motor);

    /*
     * uint16_t subtraction gives the correct wraparound behaviour
     * for the 16-bit encoder counter, provided the actual movement
     * during one sample is less than 32768 counts.
     */
    return (int16_t)((uint16_t)endCount - (uint16_t)startCount);
}

static int8_t DCMotorTest_FindMinimumRunningPWM(
        DCMotor *motor,
        int8_t direction)
{
    int8_t lastMovingPWM = RUN_START_PWM;

    /*
     * Start the motor with plenty of torque.
     */
    DCMotor_SetPWM(motor, direction * RUN_START_PWM);
    HAL_Delay(1000U);

    for (int8_t pwm = RUN_START_PWM;
         pwm >= PWM_TEST_STEP;
         pwm -= PWM_TEST_STEP)
    {
        DCMotor_SetPWM(motor, direction * pwm);

        /*
         * Let speed settle after changing PWM.
         * This also prevents momentum from the previous PWM value
         * being mistaken for sustained motion.
         */
        HAL_Delay(RUN_SETTLE_TIME_MS);

        int16_t delta =
            DCMotorTest_GetDelta(motor, RUN_SAMPLE_TIME_MS);

        OLED_Printf(
            0, 2,
            "PWM %d%% d %d",
            direction * pwm,
            delta
        );
        OLED_Refresh_Gram();

        if (abs(delta) <= MIN_TEST_COUNTS) {
            break;
        }

        lastMovingPWM = pwm;
    }

    DCMotor_SetPWM(motor, 0);

    return direction * lastMovingPWM;
}

void DCMotorTestMinimumRunningPWM()
{
    OLED_Clear();
    OLED_Printf(0, 0, "Min Running PWM");
    OLED_Printf(0, 1, "Press SW1 to start");
    OLED_Refresh_Gram();

    SW1_WhileNotPressed();

    DCMotor lmotor = { 0 };
    DCMotor rmotor = { 0 };

    DCMotor_Init(&lmotor, &MOTORBPWMSRC, false, &MOTORBENC);
    DCMotor_Init(&rmotor, &MOTORCPWMSRC, true, &MOTORCENC);

    DCMotor_Enable(&lmotor);
    DCMotor_Enable(&rmotor);

    OLED_Clear();
    OLED_Printf(0, 0, "Forward");
    OLED_Refresh_Gram();

    int8_t lForward =
        DCMotorTest_FindMinimumRunningPWM(&lmotor, 1);

    int8_t rForward =
        DCMotorTest_FindMinimumRunningPWM(&rmotor, 1);

    OLED_Clear();
    OLED_Printf(0, 0, "Forward");
    OLED_Printf(0, 1, "L: %d%%", lForward);
    OLED_Printf(0, 2, "R: %d%%", rForward);
    OLED_Printf(0, 4, "SW1: reverse");
    OLED_Refresh_Gram();

    SW1_WhileNotPressed();

    OLED_Clear();
    OLED_Printf(0, 0, "Reverse");
    OLED_Refresh_Gram();

    int8_t lReverse =
        DCMotorTest_FindMinimumRunningPWM(&lmotor, -1);

    int8_t rReverse =
        DCMotorTest_FindMinimumRunningPWM(&rmotor, -1);

    OLED_Clear();
    OLED_Printf(0, 0, "Running PWM");
    OLED_Printf(0, 1, "LF: %d%%", lForward);
    OLED_Printf(0, 2, "RF: %d%%", rForward);
    OLED_Printf(0, 3, "LR: %d%%", lReverse);
    OLED_Printf(0, 4, "RR: %d%%", rReverse);
    OLED_Refresh_Gram();
}

static void DCMotorTest_MeasureResponseCurve(
        DCMotor *motor,
        int8_t direction,
        volatile DCMotorResponsePoint *results)
{
    uint8_t index = 0;

    /*
     * Start motor reliably before beginning the descending sweep.
     */
    DCMotor_SetPWM(motor, direction * RESPONSE_MAX_PWM);
    HAL_Delay(1000U);

    for (int8_t pwm = RESPONSE_MAX_PWM;
         pwm >= RESPONSE_MIN_PWM;
         pwm -= RESPONSE_STEP)
    {
        DCMotor_SetPWM(motor, direction * pwm);

        /*
         * Exclude acceleration/deceleration transient from the
         * actual measurement.
         */
        HAL_Delay(RESPONSE_SETTLE_MS);

        int16_t delta =
            DCMotorTest_GetDelta(motor, RESPONSE_SAMPLE_MS);

        results[index].pwm = direction * pwm;
        results[index].deltaCounts = delta;
        index++;

        OLED_Printf(
            0, 2,
            "PWM %d%%",
            direction * pwm
        );
        OLED_Printf(
            0, 3,
            "dCount %d",
            delta
        );
        OLED_Refresh_Gram();
    }

    DCMotor_SetPWM(motor, 0);
}

void DCMotorTestResponseCurve()
{
    OLED_Clear();
    OLED_Printf(0, 0, "Response Curve");
    OLED_Printf(0, 1, "Press SW1 to start");
    OLED_Refresh_Gram();

    SW1_WhileNotPressed();

    DCMotor lmotor = { 0 };
    DCMotor rmotor = { 0 };

    DCMotor_Init(&lmotor, &MOTORBPWMSRC, false, &MOTORBENC);
    DCMotor_Init(&rmotor, &MOTORCPWMSRC, true, &MOTORCENC);

    DCMotor_Enable(&lmotor);
    DCMotor_Enable(&rmotor);

    OLED_Clear();
    OLED_Printf(0, 0, "Left forward");
    OLED_Refresh_Gram();

    DCMotorTest_MeasureResponseCurve(
        &lmotor,
        1,
        responseLeftForward
    );

    OLED_Clear();
    OLED_Printf(0, 0, "Left reverse");
    OLED_Refresh_Gram();

    DCMotorTest_MeasureResponseCurve(
        &lmotor,
        -1,
        responseLeftReverse
    );

    OLED_Clear();
    OLED_Printf(0, 0, "Right forward");
    OLED_Refresh_Gram();

    DCMotorTest_MeasureResponseCurve(
        &rmotor,
        1,
        responseRightForward
    );

    OLED_Clear();
    OLED_Printf(0, 0, "Right reverse");
    OLED_Refresh_Gram();

    DCMotorTest_MeasureResponseCurve(
        &rmotor,
        -1,
        responseRightReverse
    );

    OLED_Clear();
    OLED_Printf(0, 0, "Response complete");
    OLED_Printf(0, 1, "Inspect arrays");
    OLED_Printf(0, 2, "in debugger");
    OLED_Refresh_Gram();
}
