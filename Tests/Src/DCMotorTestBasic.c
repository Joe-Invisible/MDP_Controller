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
#include "WheelSpeedController.h"
#include <math.h>

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

/*
 * Brake response characterization.
 *
 * Each run:
 *   1. Drives both motors at a fixed PWM.
 *   2. Waits for speed to settle.
 *   3. Records one pre-brake speed sample.
 *   4. Applies the selected brake PWM.
 *   5. Records the speed decay at 10 ms intervals.
 *
 * 0% brake is intentionally included as the coast baseline.
 */

#define BRAKE_RESPONSE_DRIVE_PWM         65.0f
#define BRAKE_RESPONSE_SETTLE_MS        	1000U

#define BRAKE_RESPONSE_SAMPLE_MS         10U
#define BRAKE_RESPONSE_OBSERVE_MS       	1000U

#define BRAKE_RESPONSE_RESET_BRAKE_MS    300U
#define BRAKE_RESPONSE_RESET_NEUTRAL_MS  300U

#define BRAKE_RESPONSE_PHASE_DRIVE       0U
#define BRAKE_RESPONSE_PHASE_BRAKE       1U

static const float brakeResponseLevels[] = {
    0.0f,       // coast reference
    80.0f,
    82.5f,
    85.0f,
    87.5f,
    90.0f,
    92.5f,
    95.0f,
    97.5f,
    100.0f
};

#define BRAKE_RESPONSE_LEVEL_COUNT \
    (sizeof(brakeResponseLevels) / sizeof(brakeResponseLevels[0]))

#define BRAKE_RESPONSE_REPETITIONS	3U

#define BRAKE_RESPONSE_SAMPLES_PER_LEVEL \
    (1U + (BRAKE_RESPONSE_OBSERVE_MS / BRAKE_RESPONSE_SAMPLE_MS))

#define BRAKE_RESPONSE_MAX_SAMPLES \
    (BRAKE_RESPONSE_LEVEL_COUNT * BRAKE_RESPONSE_SAMPLES_PER_LEVEL * BRAKE_RESPONSE_REPETITIONS)


typedef struct {
    uint32_t timeMs;

    float brakePercent;

    float leftSpeedCps;
    float rightSpeedCps;

    int16_t leftDeltaCounts;
    int16_t rightDeltaCounts;

    uint16_t dtMs;

    uint8_t levelIndex;
    uint8_t phase;

} DCMotorBrakeResponseSample;


/*
 * Stored globally for debugger export.
 */
volatile DCMotorBrakeResponseSample
    brakeResponseLog[BRAKE_RESPONSE_MAX_SAMPLES];

volatile uint32_t brakeResponseLogCount = 0;

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


static int16_t DCMotorTest_EncoderDelta(
        int16_t current,
        int16_t previous)
{
    /*
     * Preserve correct 16-bit timer wraparound behaviour.
     */
    return (int16_t)(
        (uint16_t)current -
        (uint16_t)previous
    );
}


static void DCMotorTest_RecordBrakeResponse(
        uint8_t levelIndex,
        float brakePercent,
        uint8_t phase,
        uint32_t timeMs,
        uint32_t dtMs,
        int16_t leftCurrent,
        int16_t leftPrevious,
        int16_t rightCurrent,
        int16_t rightPrevious)
{
    if (brakeResponseLogCount >= BRAKE_RESPONSE_MAX_SAMPLES)
        return;

    DCMotorBrakeResponseSample *sample =
        (DCMotorBrakeResponseSample *)
        &brakeResponseLog[brakeResponseLogCount++];

    int16_t leftDelta =
        DCMotorTest_EncoderDelta(
            leftCurrent,
            leftPrevious);

    int16_t rightDelta =
        DCMotorTest_EncoderDelta(
            rightCurrent,
            rightPrevious);

    sample->timeMs = timeMs;
    sample->brakePercent = brakePercent;

    sample->leftDeltaCounts = leftDelta;
    sample->rightDeltaCounts = rightDelta;

    sample->dtMs = (uint16_t)dtMs;

    if (dtMs > 0U)
    {
        float cpsScale =
            1000.0f / (float)dtMs;

        sample->leftSpeedCps =
            (float)leftDelta * cpsScale;

        sample->rightSpeedCps =
            (float)rightDelta * cpsScale;
    }
    else
    {
        sample->leftSpeedCps = 0.0f;
        sample->rightSpeedCps = 0.0f;
    }

    sample->levelIndex = levelIndex;
    sample->phase = phase;
}


void DCMotorTestBrakeResponse(void)
{
    OLED_Clear();
    OLED_Printf(0, 0, "Brake Response");
    OLED_Printf(0, 1, "Lift rear wheels");
    OLED_Printf(0, 2, "SW1 to start");
    OLED_Refresh_Gram();

    SW1_WhileNotPressed();

    DCMotor lmotor = { 0 };
    DCMotor rmotor = { 0 };

    DCMotor_Init(
        &lmotor,
        &MOTORBPWMSRC,
        false,
        &MOTORBENC);

    DCMotor_Init(
        &rmotor,
        &MOTORCPWMSRC,
        true,
        &MOTORCENC);

    DCMotor_Enable(&lmotor);
    DCMotor_Enable(&rmotor);

    brakeResponseLogCount = 0;

    for (uint8_t level = 0;
         level < BRAKE_RESPONSE_LEVEL_COUNT;
         level++)
    {
    		for (uint8_t _ = 0; _ < BRAKE_RESPONSE_REPETITIONS; _++) {
			float brakePercent =
				brakeResponseLevels[level];

			/*
			 * Ensure each trial starts from a reproducible
			 * stationary condition.
			 */
			DCMotor_Brake(&lmotor);
			DCMotor_Brake(&rmotor);

			HAL_Delay(BRAKE_RESPONSE_RESET_BRAKE_MS);

			DCMotor_Neutral(&lmotor);
			DCMotor_Neutral(&rmotor);

			HAL_Delay(BRAKE_RESPONSE_RESET_NEUTRAL_MS);

			OLED_Clear();
			OLED_Printf(0, 0, "Brake Response");
			OLED_Printf(
				0, 1,
				"Run %d/%d",
				(int)level + 1,
				(int)BRAKE_RESPONSE_LEVEL_COUNT);
			OLED_Printf(
				0, 2,
				"Brake %d%%",
				(int)brakePercent);
			OLED_Printf(
				0, 3,
				"Drive %d%%",
				(int)BRAKE_RESPONSE_DRIVE_PWM);
			OLED_Refresh_Gram();

			/*
			 * Bring both wheels to approximately the same
			 * initial operating condition.
			 */
			DCMotor_SetPWM(
				&lmotor,
				BRAKE_RESPONSE_DRIVE_PWM);

			DCMotor_SetPWM(
				&rmotor,
				BRAKE_RESPONSE_DRIVE_PWM);

			HAL_Delay(BRAKE_RESPONSE_SETTLE_MS);

			/*
			 * Measure one 10 ms sample immediately before
			 * the brake command. This becomes t = 0.
			 */
			int16_t leftPrevious =
				DCMotor_GetEncoderCount(&lmotor);

			int16_t rightPrevious =
				DCMotor_GetEncoderCount(&rmotor);

			uint32_t previousTick =
				HAL_GetTick();

			HAL_Delay(BRAKE_RESPONSE_SAMPLE_MS);

			uint32_t now =
				HAL_GetTick();

			int16_t leftCurrent =
				DCMotor_GetEncoderCount(&lmotor);

			int16_t rightCurrent =
				DCMotor_GetEncoderCount(&rmotor);

			DCMotorTest_RecordBrakeResponse(
				level,
				brakePercent,
				BRAKE_RESPONSE_PHASE_DRIVE,
				0U,
				now - previousTick,
				leftCurrent,
				leftPrevious,
				rightCurrent,
				rightPrevious);

			leftPrevious = leftCurrent;
			rightPrevious = rightCurrent;
			previousTick = now;

			/*
			 * Begin the actual brake/coast trial.
			 *
			 * For brakePercent == 0 this deliberately enters
			 * NEUTRAL, giving us the coast reference trace.
			 */
			DCMotor_SetBrakePWM(
				&lmotor,
				brakePercent);

			DCMotor_SetBrakePWM(
				&rmotor,
				brakePercent);

			uint32_t brakeStartTick =
				HAL_GetTick();

			while ((HAL_GetTick() - brakeStartTick) <
				   BRAKE_RESPONSE_OBSERVE_MS)
			{
				HAL_Delay(BRAKE_RESPONSE_SAMPLE_MS);

				now = HAL_GetTick();

				leftCurrent =
					DCMotor_GetEncoderCount(&lmotor);

				rightCurrent =
					DCMotor_GetEncoderCount(&rmotor);

				uint32_t dtMs =
					now - previousTick;

				DCMotorTest_RecordBrakeResponse(
					level,
					brakePercent,
					BRAKE_RESPONSE_PHASE_BRAKE,
					now - brakeStartTick,
					dtMs,
					leftCurrent,
					leftPrevious,
					rightCurrent,
					rightPrevious);

				leftPrevious = leftCurrent;
				rightPrevious = rightCurrent;
				previousTick = now;
			}

			/*
			 * Whatever the tested brake magnitude was,
			 * stop the wheels decisively before the next run.
			 */
			DCMotor_Brake(&lmotor);
			DCMotor_Brake(&rmotor);

			HAL_Delay(BRAKE_RESPONSE_RESET_BRAKE_MS);

			DCMotor_Neutral(&lmotor);
			DCMotor_Neutral(&rmotor);

			HAL_Delay(BRAKE_RESPONSE_RESET_NEUTRAL_MS);
    		}
    }

    DCMotor_Disable(&lmotor);
    DCMotor_Disable(&rmotor);

    OLED_Clear();
    OLED_Printf(0, 0, "Brake complete");
    OLED_Printf(
        0, 1,
        "%lu samples",
        brakeResponseLogCount);
    OLED_Printf(0, 2, "Inspect debugger");
    OLED_Refresh_Gram();
}

/*
 * DCMotor brake map characterization
 *
 * The WheelSpeedController is used only to establish a repeatable
 * initial wheel speed. Once braking begins, it is no longer updated.
 *
 * Rear wheels should be lifted for this test.
 */

/* ---------------------------------------------------------------
 * Test grid
 * --------------------------------------------------------------- */

static const float brakeMapTargetSpeedsCps[] = {
    500.0f,
    750.0f,
    1000.0f,
    1500.0f,
    2000.0f
};

static const float brakeMapBrakePercents[] = {
    0.0f,       /* Coast reference */
    85.0f,
    87.5f,
    90.0f,
    92.5f,
    95.0f,
    97.5f,
    100.0f
};

#define BRAKE_MAP_SPEED_COUNT \
    (sizeof(brakeMapTargetSpeedsCps) / \
     sizeof(brakeMapTargetSpeedsCps[0]))

#define BRAKE_MAP_BRAKE_COUNT \
    (sizeof(brakeMapBrakePercents) / \
     sizeof(brakeMapBrakePercents[0]))

#define BRAKE_MAP_REPEAT_COUNT              3U

/* ---------------------------------------------------------------
 * Timing
 * --------------------------------------------------------------- */

#define BRAKE_MAP_CONTROL_PERIOD_MS        10U

#define BRAKE_MAP_SETTLE_MS              1000U
#define BRAKE_MAP_PREBRAKE_AVG_MS         200U

/*
 * 20 ms gives better encoder-speed resolution than the previous
 * ~10 ms samples while retaining enough temporal resolution for
 * the braking transient.
 */
#define BRAKE_MAP_TRACE_SAMPLE_MS           20U
#define BRAKE_MAP_TRACE_DURATION_MS        700U

#define BRAKE_MAP_RESET_BRAKE_MS           300U
#define BRAKE_MAP_RESET_NEUTRAL_MS         200U

#define BRAKE_MAP_TRIAL_COUNT \
    (BRAKE_MAP_SPEED_COUNT * \
     BRAKE_MAP_BRAKE_COUNT * \
     BRAKE_MAP_REPEAT_COUNT)

#define BRAKE_MAP_MAX_TRACE_PER_TRIAL \
    ((BRAKE_MAP_TRACE_DURATION_MS / \
      BRAKE_MAP_TRACE_SAMPLE_MS) + 2U)

#define BRAKE_MAP_MAX_TRACE_SAMPLES \
    (BRAKE_MAP_TRIAL_COUNT * \
     BRAKE_MAP_MAX_TRACE_PER_TRIAL)


/* ---------------------------------------------------------------
 * Controller configuration used only to establish initial speed
 * --------------------------------------------------------------- */

#define BRAKE_MAP_WSC_KP          0.02f
#define BRAKE_MAP_WSC_KI          0.00f

#define BRAKE_MAP_WSC_MIN_FB    -100.0f
#define BRAKE_MAP_WSC_MAX_FB     100.0f


static const WheelSpeedCalibration brakeMapLeftCalibration = {
    .forwardSlope     = 203.88f,
    .forwardOffset    = 53.03f,

    .reverseSlope     = 191.88f,
    .reverseOffset    = 53.44f,

    .startForwardPWM  = 55.0f,
    .startReversePWM  = 56.0f,

    .runForwardPWM    = 54.0f,
    .runReversePWM    = 55.0f
};


static const WheelSpeedCalibration brakeMapRightCalibration = {
    .forwardSlope     = 192.56f,
    .forwardOffset    = 53.37f,

    .reverseSlope     = 198.32f,
    .reverseOffset    = 53.49f,

    .startForwardPWM  = 56.0f,
    .startReversePWM  = 56.0f,

    .runForwardPWM    = 54.0f,
    .runReversePWM    = 55.0f
};


/* ---------------------------------------------------------------
 * Logging
 * --------------------------------------------------------------- */

/*
 * One summary per complete braking trial.
 */
typedef struct
{
    float targetSpeedCps;
    float brakePercent;

    float leftInitialMeanCps;
    float rightInitialMeanCps;

    float leftInitialStdCps;
    float rightInitialStdCps;

    int16_t leftRunOnCounts;
    int16_t rightRunOnCounts;

    uint16_t traceStartIndex;
    uint16_t traceCount;

    uint8_t speedIndex;
    uint8_t brakeIndex;
    uint8_t repeatIndex;

} DCMotorBrakeMapTrial;


/*
 * Compact trace representation.
 *
 * CPS is deliberately NOT stored because:
 *
 *      speedCps = deltaCounts * 1000 / dtMs
 *
 * Keeping raw encoder deltas avoids wasting RAM and preserves
 * the underlying measurement.
 */
typedef struct
{
    uint16_t timeMs;

    int16_t leftDeltaCounts;
    int16_t rightDeltaCounts;

    uint8_t dtMs;

    uint8_t speedIndex;
    uint8_t brakeIndex;
    uint8_t repeatIndex;

} DCMotorBrakeMapTrace;


/*
 * Global so the arrays can be exported through GDB.
 */
volatile DCMotorBrakeMapTrial
    brakeMapTrialLog[BRAKE_MAP_TRIAL_COUNT];

volatile DCMotorBrakeMapTrace
    brakeMapTraceLog[BRAKE_MAP_MAX_TRACE_SAMPLES];

volatile uint16_t brakeMapTrialLogCount = 0;
volatile uint16_t brakeMapTraceLogCount = 0;


typedef struct
{
    float leftMean;
    float rightMean;

    float leftStd;
    float rightStd;

} BrakeMapSpeedStats;


/*
 * Run both wheel-speed controllers for the requested duration.
 *
 * If stats != NULL, all measurements generated during the interval
 * are accumulated to obtain mean and standard deviation.
 */
static void DCMotorTest_RunSpeedControl(
        WheelSpeedController *leftController,
        WheelSpeedController *rightController,
        uint32_t durationMs,
        BrakeMapSpeedStats *stats)
{
    uint32_t startTick = HAL_GetTick();
    uint32_t previousTick = startTick;

    float leftSum = 0.0f;
    float rightSum = 0.0f;

    float leftSumSq = 0.0f;
    float rightSumSq = 0.0f;

    uint32_t sampleCount = 0U;

    while ((HAL_GetTick() - startTick) < durationMs)
    {
        uint32_t now = HAL_GetTick();

        if ((now - previousTick) <
            BRAKE_MAP_CONTROL_PERIOD_MS)
        {
            HAL_Delay(1U);
            continue;
        }

        uint32_t dtMs = now - previousTick;
        previousTick = now;

        float dt = (float)dtMs / 1000.0f;

        WheelSpeedController_Update(
            leftController,
            dt);

        WheelSpeedController_Update(
            rightController,
            dt);

        if (stats != NULL)
        {
            float left =
                leftController->measuredSpeedCps;

            float right =
                rightController->measuredSpeedCps;

            leftSum += left;
            rightSum += right;

            leftSumSq += left * left;
            rightSumSq += right * right;

            sampleCount++;
        }
    }

    if (stats == NULL)
        return;

    if (sampleCount == 0U)
    {
        *stats = (BrakeMapSpeedStats){0};
        return;
    }

    float n = (float)sampleCount;

    stats->leftMean = leftSum / n;
    stats->rightMean = rightSum / n;

    float leftVariance =
        leftSumSq / n -
        stats->leftMean * stats->leftMean;

    float rightVariance =
        rightSumSq / n -
        stats->rightMean * stats->rightMean;

    /*
     * Protect sqrtf() against tiny negative values caused by
     * floating-point rounding.
     */
    if (leftVariance < 0.0f)
        leftVariance = 0.0f;

    if (rightVariance < 0.0f)
        rightVariance = 0.0f;

    stats->leftStd = sqrtf(leftVariance);
    stats->rightStd = sqrtf(rightVariance);
}


static void DCMotorTest_RecordBrakeMapTrace(
        uint8_t speedIndex,
        uint8_t brakeIndex,
        uint8_t repeatIndex,
        uint32_t timeMs,
        uint32_t dtMs,
        int16_t leftCurrent,
        int16_t leftPrevious,
        int16_t rightCurrent,
        int16_t rightPrevious)
{
    if (brakeMapTraceLogCount >=
        BRAKE_MAP_MAX_TRACE_SAMPLES)
    {
        return;
    }

    volatile DCMotorBrakeMapTrace *sample =
        &brakeMapTraceLog[brakeMapTraceLogCount++];

    sample->timeMs = (uint16_t)timeMs;

    sample->leftDeltaCounts =
        DCMotorTest_EncoderDelta(
            leftCurrent,
            leftPrevious);

    sample->rightDeltaCounts =
        DCMotorTest_EncoderDelta(
            rightCurrent,
            rightPrevious);

    sample->dtMs =
        dtMs > 255U ?
        255U :
        (uint8_t)dtMs;

    sample->speedIndex = speedIndex;
    sample->brakeIndex = brakeIndex;
    sample->repeatIndex = repeatIndex;
}


/*
 * Alternate the ordering of experiments between repetitions.
 *
 * This prevents a gradual battery-voltage or motor-temperature
 * change from being perfectly correlated with increasing speed
 * or increasing brake duty.
 */
static uint8_t DCMotorTest_MapOrderedIndex(
        uint8_t position,
        uint8_t count,
        bool reverse)
{
    if (!reverse)
        return position;

    return (uint8_t)(count - 1U - position);
}


/* ---------------------------------------------------------------
 * Main brake-map test
 * --------------------------------------------------------------- */

void DCMotorTestBrakeMap(void)
{
    OLED_Clear();
    OLED_Printf(0, 0, "Brake Map");
    OLED_Printf(0, 1, "Lift rear wheels");
    OLED_Printf(0, 2, "SW1 to start");
    OLED_Refresh_Gram();

    SW1_WhileNotPressed();

    DCMotor lmotor = {0};
    DCMotor rmotor = {0};

    WheelSpeedController leftController = {0};
    WheelSpeedController rightController = {0};

    DCMotor_Init(
        &lmotor,
        &MOTORBPWMSRC,
        false,
        &MOTORBENC);

    DCMotor_Init(
        &rmotor,
        &MOTORCPWMSRC,
        true,
        &MOTORCENC);

    DCMotor_Enable(&lmotor);
    DCMotor_Enable(&rmotor);

    WheelSpeedController_Init(
        &leftController,
        &lmotor,
        BRAKE_MAP_WSC_KP,
        BRAKE_MAP_WSC_KI,
        BRAKE_MAP_WSC_MIN_FB,
        BRAKE_MAP_WSC_MAX_FB,
        &brakeMapLeftCalibration);

    WheelSpeedController_Init(
        &rightController,
        &rmotor,
        BRAKE_MAP_WSC_KP,
        BRAKE_MAP_WSC_KI,
        BRAKE_MAP_WSC_MIN_FB,
        BRAKE_MAP_WSC_MAX_FB,
        &brakeMapRightCalibration);

    brakeMapTrialLogCount = 0U;
    brakeMapTraceLogCount = 0U;


    for (uint8_t repeat = 0;
         repeat < BRAKE_MAP_REPEAT_COUNT;
         repeat++)
    {
        /*
         * Reverse the speed order on the middle repetition.
         */
        bool reverseSpeedOrder =
            (repeat == 1U);

        for (uint8_t speedPosition = 0;
             speedPosition < BRAKE_MAP_SPEED_COUNT;
             speedPosition++)
        {
            uint8_t speedIndex =
                DCMotorTest_MapOrderedIndex(
                    speedPosition,
                    BRAKE_MAP_SPEED_COUNT,
                    reverseSpeedOrder);

            float targetSpeed =
                brakeMapTargetSpeedsCps[speedIndex];

            /*
             * Alternate brake ordering as well.
             */
            bool reverseBrakeOrder =
                ((repeat + speedPosition) & 1U) != 0U;

            for (uint8_t brakePosition = 0;
                 brakePosition < BRAKE_MAP_BRAKE_COUNT;
                 brakePosition++)
            {
                uint8_t brakeIndex =
                    DCMotorTest_MapOrderedIndex(
                        brakePosition,
                        BRAKE_MAP_BRAKE_COUNT,
                        reverseBrakeOrder);

                float brakePercent =
                    brakeMapBrakePercents[brakeIndex];


                /* ------------------------------------------------
                 * Reset to stationary
                 * ------------------------------------------------ */

                DCMotor_Brake(&lmotor);
                DCMotor_Brake(&rmotor);

                HAL_Delay(
                    BRAKE_MAP_RESET_BRAKE_MS);

                DCMotor_Neutral(&lmotor);
                DCMotor_Neutral(&rmotor);

                HAL_Delay(
                    BRAKE_MAP_RESET_NEUTRAL_MS);

                /*
                 * Reset controller PI and encoder history.
                 */
                WheelSpeedController_Stop(
                    &leftController);

                WheelSpeedController_Stop(
                    &rightController);


                /* ------------------------------------------------
                 * Establish target speed
                 * ------------------------------------------------ */

                WheelSpeedController_SetTarget(
                    &leftController,
                    targetSpeed);

                WheelSpeedController_SetTarget(
                    &rightController,
                    targetSpeed);

                OLED_Clear();
                OLED_Printf(0, 0, "Brake Map");

                OLED_Printf(
                    0, 1,
                    "V:%d cps",
                    (int)targetSpeed);

                OLED_Printf(
                    0, 2,
                    "B:%d.%d%%",
                    (int)brakePercent,
                    ((int)(brakePercent * 10.0f)) % 10);

                OLED_Printf(
                    0, 3,
                    "Rep:%d/%d",
                    (int)repeat + 1,
                    (int)BRAKE_MAP_REPEAT_COUNT);

                OLED_Printf(
                    0, 4,
                    "Trial:%d/%d",
                    (int)brakeMapTrialLogCount + 1,
                    (int)BRAKE_MAP_TRIAL_COUNT);

                OLED_Refresh_Gram();


                /*
                 * Allow speed controller to settle.
                 */
                DCMotorTest_RunSpeedControl(
                    &leftController,
                    &rightController,
                    BRAKE_MAP_SETTLE_MS,
                    NULL);


                /*
                 * Measure actual initial speed over 200 ms.
                 */
                BrakeMapSpeedStats initialStats = {0};

                DCMotorTest_RunSpeedControl(
                    &leftController,
                    &rightController,
                    BRAKE_MAP_PREBRAKE_AVG_MS,
                    &initialStats);


                /* ------------------------------------------------
                 * Begin braking
                 * ------------------------------------------------ */

                int16_t leftStart =
                    DCMotor_GetEncoderCount(
                        &lmotor);

                int16_t rightStart =
                    DCMotor_GetEncoderCount(
                        &rmotor);

                int16_t leftPrevious =
                    leftStart;

                int16_t rightPrevious =
                    rightStart;

                uint16_t traceStart =
                    brakeMapTraceLogCount;

                uint32_t brakeStart =
                    HAL_GetTick();

                uint32_t previousTick =
                    brakeStart;

                /*
                 * From this point onward:
                 *
                 * DO NOT call WheelSpeedController_Update().
                 *
                 * We are measuring the raw DCMotor brake response.
                 */
                DCMotor_SetBrakePWM(
                    &lmotor,
                    brakePercent);

                DCMotor_SetBrakePWM(
                    &rmotor,
                    brakePercent);


                while ((HAL_GetTick() - brakeStart) <
                       BRAKE_MAP_TRACE_DURATION_MS)
                {
                    HAL_Delay(
                        BRAKE_MAP_TRACE_SAMPLE_MS);

                    uint32_t now =
                        HAL_GetTick();

                    uint32_t dtMs =
                        now - previousTick;

                    int16_t leftCurrent =
                        DCMotor_GetEncoderCount(
                            &lmotor);

                    int16_t rightCurrent =
                        DCMotor_GetEncoderCount(
                            &rmotor);

                    DCMotorTest_RecordBrakeMapTrace(
                        speedIndex,
                        brakeIndex,
                        repeat,
                        now - brakeStart,
                        dtMs,
                        leftCurrent,
                        leftPrevious,
                        rightCurrent,
                        rightPrevious);

                    leftPrevious =
                        leftCurrent;

                    rightPrevious =
                        rightCurrent;

                    previousTick =
                        now;
                }


                int16_t leftFinal =
                    DCMotor_GetEncoderCount(
                        &lmotor);

                int16_t rightFinal =
                    DCMotor_GetEncoderCount(
                        &rmotor);


                /* ------------------------------------------------
                 * Record trial summary
                 * ------------------------------------------------ */

                if (brakeMapTrialLogCount <
                    BRAKE_MAP_TRIAL_COUNT)
                {
                    volatile DCMotorBrakeMapTrial *trial =
                        &brakeMapTrialLog[
                            brakeMapTrialLogCount++];

                    trial->targetSpeedCps =
                        targetSpeed;

                    trial->brakePercent =
                        brakePercent;

                    trial->leftInitialMeanCps =
                        initialStats.leftMean;

                    trial->rightInitialMeanCps =
                        initialStats.rightMean;

                    trial->leftInitialStdCps =
                        initialStats.leftStd;

                    trial->rightInitialStdCps =
                        initialStats.rightStd;

                    trial->leftRunOnCounts =
                        DCMotorTest_EncoderDelta(
                            leftFinal,
                            leftStart);

                    trial->rightRunOnCounts =
                        DCMotorTest_EncoderDelta(
                            rightFinal,
                            rightStart);

                    trial->traceStartIndex =
                        traceStart;

                    trial->traceCount =
                        brakeMapTraceLogCount -
                        traceStart;

                    trial->speedIndex =
                        speedIndex;

                    trial->brakeIndex =
                        brakeIndex;

                    trial->repeatIndex =
                        repeat;
                }
            }
        }
    }


    /* ------------------------------------------------------------
     * Safe final state
     * ------------------------------------------------------------ */

    DCMotor_Brake(&lmotor);
    DCMotor_Brake(&rmotor);

    HAL_Delay(500U);

    DCMotor_Disable(&lmotor);
    DCMotor_Disable(&rmotor);


    OLED_Clear();
    OLED_Printf(0, 0, "Brake Map Done");

    OLED_Printf(
        0, 1,
        "Trials:%d",
        brakeMapTrialLogCount);

    OLED_Printf(
        0, 2,
        "Trace:%d",
        brakeMapTraceLogCount);

    OLED_Printf(0, 3, "Export debugger");

    OLED_Refresh_Gram();
}

