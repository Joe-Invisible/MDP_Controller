/*
 * WheelSpeedControllerTest.c
 *
 *  Created on: 2026年8月29日
 *      Author: Joe
 */


#include "WheelSpeedControllerTest.h"
#include "oledutils.h"
#include "userbutton.h"
#include "tim.h"
#include <math.h>

#define MOTORBPWMSRC htim9
#define MOTORCPWMSRC htim1

#define MOTORBENC htim3
#define MOTORCENC htim4

// See kp_analysis in exp/
#define TEST_KP		0.02f
#define TEST_KI		0.0f

/* Controller runs at 100 Hz. */
#define CONTROL_PERIOD_MS      10U

/* Update display much more slowly than the control loop. */
#define DISPLAY_PERIOD_MS      100U

/* Hold each target for this long. */
#define TARGET_DURATION_MS     3000U


/*
 * Speeds are deliberately kept well inside the experimentally
 * measured operating range for this initial test.
 */
static const float testTargets[] = {
     2000.0f,
     4000.0f,
     6000.0f,
        0.0f,
    -2000.0f,
    -4000.0f,
    -6000.0f,
        0.0f
};

#define NUM_TEST_TARGETS \
    (sizeof(testTargets) / sizeof(testTargets[0]))

#define DEBUGLOG 0

#if DEBUGLOG == 1
typedef struct {
    float targetCps;
    float measuredCps;
    float pwm;
    float error;
    float pidOut;
} WheelDebugSample;

#define DEBUG_SAMPLE_COUNT (NUM_TEST_TARGETS * TARGET_DURATION_MS / CONTROL_PERIOD_MS)

static WheelDebugSample debugSamples[DEBUG_SAMPLE_COUNT];
static int debugSampleIndex = 0;
#endif

static void WheelSpeedControllerTest_RunTarget(
        WheelSpeedController *controller,
        float targetCps)
{
    WheelSpeedController_SetTarget(controller, targetCps);

    /* This is a temporary fix to account for the 500ms HAL_Delay in the testing loop
     * In actual deployment the motion control task will run periodically and there
     * should be no such mismatch.
     */
    controller->previousEncoderCount =
            DCMotor_GetEncoderCount(controller->motor);

    uint32_t startTick   = HAL_GetTick();
    uint32_t lastControl = startTick;
    uint32_t lastDisplay = startTick;

    while ((HAL_GetTick() - startTick) < TARGET_DURATION_MS)
    {
        uint32_t now = HAL_GetTick();

        /*
         * Run the control loop approximately every 10 ms.
         *
         * Use the actual elapsed time rather than assuming that
         * exactly 10 ms has elapsed.
         */
        if ((now - lastControl) >= CONTROL_PERIOD_MS)
        {
            float dt =
                    (float)(now - lastControl) / 1000.0f;

            lastControl = now;

            WheelSpeedController_Update(
                    controller,
                    dt);

#if DEBUGLOG == 1
            debugSamples[debugSampleIndex].error = controller->pid.previousError;
            debugSamples[debugSampleIndex].measuredCps = controller->measuredSpeedCps;
            debugSamples[debugSampleIndex].pidOut = controller->pid.output;
            debugSamples[debugSampleIndex].pwm = controller->outputPWM;
            debugSamples[debugSampleIndex].targetCps = controller->targetSpeedCps;
            debugSampleIndex++;
#endif	/* DEBUGLOG */
        }

#if DEBUGLOG == 0	/* Do not refresh display if logging debug data */
        /*
         * OLED refreshes are relatively slow, so do not perform
         * them at control-loop frequency.
         */
        if ((now - lastDisplay) >= DISPLAY_PERIOD_MS)
        {
            lastDisplay = now;

            OLED_Printf(
                    0, 0,
                    "Target: %.0f ",
                    controller->targetSpeedCps);

            OLED_Printf(
                    0, 1,
                    "Speed:  %.0f ",
                    controller->measuredSpeedCps);

            OLED_Printf(
                    0, 2,
                    "Error:  %.0f ",
                    controller->targetSpeedCps -
                    controller->measuredSpeedCps);

            OLED_Printf(
                    0, 3,
                    "PID:    %.2f ",
                    controller->pid.output);
            OLED_Printf(
					0, 4,
					"PWM:    %.2f ",
					controller->outputPWM);

            OLED_Refresh_Gram();
        }
#endif	/* DEBUGLOG */
    }
}


static void WheelSpeedControllerTest_RunMotor(
        DCMotor *motor,
        const WheelSpeedCalibration *calibration,
        const char *motorName)
{
    WheelSpeedController controller = {0};

    if (!WheelSpeedController_Init(
            &controller,
            motor,
			TEST_KP, TEST_KI,
            calibration))
    {
        OLED_Clear();
        OLED_Printf(0, 0, "WSC Init failed");
        OLED_Refresh_Gram();
        return;
    }


    for (uint32_t i = 0; i < NUM_TEST_TARGETS; i++)
    {
#if DEBUGLOG == 0
        OLED_Clear();
        OLED_Printf(0, 0, "%s", motorName);
        OLED_Printf(
                0, 1,
                "Next: %.0f cps",
                testTargets[i]);
        OLED_Refresh_Gram();

        HAL_Delay(500U);
#endif
        WheelSpeedControllerTest_RunTarget(
                &controller,
                testTargets[i]);
    }


    WheelSpeedController_Stop(&controller);

    OLED_Clear();
    OLED_Printf(0, 0, "%s complete", motorName);
    OLED_Refresh_Gram();
}

void WheelSpeedControllerTestRun() {
	DCMotor lmotor = { 0 };
	DCMotor rmotor = { 0 };
	DCMotor_Init(&lmotor, &MOTORBPWMSRC, false, &MOTORBENC);
	DCMotor_Init(&rmotor, &MOTORCPWMSRC, true, &MOTORCENC);

	DCMotor_Enable(&lmotor);
	DCMotor_Enable(&rmotor);

	OLED_Clear();
	OLED_Printf(0, 0, "WSCtrl Test");
	OLED_Printf(0, 1, "SW1: Left Motor");
	OLED_Refresh_Gram();
	// SW1_WhileNotPressed();

	WheelSpeedControllerTest_RunMotor(&lmotor, &leftCalibration, "LEFT");

    OLED_Clear();
    OLED_Printf(0, 0, "Right complete");
    OLED_Printf(0, 1, "SW1: Right motor");
    OLED_Refresh_Gram();

    // SW1_WhileNotPressed();
    WheelSpeedControllerTest_RunMotor(&rmotor, &rightCalibration, "RIGHT");


    OLED_Clear();
    OLED_Printf(0, 0, "Test complete");
    OLED_Refresh_Gram();

}
