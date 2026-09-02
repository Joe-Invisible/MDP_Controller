/*
 * MotionControllerTest.c
 *
 *  Created on: 2026年8月31日
 *      Author: Joe
 */

#include "MotionControllerTest.h"
#include "oledutils.h"
#include "userbutton.h"
#include "RobotTestFixture.h"

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* Test configuration                                                         */
/* -------------------------------------------------------------------------- */

#define MOTION_TEST_DISTANCE_MM          (1000.0f)
#define MOTION_TEST_SPEED_CPS            (2000.0f)

#define MOTION_TEST_CONTROL_PERIOD_MS    (10U)
#define MOTION_TEST_CONTROL_PERIOD_S     (0.010f)

#define MOTION_TEST_DISPLAY_PERIOD_MS    (100U)
#define MOTION_TEST_TIMEOUT_MS           (10000U)

/*
 * 1000 mm @ 2000 CPS takes roughly 4 s.
 *
 * 512 samples at 10 ms/sample gives 5.12 s of full-rate logging, including
 * most/all of the braking phase.
 */
#define MOTION_TEST_LOG_CAPACITY         (512U)

/*
 * Current rear-wheel calibration.
 *
 * Distance is reconstructed from measured CPS solely for this diagnostic
 * logger. Since measured CPS is based on encoder-count change, integrating
 * it gives us an independent left/right distance trace.
 */
#define MOTION_TEST_ENCODER_CPR          (1560.0f)
#define MOTION_TEST_WHEEL_DIAMETER_MM    (65.5f)

/*
 * Synchronization controller parameters
 */
#define MOTION_SYNC_KP_CPS_PER_MM        10.0f
#define MOTION_SYNC_MAX_CORRECTION_CPS   100.0f

/*
 * Heading controller parameters
 */
#define HEADING_CTRL_KP		(10.0f)
#define HEADING_CTRL_KI		(0.0f)


#define DEBUGLOG 1

#define MOTION_TEST_MM_PER_COUNT \
    (3.14159265358979323846f * MOTION_TEST_WHEEL_DIAMETER_MM \
     / MOTION_TEST_ENCODER_CPR)

/* -------------------------------------------------------------------------- */
/* Debugger log                                                               */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint32_t timeMs;
    uint32_t state;

    float leftTargetCps;
    float rightTargetCps;

    float leftMeasuredCps;
    float rightMeasuredCps;

    float leftPwm;
    float rightPwm;

    float leftDistanceMm;
    float rightDistanceMm;

    float speedDifferenceCps;
    float distanceDifferenceMm;

    float yawDeg;

    float wheelSyncErrorMm;
    float wheelSyncCorrectionCps;

    float steeringCommand;
} MotionControllerTestLogSample;


/*
 * Global so that CubeIDE/GDB can export it directly.
 *
 * After the test:
 *
 *     p motionControllerTestLog
 *
 * or inspect/export the array in the Expressions/Variables window.
 *
 * motionControllerTestLogCount tells you how many entries are valid.
 */
volatile MotionControllerTestLogSample
motionControllerTestLog[MOTION_TEST_LOG_CAPACITY];

volatile uint32_t motionControllerTestLogCount = 0U;


/* -------------------------------------------------------------------------- */
/* Local diagnostic state                                                     */
/* -------------------------------------------------------------------------- */

static float motionControllerTestLeftDistanceMm = 0.0f;
static float motionControllerTestRightDistanceMm = 0.0f;


/* -------------------------------------------------------------------------- */
/* Logging helpers                                                            */
/* -------------------------------------------------------------------------- */

static void MotionControllerTest_ResetLog(void)
{
    motionControllerTestLogCount = 0U;

    motionControllerTestLeftDistanceMm = 0.0f;
    motionControllerTestRightDistanceMm = 0.0f;
}


static void MotionControllerTest_LogSample(
    RobotTestFixture *fixture,
    uint32_t elapsedMs)
{
    MotionController *motionController = &fixture->motionController;

    WheelSpeedController *leftWheel = motionController->leftWheel;
    WheelSpeedController *rightWheel = motionController->rightWheel;

    /*
     * Reconstruct the distance travelled during this 10 ms sample.
     *
     * CPS * seconds = encoder counts
     * encoder counts * mm/count = mm
     */
    motionControllerTestLeftDistanceMm +=
        leftWheel->measuredSpeedCps *
        MOTION_TEST_CONTROL_PERIOD_S *
        MOTION_TEST_MM_PER_COUNT;

    motionControllerTestRightDistanceMm +=
        rightWheel->measuredSpeedCps *
        MOTION_TEST_CONTROL_PERIOD_S *
        MOTION_TEST_MM_PER_COUNT;

    if (motionControllerTestLogCount >= MOTION_TEST_LOG_CAPACITY) {
        return;
    }

    uint32_t i = motionControllerTestLogCount;

    motionControllerTestLog[i].timeMs = elapsedMs;
    motionControllerTestLog[i].state = (uint32_t)motionController->mode;

    motionControllerTestLog[i].leftTargetCps =
        leftWheel->targetSpeedCps;

    motionControllerTestLog[i].rightTargetCps =
        rightWheel->targetSpeedCps;

    motionControllerTestLog[i].leftMeasuredCps =
        leftWheel->measuredSpeedCps;

    motionControllerTestLog[i].rightMeasuredCps =
        rightWheel->measuredSpeedCps;

    motionControllerTestLog[i].leftPwm =
        leftWheel->outputPWM;

    motionControllerTestLog[i].rightPwm =
        rightWheel->outputPWM;

    motionControllerTestLog[i].leftDistanceMm =
        motionControllerTestLeftDistanceMm;

    motionControllerTestLog[i].rightDistanceMm =
        motionControllerTestRightDistanceMm;

    motionControllerTestLog[i].speedDifferenceCps =
        rightWheel->measuredSpeedCps -
        leftWheel->measuredSpeedCps;

    motionControllerTestLog[i].distanceDifferenceMm =
        motionControllerTestRightDistanceMm -
        motionControllerTestLeftDistanceMm;

    /*
     * MotionController's integrated heading estimate.
     *
     * If your member is named something slightly different in the current
     * MotionController struct, this should be the only line that needs
     * changing.
     */
    motionControllerTestLog[i].yawDeg =
        motionController->yawDeg;

    motionControllerTestLog[i].wheelSyncErrorMm =
        motionController->wheelSyncErrorMm;

    motionControllerTestLog[i].wheelSyncCorrectionCps =
        motionController->wheelSyncCorrectionCps;

    motionControllerTestLog[i].steeringCommand =
    		motionController->steeringCommand;

    motionControllerTestLogCount++;
}


/* -------------------------------------------------------------------------- */
/* Initialisation                                                             */
/* -------------------------------------------------------------------------- */

bool MotionControllerTest_Init(RobotTestFixture *fixture)
{
    OLED_Init();
    OLED_Clear();
    OLED_Refresh_Gram();

    /*
     * Heading controller disabled for this diagnostic.
     *
     * Servo remains enabled/centred, but heading PID output is zero.
     *
     * maxSteeringCorrection must still be a valid non-zero percentage.
     */
    if (!RobotTestFixture_InitMotionController(
    		fixture,
		HEADING_CTRL_KP,      /* Kp */
		HEADING_CTRL_KI,      /* Ki */
		0.0f,      /* Kd */
		30.0f,	   /* max steering correction (%) */
		MOTION_SYNC_KP_CPS_PER_MM,
		MOTION_SYNC_MAX_CORRECTION_CPS))
    {
    	return false;
    }

    return true;
}


/* -------------------------------------------------------------------------- */
/* Test                                                                       */
/* -------------------------------------------------------------------------- */

void MotionControllerTestRun(void)
{
    RobotTestFixture fixture = { 0 };

    if (!MotionControllerTest_Init(&fixture)) {
        OLED_Printf(0, 0, "Motion Ctrl Init Failed");
        OLED_Refresh_Gram();

        SW1_WhileNotPressed();
        return;
    }

    OLED_Printf(0, 0, "Motion Ctrl Test");
    OLED_Printf(0, 1, "Start: SW1");
    OLED_Printf(0, 2, "1000mm @ 2000CPS");
    OLED_Printf(0, 3, "Heading Kp = %.2f", fixture.motionController.headingPID.kp);
    OLED_Printf(0, 4, "Sync Kp = %.2f", fixture.motionController.wheelSyncKpCpsPerMm);

    OLED_Refresh_Gram();

    SW1_WhileNotPressed();

    OLED_Clear();

    MotionControllerTest_ResetLog();

    MotionController_MoveStraight(
        &fixture.motionController,
        MOTION_TEST_DISTANCE_MM,
        MOTION_TEST_SPEED_CPS);

    uint32_t startTick = HAL_GetTick();
    uint32_t lastControlTick = startTick;
    // uint32_t lastDisplayTick = startTick;

    bool timedOut = false;

    while (fixture.motionController.mode != MOTIONCONTROLLER_IDLE) {

        uint32_t now = HAL_GetTick();

        /*
         * Hard test timeout.
         */
        if ((now - startTick) >= MOTION_TEST_TIMEOUT_MS) {
            timedOut = true;

            MotionController_Brake(&fixture.motionController);
            break;
        }

        /*
         * 100 Hz controller + diagnostic logger.
         */
        if ((now - lastControlTick) >= MOTION_TEST_CONTROL_PERIOD_MS) {
            lastControlTick += MOTION_TEST_CONTROL_PERIOD_MS;

            MotionController_Update(
                &fixture.motionController,
                MOTION_TEST_CONTROL_PERIOD_S);

            /*
             * Log immediately after the controller update so all values
             * correspond to the same completed control iteration.
             */
            MotionControllerTest_LogSample(
                &fixture,
                now - startTick);
        }
#if DEBUGLOG == 0
        /*
         * OLED is intentionally updated much more slowly than the controller.
         */
        if ((now - lastDisplayTick) >= MOTION_TEST_DISPLAY_PERIOD_MS) {
            lastDisplayTick = now;

            OLED_Printf(
                0, 0,
                "L:%4.0f R:%4.0f",
                fixture.motionController.leftWheel->measuredSpeedCps,
                fixture.motionController.rightWheel->measuredSpeedCps);

            OLED_Printf(
                0, 1,
                "Ld:%6.1f",
                motionControllerTestLeftDistanceMm);

            OLED_Printf(
                0, 2,
                "Rd:%6.1f",
                motionControllerTestRightDistanceMm);

            OLED_Printf(
                0, 3,
                "Yaw:%6.2f",
                fixture.motionController.yawDeg);

            OLED_Refresh_Gram();
        }
#endif /* DEBBUGLOG */
    }


    /*
     * If timeout occurred, continue running the MotionController while it
     * brakes instead of immediately stopping control.
     */
    if (timedOut) {
        uint32_t brakeStartTick = HAL_GetTick();

        while ((fixture.motionController.mode != MOTIONCONTROLLER_IDLE) &&
               ((HAL_GetTick() - brakeStartTick) < 3000U)) {

            uint32_t now = HAL_GetTick();

            if ((now - lastControlTick) >= MOTION_TEST_CONTROL_PERIOD_MS) {
                lastControlTick += MOTION_TEST_CONTROL_PERIOD_MS;

                MotionController_Update(
                    &fixture.motionController,
                    MOTION_TEST_CONTROL_PERIOD_S);

                MotionControllerTest_LogSample(
                    &fixture,
                    now - startTick);
            }
        }

        /*
         * Last resort if braking never reaches IDLE.
         */
        if (fixture.motionController.mode != MOTIONCONTROLLER_IDLE) {
            MotionController_Stop(&fixture.motionController);
        }
    }


    OLED_Clear();

    OLED_Printf(
        0, 0,
        "L:%6.1f",
        motionControllerTestLeftDistanceMm);

    OLED_Printf(
        0, 1,
        "R:%6.1f",
        motionControllerTestRightDistanceMm);

    OLED_Printf(
        0, 2,
        "Yaw:%6.2f",
        fixture.motionController.yawDeg);

    OLED_Printf(
        0, 3,
        "N:%lu%s",
        motionControllerTestLogCount,
        timedOut ? " TIMEOUT" : "");

    OLED_Refresh_Gram();

    /*
     * Leave the log in RAM for debugger inspection/export.
     */
    SW1_WhileNotPressed();
}
