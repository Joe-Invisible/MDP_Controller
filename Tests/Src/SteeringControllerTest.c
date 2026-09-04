/*
 * SteeringControllerTest.c
 */

#include "SteeringControllerTest.h"

#include "RobotTestFixture.h"
#include "oledutils.h"
#include "userbutton.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* -------------------------------------------------------------------------- */
/* Test configuration                                                         */
/* -------------------------------------------------------------------------- */

#define STEERING_TEST_LOG_CAPACITY        (8U)

#define STEERING_TEST_ANGLE_A_RAD         (+0.005f)
#define STEERING_TEST_ANGLE_B_RAD         (-0.005f)

#define STEERING_TEST_ANGLE_NEG_RAD       (-0.020f)
#define STEERING_TEST_ANGLE_POS_RAD       (+0.010f)

/*
 * Deliberately outside the currently bidirectionally-controllable
 * calibrated range.
 */
#define STEERING_TEST_CLAMP_HIGH_RAD      (+0.050f)
#define STEERING_TEST_CLAMP_LOW_RAD       (-0.080f)

#define STEERING_TEST_SETTLE_MS           (500U)

/*
 * Only used for the software-model checks below.
 *
 * The interpolation results are deterministic, so a small tolerance
 * is appropriate.
 */
#define STEERING_TEST_ANGLE_TOLERANCE_RAD (0.00005f)
#define STEERING_TEST_COMMAND_TOLERANCE   (0.05f)

/* -------------------------------------------------------------------------- */
/* Debugger-visible state                                                     */
/* -------------------------------------------------------------------------- */

volatile SteeringControllerTestLogSample
steeringControllerTestLog[STEERING_TEST_LOG_CAPACITY];

volatile uint32_t steeringControllerTestLogCount = 0U;

volatile bool steeringControllerTestPassed = true;

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */

static bool SteeringControllerTest_FloatNear(
    float actual,
    float expected,
    float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}


static void SteeringControllerTest_Log(
    SteeringController *controller,
    uint32_t step,
    float requestedAngleRad)
{
    if (controller == NULL)
    {
        steeringControllerTestPassed = false;
        return;
    }

    if (steeringControllerTestLogCount >=
        STEERING_TEST_LOG_CAPACITY)
    {
        steeringControllerTestPassed = false;
        return;
    }

    uint32_t i = steeringControllerTestLogCount;

    steeringControllerTestLog[i].step =
        step;

    steeringControllerTestLog[i].requestedAngleRad =
        requestedAngleRad;

    steeringControllerTestLog[i].targetAngleRad =
        SteeringController_GetTargetEffectiveAngleRad(
            controller);

    steeringControllerTestLog[i].effectiveAngleRad =
        SteeringController_GetEffectiveAngleRad(
            controller);

    steeringControllerTestLog[i].command =
        SteeringController_GetCommand(
            controller);

    steeringControllerTestLog[i].backlashActive =
        SteeringController_IsBacklashActive(
            controller);

    steeringControllerTestLog[i].movementDirection =
        SteeringController_GetMovementDirection(
            controller);

    steeringControllerTestLogCount++;
}


static void SteeringControllerTest_CheckState(
    SteeringController *controller,
    float expectedTargetAngleRad,
    float expectedEffectiveAngleRad,
    float expectedCommand)
{
    if (!SteeringControllerTest_FloatNear(
            SteeringController_GetTargetEffectiveAngleRad(
                controller),
            expectedTargetAngleRad,
            STEERING_TEST_ANGLE_TOLERANCE_RAD))
    {
        steeringControllerTestPassed = false;
    }

    if (!SteeringControllerTest_FloatNear(
            SteeringController_GetEffectiveAngleRad(
                controller),
            expectedEffectiveAngleRad,
            STEERING_TEST_ANGLE_TOLERANCE_RAD))
    {
        steeringControllerTestPassed = false;
    }

    if (!SteeringControllerTest_FloatNear(
            SteeringController_GetCommand(
                controller),
            expectedCommand,
            STEERING_TEST_COMMAND_TOLERANCE))
    {
        steeringControllerTestPassed = false;
    }
}


static void SteeringControllerTest_ShowStep(
    uint32_t step,
    SteeringController *controller)
{
    OLED_Clear();

    OLED_Printf(
        0, 0,
        "Steer Test %lu",
        step);

    OLED_Printf(
        0, 1,
        "Cmd:%6.2f",
        SteeringController_GetCommand(
            controller));

    OLED_Printf(
        0, 2,
        "T:%+.4f",
        SteeringController_GetTargetEffectiveAngleRad(
            controller));

    OLED_Printf(
        0, 3,
        "E:%+.4f",
        SteeringController_GetEffectiveAngleRad(
            controller));

    OLED_Printf(
        0, 4,
        "B:%d D:%d",
        SteeringController_IsBacklashActive(
            controller) ? 1 : 0,
        SteeringController_GetMovementDirection(
            controller));

    OLED_Refresh_Gram();
}


/*
 * Run one angle command, wait for the physical servo to settle,
 * log the controller state, and display it.
 *
 * The model itself updates immediately. The delay is only so that
 * the physical front wheels can be visually inspected as well.
 */
static void SteeringControllerTest_RunAngleStep(
    SteeringController *controller,
    uint32_t step,
    float requestedAngleRad)
{
    SteeringController_SetEffectiveAngleRad(
        controller,
        requestedAngleRad);

    HAL_Delay(
        STEERING_TEST_SETTLE_MS);

    SteeringControllerTest_Log(
        controller,
        step,
        requestedAngleRad);

    SteeringControllerTest_ShowStep(
        step,
        controller);

    SW1_WaitForPressAndRelease();
}

/* -------------------------------------------------------------------------- */
/* Test                                                                       */
/* -------------------------------------------------------------------------- */

void SteeringControllerTestRun(void)
{
    RobotTestFixture fixture = { 0 };

    steeringControllerTestLogCount = 0U;
    steeringControllerTestPassed = true;

    OLED_Init();
    OLED_Clear();

    /*
     * Only initialise what this test needs.
     */
    if (!RobotTestFixture_InitFrontWheels(
            &fixture))
    {
        OLED_Printf(
            0, 0,
            "Servo Init Failed");

        OLED_Refresh_Gram();

        SW1_WaitForPressAndRelease();
        return;
    }

    if (!RobotTestFixture_InitSteeringController(
            &fixture))
    {
        OLED_Printf(
            0, 0,
            "Steer Init Failed");

        OLED_Refresh_Gram();

        SW1_WaitForPressAndRelease();
        return;
    }

    SteeringController *controller =
        &fixture.steeringController;

    /*
     * Initial state.
     *
     * By convention SteeringController_Init() establishes:
     *
     *   command             = 0
     *   target angle        = 0
     *   effective angle     = 0
     */
    SteeringControllerTest_Log(
        controller,
        0U,
        0.0f);

    SteeringControllerTest_CheckState(
        controller,
        0.0f,
        0.0f,
        0.0f);

    SteeringControllerTest_ShowStep(
        0U,
        controller);

    SW1_WaitForPressAndRelease();

    /*
     * Step 1:
     *
     * Request positive effective angle.
     *
     * This should use the decreasing-command branch.
     *
     * From the current calibration:
     *
     *   target = +0.005 rad
     *   command ~= -10.263
     */
    SteeringControllerTest_RunAngleStep(
        controller,
        1U,
        STEERING_TEST_ANGLE_A_RAD);

    SteeringControllerTest_CheckState(
        controller,
        STEERING_TEST_ANGLE_A_RAD,
        STEERING_TEST_ANGLE_A_RAD,
        -10.2634f);

    /*
     * Step 2:
     *
     * Reverse physical steering direction.
     *
     * This should switch to the increasing-command branch.
     *
     *   target = -0.005 rad
     *   command ~= +4.427
     */
    SteeringControllerTest_RunAngleStep(
        controller,
        2U,
        STEERING_TEST_ANGLE_B_RAD);

    SteeringControllerTest_CheckState(
        controller,
        STEERING_TEST_ANGLE_B_RAD,
        STEERING_TEST_ANGLE_B_RAD,
        +4.4270f);

    /*
     * Step 3:
     *
     * Physical centre.
     *
     * Since we are approaching zero from negative effective
     * angle, this should use the decreasing-command branch.
     *
     * Current calibration predicts:
     *
     *   zero-angle command ~= -9.525
     */
    SteeringController_Centre(
        controller);

    HAL_Delay(
        STEERING_TEST_SETTLE_MS);

    SteeringControllerTest_Log(
        controller,
        3U,
        0.0f);

    SteeringControllerTest_ShowStep(
        3U,
        controller);

    SteeringControllerTest_CheckState(
        controller,
        0.0f,
        0.0f,
        -9.5250f);

    SW1_WaitForPressAndRelease();

    /*
     * Step 4:
     *
     * Larger negative effective angle.
     *
     * Increasing-command branch:
     *
     *   target = -0.020 rad
     *   command ~= +5.977
     */
    SteeringControllerTest_RunAngleStep(
        controller,
        4U,
        STEERING_TEST_ANGLE_NEG_RAD);

    SteeringControllerTest_CheckState(
        controller,
        STEERING_TEST_ANGLE_NEG_RAD,
        STEERING_TEST_ANGLE_NEG_RAD,
        +5.9765f);

    /*
     * Step 5:
     *
     * Reverse again to a positive effective angle.
     *
     * Decreasing-command branch:
     *
     *   target = +0.010 rad
     *   command ~= -11.043
     */
    SteeringControllerTest_RunAngleStep(
        controller,
        5U,
        STEERING_TEST_ANGLE_POS_RAD);

    SteeringControllerTest_CheckState(
        controller,
        STEERING_TEST_ANGLE_POS_RAD,
        STEERING_TEST_ANGLE_POS_RAD,
        -11.0435f);

    /*
     * Step 6:
     *
     * Deliberately request an angle above the common calibrated
     * range. This should clamp to the maximum angle reachable on
     * both branches.
     */
    SteeringControllerTest_RunAngleStep(
        controller,
        6U,
        STEERING_TEST_CLAMP_HIGH_RAD);

    float maxAngleRad =
        SteeringController_GetMaxEffectiveAngleRad(
            controller);

    if (!SteeringControllerTest_FloatNear(
            SteeringController_GetTargetEffectiveAngleRad(
                controller),
            maxAngleRad,
            STEERING_TEST_ANGLE_TOLERANCE_RAD))
    {
        steeringControllerTestPassed = false;
    }

    if (!SteeringControllerTest_FloatNear(
            SteeringController_GetEffectiveAngleRad(
                controller),
            maxAngleRad,
            STEERING_TEST_ANGLE_TOLERANCE_RAD))
    {
        steeringControllerTestPassed = false;
    }

    /*
     * Step 7:
     *
     * Same test at the negative limit.
     */
    SteeringControllerTest_RunAngleStep(
        controller,
        7U,
        STEERING_TEST_CLAMP_LOW_RAD);

    float minAngleRad =
        SteeringController_GetMinEffectiveAngleRad(
            controller);

    if (!SteeringControllerTest_FloatNear(
            SteeringController_GetTargetEffectiveAngleRad(
                controller),
            minAngleRad,
            STEERING_TEST_ANGLE_TOLERANCE_RAD))
    {
        steeringControllerTestPassed = false;
    }

    if (!SteeringControllerTest_FloatNear(
            SteeringController_GetEffectiveAngleRad(
                controller),
            minAngleRad,
            STEERING_TEST_ANGLE_TOLERANCE_RAD))
    {
        steeringControllerTestPassed = false;
    }

    /*
     * Final display.
     */
    OLED_Clear();

    OLED_Printf(
        0, 0,
        "Steering Test");

    OLED_Printf(
        0, 1,
        "%s",
        steeringControllerTestPassed
            ? "PASS"
            : "FAIL");

    OLED_Printf(
        0, 2,
        "N:%lu",
        steeringControllerTestLogCount);

    OLED_Refresh_Gram();

    /*
     * Leave all test state in RAM for GDB inspection.
     */
    SW1_WaitForPressAndRelease();
}
