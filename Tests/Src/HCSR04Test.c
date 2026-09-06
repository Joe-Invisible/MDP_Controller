/*
 * HCSR04Test.c
 *
 *  Created on: 2026年9月6日
 *      Author: Joe
 */

#include "HCSR04Test.h"

#include "hcsr04.h"
#include "RobotPeripherals.h"
#include "oled.h"
#include "oledutils.h"
#include "tim.h"

#include <stdint.h>


#define USS_ECHO_TIM        htim12
#define USS_ECHO_CH         TIM_CHANNEL_2

#define USS_TEST_PERIOD_MS  100U


void HCSR04TestRun(void)
{
    OLED_Init();
    OLED_Clear();

    HCSR04_Init(
        &hcsr04,
        USS_TRIG_GPIO_Port,
        USS_TRIG_Pin,
        &USS_ECHO_TIM,
        USS_ECHO_CH);

    OLED_Printf(0, 0, "HC-SR04 Test");
    OLED_Printf(0, 1, "Starting...");
    OLED_Refresh_Gram();

    HAL_Delay(500U);

    while (1)
    {
        /*
         * Start one ultrasonic measurement.
         */
        if (!HCSR04_Trigger(&hcsr04))
        {
            OLED_Printf(0, 1, "Trigger failed   ");
            OLED_Refresh_Gram();

            HAL_Delay(USS_TEST_PERIOD_MS);
            continue;
        }

        /*
         * Wait for the input-capture ISR to receive the ECHO pulse.
         *
         * We are NOT timing the pulse here. TIM12 does that in hardware.
         * This loop only waits for READY or TIMEOUT.
         */
        while (HCSR04_IsBusy(&hcsr04))
        {
            HCSR04_Update(&hcsr04);
            HAL_Delay(1U);
        }

        if (HCSR04_HasMeasurement(&hcsr04))
        {
            uint32_t pulseWidthUs =
                HCSR04_GetPulseWidthUs(&hcsr04);

            float distanceMm =
                HCSR04_GetDistanceMm(&hcsr04);

            /*
             * Fixed-width fields/spaces help overwrite the previous
             * measurement without clearing the whole OLED every cycle.
             */
            OLED_Printf(
                0, 1,
                "Dist: %7.1f mm ",
                distanceMm);

            OLED_Printf(
                0, 2,
                "Echo: %6lu us ",
                pulseWidthUs);

            OLED_Printf(
                0, 3,
                "State: READY    ");
        }
        else
        {
            OLED_Printf(
                0, 1,
                "Dist:    --- mm ");

            OLED_Printf(
                0, 2,
                "Echo:    --- us ");

            if (HCSR04_GetState(&hcsr04) == HCSR04_STATE_TIMEOUT)
            {
                OLED_Printf(
                    0, 3,
                    "State: TIMEOUT  ");
            }
            else
            {
                OLED_Printf(
                    0, 3,
                    "State: ERROR    ");
            }
        }

        OLED_Refresh_Gram();

        /*
         * HC-SR04 does not need to be triggered rapidly.
         * 100 ms is plenty for this test.
         */
        HAL_Delay(USS_TEST_PERIOD_MS);
    }
}
