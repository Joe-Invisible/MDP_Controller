/*
 * GP2Y0A21YKTest.c
 *
 *  Created on: 2026年9月20日
 *      Author: Joe
 */


/*
 * GP2Y0A21YKTest.c
 *
 * Displays raw ADC counts and voltages for both side sensors.
 */

#include "GP2Y0A21YKTest.h"

#include "SideIRSensorConfig.h"
#include "gp2y0a21yk.h"
#include "oled.h"
#include "oledutils.h"

#include <stdint.h>

#define SIDE_IR_TEST_PERIOD_MS    50U

void GP2Y0A21YKTestRun(void)
{
    GP2Y0A21YK leftSensor = {0};
    GP2Y0A21YK rightSensor = {0};
    GP2Y0A21YK_Measurement leftMeasurement = {0};
    GP2Y0A21YK_Measurement rightMeasurement = {0};
    GP2Y0A21YK_Status leftStatus;
    GP2Y0A21YK_Status rightStatus;

    OLED_Init();
    OLED_Clear();

    leftStatus = GP2Y0A21YK_Init(&leftSensor, &sideIRLeftConfig);
    rightStatus = GP2Y0A21YK_Init(&rightSensor, &sideIRRightConfig);

    if ((leftStatus != GP2Y0A21YK_OK) ||
        (rightStatus != GP2Y0A21YK_OK))
    {
        OLED_Printf(0, 0, "Side IR init fail");
        OLED_Printf(
            0,
            1,
            "L:%u R:%u",
            (unsigned int)leftStatus,
            (unsigned int)rightStatus);
        OLED_Refresh_Gram();
        return;
    }

    OLED_Printf(0, 0, "GP2Y0A21YK Test");

    while (1)
    {
        leftStatus = GP2Y0A21YK_Read(&leftSensor, &leftMeasurement);
        rightStatus = GP2Y0A21YK_Read(&rightSensor, &rightMeasurement);

        if (leftStatus == GP2Y0A21YK_OK)
        {
            OLED_Printf(
                0,
                1,
                "L %4u  %.3fV ",
                (unsigned int)leftMeasurement.rawAdc,
                leftMeasurement.voltageV);
        }
        else
        {
            OLED_Printf(
                0,
                1,
                "L error %u     ",
                (unsigned int)leftStatus);
        }

        if (rightStatus == GP2Y0A21YK_OK)
        {
            OLED_Printf(
                0,
                2,
                "R %4u  %.3fV ",
                (unsigned int)rightMeasurement.rawAdc,
                rightMeasurement.voltageV);
        }
        else
        {
            OLED_Printf(
                0,
                2,
                "R error %u     ",
                (unsigned int)rightStatus);
        }

        OLED_Refresh_Gram();
        HAL_Delay(SIDE_IR_TEST_PERIOD_MS);
    }
}
