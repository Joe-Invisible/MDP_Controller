/*
 * userbutton.c
 *
 *  Created on: 2026年8月28日
 *      Author: Joe
 */

#include "userbutton.h"
#include "gpio.h"


#define SW1_DEBOUNCE_MS    30U


SW1_State_t SW1_ReadState(void)
{
    return (SW1_State_t)
        HAL_GPIO_ReadPin(USERBTN_GPIO_Port, USERBTN_Pin);
}


void SW1_WaitForPress(void)
{
    while (1)
    {
        /*
         * Wait for the first indication of a press.
         */
        while (SW1_ReadState() != SW1_Enabled)
        {
        }

        /*
         * Allow contact bounce to settle.
         */
        HAL_Delay(SW1_DEBOUNCE_MS);

        /*
         * Accept the press only if it is still asserted.
         */
        if (SW1_ReadState() == SW1_Enabled)
        {
            return;
        }
    }
}


void SW1_WaitForRelease(void)
{
    while (1)
    {
        /*
         * Wait for the first indication of release.
         */
        while (SW1_ReadState() != SW1_Idle)
        {
        }

        HAL_Delay(SW1_DEBOUNCE_MS);

        /*
         * Accept the release only if it remains released.
         */
        if (SW1_ReadState() == SW1_Idle)
        {
            return;
        }
    }
}


void SW1_WaitForPressAndRelease(void)
{
    SW1_WaitForPress();
    SW1_WaitForRelease();
}


void SW1_WhileNotPressed(void)
{
    SW1_WaitForPress();
}
