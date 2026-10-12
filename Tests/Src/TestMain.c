/* Active standalone Task 2 route test. */
#include "Task2MotionTest.h"
#include "oled.h"
#include "led3.h"

void InvokeTest(void)
{
    OLED_Init();
    OLED_Clear();
    LED_On();
    Task2MotionTestRun();
    while (1) { /* Results remain available for export. */ }
}
