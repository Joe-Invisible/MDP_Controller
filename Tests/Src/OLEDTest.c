/*
 * OLEDTest.c
 *
 *  Created on: 2026年8月24日
 *      Author: Joe
 */


#include "OLEDTest.h"

void OLEDTestRun() {
	OLED_Clear();
	OLED_ShowString5x7(0, 0, (const char*)"Hello World!");
	OLED_ShowString5x7(0, 1, (const char*)"5x7 Font text");
	OLED_ShowString5x7(0, 2, (const char*)"Display becomes 21x7!");
	OLED_Refresh_Gram();
}
