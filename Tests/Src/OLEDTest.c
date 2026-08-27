/*
 * OLEDTest.c
 *
 *  Created on: 2026年8月24日
 *      Author: Joe
 */


#include "OLEDTest.h"

void OLEDTestRun() {
	OLED_Init();
	OLED_Clear();
	OLED_ShowString5x7(0, 0, (const char*)"Hello World!");
	OLED_Refresh_Gram();
}
