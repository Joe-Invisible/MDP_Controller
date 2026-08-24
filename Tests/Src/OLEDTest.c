/*
 * OLEDTest.c
 *
 *  Created on: 2026年8月24日
 *      Author: Joe
 */


#include "OLEDTest.h"

void OLEDTestRun() {
	OLED_Clear();
	OLED_ShowString(10, 20, (const uint8_t*)"Hello World!");
	OLED_Refresh_Gram();
}
