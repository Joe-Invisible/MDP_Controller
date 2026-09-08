/*
 * DynamicBrakeMapTest.c
 *
 *  Created on: 2026年9月8日
 *      Author: Joe
 */

#include "DynamicBrakeMap.h"
#include "WheelBrakeConfig.h"

void DynamicBrakeMapTestRun() {
	float zeroResult = DynamicBrakeMap_GetPWM(&rearWheelBrakeMap, 0.0f, 1000.0f);
	float nonZeroResult = DynamicBrakeMap_GetPWM(&rearWheelBrakeMap, 0.10f, 1000.0f);

	(void)zeroResult;
	(void)nonZeroResult;
}
