/*
 * OLEDTask.c
 *
 *  Created on: 2026年8月25日
 *      Author: Joe
 */

#include "OLEDTask.h"
#include "OLEDManager.h"
#include "OLEDManagerConfig.h"

#include "cmsis_os.h"
#include "oled.h"

// A compile-time flag because for testing we might have already called OLED_Init earlier.
#define OLED_ALREADY_INITIALIZED 0

void OLEDTask(void* _) {
	(void)_;

	/* OLED hardware and framebuffer are owned by this task. */
#if !(OLED_ALREADY_INITIALIZED == 1)
	OLED_Init();
#endif
	for (;;) {
#if !(OLED_ALREADY_INITIALIZED == 1)
		OLED_ManagerRefresh();
#endif
		osDelay(OLED_MANAGER_REFRESH_PERIOD_MS);
	}
}
