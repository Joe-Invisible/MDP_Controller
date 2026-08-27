/*
 * AppTasks.c
 * Contains thread attributes and task handles.
 *  Created on: 2026年8月24日
 *      Author: Joe
 */

#include "AppTasks.h"

#include "OLEDManager.h"
#include "OLEDTask.h"

const osThreadAttr_t OLEDTask_attributes = {
  .name = "OLEDTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
void AppTasks_Init() {
	if (OLED_ManagerInit() != OLED_OK)
		return;

	osThreadId_t OLEDTaskHandle = osThreadNew(OLEDTask, NULL, &OLEDTask_attributes);
}
#pragma GCC diagnostic pop
