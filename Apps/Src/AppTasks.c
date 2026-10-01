/*
 * AppTasks.c
 * Contains thread attributes and task handles.
 *  Created on: 2026年8月24日
 *      Author: Joe
 */

#include "AppTasks.h"

#include "usart.h"

#include "OLEDManager.h"
#include "OLEDTask.h"
#include "CommandLink.h"
#include "MotionTask.h"
#include "SensorTask.h"

const osThreadAttr_t OLEDTask_attributes = {
  .name = "OLEDTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/*
 * Owns motion and protocol state. The larger stack accommodates strtof
 * when parsing a batch; controller and batch storage are static.
 */
const osThreadAttr_t MotionTask_attributes = {
  .name = "MotionTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
void AppTasks_Init() {
	if (OLED_ManagerInit() != OLED_OK)
		return;

	osThreadId_t OLEDTaskHandle = osThreadNew(OLEDTask, NULL, &OLEDTask_attributes);

	if (!CommandLink_Init(&huart3))
		return;

	if (osThreadNew(SensorTask, NULL, &SensorTask_attributes) == NULL)
		return;

	if (osThreadNew(MotionTask, NULL, &MotionTask_attributes) == NULL)
		return;
}
#pragma GCC diagnostic pop
