/*
 * OLEDTask.c
 *
 *  Created on: 2026年8月25日
 *      Author: Joe
 */

#include "OLEDTask.h"
#include "cmsis_os.h"

void OLEDTask(void* _) {
	for (;;) {
		osDelay(1);
	}
}
