/*
 * buzzer.h
 *
 * Basic buzzer driver. Does not include initialisation.
 * Make sure GPIO init code is generated correctly by
 * CubeMX.
 *
 *  Created on: 2026年9月17日
 *      Author: Joe
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include "stm32f4xx_hal.h"

void Buzzer_On();
void Buzzer_Off();

void Buzzer_BlockingBuzz(uint32_t ms);

#endif /* INC_BUZZER_H_ */
