#ifndef INC_COMMANDLINK_H_
#define INC_COMMANDLINK_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

/*
 * Byte transport between the USART3 RX interrupt and MotionTask.
 *
 * Single producer (the ISR), single consumer (MotionTask), carried
 * by a FreeRTOS stream buffer. The ISR-side calls must run at NVIC
 * priority >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5).
 */

/**
 * Creates the stream buffer and arms the first 1-byte interrupt
 * receive. Call once, after the kernel objects can be created and
 * before MotionTask starts reading.
 */
bool CommandLink_Init(UART_HandleTypeDef *huart);

/**
 * Blocking read for MotionTask. Returns the number of bytes copied
 * into buf (0 on timeout). timeoutTicks may be osWaitForever.
 */
size_t CommandLink_ReadBytes(uint8_t *buf, size_t maxLen, uint32_t timeoutTicks);

/**
 * Blocking send. MotionTask is the sole caller; no TX mutex is needed.
 * Send one complete line per call.
 */
bool CommandLink_Send(const char *text);

/* Optional GDB-visible TX timing. No diagnostic bytes are added to the protocol.
 * Enable for measurement builds with -DCOMMANDLINK_DIAGNOSTICS=1; halt the
 * target to inspect a consistent snapshot of CommandLink_Diagnostics.
 */
#ifndef COMMANDLINK_DIAGNOSTICS
#define COMMANDLINK_DIAGNOSTICS 0
#endif
#if COMMANDLINK_DIAGNOSTICS
typedef struct {
    uint32_t sends;
    uint32_t failures;
    uint32_t lastDurationMs;
    uint32_t maxDurationMs;
} CommandLinkDiagnostics;
extern volatile CommandLinkDiagnostics CommandLink_Diagnostics;
#endif

/**
 * Forwarders for the HAL callbacks in Core/Src/hal_callback.c.
 * Each ignores handles other than the one given to CommandLink_Init.
 */
void CommandLink_IsrRxComplete(UART_HandleTypeDef *huart);
void CommandLink_IsrError(UART_HandleTypeDef *huart);

/**
 * Dropped-byte count: increments when the stream buffer is full at
 * ISR time or reception had to be re-armed after an error. A rising
 * value with a healthy sender means the consumer is stalled.
 */
uint32_t CommandLink_GetDropCount(void);

#endif /* INC_COMMANDLINK_H_ */
