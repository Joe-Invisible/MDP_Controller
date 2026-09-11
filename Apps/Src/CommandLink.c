#include "CommandLink.h"

#include <string.h>

#include "FreeRTOS.h"
#include "stream_buffer.h"

/*
 * Buffer short bursts between MotionTask polls: at
 * 115200 baud a full second of traffic is ~11.5 KB, but frames are
 * tens of bytes. The consumer polls at 100 Hz; 256 buffers normal
 * batches/status requests, not a sustained full-rate stream.
 */
#define COMMANDLINK_BUFFER_BYTES	256

#define COMMANDLINK_TX_TIMEOUT_MS	20U

static UART_HandleTypeDef *link_uart;
static StreamBufferHandle_t link_stream;
static uint8_t link_rxByte;
static volatile uint32_t link_dropCount;

bool CommandLink_Init(UART_HandleTypeDef *huart) {
	if (huart == NULL)
		return false;

	/* Trigger level 1: the consumer wakes on every byte. */
	link_stream = xStreamBufferCreate(COMMANDLINK_BUFFER_BYTES, 1);
	if (link_stream == NULL)
		return false;

	link_uart = huart;

	if (HAL_UART_Receive_IT(link_uart, &link_rxByte, 1) != HAL_OK) {
		vStreamBufferDelete(link_stream);
		link_stream = NULL;
		link_uart = NULL;
		return false;
	}

	return true;
}

size_t CommandLink_ReadBytes(uint8_t *buf, size_t maxLen, uint32_t timeoutTicks) {
	if (link_stream == NULL || buf == NULL || maxLen == 0)
		return 0;

	return xStreamBufferReceive(link_stream, buf, maxLen, timeoutTicks);
}

bool CommandLink_Send(const char *text) {
	if (link_uart == NULL || text == NULL)
		return false;

	size_t len = strlen(text);
	if (len == 0)
		return true;

    /* MotionTask is the only transmitter. Replies are short and bounded. */
    return HAL_UART_Transmit(link_uart, (const uint8_t *)text,
            (uint16_t)len, COMMANDLINK_TX_TIMEOUT_MS) == HAL_OK;
}

void CommandLink_IsrRxComplete(UART_HandleTypeDef *huart) {
	if (huart != link_uart || link_stream == NULL)
		return;

	BaseType_t woken = pdFALSE;
	if (xStreamBufferSendFromISR(link_stream, &link_rxByte, 1, &woken) != 1)
		link_dropCount++;

	/*
	 * Re-arm before yielding: the shift register only buffers one
	 * byte, so reception must be pending again as soon as possible.
	 */
	HAL_UART_Receive_IT(link_uart, &link_rxByte, 1);

	portYIELD_FROM_ISR(woken);
}

void CommandLink_IsrError(UART_HandleTypeDef *huart) {
	if (huart != link_uart)
		return;

	/*
	 * An overrun (or noise/framing error) aborts interrupt reception:
	 * without this re-arm the port stays silent forever. The byte in
	 * flight is lost either way; MotionTask resynchronises on the
	 * next '\n'.
	 */
	link_dropCount++;
	HAL_UART_Receive_IT(link_uart, &link_rxByte, 1);
}

uint32_t CommandLink_GetDropCount(void) {
	return link_dropCount;
}
