/* Tests real CommandLink with fake UART/RTOS. No hardware access.
 * cc -std=c11 -Wall -Wextra -Werror -DCOMMANDLINK_DIAGNOSTICS=1
 *    -Ihosttests/command_link_stubs -IApps/Inc hosttests/command_link_test.c
 *    -o /tmp/command_link_test
 * Also compile without the define to verify diagnostics-free behavior.
 */
#include <assert.h>
#include <string.h>
#include "../Apps/Src/CommandLink.c"

static uint32_t nowMs, txMs;
static HAL_StatusTypeDef txStatus = HAL_OK;
static unsigned sends;
static char lastSent[128];
uint32_t HAL_GetTick(void) { return nowMs; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, const uint8_t *data,
                                   uint16_t len, uint32_t timeout)
{
    assert(u != NULL && timeout == 20 && len < sizeof(lastSent));
    memcpy(lastSent, data, len); lastSent[len] = '\0';
    nowMs += txMs; sends++;
    return txStatus;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{
    (void)u; (void)b; assert(n == 1); return HAL_OK;
}
StreamBufferHandle_t xStreamBufferCreate(size_t n, size_t trigger)
{
    assert(n == 256 && trigger == 1); return &sends;
}
void vStreamBufferDelete(StreamBufferHandle_t stream) { (void)stream; }
size_t xStreamBufferReceive(StreamBufferHandle_t s, void *b, size_t n, uint32_t t)
{
    (void)s; (void)b; (void)n; (void)t; return 0;
}
size_t xStreamBufferSendFromISR(StreamBufferHandle_t s, const void *b, size_t n, BaseType_t *w)
{
    (void)s; (void)b; (void)w; return n;
}
int main(void)
{
    UART_HandleTypeDef uart = {0};
    assert(!CommandLink_Send("READY\n"));
    assert(!CommandLink_Init(NULL));
    assert(CommandLink_Init(&uart));
    assert(!CommandLink_Send(NULL));
    assert(CommandLink_Send(""));
    assert(sends == 0);
    txMs = 4;
    assert(CommandLink_Send("DONE 42\n"));
    assert(strcmp(lastSent, "DONE 42\n") == 0);
    txStatus = HAL_TIMEOUT; txMs = 20;
    assert(!CommandLink_Send("U 2 TIMEOUT - 0 3\n"));
#if COMMANDLINK_DIAGNOSTICS
    assert(CommandLink_Diagnostics.sends == 2);
    assert(CommandLink_Diagnostics.failures == 1);
    assert(CommandLink_Diagnostics.maxDurationMs == 20);
#endif
    nowMs = UINT32_MAX - 1; txMs = 3; txStatus = HAL_OK;
    assert(CommandLink_Send("STOPPED 42\n"));
#if COMMANDLINK_DIAGNOSTICS
    assert(CommandLink_Diagnostics.sends == 3);
    assert(CommandLink_Diagnostics.lastDurationMs == 3);
    assert(CommandLink_Diagnostics.maxDurationMs == 20);
#endif
    assert(sends == 3 && strcmp(lastSent, "STOPPED 42\n") == 0);
    return 0;
}
