#ifndef COMMAND_LINK_TEST_HAL_H
#define COMMAND_LINK_TEST_HAL_H
#include <stdint.h>
typedef struct { unsigned unused; } UART_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_TIMEOUT } HAL_StatusTypeDef;
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
#endif
