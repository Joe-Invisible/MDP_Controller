#ifndef COMMAND_LINK_TEST_STREAM_H
#define COMMAND_LINK_TEST_STREAM_H
#include "FreeRTOS.h"
typedef void *StreamBufferHandle_t;
StreamBufferHandle_t xStreamBufferCreate(size_t, size_t);
void vStreamBufferDelete(StreamBufferHandle_t);
size_t xStreamBufferReceive(StreamBufferHandle_t, void *, size_t, uint32_t);
size_t xStreamBufferSendFromISR(StreamBufferHandle_t, const void *, size_t, BaseType_t *);
#endif
