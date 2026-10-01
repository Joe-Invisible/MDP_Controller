#ifndef SENSOR_TASK_TEST_OS_H
#define SENSOR_TASK_TEST_OS_H
#include <stdint.h>
static inline uint32_t osKernelGetTickFreq(void) { return 1000U; }
static inline void osDelay(uint32_t ticks) { (void)ticks; }
#endif
