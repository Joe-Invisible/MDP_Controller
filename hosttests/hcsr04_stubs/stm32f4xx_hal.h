#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef struct { uint32_t Channel, counter, period, capture; } TIM_HandleTypeDef;
typedef struct { unsigned unused; } GPIO_TypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define HAL_TIM_ACTIVE_CHANNEL_1 1U
#define HAL_TIM_ACTIVE_CHANNEL_2 2U
#define HAL_TIM_ACTIVE_CHANNEL_3 3U
#define HAL_TIM_ACTIVE_CHANNEL_4 4U
#define TIM_FLAG_CC1 1U
#define TIM_FLAG_CC2 2U
#define TIM_FLAG_CC3 4U
#define TIM_FLAG_CC4 8U
#define TIM_INPUTCHANNELPOLARITY_RISING 0U
#define TIM_INPUTCHANNELPOLARITY_FALLING 1U
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
extern uint32_t testTick, testMask, testPolarity;
extern HAL_StatusTypeDef testStartResult;
extern void (*testBeforeDisable)(void);
static inline uint32_t HAL_GetTick(void) { return testTick; }
static inline uint32_t __get_PRIMASK(void) { return testMask; }
static inline void __disable_irq(void) {
    if (testBeforeDisable) { void (*hook)(void)=testBeforeDisable; testBeforeDisable=0; hook(); }
    testMask=1;
}
static inline void __set_PRIMASK(uint32_t mask) { testMask=mask; }
#define __HAL_TIM_SET_CAPTUREPOLARITY(t,c,p) ((void)(t), (void)(c), testPolarity=(p))
#define __HAL_TIM_CLEAR_FLAG(t,f) ((void)(t), (void)(f))
#define __HAL_TIM_GET_AUTORELOAD(t) ((t)->period)
#define __HAL_TIM_GET_COUNTER(t) (((t)->counter++) & (t)->period)
static inline HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *t, uint32_t c) {
    (void)t; (void)c; return testStartResult;
}
static inline HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *t, uint32_t c) {
    (void)t; (void)c; return HAL_OK;
}
static inline uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *t, uint32_t c) {
    (void)c; return t->capture;
}
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, unsigned value) {
    (void)p; (void)pin; (void)value;
}
#endif
