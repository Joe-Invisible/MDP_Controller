/*
 * OLEDManagerConfig.h
 * Compile-time configuration for the thread-safe OLED diagnostic manager.
 */

#ifndef INC_OLEDMANAGERCONFIG_H_
#define INC_OLEDMANAGERCONFIG_H_

#include "oled.h"

/* Select the low-level text renderer used by the manager. */
#define OLED_MANAGER_RENDERER             OLED_RENDERER_5X7

/* Registration pool. No dynamic allocation is used by the manager. */
#define OLED_MANAGER_MAX_CHANNELS         32U

/* Lengths include the terminating null character. */
#define OLED_MANAGER_NAME_LENGTH          12U
#define OLED_MANAGER_VALUE_LENGTH         32U

/* Maximum OLED refresh rate. 200 ms = 5 Hz. */
#define OLED_MANAGER_REFRESH_PERIOD_MS    200U

#if OLED_MANAGER_RENDERER == OLED_RENDERER_5X7
#define OLED_MANAGER_VISIBLE_ROWS         OLED_5X7_ROWS
#define OLED_MANAGER_VISIBLE_COLS         OLED_5X7_COLS
#elif OLED_MANAGER_RENDERER == OLED_RENDERER_LEGACY
#define OLED_MANAGER_VISIBLE_ROWS         (OLED_Height / 16U)
#define OLED_MANAGER_VISIBLE_COLS         (OLED_Width / 8U)
#else
#error "Unsupported OLED_MANAGER_RENDERER"
#endif

#if OLED_MANAGER_MAX_CHANNELS > 255U
#error "OLED_MANAGER_MAX_CHANNELS must fit in OLED_Handle_t.slot"
#endif

#endif /* INC_OLEDMANAGERCONFIG_H_ */
