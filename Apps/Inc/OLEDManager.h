/*
 * OLEDManager.h
 * Thread-safe diagnostic publishing interface for the OLED.
 */

#ifndef INC_OLEDMANAGER_H_
#define INC_OLEDMANAGER_H_

#include <stdint.h>

#define OLED_INVALID_SLOT    UINT8_MAX

typedef struct {
	uint8_t slot;
} OLED_Handle_t;

typedef enum {
	OLED_OK = 0,
	OLED_ERROR_NOT_INITIALIZED,
	OLED_ERROR_INVALID_ARGUMENT,
	OLED_ERROR_INVALID_HANDLE,
	OLED_ERROR_FULL,
	OLED_ERROR_OS
} OLED_Status_t;

/**
 * Initializes manager state and its mutex.
 * Call once before creating application threads that use OLED_Register/Post.
 */
OLED_Status_t OLED_ManagerInit(void);

/**
 * Registers one diagnostic field. Registration order determines display order.
 */
OLED_Status_t OLED_Register(OLED_Handle_t *handle, const char *name);

/**
 * Replaces a field's current value with formatted text.
 * Only the latest value is retained. This function is task-safe, not ISR-safe.
 */
OLED_Status_t OLED_Post(OLED_Handle_t *handle, const char *fmt, ...);

/**
 * Takes a snapshot of the latest visible fields and refreshes the OLED if any
 * visible data may have changed. Intended to be called only by OLEDTask().
 */
void OLED_ManagerRefresh(void);

#endif /* INC_OLEDMANAGER_H_ */
