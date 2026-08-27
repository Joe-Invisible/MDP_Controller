/*
 * OLEDManager.c
 * Thread-safe latest-value storage and rendering for OLED diagnostics.
 */

#include "OLEDManager.h"
#include "OLEDManagerConfig.h"

#include "cmsis_os.h"
#include "oled.h"

#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

typedef struct {
	bool used;
	char name[OLED_MANAGER_NAME_LENGTH];
	char value[OLED_MANAGER_VALUE_LENGTH];
} OLED_Channel_t;

static OLED_Channel_t channels[OLED_MANAGER_MAX_CHANNELS];
static OLED_Channel_t snapshot[OLED_MANAGER_VISIBLE_ROWS];

static osMutexId_t managerMutex;
static uint8_t channelCount;
static bool initialized;
static bool dirty;

static OLED_Status_t OLED_Lock(void) {
	if (osMutexAcquire(managerMutex, osWaitForever) != osOK)
		return OLED_ERROR_OS;
	return OLED_OK;
}

static void OLED_Unlock(void) {
	(void)osMutexRelease(managerMutex);
}

static bool OLED_HandleIsValid(const OLED_Handle_t *handle) {
	return handle != NULL
			&& handle->slot < channelCount
			&& channels[handle->slot].used;
}

static void OLED_BuildLine(char *line, size_t lineSize,
		const OLED_Channel_t *channel) {
	if (channel->name[0] != '\0' && channel->value[0] != '\0') {
		(void)snprintf(line, lineSize, "%s %s", channel->name, channel->value);
	} else if (channel->name[0] != '\0') {
		(void)snprintf(line, lineSize, "%s", channel->name);
	} else {
		(void)snprintf(line, lineSize, "%s", channel->value);
	}
}

OLED_Status_t OLED_ManagerInit(void) {
	if (initialized)
		return OLED_OK;

	memset(channels, 0, sizeof(channels));
	channelCount = 0U;
	dirty = true;

	managerMutex = osMutexNew(NULL);
	if (managerMutex == NULL)
		return OLED_ERROR_OS;

	initialized = true;
	return OLED_OK;
}

OLED_Status_t OLED_Register(OLED_Handle_t *handle, const char *name) {
	OLED_Status_t status;
	OLED_Channel_t *channel;

	if (!initialized)
		return OLED_ERROR_NOT_INITIALIZED;
	if (handle == NULL || name == NULL)
		return OLED_ERROR_INVALID_ARGUMENT;

	handle->slot = OLED_INVALID_SLOT;

	status = OLED_Lock();
	if (status != OLED_OK)
		return status;

	if (channelCount >= OLED_MANAGER_MAX_CHANNELS) {
		OLED_Unlock();
		return OLED_ERROR_FULL;
	}

	channel = &channels[channelCount];
	memset(channel, 0, sizeof(*channel));
	strncpy(channel->name, name, sizeof(channel->name) - 1U);
	channel->name[sizeof(channel->name) - 1U] = '\0';
	channel->used = true;

	handle->slot = channelCount;
	channelCount++;
	dirty = true;

	OLED_Unlock();
	return OLED_OK;
}

OLED_Status_t OLED_Post(OLED_Handle_t *handle, const char *fmt, ...) {
	OLED_Status_t status;
	char value[OLED_MANAGER_VALUE_LENGTH] = { 0 };
	va_list args;

	if (!initialized)
		return OLED_ERROR_NOT_INITIALIZED;
	if (handle == NULL || fmt == NULL)
		return OLED_ERROR_INVALID_ARGUMENT;

	va_start(args, fmt);
	if (vsnprintf(value, sizeof(value), fmt, args) < 0) {
		va_end(args);
		return OLED_ERROR_INVALID_ARGUMENT;
	}
	va_end(args);

	status = OLED_Lock();
	if (status != OLED_OK)
		return status;

	if (!OLED_HandleIsValid(handle)) {
		OLED_Unlock();
		return OLED_ERROR_INVALID_HANDLE;
	}

	if (strncmp(channels[handle->slot].value, value,
			sizeof(channels[handle->slot].value)) != 0) {
		memcpy(channels[handle->slot].value, value, sizeof(value));
		dirty = true;
	}

	OLED_Unlock();
	return OLED_OK;
}

void OLED_ManagerRefresh(void) {
	OLED_Status_t status;
	uint8_t visibleCount;
	char line[OLED_MANAGER_VISIBLE_COLS + 1U];

	if (!initialized)
		return;

	status = OLED_Lock();
	if (status != OLED_OK)
		return;

	if (!dirty) {
		OLED_Unlock();
		return;
	}

	visibleCount = channelCount;
	if (visibleCount > OLED_MANAGER_VISIBLE_ROWS)
		visibleCount = OLED_MANAGER_VISIBLE_ROWS;

	memset(snapshot, 0, sizeof(snapshot));
	memcpy(snapshot, channels, visibleCount * sizeof(OLED_Channel_t));
	dirty = false;

	OLED_Unlock();

	OLED_ClearGram();

	for (uint8_t row = 0U; row < visibleCount; row++) {
		memset(line, 0, sizeof(line));
		OLED_BuildLine(line, sizeof(line), &snapshot[row]);

#if OLED_MANAGER_RENDERER == OLED_RENDERER_5X7
		OLED_ShowString5x7(0U, row, line);
#elif OLED_MANAGER_RENDERER == OLED_RENDERER_LEGACY
		OLED_ShowString(0U, row * 16U, (const uint8_t *)line);
#endif
	}

	OLED_Refresh_Gram();
}
