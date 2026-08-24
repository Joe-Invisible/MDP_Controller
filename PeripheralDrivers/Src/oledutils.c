/*
 * oledutils.c
 *
 *  Created on: 2026年8月24日
 *      Author: Joe
 */

#include "oledutils.h"
#include "oled.h"

#include <stdio.h>
#include <stdarg.h>

#define OLED_TEXT_BUFFER_SIZE 32

void OLED_Printf(uint8_t x, uint8_t y, const char *fmt, ...)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    OLED_ShowString(x, y, (const uint8_t *)buffer);
}
