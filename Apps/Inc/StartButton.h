#ifndef STARTBUTTON_H
#define STARTBUTTON_H

#include <stdbool.h>
#include <stdint.h>

/* Nonblocking debounce. A held button at boot is not a start action. */
typedef struct {
    uint32_t changedAt;
    uint32_t count;
    bool rawPressed;
    bool stablePressed;
    bool released;
    bool pressedAfterRelease;
} StartButton;

static inline void StartButton_Update(StartButton *button, bool pressed,
                                      uint32_t nowMs) {
    if (pressed != button->rawPressed) {
        button->rawPressed = pressed;
        button->changedAt = nowMs;
    }
    if ((uint32_t)(nowMs - button->changedAt) < 30U)
        return;
    if (!pressed) {
        if (button->stablePressed && button->pressedAfterRelease)
            ++button->count;
        button->released = true;
        button->pressedAfterRelease = false;
    } else if (!button->stablePressed) {
        button->pressedAfterRelease = button->released;
    }
    button->stablePressed = pressed;
}

#endif
