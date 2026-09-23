#include "StartButton.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
    StartButton button = {0};
    /* Held at boot: releasing must not start a run. */
    StartButton_Update(&button, true, 0);
    StartButton_Update(&button, true, 31);
    StartButton_Update(&button, false, 50);
    StartButton_Update(&button, false, 81);
    assert(button.count == 0);
    /* Bounce is ignored, a full stable press/release counts once. */
    StartButton_Update(&button, true, 90);
    StartButton_Update(&button, false, 100);
    StartButton_Update(&button, true, 110);
    StartButton_Update(&button, true, 141);
    StartButton_Update(&button, true, 200);
    assert(button.count == 0);
    StartButton_Update(&button, false, 210);
    StartButton_Update(&button, true, 220);
    StartButton_Update(&button, false, 230);
    StartButton_Update(&button, false, 261);
    StartButton_Update(&button, false, 300);
    assert(button.count == 1);
    /* Unsigned tick wrap still debounces correctly. */
    StartButton_Update(&button, true, UINT32_MAX - 10U);
    StartButton_Update(&button, true, 25);
    StartButton_Update(&button, false, 30);
    StartButton_Update(&button, false, 61);
    assert(button.count == 2);
    puts("button tests passed");
}
