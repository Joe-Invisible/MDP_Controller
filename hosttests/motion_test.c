#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "CommandMotion.h"

int main(void) {
    CommandMotion motion;
    Command cmd = { COMMAND_FORWARD, 1000.0f };
    assert(CommandMotion_Resolve(&cmd, &motion));
    assert(motion.distanceMm == 1000.0f && motion.radiusMm == 0.0f);
    assert(motion.speedCps == 5000.0f);
    cmd.type = COMMAND_BACKWARD;
    assert(CommandMotion_Resolve(&cmd, &motion));
    assert(motion.distanceMm == -1000.0f && motion.radiusMm == 0.0f);

    /* Quarter-circle centre path, with opposite curvature for left/right. */
    cmd = (Command){ COMMAND_LEFT, 90.0f };
    assert(CommandMotion_Resolve(&cmd, &motion));
    assert(fabsf(motion.distanceMm - 431.9690f) < 0.001f);
    assert(motion.radiusMm == 275.0f && motion.speedCps == 2000.0f);
    cmd.type = COMMAND_RIGHT;
    assert(CommandMotion_Resolve(&cmd, &motion));
    assert(fabsf(motion.distanceMm - 431.9690f) < 0.001f);
    assert(motion.radiusMm == -275.0f && motion.speedCps == 2000.0f);
    cmd.param = 360.0f;
    assert(CommandMotion_Resolve(&cmd, &motion));
    assert(fabsf(motion.distanceMm - 1727.8760f) < 0.001f);
    cmd.param = 0.5f;
    assert(CommandMotion_Resolve(&cmd, &motion));
    assert(fabsf(motion.distanceMm - 2.399828f) < 0.00001f);

    const float invalid[] = { 0.0f, -90.0f, 360.1f, INFINITY, NAN };
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        cmd.param = invalid[i];
        assert(!CommandMotion_Resolve(&cmd, &motion));
    }
    cmd = (Command){ COMMAND_STOP, 1.0f };
    assert(!CommandMotion_Resolve(&cmd, &motion));
    assert(!CommandMotion_Resolve(NULL, &motion));
    assert(!CommandMotion_Resolve(&cmd, NULL));
    puts("motion_test: all checks passed");
}
