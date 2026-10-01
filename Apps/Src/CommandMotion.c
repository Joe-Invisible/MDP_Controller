#include "CommandMotion.h"
#include "UltrasonicApproachConfig.h"

#include <math.h>

bool CommandMotion_Resolve(const Command *command, CommandMotion *motion) {
    if (!command || !motion || !isfinite(command->param) || command->param <= 0.0f)
        return false;

    CommandMotion result = {
        .distanceMm = command->param,
        .radiusMm = 0.0f,
        .speedCps = COMMANDMOTION_STRAIGHT_SPEED_CPS,
    };
    switch (command->type) {
    case COMMAND_FORWARD:
        break;
    case COMMAND_BACKWARD:
        result.distanceMm = -command->param;
        break;
    case COMMAND_ULTRASONIC:
        if (command->param < ULTRASONIC_APPROACH_MIN_TARGET_MM ||
            command->param > ULTRASONIC_APPROACH_MAX_TARGET_MM)
            return false;
        /* Runtime derives a tighter travel bound from the initial reading. */
        result.distanceMm = ULTRASONIC_APPROACH_MAX_TRAVEL_MM;
        result.speedCps = 0.0f;
        result.speedMmps = ULTRASONIC_APPROACH_CRUISE_MMPS;
        break;
    case COMMAND_LEFT:
    case COMMAND_RIGHT:
        if (command->param > COMMANDMOTION_MAX_TURN_DEG)
            return false;
        /* Forward arc: s = |R| * angle in radians. */
        result.distanceMm = COMMANDMOTION_TURN_RADIUS_MM *
                            (command->param * (3.14159265358979323846f / 180.0f));
        result.radiusMm = command->type == COMMAND_LEFT
            ? COMMANDMOTION_TURN_RADIUS_MM : -COMMANDMOTION_TURN_RADIUS_MM;
        result.speedCps = COMMANDMOTION_TURN_SPEED_CPS;
        break;
    default:
        return false;
    }
    *motion = result;
    return true;
}
