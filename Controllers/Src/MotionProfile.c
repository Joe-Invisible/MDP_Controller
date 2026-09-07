/*
 * MotionProfile.c
 *
 *  Created on: 2026年9月7日
 *      Author: Joe
 */


#include "MotionProfile.h"

#include <math.h>
#include <stddef.h>

static float MotionProfile_Min3(
    float a,
    float b,
    float c)
{
    float min = a;

    if (b < min)
        min = b;

    if (c < min)
        min = c;

    return min;
}

bool MotionProfile_Init(
    MotionProfile *profile,
    float accelerationMmps2,
    float decelerationMmps2)
{
    if (profile == NULL ||
        accelerationMmps2 <= 0.0f ||
        decelerationMmps2 <= 0.0f)
    {
        return false;
    }

    *profile = (MotionProfile){0};

    profile->accelerationMmps2 = accelerationMmps2;
    profile->decelerationMmps2 = decelerationMmps2;

    return true;
}

bool MotionProfile_Start(
    MotionProfile *profile,
    float distanceMm,
    float maxSpeedMmps)
{
    if (profile == NULL ||
        distanceMm <= 0.0f ||
        maxSpeedMmps <= 0.0f ||
        profile->accelerationMmps2 <= 0.0f ||
        profile->decelerationMmps2 <= 0.0f)
    {
        return false;
    }

    profile->targetDistanceMm = distanceMm;
    profile->maxSpeedMmps = maxSpeedMmps;
    profile->targetSpeedMmps = 0.0f;
    profile->active = true;

    return true;
}

float MotionProfile_Update(
    MotionProfile *profile,
    float travelledDistanceMm,
    float dt)
{
    if (profile == NULL || !profile->active)
        return 0.0f;

    if (dt <= 0.0f)
        return profile->targetSpeedMmps;

    if (travelledDistanceMm < 0.0f)
        travelledDistanceMm = 0.0f;

    float remainingDistanceMm =
        profile->targetDistanceMm - travelledDistanceMm;

    if (remainingDistanceMm <= 0.0f)
    {
        MotionProfile_Stop(profile);
        return 0.0f;
    }

    /*
     * Time-domain acceleration limit.
     *
     * This allows the profile to move away from zero even when
     * traveled distance is initially zero.
     */
    float accelerationLimitedSpeedMmps =
        profile->targetSpeedMmps +
        profile->accelerationMmps2 * dt;

    /*
     * Maximum speed from which zero speed can still be reached
     * over the remaining distance using the configured
     * deceleration magnitude:
     *
     *     v^2 = 2 a s
     */
    float brakingLimitedSpeedMmps =
        sqrtf(
            2.0f *
            profile->decelerationMmps2 *
            remainingDistanceMm);

    /*
     * This automatically generates a triangular profile
     * instead if the requested distance is too short to
     * attain the max speed
     */
    profile->targetSpeedMmps = MotionProfile_Min3(
        profile->maxSpeedMmps,
        accelerationLimitedSpeedMmps,
        brakingLimitedSpeedMmps);

    return profile->targetSpeedMmps;
}

void MotionProfile_Stop(MotionProfile *profile)
{
    if (profile == NULL)
        return;

    profile->targetSpeedMmps = 0.0f;
    profile->active = false;
}

bool MotionProfile_IsActive(const MotionProfile *profile)
{
    return profile != NULL && profile->active;
}
