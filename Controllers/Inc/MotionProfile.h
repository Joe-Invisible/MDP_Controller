/*
 * MotionProfile.h
 *
 * Generates a bounded vehicle-speed reference for finite-distance or caller-terminated motion.
 * All distances and speeds are unsigned magnitudes.
 *
 *  Created on: 2026年9月7日
 *      Author: Joe
 */

#ifndef INC_MOTIONPROFILE_H_
#define INC_MOTIONPROFILE_H_

#include <stdbool.h>

typedef struct MotionProfile
{
    /*
     * Profile limits
     */
    float accelerationMmps2;
    float decelerationMmps2;

    /*
     * Current motion request
     */
    float targetDistanceMm;
    float maxSpeedMmps;
    /* Optional soft cruise request. Zero inherits maxSpeedMmps. */
    float requestedCruiseSpeedMmps;

    /*
     * Current generated speed reference
     */
    float targetSpeedMmps;

    bool active;
    bool continuous; /* No distance completion; caller stops the profile. */

    float completionToleranceMm;

} MotionProfile;

/**
 * @brief Initializes a motion profile.
 * 		accelerationMmps2 and decelerationMmps2
 * 		must be positive magnitudes.
 */
bool MotionProfile_Init(MotionProfile *profile,
                        float accelerationMmps2,
                        float decelerationMmps2,
                        float completionToleranceMm);

/**
 * @brief Starts a rest-to-rest motion profile.
 * 		distanceMm and maxSpeedMmps must be
 * 		positive magnitudes.
 */
bool MotionProfile_Start(MotionProfile *profile, float distanceMm, float maxSpeedMmps);

/* Start acceleration-limited cruise without a distance endpoint. */
bool MotionProfile_StartContinuous(MotionProfile *profile, float maxSpeedMmps);

/* Change cruise speed while active, preserving distance, speed and progress.
 * The hard max/stopping envelope still wins; requests use normal accel/decel. */
bool MotionProfile_RequestCruiseSpeed(MotionProfile *profile, float speedMmps);

/**
 * @brief Updates the speed reference using travelled
 * 		distance measured from the start of the motion
 * 		and the elapsed control-loop time.
 *
 * Returns the unsigned target vehicle speed in mm/s.
 */
float MotionProfile_Update(MotionProfile *profile, float travelledDistanceMm, float dt);

/* Same acceleration/completion rules, with a stopping-speed envelope computed
 * by a path planner for varying deceleration along the remaining distance. */
float MotionProfile_UpdateWithBrakingLimit(MotionProfile *profile,
                                           float travelledDistanceMm,
                                           float dt,
                                           float brakingSpeedLimitMmps);

/**
 * @brief Stops profile generation and resets the
 * 		target speed to zero.
 */
void MotionProfile_Stop(MotionProfile *profile);

/**
 * @brief Returns whether a profile is currently active.
 */
bool MotionProfile_IsActive(const MotionProfile *profile);

#endif /* INC_MOTIONPROFILE_H_ */
