/*
 * DynamicBrakeMap.h
 *
 * Maps a normalized braking demand and current wheel speed
 * to the physical H-bridge brake PWM required by the motor.
 *
 *	Created on: 2026年9月8日
 *      Author: Joe
 */

#ifndef INC_DYNAMICBRAKEMAP_H_
#define INC_DYNAMICBRAKEMAP_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * pwmTable is stored row-major:
 *
 *   [speed 0, demand 0]
 *   [speed 0, demand 1]
 *   ...
 *   [speed 1, demand 0]
 *   ...
 *
 * speedKnotsCps and demandKnots must both be strictly increasing.
 *
 * brakeDemand semantics:
 *
 *   0.0f
 *       No active braking. Returns 0% PWM / coast.
 *
 *   > 0.0f
 *       Active braking.
 *
 * The first demand knot represents the minimum calibrated
 * non-zero braking demand. Requests between 0 and that knot
 * are promoted to the first knot rather than extrapolated.
 */
typedef struct
{
    const float *speedKnotsCps;
    uint8_t speedCount;

    const float *demandKnots;
    uint8_t demandCount;

    const float *pwmTable;

} DynamicBrakeMap;


bool DynamicBrakeMap_ValidateMap(
        const DynamicBrakeMap *map);


/*
 * Returns physical brake PWM percentage [0, 100].
 *
 * speedCps:
 *   Sign is ignored; braking depends on speed magnitude.
 *
 * Speeds outside the calibrated range are clamped to the
 * nearest calibrated speed rather than extrapolated.
 */
float DynamicBrakeMap_GetPWM(
        const DynamicBrakeMap *map,
        float brakeDemand,
        float speedCps);

#endif /* INC_DYNAMICBRAKEMAP_H_ */
