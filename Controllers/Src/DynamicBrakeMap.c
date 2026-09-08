/*
 * DynamicBrakeMap.c
 *
 *  Created on: 2026年9月8日
 *      Author: Joe
 */


#include "DynamicBrakeMap.h"

#include <math.h>
#include <stddef.h>


static float DynamicBrakeMap_Lerp(
        float x,
        float x0,
        float x1,
        float y0,
        float y1)
{
    if (x1 <= x0)
        return y0;

    float t = (x - x0) / (x1 - x0);

    if (t < 0.0f)
        t = 0.0f;
    else if (t > 1.0f)
        t = 1.0f;

    return y0 + t * (y1 - y0);
}


static uint8_t DynamicBrakeMap_FindLowerIndex(
        const float *knots,
        uint8_t count,
        float value)
{
    if (count <= 1U)
        return 0U;

    if (value <= knots[0])
        return 0U;

    for (uint8_t i = 0U; i < count - 1U; i++)
    {
        if (value <= knots[i + 1U])
            return i;
    }

    return count - 2U;
}


static float DynamicBrakeMap_TableValue(
        const DynamicBrakeMap *map,
        uint8_t speedIndex,
        uint8_t demandIndex)
{
    uint16_t index =
        (uint16_t)speedIndex * map->demandCount
        + demandIndex;

    return map->pwmTable[index];
}


bool DynamicBrakeMap_ValidateMap(
        const DynamicBrakeMap *map)
{
    if (map == NULL ||
        map->speedKnotsCps == NULL ||
        map->demandKnots == NULL ||
        map->pwmTable == NULL ||
        map->speedCount == 0U ||
        map->demandCount == 0U)
    {
        return false;
    }

    for (uint8_t i = 1U; i < map->speedCount; i++)
    {
        if (map->speedKnotsCps[i] <=
            map->speedKnotsCps[i - 1U])
        {
            return false;
        }
    }

    for (uint8_t i = 1U; i < map->demandCount; i++)
    {
        if (map->demandKnots[i] <=
            map->demandKnots[i - 1U])
        {
            return false;
        }
    }

    uint16_t elementCount =
        (uint16_t)map->speedCount *
        (uint16_t)map->demandCount;

    for (uint16_t i = 0U; i < elementCount; i++)
    {
        if (map->pwmTable[i] < 0.0f ||
            map->pwmTable[i] > 100.0f)
        {
            return false;
        }
    }

    return true;
}


float DynamicBrakeMap_GetPWM(
        const DynamicBrakeMap *map,
        float brakeDemand,
        float speedCps)
{
    if (!DynamicBrakeMap_ValidateMap(map))
        return 0.0f;

    /*
     * Exact zero means coast.
     *
     * This is deliberately discontinuous from the active-braking
     * branch because the hardware has a minimum useful brake duty.
     */
    if (brakeDemand <= 0.0f)
        return 0.0f;


    float speed = fabsf(speedCps);

    /*
     * Do not extrapolate outside the measured speed range.
     */
    if (speed < map->speedKnotsCps[0])
        speed = map->speedKnotsCps[0];

    float maxSpeed =
        map->speedKnotsCps[map->speedCount - 1U];

    if (speed > maxSpeed)
        speed = maxSpeed;


    /*
     * Any positive demand below the minimum calibrated demand
     * receives the minimum calibrated active-brake command.
     */
    if (brakeDemand < map->demandKnots[0])
        brakeDemand = map->demandKnots[0];

    float maxDemand =
        map->demandKnots[map->demandCount - 1U];

    if (brakeDemand > maxDemand)
        brakeDemand = maxDemand;


    uint8_t speed0 =
        DynamicBrakeMap_FindLowerIndex(
            map->speedKnotsCps,
            map->speedCount,
            speed);

    uint8_t speed1 =
        map->speedCount == 1U ?
        speed0 :
        speed0 + 1U;


    uint8_t demand0 =
        DynamicBrakeMap_FindLowerIndex(
            map->demandKnots,
            map->demandCount,
            brakeDemand);

    uint8_t demand1 =
        map->demandCount == 1U ?
        demand0 :
        demand0 + 1U;


    /*
     * Interpolate brake PWM along the demand axis at each
     * neighbouring speed row.
     */
    float pwmAtSpeed0 =
        DynamicBrakeMap_Lerp(
            brakeDemand,
            map->demandKnots[demand0],
            map->demandKnots[demand1],
            DynamicBrakeMap_TableValue(
                map, speed0, demand0),
            DynamicBrakeMap_TableValue(
                map, speed0, demand1));


    float pwmAtSpeed1 =
        DynamicBrakeMap_Lerp(
            brakeDemand,
            map->demandKnots[demand0],
            map->demandKnots[demand1],
            DynamicBrakeMap_TableValue(
                map, speed1, demand0),
            DynamicBrakeMap_TableValue(
                map, speed1, demand1));


    /*
     * Then interpolate between speed rows.
     */
    return DynamicBrakeMap_Lerp(
        speed,
        map->speedKnotsCps[speed0],
        map->speedKnotsCps[speed1],
        pwmAtSpeed0,
        pwmAtSpeed1);
}
