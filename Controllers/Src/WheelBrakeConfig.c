/*
 * WheelBrakeConfig.c
 *
 * Rear-wheel dynamic-brake calibration.
 *
 * Source experiment:
 *   8 Sep 2026, 14:27 SGT
 *   Battery: 10.95 V
 *   Rear wheels lifted
 *
 * brakeDemand is normalized braking authority derived from
 * reduction in wheel run-on distance between coast and
 * full dynamic braking.
 *
 * It is NOT braking torque percentage or current percentage.
 *
 *  Created on: 2026年9月8日
 *      Author: Joe
 */


#include "WheelBrakeConfig.h"


static const float rearBrakeSpeedKnotsCps[] =
{
    500.0f,
    750.0f,
    1000.0f,
    1500.0f,
    2000.0f
};


/*
 * We intentionally calibrate only from 25% authority upward.
 *
 * The experiment did not sufficiently characterize braking
 * below this level, particularly at high wheel speed.
 *
 * Therefore:
 *
 *   demand == 0       -> coast
 *   0 < demand < 0.25 -> promoted to 0.25
 */
static const float rearBrakeDemandKnots[] =
{
    0.25f,
    0.50f,
    0.75f,
    1.00f
};


/*
 * Physical H-bridge brake PWM [%].
 *
 * Rows:    wheel speed
 * Columns: normalized brake demand
 *
 *              0.25    0.50    0.75    1.00
 */
static const float rearBrakePwmTable[] =
{
    /*  500 CPS */
    95.6f,  97.4f,  98.7f, 100.0f,

    /*  750 CPS */
    93.2f,  95.9f,  98.0f, 100.0f,

    /* 1000 CPS */
    91.8f,  94.8f,  97.4f, 100.0f,

    /* 1500 CPS */
    88.6f,  92.1f,  95.6f, 100.0f,

    /* 2000 CPS */
    85.5f,  89.8f,  94.1f, 100.0f
};


const DynamicBrakeMap rearWheelBrakeMap =
{
    .speedKnotsCps =
        rearBrakeSpeedKnotsCps,

    .speedCount =
        sizeof(rearBrakeSpeedKnotsCps) /
        sizeof(rearBrakeSpeedKnotsCps[0]),

    .demandKnots =
        rearBrakeDemandKnots,

    .demandCount =
        sizeof(rearBrakeDemandKnots) /
        sizeof(rearBrakeDemandKnots[0]),

    .pwmTable =
        rearBrakePwmTable
};
