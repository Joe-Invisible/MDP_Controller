/*
 * ICM20948Test.c
 *
 *  Created on: 2026年8月28日
 *      Author: Joe
 */

#include "ICM20948Test.h"
#include "oledutils.h"

#include "i2c.h"

#include <math.h>


#define ICM20948_TEST_SAMPLE_COUNT       500U
#define ICM20948_TEST_SAMPLE_DELAY_MS    10U


void ICM20948TestRun()
{
    ICM20948 imu = { 0 };
    ICM20948Measurement meas = { 0 };

    float mean = 0.0f;
    float m2 = 0.0f;

    OLED_Clear();
    OLED_Printf(0, 0, "Keep IMU still");
    OLED_Refresh_Gram();

    /* Give the user time to leave the robot stationary. */
    HAL_Delay(1000U);

    if (!ICM20948_Init(&imu, &hi2c2)) {
        OLED_Clear();
        OLED_Printf(0, 0, "IMU Init Failed.");
        OLED_Refresh_Gram();
        return;
    }

    OLED_Clear();
    OLED_Printf(0, 0, "Testing gyro Z");
    OLED_Printf(0, 1, "Bias: %.4f", imu.gyroBias.z);
    OLED_Printf(0, 2, "Collecting...");
    OLED_Refresh_Gram();

    /*
     * Calculate mean and variance using Welford's algorithm.
     *
     * This avoids storing all samples and is numerically more stable
     * than calculating sum(x^2) - mean^2.
     */
    for (uint32_t i = 0; i < ICM20948_TEST_SAMPLE_COUNT; i++) {
        if (!ICM20948_ReadMeasurement(&imu, &meas)) {
            OLED_Clear();
            OLED_Printf(0, 0, "IMU Read Failed.");
            OLED_Printf(0, 1, "Sample: %lu", i);
            OLED_Refresh_Gram();
            return;
        }

        float gyroZ = meas.gyroDps.z;

        float delta = gyroZ - mean;
        mean += delta / (float)(i + 1U);

        float delta2 = gyroZ - mean;
        m2 += delta * delta2;

        /*
         * Gyro ODR is 112.5 Hz (~8.89 ms/sample).
         * Waiting 10 ms should give us a new measurement each iteration.
         */
        HAL_Delay(ICM20948_TEST_SAMPLE_DELAY_MS);
    }

    float variance =
        m2 / (float)(ICM20948_TEST_SAMPLE_COUNT - 1U);

    float standardDeviation = sqrtf(variance);

    /*
     * Approximate yaw drift caused by the remaining mean offset if it
     * remained constant for 10 seconds.
     */
    float estimatedDrift10s = mean * 10.0f;

    OLED_Clear();
    OLED_Printf(0, 0, "gyroZ stationary");
    OLED_Printf(0, 1, "Bias %.4f dps", imu.gyroBias.z);
    OLED_Printf(0, 2, "Mean %.4f dps", mean);
    OLED_Printf(0, 3, "Std  %.4f dps", standardDeviation);
    OLED_Printf(0, 4, "10s  %.2f deg", estimatedDrift10s);
    OLED_Printf(0, 5, "N = %lu", ICM20948_TEST_SAMPLE_COUNT);
    OLED_Refresh_Gram();
}
