/*
 * ICM20948.h
 *
 * Driver for the ICM-20948 9-axis IMU.
 *
 *  Created on: 2026年8月28日
 *      Author: Joe
 */

#ifndef INC_ICM20948_H_
#define INC_ICM20948_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

/* --------------------------------------------------------------------------
 * Device
 * -------------------------------------------------------------------------- */

#define ICM20948_I2C_ADDR_7BIT          0x68U
#define ICM20948_I2C_ADDR               (ICM20948_I2C_ADDR_7BIT << 1U)

#define ICM20948_WHO_AM_I_VALUE         0xEAU

#define ICM20948_I2C_TIMEOUT_MS         100U


/* --------------------------------------------------------------------------
 * Register banks
 * -------------------------------------------------------------------------- */

#define ICM20948_BANK_0                 0x00U
#define ICM20948_BANK_1                 0x10U
#define ICM20948_BANK_2                 0x20U
#define ICM20948_BANK_3                 0x30U

/* Present in every register bank. */
#define ICM20948_REG_BANK_SEL           0x7FU


/* --------------------------------------------------------------------------
 * User Bank 0
 * -------------------------------------------------------------------------- */

#define ICM20948_REG_WHO_AM_I           0x00U
#define ICM20948_REG_USER_CTRL          0x03U
#define ICM20948_REG_LP_CONFIG          0x05U
#define ICM20948_REG_PWR_MGMT_1         0x06U
#define ICM20948_REG_PWR_MGMT_2         0x07U

#define ICM20948_REG_INT_PIN_CFG        0x0FU
#define ICM20948_REG_INT_ENABLE_1       0x11U
#define ICM20948_REG_INT_STATUS_1       0x1AU

#define ICM20948_REG_ACCEL_XOUT_H       0x2DU
#define ICM20948_REG_ACCEL_XOUT_L       0x2EU
#define ICM20948_REG_ACCEL_YOUT_H       0x2FU
#define ICM20948_REG_ACCEL_YOUT_L       0x30U
#define ICM20948_REG_ACCEL_ZOUT_H       0x31U
#define ICM20948_REG_ACCEL_ZOUT_L       0x32U

#define ICM20948_REG_GYRO_XOUT_H        0x33U
#define ICM20948_REG_GYRO_XOUT_L        0x34U
#define ICM20948_REG_GYRO_YOUT_H        0x35U
#define ICM20948_REG_GYRO_YOUT_L        0x36U
#define ICM20948_REG_GYRO_ZOUT_H        0x37U
#define ICM20948_REG_GYRO_ZOUT_L        0x38U

#define ICM20948_REG_TEMP_OUT_H         0x39U
#define ICM20948_REG_TEMP_OUT_L         0x3AU


/* --------------------------------------------------------------------------
 * User Bank 2
 * -------------------------------------------------------------------------- */

#define ICM20948_REG_GYRO_SMPLRT_DIV    0x00U
#define ICM20948_REG_GYRO_CONFIG_1      0x01U
#define ICM20948_REG_GYRO_CONFIG_2      0x02U

#define ICM20948_REG_ACCEL_SMPLRT_DIV_1 0x10U
#define ICM20948_REG_ACCEL_SMPLRT_DIV_2 0x11U
#define ICM20948_REG_ACCEL_CONFIG       0x14U
#define ICM20948_REG_ACCEL_CONFIG_2     0x15U


/* --------------------------------------------------------------------------
 * PWR_MGMT_1
 * -------------------------------------------------------------------------- */

#define ICM20948_PWR_MGMT_1_RESET       0x80U
#define ICM20948_PWR_MGMT_1_CLK_AUTO    0x01U

/* PWR_MGMT_2: all zeroes means all accel/gyro axes enabled. */
#define ICM20948_PWR_MGMT_2_ENABLE_ALL  0x00U


/* --------------------------------------------------------------------------
 * Default gyro configuration
 *
 * DLPF_CFG = 3  -> ~51.2 Hz bandwidth
 * FS_SEL   = 1  -> +/-500 dps
 * FCHOICE  = 1  -> DLPF enabled
 * -------------------------------------------------------------------------- */

#define ICM20948_GYRO_DLPF_CFG_3        (3U << 3U)
#define ICM20948_GYRO_FS_500_DPS        (1U << 1U)
#define ICM20948_GYRO_DLPF_ENABLE       0x01U

#define ICM20948_GYRO_CONFIG_DEFAULT    \
	(ICM20948_GYRO_DLPF_CFG_3 |         \
	 ICM20948_GYRO_FS_500_DPS |         \
	 ICM20948_GYRO_DLPF_ENABLE)


/* --------------------------------------------------------------------------
 * Default accelerometer configuration
 *
 * DLPF_CFG = 3  -> ~50.4 Hz bandwidth
 * FS_SEL   = 1  -> +/-4 g
 * FCHOICE  = 1  -> DLPF enabled
 * -------------------------------------------------------------------------- */

#define ICM20948_ACCEL_DLPF_CFG_3       (3U << 3U)
#define ICM20948_ACCEL_FS_4G            (1U << 1U)
#define ICM20948_ACCEL_DLPF_ENABLE      0x01U

#define ICM20948_ACCEL_CONFIG_DEFAULT   \
	(ICM20948_ACCEL_DLPF_CFG_3 |        \
	 ICM20948_ACCEL_FS_4G |             \
	 ICM20948_ACCEL_DLPF_ENABLE)


/*
 * ODR = 1125 / (1 + divider)
 *
 * divider = 9 -> 112.5 Hz
 */
#define ICM20948_GYRO_SMPLRT_DIV_DEFAULT    9U
#define ICM20948_ACCEL_SMPLRT_DIV_DEFAULT   9U

#define GYRO_SMPLRT_DIV	ICM20948_GYRO_SMPLRT_DIV_DEFAULT

/* --------------------------------------------------------------------------
 * Measurement conversion
 * -------------------------------------------------------------------------- */

/*
 * Current default configuration:
 *   Accelerometer: +/-4 g   -> 8192 LSB/g
 *   Gyroscope:     +/-500 dps -> 65.5 LSB/(dps)
 */
#define ICM20948_ACCEL_LSB_PER_G        8192.0f
#define ICM20948_GYRO_LSB_PER_DPS       65.5f

#define ICM20948_TEMP_LSB_PER_C         333.87f
#define ICM20948_TEMP_OFFSET_C          21.0f

#define ICM20948_MEASUREMENT_SIZE       14U

typedef struct {
	float x;
	float y;
	float z;
} ICM20948f32Vector3;

typedef struct {
	int32_t x;
	int32_t y;
	int32_t z;
} ICM20948i32Vector3;

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint16_t devAddr;
	ICM20948f32Vector3 gyroBias;
} ICM20948;


typedef struct {
	int16_t accelX;
	int16_t accelY;
	int16_t accelZ;

	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;

	int16_t temperature;
} ICM20948RawMeasurement;




typedef struct {
	/* Acceleration in g. */
	ICM20948f32Vector3 accelG;

	/* Angular velocity in degrees per second. */
	ICM20948f32Vector3 gyroDps;

	/* Internal sensor temperature in degrees Celsius. */
	float temperatureC;
} ICM20948Measurement;


bool ICM20948_Init(ICM20948 *imu, I2C_HandleTypeDef *hi2c);

bool ICM20948_ReadRaw(
		ICM20948 *imu,
		ICM20948RawMeasurement *measurement);

bool ICM20948_ReadMeasurement(
		ICM20948 *imu,
		ICM20948Measurement *measurement);

#endif /* INC_ICM20948_H_ */
