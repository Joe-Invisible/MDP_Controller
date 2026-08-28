/*
 * ICM20948.c
 *
 * Driver for the ICM-20948 9-axis IMU.
 *
 *  Created on: 2026年8月28日
 *      Author: Joe
 */


#include "icm20948.h"

#define ICM20948_CALIBRATION_SAMPLE_COUNT 100

#if ICM20948_CALIBRATION_SAMPLE_COUNT <= 0
#error "Calibration sample count must be positive!"
#endif /* ICM20948_CALIBRATION_SAMPLE_COUNT */

 /**
  * For converting Big-Endian measurements
  */
static int16_t ICM20948_MakeInt16(uint8_t high, uint8_t low) {
	return (int16_t)(
		((uint16_t)high << 8U) |
		(uint16_t)low);
}

static bool ICM20948_WriteReg(ICM20948* imu, uint8_t reg, uint8_t value) {
	return HAL_I2C_Mem_Write(
		imu->hi2c,
		imu->devAddr,
		reg,
		I2C_MEMADD_SIZE_8BIT,
		&value,
		1U,
		ICM20948_I2C_TIMEOUT_MS) == HAL_OK;
}


static bool ICM20948_ReadRegs(
	ICM20948* imu,
	uint8_t reg,
	uint8_t* data,
	uint16_t size) {

	return HAL_I2C_Mem_Read(
		imu->hi2c,
		imu->devAddr,
		reg,
		I2C_MEMADD_SIZE_8BIT,
		data,
		size,
		ICM20948_I2C_TIMEOUT_MS) == HAL_OK;
}


static bool ICM20948_ReadReg(
	ICM20948* imu,
	uint8_t reg,
	uint8_t* value) {

	return ICM20948_ReadRegs(
		imu,
		reg,
		value,
		1U);
}


static bool ICM20948_SelectBank(ICM20948* imu, uint8_t bank) {
	return ICM20948_WriteReg(
		imu,
		ICM20948_REG_BANK_SEL,
		bank);
}

static bool ICM20948_EstimateGyroBias(ICM20948* imu) {
	ICM20948RawMeasurement meas = { 0 };
	ICM20948i32Vector3 bias = { 0 };
	imu->gyroBias.x = 0;
	imu->gyroBias.y = 0;
	imu->gyroBias.z = 0;
	for (int i = 0; i < ICM20948_CALIBRATION_SAMPLE_COUNT; i++) {
		if (!ICM20948_ReadRaw(imu, &meas))
				return false;
		bias.x += meas.gyroX;
		bias.y += meas.gyroY;
		bias.z += meas.gyroZ;

		// Wait approximately for the next sampling period
		HAL_Delay(GYRO_SMPLRT_DIV);
	}

	imu->gyroBias.x = (float)bias.x / (float)ICM20948_CALIBRATION_SAMPLE_COUNT / ICM20948_GYRO_LSB_PER_DPS;
	imu->gyroBias.y = (float)bias.y / (float)ICM20948_CALIBRATION_SAMPLE_COUNT / ICM20948_GYRO_LSB_PER_DPS;
	imu->gyroBias.z = (float)bias.z / (float)ICM20948_CALIBRATION_SAMPLE_COUNT / ICM20948_GYRO_LSB_PER_DPS;

	return true;
}


bool ICM20948_Init(ICM20948* imu, I2C_HandleTypeDef* hi2c) {
	uint8_t whoAmI = 0xff;

	if (imu == NULL || hi2c == NULL)
		return false;

	imu->hi2c = hi2c;
	imu->devAddr = ICM20948_I2C_ADDR;


	/* Verify device identity. */
	if (!ICM20948_SelectBank(imu, ICM20948_BANK_0))
		return false;

	if (!ICM20948_ReadReg(imu, ICM20948_REG_WHO_AM_I, &whoAmI))
		return false;

	if (whoAmI != ICM20948_WHO_AM_I_VALUE)
		return false;


	/* Reset the device. */
	if (!ICM20948_WriteReg(
		imu,
		ICM20948_REG_PWR_MGMT_1,
		ICM20948_PWR_MGMT_1_RESET))
		return false;

	/**
	 * Wait blindly hoping that reset bit then becomes cleared
	 * We could do this more elegantly by directly polling the reset bit,
	 * but would need to ensure I2C becomes reliable after reset.
	 */
	HAL_Delay(100U);


	/*
	 * Reset restores the register bank and power-management registers
	 * to their defaults, so explicitly select Bank 0 again.
	 */
	if (!ICM20948_SelectBank(imu, ICM20948_BANK_0))
		return false;


	/*
	 * Wake the device and select the best available clock source.
	 * CLKSEL = 1 allows the device to use its PLL when available.
	 */
	if (!ICM20948_WriteReg(
		imu,
		ICM20948_REG_PWR_MGMT_1,
		ICM20948_PWR_MGMT_1_CLK_AUTO))
		return false;


	/* Enable all accelerometer and gyroscope axes. */
	if (!ICM20948_WriteReg(
		imu,
		ICM20948_REG_PWR_MGMT_2,
		ICM20948_PWR_MGMT_2_ENABLE_ALL))
		return false;

	/* Accelerometer and gyroscope configuration lives in Bank 2. */
	if (!ICM20948_SelectBank(imu, ICM20948_BANK_2))
		return false;


	/* Gyroscope: 112.5 Hz ODR, +/-500 dps, ~51 Hz DLPF. */
	if (!ICM20948_WriteReg(
		imu,
		ICM20948_REG_GYRO_SMPLRT_DIV,
		ICM20948_GYRO_SMPLRT_DIV_DEFAULT))
		return false;

	if (!ICM20948_WriteReg(
		imu,
		ICM20948_REG_GYRO_CONFIG_1,
		ICM20948_GYRO_CONFIG_DEFAULT))
		return false;


	/* Accelerometer: 112.5 Hz ODR, +/-4 g, ~50 Hz DLPF. */
	if (!ICM20948_WriteReg(
		imu,
		ICM20948_REG_ACCEL_SMPLRT_DIV_1,
		(uint8_t)((ICM20948_ACCEL_SMPLRT_DIV_DEFAULT >> 8U) & 0x0FU)))
		return false;

	if (!ICM20948_WriteReg(
		imu,
		ICM20948_REG_ACCEL_SMPLRT_DIV_2,
		(uint8_t)(ICM20948_ACCEL_SMPLRT_DIV_DEFAULT & 0xFFU)))
		return false;

	if (!ICM20948_WriteReg(
		imu,
		ICM20948_REG_ACCEL_CONFIG,
		ICM20948_ACCEL_CONFIG_DEFAULT))
		return false;


	/*
	 * Leave the device in Bank 0 since this is where the sensor-output
	 * registers are located.
	 */
	if (!ICM20948_SelectBank(imu, ICM20948_BANK_0))
		return false;

	// wait for gyro startup from sleep state
	HAL_Delay(100U);

	if (!ICM20948_EstimateGyroBias(imu))
		return false;

	return true;
}

bool ICM20948_ReadRaw(
	ICM20948* imu,
	ICM20948RawMeasurement* measurement) {

	uint8_t data[ICM20948_MEASUREMENT_SIZE] = { 0 };

	if (imu == NULL || measurement == NULL)
		return false;

	/*
	 * Measurement registers are located in User Bank 0.
	 */
	if (!ICM20948_SelectBank(imu, ICM20948_BANK_0))
		return false;

	/*
	 * Read ACCEL_XOUT_H through TEMP_OUT_L in one burst.
	 */
	if (!ICM20948_ReadRegs(
		imu,
		ICM20948_REG_ACCEL_XOUT_H,
		data,
		sizeof(data)))
		return false;

	measurement->accelX =
		ICM20948_MakeInt16(data[0], data[1]);

	measurement->accelY =
		ICM20948_MakeInt16(data[2], data[3]);

	measurement->accelZ =
		ICM20948_MakeInt16(data[4], data[5]);

	measurement->gyroX =
		ICM20948_MakeInt16(data[6], data[7]);

	measurement->gyroY =
		ICM20948_MakeInt16(data[8], data[9]);

	measurement->gyroZ =
		ICM20948_MakeInt16(data[10], data[11]);

	measurement->temperature =
		ICM20948_MakeInt16(data[12], data[13]);

	return true;
}

bool ICM20948_ReadMeasurement(
	ICM20948* imu,
	ICM20948Measurement* measurement) {

	ICM20948RawMeasurement raw;

	if (imu == NULL || measurement == NULL)
		return false;

	if (!ICM20948_ReadRaw(imu, &raw))
		return false;

	/* Accelerometer: raw value -> g. */
	measurement->accelG.x =
		(float)raw.accelX / ICM20948_ACCEL_LSB_PER_G;

	measurement->accelG.y =
		(float)raw.accelY / ICM20948_ACCEL_LSB_PER_G;

	measurement->accelG.z =
		(float)raw.accelZ / ICM20948_ACCEL_LSB_PER_G;


	/* Gyroscope: raw value -> degrees per second. */
	measurement->gyroDps.x =
		(float)raw.gyroX / ICM20948_GYRO_LSB_PER_DPS;
	measurement->gyroDps.x -= imu->gyroBias.x;

	measurement->gyroDps.y =
		(float)raw.gyroY / ICM20948_GYRO_LSB_PER_DPS;
	measurement->gyroDps.y -= imu->gyroBias.y;

	measurement->gyroDps.z =
		(float)raw.gyroZ / ICM20948_GYRO_LSB_PER_DPS;
	measurement->gyroDps.z -= imu->gyroBias.z;


	/*
	 * Temperature:
	 *
	 * T = raw / 333.87 + 21
	 */
	measurement->temperatureC =
		((float)raw.temperature / ICM20948_TEMP_LSB_PER_C)
		+ ICM20948_TEMP_OFFSET_C;

	return true;
}
