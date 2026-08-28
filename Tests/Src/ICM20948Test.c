/*
 * ICM20948Test.c
 *
 *  Created on: 2026年8月28日
 *      Author: Joe
 */

#include "ICM20948Test.h"
#include "oledutils.h"
#include "i2c.h"
#include "userbutton.h"
#include "math.h"

void ICM20948TestRun() {
	ICM20948 imu = { 0 };
	ICM20948Measurement meas = { 0 };
	OLED_Clear();
	if (ICM20948_Init(&imu, &hi2c2)) {
		OLED_Printf(0, 0, "IMU Init OK.");
	} else {
		// We should return if init fails, but I guess it never would
		// Gloria, Gloria! In excelsis ICM20948!
		OLED_Printf(0, 0, "IMU Init Failed.");
	}

	OLED_Printf(0, 1, "d %.2f %.2f %.2f", imu.gyroBias.x, imu.gyroBias.y, imu.gyroBias.z);

	OLED_Refresh_Gram();

	HAL_Delay(2000);

	int reads = 0;
	for (;;) {
		/**
		 * Allow stopping with user button press
		 */
		if (SW1_ReadState() == SW1_Enabled) break;

		if (!ICM20948_ReadMeasurement(&imu, &meas)) break;

		float accMagnitude = (float)sqrt(
				meas.accelG.x * meas.accelG.x +
				meas.accelG.y * meas.accelG.y +
				meas.accelG.z * meas.accelG.z

			);

		OLED_Printf(0, 2, "g %.2f %.2f %.2f", meas.gyroDps.x, meas.gyroDps.y, meas.gyroDps.z);
		OLED_Printf(0, 3, "T %.2f", meas.temperatureC);
		OLED_Printf(0, 4, "a_mag %.2f", accMagnitude);

		OLED_Printf(0, 5, "%d Reads", ++reads);

		OLED_Refresh_Gram();

		HAL_Delay(200);

	}

}
