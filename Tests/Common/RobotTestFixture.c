/*
 * RobotTestFixture.c
 *
 *  Created on: 2026年8月31日
 *      Author: Joe
 */

#include "tim.h"
#include "i2c.h"

#include "RobotTestFixture.h"
#include "RobotPeripherals.h"

#define MOTORBPWMSRC htim9
#define MOTORCPWMSRC htim1

#define MOTORBENC htim3
#define MOTORCENC htim4

#define SERVOPWMSRC htim8
#define SERVOPWMCH TIM_CHANNEL_1

#define CONFIGURE_DYNAMIC_BRAKES 1

#if CONFIGURE_DYNAMIC_BRAKES == 1
static const WheelSpeedBrakeConfig wheelSpeedBrakeConfig = {.map = &rearWheelBrakeMap,

                                                            .engageOverspeedCps = 200.0f,
                                                            .releaseOverspeedCps = 100.0f,

                                                            .fullDemandOverspeedCps = 800.0f};
#endif

bool RobotTestFixture_ReadUltrasonicMm(float *distanceMm)
{
    if (distanceMm == NULL) return false;
    *distanceMm = 0.0f;
    HCSR04_Init(&hcsr04, USS_TRIG_GPIO_Port, USS_TRIG_Pin, &htim12, TIM_CHANNEL_2);
    if (!HCSR04_Trigger(&hcsr04)) return false;
    while (HCSR04_IsBusy(&hcsr04))
    {
        HCSR04_Update(&hcsr04);
        HAL_Delay(1U);
    }
    if (!HCSR04_HasMeasurement(&hcsr04)) return false;
    float range = HCSR04_GetDistanceMm(&hcsr04);
    if (range < 20.0f || range > 4000.0f) return false;
    *distanceMm = range;
    return true;
}

bool RobotTestFixture_InitIMU(RobotTestFixture *fixture)
{
    if (fixture == NULL) return false;
    if (!ICM20948_Init(&fixture->imu, &hi2c2)) return false;

    return true;
}

bool RobotTestFixture_InitRearWheels(RobotTestFixture *fixture)
{

    if (fixture == NULL) return false;

    if (!DCMotor_Init(&fixture->leftRearWheel, &MOTORBPWMSRC, false, &MOTORBENC) ||
        !DCMotor_Init(&fixture->rightRearWheel, &MOTORCPWMSRC, true, &MOTORCENC))
    {
        return false;
    }

    if (!DCMotor_Enable(&fixture->leftRearWheel)) return false;
    if (!DCMotor_Enable(&fixture->rightRearWheel)) return false;

    return true;
}

bool RobotTestFixture_InitFrontWheels(RobotTestFixture *fixture)
{
    if (fixture == NULL) return false;

    if (!Servo_Init(&fixture->steeringServo,
                    &SERVOPWMSRC,
                    SERVOPWMCH,
                    CHASSIS_STEER_MIN_PULSE_US,
                    CHASSIS_STEER_CTR_PULSE_US,
                    CHASSIS_STEER_MAX_PULSE_US))
        return false;

    if (!Servo_Enable(&fixture->steeringServo)) return false;

    return true;
}

bool RobotTestFixture_InitWheelControllers(RobotTestFixture *fixture)
{
    if (fixture == NULL) return false;

    if (!WheelSpeedController_Init(&fixture->leftWheelController,
                                   &fixture->leftRearWheel,
                                   WHEELSPEEDCONTROLLER_KP,
                                   WHEELSPEEDCONTROLLER_KI,
                                   WHEELSPEEDCONTROLLER_MIN_FEEDBACK,
                                   WHEELSPEEDCONTROLLER_MAX_FEEDBACK,
                                   &leftCalibration))
    {
        return false;
    }

    if (!WheelSpeedController_Init(&fixture->rightWheelController,
                                   &fixture->rightRearWheel,
                                   WHEELSPEEDCONTROLLER_KP,
                                   WHEELSPEEDCONTROLLER_KI,
                                   WHEELSPEEDCONTROLLER_MIN_FEEDBACK,
                                   WHEELSPEEDCONTROLLER_MAX_FEEDBACK,
                                   &rightCalibration))
    {
        return false;
    }

    return true;
}

bool RobotTestFixture_InitSteeringController(RobotTestFixture *fixture)
{
    if (fixture == NULL) return false;

    if (!SteeringController_Init(
            &fixture->steeringController, &fixture->steeringServo, &steeringCalibration))
    {
        return false;
    }

    return true;
}

bool RobotTestFixture_InitMotionController(RobotTestFixture *fixture,
                                           const MotionControllerConfig *config)
{
    if (fixture == NULL || config == NULL) return false;

    if (!RobotTestFixture_InitIMU(fixture)) return false;

    if (!RobotTestFixture_InitRearWheels(fixture)) return false;

    if (!RobotTestFixture_InitFrontWheels(fixture)) return false;

    if (!RobotTestFixture_InitSteeringController(fixture)) return false;

    if (!RobotTestFixture_InitWheelControllers(fixture)) return false;

    if (MotionController_Init(&fixture->motionController,
                              &fixture->leftWheelController,
                              &fixture->rightWheelController,
                              &fixture->steeringController,
                              &fixture->imu,
                              config) != MOTIONCONTROLLER_STATUS_OK)
        return false;

#if CONFIGURE_DYNAMIC_BRAKES == 1
    WheelSpeedController_ConfigureBrake(&fixture->leftWheelController, &wheelSpeedBrakeConfig);
    WheelSpeedController_ConfigureBrake(&fixture->rightWheelController, &wheelSpeedBrakeConfig);
#endif

    return true;
}
