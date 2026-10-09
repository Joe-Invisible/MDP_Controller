#include "WheelSpeedController.h"
#include "WheelSpeedControllerConfig.h"
#include "WheelBrakeConfig.h"
#include "MotionControllerConfig.h"
typedef struct { float raw; } Servo;
typedef struct { DCMotor leftRearWheel, rightRearWheel; WheelSpeedController leftWheelController, rightWheelController; Servo steeringServo; } RobotTestFixture;
bool RobotTestFixture_InitRearWheels(RobotTestFixture *f);
bool RobotTestFixture_InitFrontWheels(RobotTestFixture *f);
bool RobotTestFixture_InitWheelControllers(RobotTestFixture *f);
void Servo_SetSteering(Servo *s, float value);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
