#include "Task2ObstacleSensors.h"
#include "RobotPeripherals.h"
#include "SideIRSensorConfig.h"
#include "tim.h"
static GP2Y0A21YK leftIR, rightIR;
static bool pending, started;
static uint32_t lastTrigger;
static Task2RoutePose triggerPose;

bool Task2ObstacleSensors_Init(const Task2ObstacleSensorConfig *c)
{
    pending = started = false;
    HCSR04_Init(&hcsr04, USS_TRIG_GPIO_Port, USS_TRIG_Pin, &htim12, TIM_CHANNEL_2);
    return !c->sideIREnabled || (GP2Y0A21YK_Init(&leftIR, &sideIRLeftConfig) == GP2Y0A21YK_OK &&
        GP2Y0A21YK_Init(&rightIR, &sideIRRightConfig) == GP2Y0A21YK_OK);
}
uint32_t Task2ObstacleSensors_Poll(const Task2ObstacleSensorConfig *c, uint32_t now,
    const Task2RoutePose *pose, Task2ObstacleObservation out[TASK2_SENSOR_COUNT])
{
    uint32_t mask = 0U;
    HCSR04_Update(&hcsr04);
    if (pending && !HCSR04_IsBusy(&hcsr04))
    {
        out[0] = (Task2ObstacleObservation){.valid = HCSR04_HasMeasurement(&hcsr04),
            .value = HCSR04_HasMeasurement(&hcsr04) ? HCSR04_GetDistanceMm(&hcsr04) : 0.0f,
            .fault = HCSR04_GetState(&hcsr04) != HCSR04_STATE_READY &&
                     HCSR04_GetState(&hcsr04) != HCSR04_STATE_TIMEOUT,
            .pose = triggerPose};
        pending = false;
        mask |= 1U;
    }
    if ((!started || now - lastTrigger >= c->samplePeriodMs) && !pending)
    {
        started = true;
        lastTrigger = now;
        triggerPose = *pose;
        pending = HCSR04_Trigger(&hcsr04);
        if (!pending)
        {
            out[0] = (Task2ObstacleObservation){.fault = true, .pose = *pose};
            mask |= 1U;
        }
        if (c->sideIREnabled)
        {
            GP2Y0A21YK_Measurement measurement;
            bool ok = GP2Y0A21YK_Read(&leftIR, &measurement) == GP2Y0A21YK_OK;
            out[1] = (Task2ObstacleObservation){.valid = ok, .fault = !ok,
                .value = ok ? measurement.voltageV : 0.0f, .pose = *pose};
            ok = GP2Y0A21YK_Read(&rightIR, &measurement) == GP2Y0A21YK_OK;
            out[2] = (Task2ObstacleObservation){.valid = ok, .fault = !ok,
                .value = ok ? measurement.voltageV : 0.0f, .pose = *pose};
            mask |= 6U;
        }
    }
    return mask;
}
void Task2ObstacleSensors_Stop(void)
{
    if (pending)
        HAL_TIM_IC_Stop_IT(hcsr04.htim, hcsr04.channel);
    pending = false;
    hcsr04.state = HCSR04_STATE_IDLE;
}
