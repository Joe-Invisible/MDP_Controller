#include "MotionTask.h"

#include "cmsis_os.h"
#include "tim.h"
#include "i2c.h"

#include "rwdriver.h"
#include "fwdriver.h"
#include "icm20948.h"

#include "WheelSpeedController.h"
#include "WheelSpeedControllerConfig.h"
#include "SteeringController.h"
#include "SteeringControllerConfig.h"
#include "MotionController.h"
#include "MotionControllerConfig.h"
#include "WheelBrakeConfig.h"

#include "CommandLink.h"
#include "CommandSession.h"
#include "OLEDManager.h"

#include <string.h>

#define MOTORBPWMSRC	htim9
#define MOTORCPWMSRC	htim1
#define MOTORBENC		htim3
#define MOTORCENC		htim4
#define SERVOPWMSRC		htim8
#define SERVOPWMCH		TIM_CHANNEL_1

#define MOTIONTASK_CONTROL_PERIOD_MS	10U
#define MOTIONTASK_CONTROL_PERIOD_S		0.010f

/*
 * Gains validated on hardware by MotionControllerTest; see the
 * calibration reports in exp/.
 */
#define MOTIONTASK_HEADING_KP			0.80f
#define MOTIONTASK_HEADING_KI			0.0f
#define MOTIONTASK_HEADING_KD			0.0f
#define MOTIONTASK_HEADING_LIMIT_RAD	0.010f

#define MOTIONTASK_SYNC_KP_CPS_PER_MM	10.0f
#define MOTIONTASK_SYNC_MAX_CORR_CPS	100.0f

#define MOTIONTASK_ACCELERATION_MMPS2	500.0f
#define MOTIONTASK_DECELERATION_MMPS2	250.0f

/*
 * Cruise speed applied to every F/B command until the protocol
 * carries an explicit speed.
 */
#define MOTIONTASK_CRUISE_SPEED_CPS		5000.0f

static const WheelSpeedBrakeConfig wheelSpeedBrakeConfig = {
	.map = &rearWheelBrakeMap,
	.engageOverspeedCps = 200.0f,
	.releaseOverspeedCps = 100.0f,
	.fullDemandOverspeedCps = 800.0f,
};

/*
 * The whole control stack lives here, not on the task stack, and
 * nothing outside this file may touch it.
 */
static DCMotor leftRearWheel;
static DCMotor rightRearWheel;
static Servo steeringServo;
static ICM20948 imu;
static WheelSpeedController leftWheelController;
static WheelSpeedController rightWheelController;
static SteeringController steeringController;
static MotionController motionController;

static OLED_Handle_t cmdStatus;

static bool MotionTask_InitControllers(void) {
	if (!ICM20948_Init(&imu, &hi2c2))
		return false;

	if (!DCMotor_Init(&leftRearWheel, &MOTORBPWMSRC, false, &MOTORBENC) ||
		!DCMotor_Init(&rightRearWheel, &MOTORCPWMSRC, true, &MOTORCENC))
		return false;

	if (!DCMotor_Enable(&leftRearWheel) || !DCMotor_Enable(&rightRearWheel))
		return false;

	if (!Servo_Init(&steeringServo, &SERVOPWMSRC, SERVOPWMCH,
			CHASSIS_STEER_MIN_PULSE_US,
			CHASSIS_STEER_CTR_PULSE_US,
			CHASSIS_STEER_MAX_PULSE_US))
		return false;

	if (!Servo_Enable(&steeringServo))
		return false;

	if (!SteeringController_Init(&steeringController,
			&steeringServo, &steeringCalibration))
		return false;

	if (!WheelSpeedController_Init(&leftWheelController, &leftRearWheel,
			WHEELSPEEDCONTROLLER_KP, WHEELSPEEDCONTROLLER_KI,
			WHEELSPEEDCONTROLLER_MIN_FEEDBACK,
			WHEELSPEEDCONTROLLER_MAX_FEEDBACK,
			&leftCalibration))
		return false;

	if (!WheelSpeedController_Init(&rightWheelController, &rightRearWheel,
			WHEELSPEEDCONTROLLER_KP, WHEELSPEEDCONTROLLER_KI,
			WHEELSPEEDCONTROLLER_MIN_FEEDBACK,
			WHEELSPEEDCONTROLLER_MAX_FEEDBACK,
			&rightCalibration))
		return false;

	if (!MotionController_Init(&motionController,
			&leftWheelController, &rightWheelController,
			&steeringController, &imu, &kinematics,
			MOTIONTASK_HEADING_KP, MOTIONTASK_HEADING_KI,
			MOTIONTASK_HEADING_KD, MOTIONTASK_HEADING_LIMIT_RAD,
			MOTIONTASK_SYNC_KP_CPS_PER_MM, MOTIONTASK_SYNC_MAX_CORR_CPS,
			MOTIONTASK_ACCELERATION_MMPS2, MOTIONTASK_DECELERATION_MMPS2))
		return false;

	WheelSpeedController_ConfigureBrake(
			&leftWheelController, &wheelSpeedBrakeConfig);
	WheelSpeedController_ConfigureBrake(
			&rightWheelController, &wheelSpeedBrakeConfig);

	return true;
}

/*
 * Deterministic preconditioned centering so every boot starts from
 * the same mechanical steering state (same sequence as the motion
 * tests).
 */
static void MotionTask_CentreSteering(void) {
	SteeringController_StartCentre(&steeringController);
	while (!SteeringController_UpdateCentre(&steeringController,
			MOTIONTASK_CONTROL_PERIOD_S)) {
		osDelay(MOTIONTASK_CONTROL_PERIOD_MS);
	}
	/* Mechanical settle time. */
	osDelay(100U);
}

/* All protocol and controller state belongs to this task. */
static CommandSession session;
static char reply[COMMANDSESSION_REPLY_SIZE];

#define MOTIONTASK_MAX_FRAME_LEN 64U
#define MOTIONTASK_RX_BYTES_PER_TICK 64U

/*
 * Bounded polling: at most one line and 64 bytes per control tick.
 * The Pi sends one batch, then waits for DONE; this is not a streaming
 * command queue. STATUS and S are read even while a movement is busy.
 */
static bool MotionTask_ReadLine(char **line) {
    static char buffer[MOTIONTASK_MAX_FRAME_LEN + 1U];
    static size_t length;
    static bool discarding;
    static uint32_t drops;

    for (unsigned i = 0; i < MOTIONTASK_RX_BYTES_PER_TICK; ++i) {
        uint8_t byte;
        if (CommandLink_ReadBytes(&byte, 1, 0) != 1)
            break;
        uint32_t currentDrops = CommandLink_GetDropCount();
        if (currentDrops != drops) {
            drops = currentDrops;
            discarding = true;
        }
        if (byte == '\n') {
            if (discarding) {
                CommandLink_Send("NAK - FRAME\n");
            } else {
                buffer[length] = '\0';
                *line = buffer;
            }
            bool complete = !discarding;
            length = 0;
            discarding = false;
            return complete;
        }
        if (byte == '\r')
            continue;
        if (discarding)
            continue;
        if (byte == 0 || length == MOTIONTASK_MAX_FRAME_LEN) {
            discarding = true;
            continue;
        }
        buffer[length++] = (char)byte;
    }
    return false;
}

static bool MotionTask_Start(const Command *cmd) {
    float distance = cmd->type == COMMAND_BACKWARD ? -cmd->param : cmd->param;
    OLED_Post(&cmdStatus, "%c %.0f",
              cmd->type == COMMAND_BACKWARD ? 'B' : 'F', cmd->param);
    return MotionController_MoveStraight(&motionController,
                                        distance, MOTIONTASK_CRUISE_SPEED_CPS);
}

static void MotionTask_Fault(const char *reason) {
    /* Brake actively; the session stays faulted until standalone S. */
    MotionController_Brake(&motionController);
    CommandSession_Fault(&session, reason, reply);
    CommandLink_Send(reply);
    OLED_Post(&cmdStatus, "FAULT %s", reason);
}

void MotionTask(void *argument) {
    (void)argument;
    OLED_Register(&cmdStatus, "CMD");
    CommandSession_Init(&session);

    if (!MotionTask_InitControllers()) {
        OLED_Post(&cmdStatus, "INIT FAIL");
        CommandLink_Send("FAULT - INIT\n");
        /* Keep STATUS usable, but never accept motion without hardware. */
        for (;;) {
            char *line;
            if (MotionTask_ReadLine(&line))
                CommandLink_Send("FAULT - INIT\n");
            osDelay(MOTIONTASK_CONTROL_PERIOD_MS);
        }
    }

    MotionTask_CentreSteering();
    OLED_Post(&cmdStatus, "READY");
    CommandLink_Send("READY\n");

    bool commandRunning = false;
    uint32_t nextWake = osKernelGetTickCount();
    uint32_t previousUpdate = nextWake;

    for (;;) {
        bool busy = MotionController_IsBusy(&motionController);
        if (!busy && commandRunning) {
            CommandSession_CommandDone(&session, reply);
            CommandLink_Send(reply);
            commandRunning = false;
        }
        if (!busy) {
            CommandSession_Stopped(&session, reply);
            CommandLink_Send(reply);
        }

        char *line;
        if (MotionTask_ReadLine(&line)) {
            if (CommandSession_Receive(&session, line, reply)) {
                MotionController_Brake(&motionController);
                commandRunning = false;
            }
            CommandLink_Send(reply);
        }

        /* One command at a time, all from the one accepted batch. */
        if (!commandRunning && !MotionController_IsBusy(&motionController)) {
            const Command *cmd = CommandSession_Current(&session);
            if (cmd != NULL) {
                if (MotionTask_Start(cmd))
                    commandRunning = true;
                else
                    MotionTask_Fault("REJECTED");
            }
        }

        uint32_t now = osKernelGetTickCount();
        uint32_t elapsedTicks = now - previousUpdate;
        float dt = elapsedTicks ? (float)elapsedTicks / osKernelGetTickFreq()
                                : MOTIONTASK_CONTROL_PERIOD_S;
        previousUpdate = now;
        if (!MotionController_Update(&motionController, dt)) {
            MotionTask_Fault("UPDATE");
            commandRunning = false;
        }

        nextWake += MOTIONTASK_CONTROL_PERIOD_MS;
        /* Do not spin through missed deadlines after an I2C/TX timeout. */
        if ((int32_t)(nextWake - osKernelGetTickCount()) <= 0)
            nextWake = osKernelGetTickCount() + MOTIONTASK_CONTROL_PERIOD_MS;
        osDelayUntil(nextWake);
    }
}
