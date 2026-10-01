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
#include "CommandMotion.h"
#include "CommandProgress.h"
#include "MotionWatchdog.h"
#include "MotionDiagnostics.h"
#include "StartButton.h"
#include "SensorTask.h"
#include "UltrasonicApproach.h"
#include "userbutton.h"
#include "OLEDManager.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define MOTORBPWMSRC	htim9
#define MOTORCPWMSRC	htim1
#define MOTORBENC		htim3
#define MOTORCENC		htim4
#define SERVOPWMSRC		htim8
#define SERVOPWMCH		TIM_CHANNEL_1

#define MOTIONTASK_CONTROL_PERIOD_MS	10U
#define MOTIONTASK_CONTROL_PERIOD_S		0.010f

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
/* MotionController retains this configuration pointer for its lifetime. */
static MotionControllerConfig runtimeMotionConfig;

static OLED_Handle_t cmdStatus;

/* NULL means success; a precise reason remains readable through STATUS. */
static const char *MotionTask_InitControllers(void) {
	if (!ICM20948_Init(&imu, &hi2c2))
		return "INIT_IMU";

	if (!DCMotor_Init(&leftRearWheel, &MOTORBPWMSRC, false, &MOTORBENC) ||
		!DCMotor_Init(&rightRearWheel, &MOTORCPWMSRC, true, &MOTORCENC))
		return "INIT_MOTOR";

	if (!DCMotor_Enable(&leftRearWheel) || !DCMotor_Enable(&rightRearWheel))
		return "INIT_MOTOR_ENABLE";

	if (!Servo_Init(&steeringServo, &SERVOPWMSRC, SERVOPWMCH,
			CHASSIS_STEER_MIN_PULSE_US,
			CHASSIS_STEER_CTR_PULSE_US,
			CHASSIS_STEER_MAX_PULSE_US))
		return "INIT_SERVO";

	if (!Servo_Enable(&steeringServo))
		return "INIT_SERVO_ENABLE";

	if (!SteeringController_Init(&steeringController,
			&steeringServo, &steeringCalibration))
		return "INIT_STEERING";

	if (!WheelSpeedController_Init(&leftWheelController, &leftRearWheel,
			WHEELSPEEDCONTROLLER_KP, WHEELSPEEDCONTROLLER_KI,
			WHEELSPEEDCONTROLLER_MIN_FEEDBACK,
			WHEELSPEEDCONTROLLER_MAX_FEEDBACK,
			&leftCalibration))
		return "INIT_LEFT_SPEED";

	if (!WheelSpeedController_Init(&rightWheelController, &rightRearWheel,
			WHEELSPEEDCONTROLLER_KP, WHEELSPEEDCONTROLLER_KI,
			WHEELSPEEDCONTROLLER_MIN_FEEDBACK,
			WHEELSPEEDCONTROLLER_MAX_FEEDBACK,
			&rightCalibration))
		return "INIT_RIGHT_SPEED";

	/*
	 * Inherit the shared geometry and arc calibration, but preserve the
	 * UART runtime's pre-merge straight-driving tuning. The calibration
	 * tests continue to use motionControllerConfig directly.
	 */
	runtimeMotionConfig = motionControllerConfig;
	runtimeMotionConfig.headingKp = 0.80f;
	runtimeMotionConfig.headingKi = 0.0f;
	runtimeMotionConfig.headingKd = 0.0f;
	runtimeMotionConfig.maxHeadingSteeringAngleRad = 0.010f;
	if (MotionController_Init(&motionController,
			&leftWheelController, &rightWheelController,
			&steeringController, &imu,
			&runtimeMotionConfig) != MOTIONCONTROLLER_STATUS_OK)
		return "INIT_MOTION";

	WheelSpeedController_ConfigureBrake(
			&leftWheelController, &wheelSpeedBrakeConfig);
	WheelSpeedController_ConfigureBrake(
			&rightWheelController, &wheelSpeedBrakeConfig);

	return NULL;
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
static MotionWatchdog watchdog;
static UltrasonicApproach ultrasonicApproach;
static bool ultrasonicCommand;
static CommandProgress commandProgress;
static bool progressTracking;
static char progressReply[COMMANDPROGRESS_REPLY_SIZE];

/* Copy only for a Q P request and once at the final stationary position.
 * No formatting, UART traffic or history maintenance in the control loop.
 */
static void MotionTask_SampleProgress(void) {
    if (!progressTracking) return;
    CommandProgressPhase phase = MotionController_IsBusy(&motionController)
        ? (motionController.mode == MOTIONCONTROLLER_BRAKING
            ? COMMANDPROGRESS_BRAKING : COMMANDPROGRESS_ACTIVE)
        : COMMANDPROGRESS_IDLE;
    CommandProgress_Sample(&commandProgress, motionController.travelledDistanceMm,
                           motionController.yawDeg, phase);
    if (phase == COMMANDPROGRESS_IDLE) progressTracking = false;
}

static MotionWatchPhase MotionTask_WatchPhase(void) {
    if (motionController.mode == MOTIONCONTROLLER_ARC_PREPARING ||
        motionController.mode == MOTIONCONTROLLER_STRAIGHT_PREPARING)
        return WATCH_PREPARE;
    if (motionController.mode == MOTIONCONTROLLER_BRAKING)
        return WATCH_BRAKE;
    return WATCH_MOVE;
}

#if MOTION_DIAGNOSTICS
static MotionDiagnostics diagnostics;
static float diagnosticLastDtMs, diagnosticMaxDtMs;
static char diagnosticReply[MOTION_DIAGNOSTICS_REPLY_SIZE];

static void MotionTask_Capture(const char *event) {
    if (session.next >= session.count) return;
    const Command *cmd = &session.commands[session.next];
    static const char letters[] = "FBLRSU";
    static const char *const modes[] = { "IDLE", "STRAIGHT", "ARC", "BRAKING", "PREPARE", "STRAIGHT_PREPARE" };
    diagnostics = (MotionDiagnostics){
        .valid = true, .numbered = session.hasSeq, .seq = session.seq,
        .step = (unsigned)session.next + 1U, .command = letters[cmd->type],
        .parameter = cmd->param, .event = event,
        .mode = (unsigned)motionController.mode < sizeof(modes)/sizeof(modes[0])
            ? modes[motionController.mode] : "UNKNOWN",
        .targetMm = motionController.targetDistanceMm,
        .travelledMm = motionController.travelledDistanceMm,
        .leftMm = motionController.leftTravelMm,
        .rightMm = motionController.rightTravelMm,
        .leftCps = leftWheelController.measuredSpeedCps,
        .rightCps = rightWheelController.measuredSpeedCps,
        .steeringCommand = steeringController.command,
        .yawDeg = motionController.yawDeg,
        .profileSpeedMmps = motionController.motionProfile.targetSpeedMmps,
        .leftTargetCps = leftWheelController.targetSpeedCps,
        .rightTargetCps = rightWheelController.targetSpeedCps,
        .leftPWM = leftWheelController.outputPWM,
        .rightPWM = rightWheelController.outputPWM,
        .leftBrakePWM = leftWheelController.brakePWM,
        .rightBrakePWM = rightWheelController.brakePWM,
        .leftActuator = (unsigned)leftWheelController.actuatorMode,
        .rightActuator = (unsigned)rightWheelController.actuatorMode,
        .leftMotorMode = (unsigned)leftRearWheel.state.mode,
        .rightMotorMode = (unsigned)rightRearWheel.state.mode,
        .leftMotorDuty = leftRearWheel.state.activeDutyCycle,
        .rightMotorDuty = rightRearWheel.state.activeDutyCycle,
        .lastDtMs = diagnosticLastDtMs,
        .maxDtMs = diagnosticMaxDtMs,
        .leftArr = MOTORBPWMSRC.Instance->ARR,
        .leftCcr1 = MOTORBPWMSRC.Instance->CCR1,
        .leftCcr2 = MOTORBPWMSRC.Instance->CCR2,
        .leftCr1 = MOTORBPWMSRC.Instance->CR1,
        .leftCcer = MOTORBPWMSRC.Instance->CCER,
        .rightArr = MOTORCPWMSRC.Instance->ARR,
        .rightCcr1 = MOTORCPWMSRC.Instance->CCR1,
        .rightCcr2 = MOTORCPWMSRC.Instance->CCR2,
        .rightCr1 = MOTORCPWMSRC.Instance->CR1,
        .rightCcer = MOTORCPWMSRC.Instance->CCER,
        .rightBdtr = MOTORCPWMSRC.Instance->BDTR,
    };
}
#else
#define MotionTask_Capture(event) ((void)0)
#endif

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

static const char *MotionTask_Start(const Command *cmd) {
    ultrasonicCommand = false;
    progressTracking = false;
    CommandProgress_Attempt(&commandProgress, (unsigned)session.next + 1U, cmd);
    CommandMotion motion;
    if (!CommandMotion_Resolve(cmd, &motion))
        return "REJECTED";
    /* Called only while idle. Configure the existing profile through its API;
     * the shared controller/calibration files remain unchanged. Reapply on
     * every command so an arc following a straight restores shared tolerance.
     * Profile completion still enters the controller's normal braking state.
     */
    float completionToleranceMm = motion.radiusMm == 0.0f
        ? COMMANDMOTION_STRAIGHT_COMPLETION_TOLERANCE_MM
        : runtimeMotionConfig.motionCompletionToleranceMm;
    if (cmd->type == COMMAND_ULTRASONIC)
        completionToleranceMm = ULTRASONIC_APPROACH_STOP_MARGIN_MM;
    if (!MotionProfile_Init(&motionController.motionProfile,
                            runtimeMotionConfig.motionAccelerationMmps2,
                            runtimeMotionConfig.motionDecelerationMmps2,
                            completionToleranceMm))
        return "REJECTED";
    float mmPerCount = 3.14159265358979323846f *
        runtimeMotionConfig.kinematics->rearWheelDiameterMm /
        runtimeMotionConfig.kinematics->rearEncoderCountsPerRev;
    if (cmd->type == COMMAND_ULTRASONIC) {
        UltrasonicReading reading = SensorTask_GetUltrasonic();
        const char *fault = UltrasonicApproach_Start(&ultrasonicApproach,
            cmd->param, &reading, HAL_GetTick());
        if (fault != NULL) return fault;
        motion.distanceMm = ultrasonicApproach.travelLimitMm;
        motion.speedCps = motion.speedMmps / mmPerCount;
        ultrasonicCommand = true;
    }
    if (!MotionWatchdog_Start(&watchdog, HAL_GetTick(), motion.distanceMm,
                             motion.speedCps * mmPerCount,
                             motion.radiusMm != 0.0f ? WATCH_PREPARE : WATCH_MOVE))
        return "REJECTED";
    static const char letters[] = "FBLRSU";
    OLED_Post(&cmdStatus, "%c %.0f", letters[cmd->type], cmd->param);
    if (motion.radiusMm != 0.0f)
        return MotionController_MoveArc(&motionController, motion.distanceMm,
                                        motion.radiusMm, motion.speedCps)
               == MOTIONCONTROLLER_STATUS_OK ? NULL : "REJECTED";
    return MotionController_MoveStraight(&motionController,
                                        motion.distanceMm, motion.speedCps)
           == MOTIONCONTROLLER_STATUS_OK ? NULL : "REJECTED";
}

static void MotionTask_Fault(const char *reason) {
    MotionTask_Capture(reason);
    ultrasonicCommand = false;
    /* Brake actively; the session stays faulted until standalone S.
     * A U range-verification fault can occur after an earlier stop: track
     * this braking phase too, without fabricating pre-start measurements.
     */
    if (commandProgress.measured) progressTracking = true;
    MotionController_Brake(&motionController);
    CommandSession_Fault(&session, reason, reply);
    CommandLink_Send(reply);
    OLED_Post(&cmdStatus, "FAULT %s", reason);
}

/* Returns false once this U command has completed or faulted. */
static bool MotionTask_UpdateUltrasonic(void) {
    UltrasonicReading reading = SensorTask_GetUltrasonic();
    float mmPerCount = 3.14159265358979323846f *
        runtimeMotionConfig.kinematics->rearWheelDiameterMm /
        runtimeMotionConfig.kinematics->rearEncoderCountsPerRev;
    float speedMmps = 0.5f * (fabsf(leftWheelController.measuredSpeedCps) +
        fabsf(rightWheelController.measuredSpeedCps)) * mmPerCount;
    UltrasonicApproachAction action = UltrasonicApproach_Update(
        &ultrasonicApproach, &reading, HAL_GetTick(),
        motionController.travelledDistanceMm, speedMmps,
        MotionController_IsBusy(&motionController),
        motionController.mode == MOTIONCONTROLLER_BRAKING);
    if (action == ULTRASONIC_APPROACH_FAULT) {
        MotionTask_Fault(ultrasonicApproach.fault);
        return false;
    } else if (action == ULTRASONIC_APPROACH_BRAKE) {
        MotionController_Brake(&motionController);
    } else if (action == ULTRASONIC_APPROACH_DRIVE) {
        /* Adapt only the endpoint, in the same encoder coordinate system as F.
         * MotionProfile already provides acceleration and distance-based
         * deceleration; do not restart it or replace the wheel controller.
         * This sole owner writes before MotionController_Update below.
         */
        motionController.motionProfile.targetDistanceMm =
            ultrasonicApproach.profileTargetMm;
    } else if (action == ULTRASONIC_APPROACH_DONE) {
        MotionTask_Capture("DONE");
        CommandSession_CommandDone(&session, reply);
        CommandLink_Send(reply);
        ultrasonicCommand = false;
        return false;
    }
    return true;
}

/* Querying never changes the motion session or consumes a batch ID. */
static bool MotionTask_SensorQuery(const char *line) {
    char sensorReply[SENSOR_REPLY_SIZE];
    if (strcmp(line, "Q U") == 0) {
        UltrasonicReading reading = SensorTask_GetUltrasonic();
        SensorReading_Format(&reading, HAL_GetTick(), sensorReply, sizeof(sensorReply));
    } else if (strcmp(line, "Q I") == 0) {
        IRPairReading reading = SensorTask_GetIR();
        SensorReading_FormatIR(&reading, HAL_GetTick(), sensorReply, sizeof(sensorReply));
    } else {
        return false;
    }
    CommandLink_Send(sensorReply);
    return true;
}

/* G is an explicit idle-only reboot, not a movement or a CPU-lockup recovery.
 * Use timer handles directly: the INIT fault loop may have only partially
 * initialized motor objects. Timers are initialized before tasks are started.
 * The blocking transport completes RESETTING before the CMSIS system reset.
 */
static void MotionTask_Reset(void) {
    __HAL_TIM_SET_COMPARE(&MOTORBPWMSRC, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&MOTORBPWMSRC, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&MOTORCPWMSRC, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&MOTORCPWMSRC, TIM_CHANNEL_2, 0U);
    (void)HAL_TIM_PWM_Stop(&MOTORBPWMSRC, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&MOTORBPWMSRC, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&MOTORCPWMSRC, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&MOTORCPWMSRC, TIM_CHANNEL_2);
    (void)CommandLink_Send("RESETTING\n");
    NVIC_SystemReset();
}

void MotionTask(void *argument) {
    (void)argument;
    OLED_Register(&cmdStatus, "CMD");
    CommandSession_Init(&session);

    const char *initError = MotionTask_InitControllers();
    if (initError != NULL) {
        OLED_Post(&cmdStatus, "%s", initError);
        snprintf(reply, sizeof(reply), "FAULT - %s\n", initError);
        CommandLink_Send(reply);
        /* Keep STATUS usable, but never accept motion without hardware. */
        for (;;) {
            char *line;
            if (MotionTask_ReadLine(&line)) {
                if (strcmp(line, "G") == 0)
                    MotionTask_Reset();
                else if (!MotionTask_SensorQuery(line))
                    CommandLink_Send(reply);
            }
            osDelay(MOTIONTASK_CONTROL_PERIOD_MS);
        }
    }

    MotionTask_CentreSteering();
    OLED_Post(&cmdStatus, "READY");
    CommandLink_Send("READY\n");

    bool commandRunning = false;
    StartButton startButton = {0};
    uint32_t nextWake = osKernelGetTickCount();
    uint32_t previousUpdate = nextWake;

    for (;;) {
        StartButton_Update(&startButton, SW1_ReadState() == SW1_Enabled,
                           HAL_GetTick());
        bool busy = MotionController_IsBusy(&motionController);
        if (!busy) MotionTask_SampleProgress();
        if (!busy && commandRunning && !ultrasonicCommand) {
            MotionTask_Capture("DONE");
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
#if MOTION_DIAGNOSTICS
            if (strcmp(line, "D") == 0 || strcmp(line, "D W") == 0 ||
                strcmp(line, "D H") == 0) {
                if (commandRunning) MotionTask_Capture("LIVE");
                if (strcmp(line, "D H") == 0)
                    MotionDiagnostics_FormatHardware(&diagnostics, diagnosticReply,
                                                      sizeof(diagnosticReply));
                else if (strcmp(line, "D W") == 0)
                    MotionDiagnostics_FormatWheels(&diagnostics, diagnosticReply,
                                                   sizeof(diagnosticReply));
                else
                    MotionDiagnostics_Format(&diagnostics, diagnosticReply,
                                             sizeof(diagnosticReply));
                CommandLink_Send(diagnosticReply);
                reply[0] = '\0';
            } else
#endif
            if (strcmp(line, "G") == 0) {
                if (CommandSession_CanReset(&session,
                        MotionController_IsBusy(&motionController)))
                    MotionTask_Reset();
                else
                    (void)snprintf(reply, sizeof(reply), "NAK - BUSY\n");
            } else if (strcmp(line, "Q P") == 0) {
                MotionTask_SampleProgress();
                CommandProgress_Format(&session, &commandProgress,
                                       progressReply, sizeof(progressReply));
                CommandLink_Send(progressReply);
                reply[0] = '\0';
            } else if (MotionTask_SensorQuery(line)) {
                reply[0] = '\0';
            } else if (strcmp(line, "BUTTON") == 0) {
                snprintf(reply, sizeof(reply), "BUTTON %lu %u\n",
                         (unsigned long)startButton.count,
                         startButton.rawPressed || startButton.stablePressed
                             ? 1U : 0U);
            } else {
                if (strcmp(line, "S") == 0 && commandRunning)
                    MotionTask_Capture("STOP");
                if (CommandSession_Receive(&session, line, reply)) {
                    if (commandProgress.measured) progressTracking = true;
                    MotionController_Brake(&motionController);
                    commandRunning = false;
                    ultrasonicCommand = false;
                }
            }
            CommandLink_Send(reply);
        }

        /* One command at a time, all from the one accepted batch. */
        if (!commandRunning && !MotionController_IsBusy(&motionController)) {
            const Command *cmd = CommandSession_Current(&session);
            if (cmd != NULL) {
                const char *startFault = MotionTask_Start(cmd);
                if (startFault == NULL) {
                    commandRunning = true;
                    progressTracking = true;
                    MotionTask_SampleProgress();
#if MOTION_DIAGNOSTICS
                    diagnosticLastDtMs = diagnosticMaxDtMs = 0.0f;
#endif
                    MotionTask_Capture("LIVE");
                } else
                    MotionTask_Fault(startFault);
            }
        }

        if (commandRunning && ultrasonicCommand)
            commandRunning = MotionTask_UpdateUltrasonic();

        uint32_t now = osKernelGetTickCount();
        uint32_t elapsedTicks = now - previousUpdate;
        float dt = elapsedTicks ? (float)elapsedTicks / osKernelGetTickFreq()
                                : MOTIONTASK_CONTROL_PERIOD_S;
        previousUpdate = now;
#if MOTION_DIAGNOSTICS
        diagnosticLastDtMs = dt * 1000.0f;
        if (commandRunning && diagnosticLastDtMs > diagnosticMaxDtMs)
            diagnosticMaxDtMs = diagnosticLastDtMs;
#endif
        if (MotionController_Update(&motionController, dt)
                != MOTIONCONTROLLER_STATUS_OK) {
            MotionTask_Fault("UPDATE");
            commandRunning = false;
        }
        if (commandRunning && MotionController_IsBusy(&motionController)) {
            const char *fault = MotionWatchdog_Check(&watchdog, HAL_GetTick(),
                MotionTask_WatchPhase(), motionController.motionDirection *
                motionController.travelledDistanceMm);
            if (fault != NULL) {
                MotionTask_Fault(fault);
                commandRunning = false;
            }
        }

        nextWake += MOTIONTASK_CONTROL_PERIOD_MS;
        /* Do not spin through missed deadlines after an I2C/TX timeout. */
        if ((int32_t)(nextWake - osKernelGetTickCount()) <= 0)
            nextWake = osKernelGetTickCount() + MOTIONTASK_CONTROL_PERIOD_MS;
        osDelayUntil(nextWake);
    }
}
