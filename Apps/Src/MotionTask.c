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
#include "Task2Run.h"
#include "Task2Wire.h"
#include "Task2PlanFrames.h"

#ifndef TASK2_PLAN_HOST
#define TASK2_PLAN_HOST 1   /* Plan Task 2 on a host when one answers; the STM plans itself otherwise. */
#endif
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

static MotionControllerConfig commandMotionConfig, task2MotionConfig;
static MotionControllerArcConfig task2ArcConfig;
static MotionControllerArcFeedforwardPoint task2NegativePoints[9];
static const MotionControllerArcFeedforwardPoint task2PositivePoints[] = {
    {0.0000005f, -0.05208333f}, {0.000054f, -5.625f}, {0.000183f, -11.25f}, {0.000436f, -16.875f},
    {1.0f / 1500.0f, -22.5f}, {1.0f / 1000.0f, -27.7f},
    {1.0f / 500.0f, -47.5f}, {1.0f / 300.0f, -75.0f}, {1.0f / 275.0f, -81.7f},
};
static bool MotionTask_ConfigureTask2(void) {
    commandMotionConfig = runtimeMotionConfig;
    task2MotionConfig = runtimeMotionConfig;
    task2MotionConfig.straightYawRateKp = task2MotionConfig.arcYawRateKp = 200.0f;
    task2MotionConfig.straightYawRateKi = task2MotionConfig.arcYawRateKi = 100.0f;
    task2MotionConfig.straightYawRateKd = task2MotionConfig.arcYawRateKd = 0.0f;
    task2MotionConfig.straightHeadingKpPerSec = task2MotionConfig.arcHeadingKpPerSec = 0.2f;
    task2MotionConfig.straightAccelerationMmps2 = task2MotionConfig.straightDecelerationMmps2 = 3000.0f;
    task2MotionConfig.arcAccelerationMmps2 = task2MotionConfig.arcDecelerationMmps2 = 2000.0f;
    task2MotionConfig.useMatchedYawRateReferenceFilter = false;
    task2ArcConfig = arcMotionConfig;
    task2ArcConfig.positivePoints = task2PositivePoints;
    task2ArcConfig.positivePointCount = sizeof(task2PositivePoints) / sizeof(task2PositivePoints[0]);
    uint32_t count = arcMotionConfig.negativePointCount;
    if (!count || count >= sizeof(task2NegativePoints) / sizeof(task2NegativePoints[0])) return false;
    for (uint32_t i = 0; i < count; ++i) task2NegativePoints[i] = arcMotionConfig.negativePoints[i];
    const MotionControllerArcFeedforwardPoint *near = &task2NegativePoints[count - 1U];
    task2NegativePoints[count] = (MotionControllerArcFeedforwardPoint){-0.0000005f,
        near->rawSteeringCommand * -0.0000005f / near->curvaturePerMm};
    task2ArcConfig.negativePoints = task2NegativePoints;
    task2ArcConfig.negativePointCount = count + 1U;
    task2MotionConfig.arcConfig = &task2ArcConfig;
    return true;
}

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
static Task2Run task2;
_Static_assert(TASK2COMMANDS_REPLY_SIZE == COMMANDSESSION_REPLY_SIZE,
               "Task 2 replies share the session reply buffer");
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
    static const char *const modes[] = { "IDLE", "STRAIGHT", "ARC", "BRAKING", "PREPARE", "STRAIGHT_PREPARE" };
    diagnostics = (MotionDiagnostics){
        .valid = true, .numbered = session.hasSeq, .seq = session.seq,
        .step = (unsigned)session.next + 1U, .command = CommandParser_CommandName(cmd->type),
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
/* Set by the last MotionTask_ReadFrame: it stopped because no byte was waiting, not on its budget. */
static bool readFrameIdle;

static bool MotionTask_ReadFrame(char **line) {
    static char buffer[MOTIONTASK_MAX_FRAME_LEN + 1U];
    static size_t length;
    static bool discarding;
    static uint32_t drops;

    readFrameIdle = false;
    for (unsigned i = 0; i < MOTIONTASK_RX_BYTES_PER_TICK; ++i) {
        uint8_t byte;
        if (CommandLink_ReadBytes(&byte, 1, 0) != 1) {
            readFrameIdle = true;
            break;
        }
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

/* Lines that arrived while a Task 2 plan was awaited, handed out before new input. A stop goes first. */
#define MOTIONTASK_DEFERRED_LINES 6U
static char deferredLines[MOTIONTASK_DEFERRED_LINES][MOTIONTASK_MAX_FRAME_LEN + 1U];
static unsigned deferredHead, deferredCount;
static uint32_t deferredDropped;

static void MotionTask_Defer(const char *line, bool first) {
    if (deferredCount == MOTIONTASK_DEFERRED_LINES) {
        if (!first) { ++deferredDropped; return; }
        --deferredCount;                    /* a stop displaces the newest queued line */
        ++deferredDropped;
    }
    unsigned slot = first ? (deferredHead + MOTIONTASK_DEFERRED_LINES - 1U) % MOTIONTASK_DEFERRED_LINES
                          : (deferredHead + deferredCount) % MOTIONTASK_DEFERRED_LINES;
    if (first) deferredHead = slot;
    (void)snprintf(deferredLines[slot], sizeof(deferredLines[slot]), "%s", line);
    ++deferredCount;
}

static bool MotionTask_ReadLine(char **line) {
    static char held[MOTIONTASK_MAX_FRAME_LEN + 1U];
    if (deferredCount) {
        memcpy(held, deferredLines[deferredHead], sizeof(held));
        deferredHead = (deferredHead + 1U) % MOTIONTASK_DEFERRED_LINES;
        --deferredCount;
        *line = held;
        return true;
    }
    return MotionTask_ReadFrame(line);
}

#if TASK2_PLAN_HOST
/*
 * Task 2 planning on a host, through the Pi. The request goes out as one line, "PQ <gen> <hex>"; the
 * reply comes back in frames (Task2PlanFrames). The robot is stationary at every planning point.
 * While waiting, a stop ends the wait at once and every other line is kept, in order, for the main
 * loop: status queries and arrows are answered late by up to the timeout, not lost unless more than
 * MOTIONTASK_DEFERRED_LINES arrive (counted in deferredDropped). A stop is also looked for between
 * request chunks and once more before any answer is returned, so a malformed or missing reply never
 * hides a stop that has already arrived.
 */
#define TASK2_PLAN_TIMEOUT_MS 1500U
static char planHex[2U * TASK2_WIRE_MAX_BYTES + 1U];
static Task2PlanFrames planFrames;

/* Everything already received, up to a bound that covers the whole receive buffer: true if it held a
 * stop (kept first for the main loop). A frame that ends a read budget, or is discarded as too long,
 * does not end the search; only an empty UART does. */
/* CommandLink buffers 256 bytes: at most 256 one-byte (empty) lines; a long line takes 64 bytes a read. */
#define TASK2_PLAN_DRAIN_READS 264U
static bool MotionTask_Task2PlanStop(void) {
    char *line;
    bool stop = false;
    for (unsigned reads = 0U; reads < TASK2_PLAN_DRAIN_READS; ++reads) {
        if (!MotionTask_ReadFrame(&line)) {
            if (readFrameIdle) break;
            continue;
        }
        Task2PlanFrame kind = Task2PlanFrames_Feed(&planFrames, line);
        if (kind == TASK2_PLAN_FRAME_STOP) stop = true;
        else if (kind == TASK2_PLAN_FRAME_OTHER) MotionTask_Defer(line, false);
    }
    if (stop) MotionTask_Defer("S", true);
    return stop;
}

static size_t MotionTask_Task2Exchange(uint16_t generation, const uint8_t *request, size_t length,
                                       uint8_t *reply, size_t capacity) {
    size_t hex = Task2Wire_ToHex(request, length, planHex, sizeof(planHex));
    if (!hex) return 0U;
    Task2PlanFrames_Begin(&planFrames, generation);
    char head[16];
    (void)snprintf(head, sizeof(head), "PQ %u ", (unsigned)generation);
    CommandLink_Send(head);
    for (size_t at = 0U; at < hex; at += 160U) {     /* each send within CommandLink's 20 ms */
        char chunk[161];
        size_t k = hex - at < 160U ? hex - at : 160U;
        memcpy(chunk, planHex + at, k);
        chunk[k] = '\0';
        CommandLink_Send(chunk);
        if (MotionTask_Task2PlanStop()) {
            CommandLink_Send("\n");                 /* the cut request is never answered usefully */
            return TASK2_PLAN_CANCELLED;
        }
    }
    CommandLink_Send("\n");
    uint32_t started = HAL_GetTick();
    while (HAL_GetTick() - started < TASK2_PLAN_TIMEOUT_MS) {
        char *line;
        while (MotionTask_ReadFrame(&line)) {
            switch (Task2PlanFrames_Feed(&planFrames, line)) {
            case TASK2_PLAN_FRAME_STOP:
                MotionTask_Defer(line, true);
                return TASK2_PLAN_CANCELLED;
            case TASK2_PLAN_FRAME_OTHER:
                MotionTask_Defer(line, false);
                break;
            case TASK2_PLAN_FRAME_DONE: {
                size_t total = Task2PlanFrames_Length(&planFrames);
                if (total > capacity) return 0U;
                memcpy(reply, planFrames.data, total);
                return total;
            }
            case TASK2_PLAN_FRAME_NONE:
            case TASK2_PLAN_FRAME_BAD:
                return 0U;
            default:
                break;
            }
        }
        osDelay(1);
    }
    return 0U;
}

static size_t MotionTask_Task2Plan(void *context, uint16_t generation, const uint8_t *request, size_t length,
                                   uint8_t *reply, size_t capacity) {
    (void)context;
    size_t got = MotionTask_Task2Exchange(generation, request, length, reply, capacity);
    if (got == TASK2_PLAN_CANCELLED) return got;
    return MotionTask_Task2PlanStop() ? TASK2_PLAN_CANCELLED : got;
}
#endif

/* One Task 2 primitive, guarded by the movement watchdog like any batch command. */
/* A Task 2 move was launched in this pass, possibly after a long planning wait. */
static bool task2Launched;

static const char *MotionTask_Task2Move(float distanceMm, float radiusMm, float speedCps) {
    if (MotionController_IsBusy(&motionController))
        return "T2_BUSY";
    float completionToleranceMm = radiusMm == 0.0f
        ? COMMANDMOTION_STRAIGHT_COMPLETION_TOLERANCE_MM
        : runtimeMotionConfig.motionCompletionToleranceMm;
    if (!MotionProfile_Init(&motionController.motionProfile,
                            (radiusMm == 0.0f ? runtimeMotionConfig.straightAccelerationMmps2 : runtimeMotionConfig.arcAccelerationMmps2),
                            (radiusMm == 0.0f ? runtimeMotionConfig.straightDecelerationMmps2 : runtimeMotionConfig.arcDecelerationMmps2),
                            completionToleranceMm))
        return "REJECTED";
    float mmPerCount = 3.14159265358979323846f *
        runtimeMotionConfig.kinematics->rearWheelDiameterMm /
        runtimeMotionConfig.kinematics->rearEncoderCountsPerRev;
    if (!MotionWatchdog_Start(&watchdog, HAL_GetTick(), distanceMm,
                             fabsf(speedCps) * mmPerCount,
                             radiusMm != 0.0f ? WATCH_PREPARE : WATCH_MOVE))
        return "REJECTED";
    MotionControllerStatus status = radiusMm != 0.0f
        ? MotionController_MoveArc(&motionController, distanceMm, radiusMm, speedCps)
        : MotionController_MoveStraight(&motionController, distanceMm, speedCps);
    task2Launched |= status == MOTIONCONTROLLER_STATUS_OK;
    return status == MOTIONCONTROLLER_STATUS_OK ? NULL : "REJECTED";
}

static const char *MotionTask_Task2Straight(void *context, float distanceMm, float speedCps) {
    (void)context;
    return MotionTask_Task2Move(distanceMm, 0.0f, speedCps);
}

static const char *MotionTask_Task2Arc(void *context, float distanceMm, float radiusMm,
                                       float speedCps) {
    (void)context;
    return radiusMm != 0.0f ? MotionTask_Task2Move(distanceMm, radiusMm, speedCps) : "REJECTED";
}

static bool MotionTask_Task2Busy(void *context) {
    (void)context;
    return MotionController_IsBusy(&motionController);
}

static void MotionTask_Task2Brake(void *context) {
    (void)context;
    MotionController_Brake(&motionController);
}

static const char *MotionTask_Task2Speed(void *context, float speedCps) {
    (void)context;
    return MotionController_RequestCruiseSpeed(&motionController, speedCps)
        == MOTIONCONTROLLER_STATUS_OK ? NULL : "T2_SPEED";
}
static const char *MotionTask_Task2Continuous(void *context, float speedCps) {
    (void)context;
    if (MotionController_IsBusy(&motionController)) return "T2_BUSY";
    /* Time/progress watchdog, independent of inferred width. No motion endpoint. */
    if (!MotionWatchdog_Start(&watchdog, HAL_GetTick(), 1.0f, 1.0f, WATCH_PREPARE)) return "REJECTED";
    watchdog.limitMs = 30000U;
    MotionControllerStatus status = MotionController_MoveStraightContinuous(&motionController, speedCps);
    task2Launched |= status == MOTIONCONTROLLER_STATUS_OK;
    return status == MOTIONCONTROLLER_STATUS_OK ? NULL : "REJECTED";
}
static Task2MotionReading MotionTask_Task2Motion(void *context) {
    (void)context;
    return (Task2MotionReading){
        .distanceMm = motionController.travelledDistanceMm,
        .yawRad = motionController.yawDeg * 3.14159265358979323846f / 180.0f,
        .moving = MotionController_IsBusy(&motionController),
        .braking = motionController.mode == MOTIONCONTROLLER_BRAKING,
    };
}
static UltrasonicReading MotionTask_Task2USS(void *context) {
    (void)context; return SensorTask_GetUltrasonic();
}
static IRPairReading MotionTask_Task2IR(void *context) {
    (void)context; return SensorTask_GetIR();
}

static Task2CourseIo task2CourseIo = {
    .moveStraight = MotionTask_Task2Straight,
    .moveArc = MotionTask_Task2Arc,
    .busy = MotionTask_Task2Busy,
    .brake = MotionTask_Task2Brake,
    .requestSpeed = MotionTask_Task2Speed,
    .moveContinuous = MotionTask_Task2Continuous,
    .motion = MotionTask_Task2Motion,
    .ultrasonic = MotionTask_Task2USS,
    .infrared = MotionTask_Task2IR,
};

#if TASK2_DIAGNOSTICS
static char task2DiagLines[TASK2_DIAG_PARTS][TASK2_DIAG_LINE_SIZE];

/* Microseconds from the core cycle counter for Task 2 planning time. Differences within one planning
 * call are exact; the count itself is only a running total. */
static uint32_t MotionTask_Task2ElapsedUs(void *context) {
    static uint32_t lastCycles, us, spare;
    (void)context;
    uint32_t now = DWT->CYCCNT, perUs = SystemCoreClock / 1000000U;
    uint32_t cycles = now - lastCycles + spare;
    lastCycles = now;
    us += cycles / perUs;
    spare = cycles % perUs;
    return us;
}
#endif

static void MotionTask_SendTask2Events(void) {
    char event[TASK2COMMANDS_REPLY_SIZE];
    while (Task2Run_NextEvent(&task2, event))
        CommandLink_Send(event);
}

static const char *MotionTask_Start(const Command *cmd) {
    ultrasonicCommand = false;
    progressTracking = false;
    CommandProgress_Attempt(&commandProgress, (unsigned)session.next + 1U, cmd);
    runtimeMotionConfig = cmd->type == COMMAND_TASK2 ? task2MotionConfig : commandMotionConfig;
    /* Switch at a stopped batch boundary, retaining command-mode wheel tuning. */
    leftWheelController.pid.kp = rightWheelController.pid.kp =
        cmd->type == COMMAND_TASK2 ? 0.03f : WHEELSPEEDCONTROLLER_KP;
    leftWheelController.pid.ki = rightWheelController.pid.ki =
        cmd->type == COMMAND_TASK2 ? 0.0f : WHEELSPEEDCONTROLLER_KI;
    leftWheelController.pid.kd = rightWheelController.pid.kd = 0.0f;
    if (cmd->type == COMMAND_TASK2) {
        Task2CommandArrow arrow1 = cmd->param > 0.0f ? TASK2_COMMAND_ARROW_LEFT
                                 : cmd->param < 0.0f ? TASK2_COMMAND_ARROW_RIGHT : TASK2_COMMAND_ARROW_NONE;
        const char *fault = Task2Run_Start(&task2, session.seq, arrow1, &task2CourseIo, HAL_GetTick());
        if (fault != NULL)
            return fault;
#if TASK2_COURSE_DRY_RUN
        OLED_Post(&cmdStatus, "T2 DRY RUN %u", (unsigned)session.seq);
#else
        OLED_Post(&cmdStatus, "T2 run %u", (unsigned)session.seq);
#endif
        return NULL;
    }
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
                            (motion.radiusMm == 0.0f ? runtimeMotionConfig.straightAccelerationMmps2 : runtimeMotionConfig.arcAccelerationMmps2),
                            (motion.radiusMm == 0.0f ? runtimeMotionConfig.straightDecelerationMmps2 : runtimeMotionConfig.arcDecelerationMmps2),
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
    OLED_Post(&cmdStatus, "%s %.0f", CommandParser_CommandName(cmd->type), cmd->param);
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
    Task2Run_FaultReason(&task2, reason);
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

/* Returns false once the course has finished or faulted. */
static bool MotionTask_UpdateTask2(void) {
    const char *fault = NULL;
    Task2RunStatus status = Task2Run_Update(&task2, MotionController_IsBusy(&motionController),
                                            HAL_GetTick(), &fault);
    if (status == TASK2RUN_FAULT) {
        MotionTask_SendTask2Events();
        MotionTask_Fault(fault);
        return false;
    }
    /* DONE means parked and stationary; until then the watchdog keeps guarding. */
    if (status == TASK2RUN_DONE) {
        MotionTask_SendTask2Events();
        MotionTask_Capture("DONE");
        CommandSession_CommandDone(&session, reply);
        CommandLink_Send(reply);
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
    Task2Run_Init(&task2);

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

    if (!MotionTask_ConfigureTask2()) {
        CommandLink_Send("FAULT - T2_CONFIG\n");
        return;
    }
    task2CourseIo.mmPerCount = 3.14159265358979323846f *
        runtimeMotionConfig.kinematics->rearWheelDiameterMm / runtimeMotionConfig.kinematics->rearEncoderCountsPerRev;
    task2CourseIo.decelerationMmps2 = task2MotionConfig.arcDecelerationMmps2;
#if TASK2_PLAN_HOST
    task2CourseIo.plan = MotionTask_Task2Plan;
#endif
#if TASK2_DIAGNOSTICS
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    task2CourseIo.elapsedUs = MotionTask_Task2ElapsedUs;
#endif
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
        Task2Run_Idle(&task2, &task2CourseIo, busy, HAL_GetTick());
        if (!busy) MotionTask_SampleProgress();
        if (!busy && commandRunning && !ultrasonicCommand && !task2.active) {
            MotionTask_Capture("DONE");
            CommandSession_CommandDone(&session, reply);
            CommandLink_Send(reply);
            commandRunning = false;
        }
        if (!busy) {
            Task2Run_Stationary(&task2);
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
#if TASK2_DIAGNOSTICS
            /* Task 2 planning records, only while stationary: each send blocks the loop briefly. */
            if (strncmp(line, "D T", 3) == 0 && (line[3] == '\0' || line[3] == ' ')) {
                if (MotionController_IsBusy(&motionController)) {
                    (void)snprintf(reply, sizeof(reply), "DT - BUSY\n");
                } else {
                    unsigned n = Task2Run_Diagnostics(&task2, line, task2DiagLines, TASK2_DIAG_PARTS);
                    for (unsigned i = 0U; i < n; ++i)
                        CommandLink_Send(task2DiagLines[i]);
                    if (n == 0U)
                        (void)snprintf(reply, sizeof(reply), "DT - NONE\n");
                    else
                        reply[0] = '\0';
                }
            } else if (strncmp(line, "D R", 3) == 0 && (line[3] == '\0' || line[3] == ' ')) {
                /* The run trace, a block at a time, only between runs: during one it still grows, and blocks of
                 * one fetch must come from the same trace. */
                if (MotionController_IsBusy(&motionController) || task2.active || task2.stopping || task2.finishing) {
                    (void)snprintf(reply, sizeof(reply), "DR - BUSY\n");
                } else {
                    unsigned n = Task2Run_Trace(&task2, line, task2DiagLines, TASK2_DIAG_PARTS);
                    for (unsigned i = 0U; i < n; ++i)
                        CommandLink_Send(task2DiagLines[i]);
                    if (n == 0U)
                        (void)snprintf(reply, sizeof(reply), "DR - NONE\n");
                    else
                        reply[0] = '\0';
                }
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
            } else if (Task2Run_Receive(&task2, line, reply)) {
                /* A/W/Q T: answered from the course state, never a batch. */
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
                    Task2Run_Stop(&task2);
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
        if (commandRunning && task2.active)
            commandRunning = MotionTask_UpdateTask2();
        MotionTask_SendTask2Events();

        uint32_t now = osKernelGetTickCount();
        /* A move launched in this pass starts its clock now: a planning wait before it is not time it has
         * spent, and charging it would skip the steering settle. */
        if (task2Launched) {
            previousUpdate = now;
            task2Launched = false;
        }
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
