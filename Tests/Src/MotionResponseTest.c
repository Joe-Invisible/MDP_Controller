#include "MotionResponseTest.h"
#include "RobotTestFixture.h"
#include "oledutils.h"
#include "userbutton.h"
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846f
#define CONTROL_MS 10U
#define RUN_TIMEOUT_MS 8000U
#define BRAKE_TIMEOUT_MS 3000U
#define CENTRE_TIMEOUT_MS 3000U
#define SETTLING_SEC 0.5f
#define PADDING_MM 281.0f
/* Deliberately one bounded validation, not another gain sweep. */
#define TARGET_YAW_DEG 90.0f

static RobotTestFixture fixture;
MotionResponsePlan motionResponsePlan;
MotionControllerConfig motionResponseConfig;
MotionResponseTestSample motionResponseLog[MOTION_RESPONSE_LOG_CAPACITY];
MotionResponseTestResult motionResponseResult;
uint32_t motionResponseLogCount;
bool motionResponseFinished;
static MotionResponsePose estimatedPose;
static float previousProgress, previousYawRad;
_Static_assert(sizeof(MotionResponseTestSample) == 96U, "Recheck RAM budget");

static bool Configure(void)
{
    motionResponseConfig = motionControllerConfig;
    motionResponseConfig.useLegacyStraightSteering = false;
    motionResponseConfig.straightYawRateKp = motionResponseConfig.arcYawRateKp = 0;
    motionResponseConfig.straightYawRateKi = motionResponseConfig.arcYawRateKi = 0;
    motionResponseConfig.straightYawRateKd = motionResponseConfig.arcYawRateKd = 0;
    motionResponseConfig.straightHeadingKpPerSec = motionResponseConfig.arcHeadingKpPerSec = 0;
    motionResponseConfig.arcYawRateFilterTauSec = .1f;
    motionResponseConfig.useMatchedYawRateReferenceFilter = true;
    motionResponseConfig.straightAccelerationMmps2 = motionResponseConfig.arcAccelerationMmps2 =
        MOTION_RESPONSE_ACCELERATION_MMPS2;
    motionResponseConfig.straightDecelerationMmps2 = motionResponseConfig.arcDecelerationMmps2 =
        MOTION_RESPONSE_ACCELERATION_MMPS2;
    /* The measured schedule assumes this wheel policy and zero raw centre.
     * Reject a changed production snapshot instead of silently changing inputs. */
    return motionResponseConfig.straightSteeringFeedforwardCommand == 0 &&
        motionResponseConfig.wheelSyncKpCpsPerMm == 10 &&
        motionResponseConfig.maxWheelSyncCorrectionCps == 100 &&
        MotionResponsePlan_Prepare(&motionResponsePlan, motionResponseConfig.kinematics,
                                  TARGET_YAW_DEG*PI/180);
}

static bool Centre(void)
{
    SteeringController_StartCentre(&fixture.steeringController);
    uint32_t start = HAL_GetTick();
    while (SteeringController_IsCentring(&fixture.steeringController))
    {
        HAL_Delay(CONTROL_MS);
        SteeringController_UpdateCentre(&fixture.steeringController, CONTROL_MS*.001f);
        if (HAL_GetTick()-start >= CENTRE_TIMEOUT_MS) return false;
    }
    SteeringController_SetRawCommand(&fixture.steeringController, 0);
    HAL_Delay((uint32_t)(SETTLING_SEC*1000));
    return true;
}

static void ShowPreparation(void)
{
    for (float s = 0; s < motionResponsePlan.totalDistanceMm+1; s += 1)
    {
        MotionResponsePose pose;
        MotionResponsePlan_Pose(&motionResponsePlan, s, &pose);
        motionResponseResult.minXmm = fminf(motionResponseResult.minXmm, pose.xMm);
        motionResponseResult.maxXmm = fmaxf(motionResponseResult.maxXmm, pose.xMm);
        motionResponseResult.minYmm = fminf(motionResponseResult.minYmm, pose.yMm);
        motionResponseResult.maxYmm = fmaxf(motionResponseResult.maxYmm, pose.yMm);
    }
    const MotionControllerConfig *c = fixture.motionController.config;
    OLED_Clear();
    OLED_Printf(0,0,"Response %.0fdeg %.0fCPS", TARGET_YAW_DEG, MOTION_RESPONSE_SPEED_CPS);
    OLED_Printf(0,1,"Extra arc %.0f mm", motionResponsePlan.extraArcMm);
    OLED_Printf(0,2,"Travel %.0f mm", motionResponsePlan.totalDistanceMm);
    OLED_Printf(0,3,"Area %.0fx%.0f mm", motionResponseResult.maxXmm-motionResponseResult.minXmm+2*PADDING_MM,
                motionResponseResult.maxYmm-motionResponseResult.minYmm+2*PADDING_MM);
    OLED_Printf(0,4,"S P%.0f I%.0f D%.0f H%.1f",c->straightYawRateKp,c->straightYawRateKi,
                c->straightYawRateKd,c->straightHeadingKpPerSec);
    OLED_Printf(0,5,"A P%.0f I%.0f D%.0f H%.1f",c->arcYawRateKp,c->arcYawRateKi,
                c->arcYawRateKd,c->arcHeadingKpPerSec);
    OLED_Printf(0,6,"Wheel P%.2f sync%.0f",fixture.leftWheelController.pid.kp,c->wheelSyncKpCpsPerMm);
    OLED_Printf(0,7,"Reset pose; SW1 start");
    OLED_Refresh_Gram();
}

static void UpdateEstimatedPose(void)
{
    const MotionController *c = &fixture.motionController;
    if (c->mode == MOTIONCONTROLLER_PROFILE_PREPARING) return;
    float yaw = c->yawDeg*PI/180;
    float ds = c->travelledDistanceMm-previousProgress;
    float mid = .5f*(yaw+previousYawRad);
    estimatedPose.xMm += ds*cosf(mid);
    estimatedPose.yMm += ds*sinf(mid);
    estimatedPose.yawRad = yaw;
    previousProgress = c->travelledDistanceMm;
    previousYawRad = yaw;
}

static void Log(uint32_t elapsed, float rawRate, bool terminal)
{
    uint32_t slot = motionResponseLogCount < MOTION_RESPONSE_LOG_CAPACITY
        ? motionResponseLogCount++ : MOTION_RESPONSE_LOG_CAPACITY-1U;
    const MotionController *c = &fixture.motionController;
    MotionPathSample sample;
    MotionResponsePose pose;
    MotionResponsePlan_Evaluate(&motionResponsePlan,c->travelledDistanceMm,&sample);
    MotionResponsePlan_Pose(&motionResponsePlan,c->travelledDistanceMm,&pose);
    uint8_t phase = MotionResponsePlan_Phase(&motionResponsePlan,c->travelledDistanceMm);
    if (c->mode == MOTIONCONTROLLER_PROFILE_PREPARING) phase = 0;
    if (c->mode == MOTIONCONTROLLER_BRAKING) phase = 6;
    if (terminal) phase = 7;
    motionResponseLog[slot] = (MotionResponseTestSample){
        .timeMs=elapsed,.mode=(uint8_t)c->mode,.phase=phase,
        .progressMm=c->travelledDistanceMm,.rawReference=sample.steeringFeedforwardCommand,
        .rawCommand=fixture.steeringController.command,.rawCommandRatePerSec=rawRate,
        .bodyCurvaturePerMm=sample.curvaturePerMm,
        .wheelCurvaturePerMm=sample.wheelReferenceCurvaturePerMm,
        .desiredYawDeg=sample.desiredYawRad*180/PI,.yawDeg=c->yawDeg,
        .yawRateDps=c->yawRateDps,.filteredYawRateDps=c->filteredYawRateDps,
        .expectedXmm=pose.xMm,.expectedYmm=pose.yMm,
        .estimatedXmm=estimatedPose.xMm,.estimatedYmm=estimatedPose.yMm,
        .leftTargetCps=fixture.leftWheelController.targetSpeedCps,
        .rightTargetCps=fixture.rightWheelController.targetSpeedCps,
        .leftMeasuredCps=fixture.leftWheelController.measuredSpeedCps,
        .rightMeasuredCps=fixture.rightWheelController.measuredSpeedCps,
        .wheelSyncErrorMm=c->wheelSyncErrorMm,.wheelSyncCorrectionCps=c->wheelSyncCorrectionCps,
        .measuredCentreSpeedMmps=c->measuredCentreSpeedMmps,
        .geometricYawRateDps=c->arcUnfilteredFeedforwardYawRateRadPerSec*180/PI
    };
}

static void Run(void)
{
    MotionPathProfile path = {.signedDistanceMm=motionResponsePlan.totalDistanceMm,
        .maxSpeedCps=MOTION_RESPONSE_SPEED_CPS,.steeringSettlingTimeSec=SETTLING_SEC,
        .evaluate=MotionResponsePlan_Evaluate,.context=&motionResponsePlan,
        .steeringCommandRatePerSec=480,.terminalYawPriority=false};
    motionResponseResult.status = MotionController_FollowProfile(&fixture.motionController,&path);
    if (motionResponseResult.status != MOTIONCONTROLLER_STATUS_OK)
    { motionResponseResult.outcome=RESPONSE_CONTROLLER_ERROR; return; }
    uint32_t start=HAL_GetTick(), previous=start, abortStart=0;
    bool aborting=false;
    float previousRaw=fixture.steeringController.command, rawRate=0;
    Log(0,0,false);
    while (MotionController_IsBusy(&fixture.motionController))
    {
        uint32_t now=HAL_GetTick(), interval=now-previous;
        if (interval < CONTROL_MS) continue;
        previous=now;
        if (interval > motionResponseResult.maxUpdateIntervalMs)
            motionResponseResult.maxUpdateIntervalMs=interval;
        MotionResponseOutcome reason=RESPONSE_NOT_RUN;
        if (SW1_ReadState()==SW1_Enabled) reason=RESPONSE_CANCELLED;
        else if (now-start >= RUN_TIMEOUT_MS) reason=RESPONSE_TIMED_OUT;
        else if (motionResponseLogCount >= MOTION_RESPONSE_LOG_CAPACITY-1U) reason=RESPONSE_LOG_FULL;
        if (!aborting && reason != RESPONSE_NOT_RUN)
        {
            motionResponseResult.outcome=reason;
            MotionController_Brake(&fixture.motionController);
            aborting=true; abortStart=now;
        }
        if (aborting && now-abortStart >= BRAKE_TIMEOUT_MS)
        {
            motionResponseResult.outcome=RESPONSE_BRAKE_TIMED_OUT;
            MotionController_Stop(&fixture.motionController); break;
        }
        motionResponseResult.status=MotionController_Update(&fixture.motionController,interval*.001f);
        UpdateEstimatedPose();
        rawRate=(fixture.steeringController.command-previousRaw)/(interval*.001f);
        previousRaw=fixture.steeringController.command;
        if (motionResponseResult.status != MOTIONCONTROLLER_STATUS_OK ||
            fabsf(fixture.motionController.arcSteeringCorrectionCommand) > .0001f)
        {
            motionResponseResult.outcome=RESPONSE_CONTROLLER_ERROR;
            MotionController_Stop(&fixture.motionController); break;
        }
        if (MotionController_IsBusy(&fixture.motionController) &&
            motionResponseLogCount < MOTION_RESPONSE_LOG_CAPACITY-1U) Log(now-start,rawRate,false);
    }
    motionResponseResult.elapsedMs=HAL_GetTick()-start;
    motionResponseResult.estimatedEnd=estimatedPose;
    motionResponseResult.measuredDistanceMm=fixture.motionController.travelledDistanceMm;
    if (motionResponseResult.outcome==RESPONSE_NOT_RUN) motionResponseResult.outcome=RESPONSE_COMPLETED;
    Log(motionResponseResult.elapsedMs,rawRate,true);
}

__attribute__((noinline)) void MotionResponseTestFinished(void)
{ __asm volatile ("" ::: "memory"); }

void MotionResponseTestRun(void)
{
    motionResponseFinished=false; motionResponseLogCount=0;
    memset(&motionResponseResult,0,sizeof(motionResponseResult));
    memset(&estimatedPose,0,sizeof(estimatedPose)); previousProgress=previousYawRad=0;
    if (!Configure() || !RobotTestFixture_InitMotionController(&fixture,&motionResponseConfig))
    { motionResponseResult.outcome=RESPONSE_PREPARATION_ERROR; goto finished; }
    fixture.leftWheelController.pid.kp=fixture.rightWheelController.pid.kp=.03f;
    fixture.leftWheelController.pid.ki=fixture.rightWheelController.pid.ki=0;
    fixture.leftWheelController.pid.kd=fixture.rightWheelController.pid.kd=0;
    MotionResponsePlan_Pose(&motionResponsePlan,motionResponsePlan.totalDistanceMm,&motionResponseResult.plannedEnd);
    if (!Centre()) { motionResponseResult.outcome=RESPONSE_PREPARATION_ERROR; goto finished; }
    ShowPreparation(); SW1_WaitForPressAndRelease(); Run();
finished:
    motionResponseFinished=true;
    OLED_Clear(); OLED_Printf(0,0,"Response code%u",(unsigned)motionResponseResult.outcome);
    OLED_Printf(0,1,"Yaw %.1f / %.1f",motionResponseResult.estimatedEnd.yawRad*180/PI,
                motionResponseResult.plannedEnd.yawRad*180/PI);
    OLED_Printf(0,2,"Export this run"); OLED_Refresh_Gram();
    MotionResponseTestFinished();
}
