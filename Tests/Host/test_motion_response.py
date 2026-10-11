#!/usr/bin/env python3
"""Check response geometry, reference separation, robot flow and GDB export.
Hardware is mocked; passing this does not validate the physical response model.
"""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
from test_unified_motion import HAL_STUB, FIXTURE_HARNESS
from test_steering_ramp import HARNESS as RAMP_HARNESS
ROOT = Path(__file__).resolve().parents[2]
# Reuse the hardware simulator, not the ramp test's assertions or profile.
MOCKS = RAMP_HARNESS[len(FIXTURE_HARNESS):].split('void SW1_WaitForPressAndRelease(void)')[0]
MOCKS = MOCKS.replace('SteeringRampResponseTest.h', 'MotionResponseTest.h')
HARNESS = FIXTURE_HARNESS + MOCKS + r'''
#include "SteeringRampProfile.h"
void SW1_WaitForPressAndRelease(void)
{
    assert(starts == 0);
    assert(strstr(rows[2], "Travel") && strstr(rows[3], "Area"));
    assert(strstr(rows[4], "S P0 I0 D0 H0.0") && strstr(rows[5], "A P0 I0 D0 H0.0"));
    near(robot->leftWheelController.pid.kp, .03f, 0);
    near(robot->rightWheelController.pid.ki, 0, 0);
    near(robot->motionController.config->wheelSyncKpCpsPerMm, 10, 0);
    near(robot->motionController.config->maxWheelSyncCorrectionCps, 100, 0);
    robot->leftRearWheel.state.encCount = robot->rightRearWheel.state.encCount = 32760;
    trialStart=tick; ++starts;
}
int SW1_ReadState(void)
{ return (scenario == 1 || scenario == 6) && tick-trialStart >= 2000; }

static void checkGeometry(void)
{
    MotionResponsePlan p;
    assert(MotionResponsePlan_Prepare(&p, &kinematics, PI/2));
    assert(p.extraArcMm > 380 && p.extraArcMm < 390);
    assert(p.totalDistanceMm > 1800 && p.totalDistanceMm < 1810);
    MotionResponsePose end;
    assert(MotionResponsePlan_Pose(&p, p.totalDistanceMm, &end));
    near(end.yawRad, PI/2, .000002f);
    MotionPathSample sample;
    assert(MotionResponsePlan_Evaluate(&p, p.totalDistanceMm+100, &sample));
    near(sample.desiredYawRad, PI/2, .000002f);
    near(sample.brakingSpeedLimitCps, 0, 0);
    MotionResponsePose integrated={0};
    float ds=.2f;
    double x=0,y=0;
    for (unsigned j=0;j<(unsigned)ceilf(p.totalDistanceMm/ds);++j) {
        float s=j*ds;
        float step=fminf((j+1)*ds,p.totalDistanceMm)-s;
        MotionResponsePose mid;
        assert(MotionResponsePlan_Pose(&p,s+step*.5f,&mid));
        x+=step*cosf(mid.yawRad); y+=step*sinf(mid.yawRad);
    }
    integrated.xMm=(float)x; integrated.yMm=(float)y;
    near(integrated.xMm,end.xMm,.02f); near(integrated.yMm,end.yMm,.02f);
    for (uint32_t i=1;i<p.nodeCount-1;++i) {
        MotionResponsePose a,b;
        float s=p.nodes[i].progressMm;
        MotionResponsePlan_Pose(&p,s-.001f,&a); MotionResponsePlan_Pose(&p,s+.001f,&b);
        assert(hypotf(b.xMm-a.xMm,b.yMm-a.yMm)<.003f);
        assert(fabsf(b.yawRad-a.yawRad)<.00002f);
    }
    MotionPathSample entry,exit,tail;
    MotionResponsePlan_Evaluate(&p,400,&entry);
    MotionResponsePlan_Evaluate(&p,870.757142f+p.extraArcMm,&exit);
    MotionResponsePlan_Evaluate(&p,1100+p.extraArcMm,&tail);
    near(entry.steeringFeedforwardCommand,exit.steeringFeedforwardCommand,.001f);
    near(entry.wheelReferenceCurvaturePerMm,exit.wheelReferenceCurvaturePerMm,.000001f);
    assert(exit.curvaturePerMm > entry.curvaturePerMm*2);
    near(tail.steeringFeedforwardCommand,0,0); near(tail.wheelReferenceCurvaturePerMm,0,0);
    assert(tail.curvaturePerMm > .00008f); /* residual turning, not a straight */
    for (float yaw=.87f; yaw<=1.75f; yaw+=.1f) {
        assert(MotionResponsePlan_Prepare(&p,&kinematics,yaw));
        MotionResponsePlan_Pose(&p,p.totalDistanceMm,&end); near(end.yawRad,yaw,.000003f);
    }
    assert(!MotionResponsePlan_Prepare(&p,&kinematics,PI));
    assert(!p.prepared);
    assert(!MotionResponsePlan_Prepare(&p,&kinematics,0));
    assert(!MotionResponsePlan_Prepare(&p,&kinematics,-PI/2));
    assert(!MotionResponsePlan_Prepare(&p,&kinematics,NAN));
    RobotKinematics invalid=kinematics; invalid.rearWheelDiameterMm=NAN;
    assert(!MotionResponsePlan_Prepare(&p,&invalid,PI/2));
    invalid=kinematics; invalid.rearTrackWidthMm+=1;
    assert(!MotionResponsePlan_Prepare(&p,&invalid,PI/2));
    assert(!MotionResponsePlan_Evaluate(NULL,0,&sample));
    assert(MotionResponsePlan_Prepare(&p,&kinematics,PI/2));
    assert(!MotionResponsePlan_Evaluate(&p,NAN,&sample));
    const char *csv=getenv("MOTION_RESPONSE_PATH_CSV");
    if (csv) {
        FILE *stream=fopen(csv,"w"); assert(stream);
        fputs("progressMm,xMm,yMm,yawDeg,phase,rawCommand,bodyCurvaturePerMm,wheelCurvaturePerMm\n",stream);
        for (unsigned j=0;j<=(unsigned)ceilf(p.totalDistanceMm/2);++j) {
            float s=fminf(j*2,p.totalDistanceMm);
            MotionResponsePose pose; MotionPathSample reference;
            MotionResponsePlan_Pose(&p,s,&pose); MotionResponsePlan_Evaluate(&p,s,&reference);
            fprintf(stream,"%.6f,%.6f,%.6f,%.6f,%u,%.6f,%.9f,%.9f\n",s,pose.xMm,pose.yMm,
                pose.yawRad*180/PI,MotionResponsePlan_Phase(&p,s),reference.steeringFeedforwardCommand,
                reference.curvaturePerMm,reference.wheelReferenceCurvaturePerMm);
        }
        fclose(stream);
    }
    printf("Predicted endpoint: %.2f, %.2f mm; %.2fdeg; travel %.2f mm\n",
        p.nodes[p.nodeCount-1].pose.xMm,p.nodes[p.nodeCount-1].pose.yMm,
        p.requestedYawRad*180/PI,p.totalDistanceMm);
}
static void checkCharacterizedActuation(void)
{
    Fixture f; setup(&f,false);
    static const MotionControllerArcFeedforwardPoint points[]={
        {.000054f,-5.625f},{.000183f,-11.25f},{.000436f,-16.875f},
        {1.0f/1500,-22.5f},{1.0f/1000,-27.7f},{1.0f/500,-47.5f},
        {1.0f/300,-75},{1.0f/275,-81.7f}};
    MotionControllerArcConfig arc=*f.config.arcConfig;
    arc.positivePoints=points; arc.positivePointCount=8;
    f.config.arcConfig=&arc; assert(initMotion(&f)==0);
    SteeringRampTrial trial={4500,200,525,350,300,350,150};
    SteeringRampProfile baseline;
    assert(SteeringRampProfile_Build(&baseline,&f.motion,&trial));
    MotionResponsePlan p; assert(MotionResponsePlan_Prepare(&p,&kinematics,PI/2));
    for (unsigned j=0;j<=1421;++j) {
        float s=fminf(j,baseline.totalDistanceMm);
        float mapped=s+(s>785.378571f ? p.extraArcMm : 0);
        MotionPathSample original,response;
        assert(SteeringRampProfile_Evaluate(&baseline,s,&original));
        assert(MotionResponsePlan_Evaluate(&p,mapped,&response));
        near(response.steeringFeedforwardCommand,SteeringRampProfile_RawCommand(&baseline,s),.0001f);
        near(response.wheelReferenceCurvaturePerMm,original.curvaturePerMm,.00000001f);
    }
}
static unsigned badKind;
static bool separated(const void *context,float progress,MotionPathSample *s)
{
    (void)context;
    *s=(MotionPathSample){.curvaturePerMm=-.003f,.desiredYawRad=-.1f,
        .speedLimitCps=1000,.straightTuningWeight=0,.actuationProvided=true,
        .steeringFeedforwardCommand=-5,.wheelReferenceCurvaturePerMm=.0005f};
    if (badKind==1) s->steeringFeedforwardCommand=101;
    if (badKind==2) s->wheelReferenceCurvaturePerMm=NAN;
    if (badKind==3 && progress > 1 && progress < 99) s->steeringFeedforwardCommand=NAN;
    return true;
}
static void checkSeparation(void)
{
    Fixture f; setup(&f,false);
    f.config.straightYawRateKp=f.config.straightYawRateKi=f.config.straightYawRateKd=0;
    f.config.arcYawRateKp=f.config.arcYawRateKi=f.config.arcYawRateKd=0;
    f.config.straightHeadingKpPerSec=f.config.arcHeadingKpPerSec=0;
    assert(initMotion(&f)==0);
    MotionPathProfile profile={.signedDistanceMm=100,.maxSpeedCps=1000,
        .steeringSettlingTimeSec=.5f,.evaluate=separated};
    for (badKind=1;badKind<=2;++badKind) {
        float before=f.steering.command;
        assert(MotionController_FollowProfile(&f.motion,&profile)==MOTIONCONTROLLER_STATUS_PROFILE_ERROR);
        assert(!MotionController_IsBusy(&f.motion)); near(f.steering.command,before,0);
    }
    badKind=0;
    assert(MotionController_FollowProfile(&f.motion,&profile)==0);
    prepare(&f,MOTIONCONTROLLER_PROFILE_PREPARING,MOTIONCONTROLLER_PROFILE);
    f.left.measuredSpeedCps=f.right.measuredSpeedCps=500;
    f.leftMotor.state.encCount+=40; f.rightMotor.state.encCount+=40;
    update(&f,DT);
    near(f.steering.command,-5,0); near(f.motion.targetCurvaturePerMm,-.003f,0);
    assert(f.motion.arcUnfilteredFeedforwardYawRateRadPerSec < 0);
    assert(f.motion.desiredWheelTravelDifferenceMm > 0);
    assert(f.right.targetSpeedCps > f.left.targetSpeedCps);
    near(f.motion.pathSample.desiredYawRad,-.1f,0);
    badKind=3;
    assert(MotionController_Update(&f.motion,DT)==MOTIONCONTROLLER_STATUS_PROFILE_ERROR);
    assert(!MotionController_IsBusy(&f.motion));
}
static void reset(unsigned selected)
{ scenario=selected; tick=trialStart=starts=0; leftFraction=rightFraction=0; robot=NULL; }
static void checkRobot(void)
{
    for (unsigned selected=0;selected<=7;++selected) {
        reset(selected); MotionResponseTestRun();
        assert(motionResponseFinished && motionResponseLogCount<=MOTION_RESPONSE_LOG_CAPACITY);
        MotionResponseOutcome expected=selected==0 ? RESPONSE_COMPLETED :
            selected==1 ? RESPONSE_CANCELLED : selected==3 ? RESPONSE_CONTROLLER_ERROR :
            selected==4 ? RESPONSE_PREPARATION_ERROR : selected==6 ? RESPONSE_BRAKE_TIMED_OUT :
            selected==7 ? RESPONSE_TIMED_OUT : RESPONSE_LOG_FULL;
        assert(motionResponseResult.outcome==expected);
        if (selected==4) { assert(starts==0 && motionResponseLogCount==0); continue; }
        assert(starts==1 && motionResponseLogCount>0);
        assert(motionResponseLog[motionResponseLogCount-1].phase==7);
        assert(motionResponseLog[motionResponseLogCount-1].mode==MOTIONCONTROLLER_IDLE);
        if (selected==0) {
            unsigned phases[8]={0};
            for (uint32_t i=0;i<motionResponseLogCount;++i) {
                MotionResponseTestSample *s=&motionResponseLog[i]; ++phases[s->phase];
                assert(isfinite(s->expectedXmm) && isfinite(s->estimatedXmm));
                if (s->mode==MOTIONCONTROLLER_PROFILE) near(s->rawCommand,s->rawReference,.00001f);
            }
            for (unsigned i=0;i<8;++i) assert(phases[i]>0);
            near(motionResponseResult.estimatedEnd.yawRad,PI/2,.025f);
            printf("Mock endpoint run: %ums, %u samples; area %.0fx%.0f mm\n",
                motionResponseResult.elapsedMs,motionResponseLogCount,
                motionResponseResult.maxXmm-motionResponseResult.minXmm+562,
                motionResponseResult.maxYmm-motionResponseResult.minYmm+562);
        }
        if (selected==3) assert(motionResponseResult.status==MOTIONCONTROLLER_STATUS_IMU_ERROR);
    }
}
int main(void)
{
    checkGeometry(); checkCharacterizedActuation(); checkSeparation(); checkRobot();
    puts("PASS: asymmetric response geometry, bounded heading fit, explicit actuator/wheel/body references, "
         "endpoint run, cancellation, faults, log bounds and watchdogs");
    return 0;
}
'''

EXPORT_FIXTURE = r'''
#include "MotionResponseTest.h"
#include "RobotTestFixture.h"
MotionControllerConfig motionResponseConfig={.kinematics=&kinematics};
static RobotTestFixture fixture={.motionController={.initialized=INITIALIZED,
    .config=&motionResponseConfig,.leftWheel=&fixture.leftWheelController,
    .rightWheel=&fixture.rightWheelController,.pathProfile={.steeringCommandRatePerSec=480}}};
MotionResponsePlan motionResponsePlan={.prepared=true,.totalDistanceMm=1804};
MotionResponseTestResult motionResponseResult={.outcome=RESPONSE_COMPLETED};
MotionResponseTestSample motionResponseLog[MOTION_RESPONSE_LOG_CAPACITY]={
    {.phase=2,.rawReference=-5.0f,.bodyCurvaturePerMm=.0001f,.wheelCurvaturePerMm=.000054f},
    {.phase=4,.rawReference=-5.0f,.bodyCurvaturePerMm=.0005f,.wheelCurvaturePerMm=.000054f},
    {.phase=5,.rawReference=0.0f,.bodyCurvaturePerMm=.0001f,.wheelCurvaturePerMm=0.0f}};
uint32_t motionResponseLogCount=LOG_COUNT;
bool motionResponseFinished=true;
static const void *volatile roots[7];
int main(void) {
    roots[0]=&fixture; roots[1]=&motionResponsePlan; roots[2]=&motionResponseResult;
    roots[3]=motionResponseLog; roots[4]=&motionResponseLogCount;
    roots[5]=&motionResponseConfig; roots[6]=&motionResponseFinished;
    return 0;
}
'''


def check_export(gdb, work):
    (work/'MotionResponseTest.c').write_text(EXPORT_FIXTURE)
    for count, initialized in [(3,1),(700,1),(0,1),(701,1),(3,0)]:
        elf, export=work/'export-fixture',work/'export.txt'
        command=shlex.split(os.environ.get('CC','gcc'))+[
            '-std=c11','-g','-O0','-fno-pie','-no-pie','-ffunction-sections','-fdata-sections','-Wl,--gc-sections',
            f'-DLOG_COUNT={count}',f'-DINITIALIZED={initialized}',
            f'-I{work}',f'-I{ROOT/"Controllers/Inc"}',f'-I{ROOT/"PeripheralDrivers/Inc"}',
            f'-I{ROOT/"Tests/Inc"}',f'-I{ROOT/"Tests/Common"}',str(work/'MotionResponseTest.c'),
            str(ROOT/'Controllers/Src/MotionControllerConfig.c'),'-o',str(elf)]
        subprocess.run(command,check=True)
        export.unlink(missing_ok=True)
        result=subprocess.run([gdb,'-q','--nx','-batch',str(elf),'-ex','set pagination off',
            '-ex',f'source {ROOT}/exp/gdb_scripts/export_motion_response.gdb',
            '-ex','set $motion_response_battery_v = 11.30',
            '-ex',f'motion-response-export {export}'],capture_output=True,text=True)
        assert result.returncode==0,result.stdout+result.stderr
        text=export.read_text()
        assert text.rstrip().endswith('=== EXPORT COMPLETE ===') and '11.300' in text
        if count in (3,700):
            assert 'phase = 5' in text and 'bodyCurvaturePerMm' in text and 'wheelCurvaturePerMm' in text
        else: assert 'sample array was not read' in text
        assert ('steeringCommandRatePerSec = 480' in text)==bool(initialized)
    print('PASS: GDB export, full 67 KB array, file-qualified fixture, empty/corrupt counts and init failure')


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--gdb'); args=parser.parse_args()
    sources=['MotionController','MotionControllerConfig','MotionResponsePlan','SteeringController',
             'SteeringControllerConfig','PIDController','MotionProfile','RobotKinematics']
    with tempfile.TemporaryDirectory(prefix='mdp-response-') as directory:
        work=Path(directory)
        (work/'stm32f4xx_hal.h').write_text(HAL_STUB+'\nuint32_t HAL_GetTick(void);\nvoid HAL_Delay(uint32_t ms);\n')
        (work/'oledutils.h').write_text('#include <stdint.h>\nvoid OLED_Clear(void);\nvoid OLED_Refresh_Gram(void);\n'
                                       'void OLED_Printf(uint8_t,uint8_t,const char *,...);\n')
        (work/'userbutton.h').write_text('void SW1_WaitForPressAndRelease(void);\nint SW1_ReadState(void);\n#define SW1_Enabled 1\n')
        (work/'test.c').write_text(HARNESS)
        command=shlex.split(os.environ.get('CC','gcc'))+['-std=c11','-Wall','-Wextra','-Werror','-pedantic','-O1','-g',
            f'-I{work}',f'-I{ROOT/"Controllers/Inc"}',f'-I{ROOT/"PeripheralDrivers/Inc"}',
            f'-I{ROOT/"Tests/Inc"}',f'-I{ROOT/"Tests/Common"}',str(work/'test.c'),str(ROOT/'Tests/Src/MotionResponseTest.c'),
            str(ROOT/'Tests/Common/SteeringRampProfile.c')]
        command += [str(ROOT/f'Controllers/Src/{name}.c') for name in sources]
        env=os.environ.copy(); env['ASAN_OPTIONS']=env.get('ASAN_OPTIONS','')+':detect_leaks=0'
        subprocess.run(command+['-fsanitize=address,undefined','-fno-sanitize-recover=all','-lm','-o',str(work/'test')],check=True)
        subprocess.run([str(work/'test')],check=True,env=env)
        if args.gdb: check_export(args.gdb,work)


if __name__=='__main__': main()
