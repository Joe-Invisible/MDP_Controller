/*
 * RobotKinematicsTest.c
 *
 * Created on: 2026年9月9日
 * Author: Joe
 */

#include "RobotKinematicsTest.h"

#include "RobotKinematics.h"

#include <math.h>
#include <stdbool.h>


#define TEST_WHEELBASE_MM              145.0f
#define TEST_REAR_TRACK_WIDTH_MM       185.0f

#define TEST_CENTRE_SPEED_CPS          2000.0f
#define TEST_RADIUS_MM                 4000.0f

#define TEST_CURVATURE_PER_MM          (1.0f / TEST_RADIUS_MM)

#define ANGLE_TOLERANCE_RAD            0.000001f
#define SPEED_TOLERANCE_CPS            0.01f
#define CURVATURE_TOLERANCE_PER_MM     0.00000001f


typedef struct
{
    float curvaturePerMm;

    float expectedSteeringAngleRad;
    float actualSteeringAngleRad;

    float expectedLeftSpeedCps;
    float actualLeftSpeedCps;

    float expectedRightSpeedCps;
    float actualRightSpeedCps;

    bool steeringPassed;
    bool leftSpeedPassed;
    bool rightSpeedPassed;

    bool passed;

} RobotKinematicsCaseResult;


typedef struct
{
    float inputCurvaturePerMm;
    float steeringAngleRad;
    float recoveredCurvaturePerMm;

    bool passed;

} RobotKinematicsRoundTripResult;


/*
 * Static results are intentionally retained so they can be
 * inspected directly using the debugger.
 */
static RobotKinematicsCaseResult straightResult;
static RobotKinematicsCaseResult leftCurveResult;
static RobotKinematicsCaseResult rightCurveResult;

static RobotKinematicsRoundTripResult roundTripResult;

static bool robotKinematicsTestPassed;


static bool FloatNear(
    float actual,
    float expected,
    float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}


static void RobotKinematicsTest_RunCase(
    const RobotKinematics *kinematics,
    float curvaturePerMm,
    float centreSpeedCps,
    float expectedSteeringAngleRad,
    float expectedLeftSpeedCps,
    float expectedRightSpeedCps,
    RobotKinematicsCaseResult *result)
{
    *result = (RobotKinematicsCaseResult){0};

    result->curvaturePerMm =
        curvaturePerMm;

    result->expectedSteeringAngleRad =
        expectedSteeringAngleRad;

    result->expectedLeftSpeedCps =
        expectedLeftSpeedCps;

    result->expectedRightSpeedCps =
        expectedRightSpeedCps;


    result->actualSteeringAngleRad =
        RobotKinematics_GetSteeringAngleRad(
            kinematics,
            curvaturePerMm);


    RobotKinematics_GetRearWheelSpeedTargets(
        kinematics,
        centreSpeedCps,
        curvaturePerMm,
        &result->actualLeftSpeedCps,
        &result->actualRightSpeedCps);


    result->steeringPassed =
        FloatNear(
            result->actualSteeringAngleRad,
            result->expectedSteeringAngleRad,
            ANGLE_TOLERANCE_RAD);

    result->leftSpeedPassed =
        FloatNear(
            result->actualLeftSpeedCps,
            result->expectedLeftSpeedCps,
            SPEED_TOLERANCE_CPS);

    result->rightSpeedPassed =
        FloatNear(
            result->actualRightSpeedCps,
            result->expectedRightSpeedCps,
            SPEED_TOLERANCE_CPS);


    result->passed =
        result->steeringPassed &&
        result->leftSpeedPassed &&
        result->rightSpeedPassed;
}


void RobotKinematicsTestRun(void)
{
    RobotKinematics kinematics = {
        .rearEncoderCountsPerRev = 0,
        .rearWheelDiameterMm = 0.0f,
        .wheelbaseMm = TEST_WHEELBASE_MM,
        .rearTrackWidthMm = TEST_REAR_TRACK_WIDTH_MM
    };


    /*
     * Test 1: Straight motion.
     *
     * curvature = 0
     * steering = 0
     * vL = vR = centre speed
     */
    RobotKinematicsTest_RunCase(
        &kinematics,
        0.0f,
        TEST_CENTRE_SPEED_CPS,

        0.0f,
        2000.0f,
        2000.0f,

        &straightResult);


    /*
     * Test 2: Positive curvature.
     *
     * R = 4000 mm
     * curvature = +0.00025 /mm
     *
     * delta = atan(145 / 4000)
     *       = 0.0362341 rad
     *
     * vL = 1953.75 CPS
     * vR = 2046.25 CPS
     */
    RobotKinematicsTest_RunCase(
        &kinematics,
        TEST_CURVATURE_PER_MM,
        TEST_CENTRE_SPEED_CPS,

        0.036234134f,
        1953.75f,
        2046.25f,

        &leftCurveResult);


    /*
     * Test 3: Negative curvature.
     *
     * Everything should mirror about zero.
     */
    RobotKinematicsTest_RunCase(
        &kinematics,
        -TEST_CURVATURE_PER_MM,
        TEST_CENTRE_SPEED_CPS,

        -0.036234134f,
        2046.25f,
        1953.75f,

        &rightCurveResult);


    /*
     * Test 4:
     * curvature -> steering angle -> curvature
     *
     * This checks consistency between the forward and
     * inverse bicycle-model calculations.
     */
    roundTripResult =
        (RobotKinematicsRoundTripResult){0};

    roundTripResult.inputCurvaturePerMm =
        TEST_CURVATURE_PER_MM;

    roundTripResult.steeringAngleRad =
        RobotKinematics_GetSteeringAngleRad(
            &kinematics,
            roundTripResult.inputCurvaturePerMm);

    roundTripResult.recoveredCurvaturePerMm =
        RobotKinematics_GetCurvaturePerMm(
            &kinematics,
            roundTripResult.steeringAngleRad);

    roundTripResult.passed =
        FloatNear(
            roundTripResult.recoveredCurvaturePerMm,
            roundTripResult.inputCurvaturePerMm,
            CURVATURE_TOLERANCE_PER_MM);


    robotKinematicsTestPassed =
        straightResult.passed &&
        leftCurveResult.passed &&
        rightCurveResult.passed &&
        roundTripResult.passed;
}
