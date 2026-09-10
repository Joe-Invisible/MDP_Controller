/*
 * RobotKinematics.c
 *
 *  Created on: 2026年9月9日
 *      Author: Joe
 */


#include "RobotKinematics.h"

#include <math.h>


float RobotKinematics_GetCurvaturePerMm(
    const RobotKinematics *kinematics,
    float steeringAngleRad)
{
    return tanf(steeringAngleRad) /
           kinematics->wheelbaseMm;
}


float RobotKinematics_GetSteeringAngleRad(
    const RobotKinematics *kinematics,
    float curvaturePerMm)
{
    return atanf(
        kinematics->wheelbaseMm *
        curvaturePerMm);
}


void RobotKinematics_GetRearWheelSpeedTargets(
    const RobotKinematics *kinematics,
    float centreSpeedCps,
    float curvaturePerMm,
    float *leftSpeedCps,
    float *rightSpeedCps)
{
    float halfTrackCurvature =
        0.5f *
        kinematics->rearTrackWidthMm *
        curvaturePerMm;

    *leftSpeedCps =
        centreSpeedCps *
        (1.0f - halfTrackCurvature);

    *rightSpeedCps =
        centreSpeedCps *
        (1.0f + halfTrackCurvature);
}
