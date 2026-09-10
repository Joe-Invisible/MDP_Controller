/*
 * MotionController.c
 *
 * Created on: 2026年8月30日
 * Author: Joe
 */

#include "MotionController.h"

#include <math.h>
#include <stddef.h>

#define MOTION_PI 3.14159265358979323846f


static float MotionController_GetMmPerCount(
    const MotionController *controller)
{
    /*
     * Why are we recomputing a constant each time?
     * Does the wheel grow as the robot moves?
     * Relativistic physics?
     */
    return MOTION_PI *
        controller->kinematics->rearWheelDiameterMm /
        (float)controller->kinematics->rearEncoderCountsPerRev;
}

static float MotionController_GetWheelReferenceCurvaturePerMm(
    const MotionController *controller)
{
    /*
     * ARC:
     * Rear-wheel coordination follows the requested path
     * curvature directly. It must not depend on the estimated
     * steering angle or wheelbase-based steering model.
     *
     * STRAIGHT:
     * Preserve the existing behaviour. Small heading
     * corrections have already been experimentally validated
     * with the existing steering calibration, so rear-wheel
     * coordination continues to accommodate those corrections.
     */
    if (controller->mode == MOTIONCONTROLLER_ARC)
    {
        return controller->targetCurvaturePerMm;
    }

    float steeringAngleRad =
        SteeringController_GetEffectiveAngleRad(
            controller->steering);

    return RobotKinematics_GetCurvaturePerMm(
        controller->kinematics,
        steeringAngleRad);
}

static void MotionController_ResetOdometry(
    MotionController *controller)
{
    controller->leftTravelMm = 0.0f;
    controller->rightTravelMm = 0.0f;
    controller->travelledDistanceMm = 0.0f;

    controller->desiredWheelTravelDifferenceMm = 0.0f;
    controller->wheelSyncErrorMm = 0.0f;

    controller->previousLeftEncoderCount =
        DCMotor_GetEncoderCount(controller->leftWheel->motor);

    controller->previousRightEncoderCount =
        DCMotor_GetEncoderCount(controller->rightWheel->motor);
}


static void MotionController_UpdateOdometry(
    MotionController *controller)
{
    int16_t currentLeft =
        DCMotor_GetEncoderCount(controller->leftWheel->motor);

    int16_t currentRight =
        DCMotor_GetEncoderCount(controller->rightWheel->motor);

    /*
     * Use the same wrap-safe subtraction used by
     * WheelSpeedController.
     */
    int16_t deltaLeft = (int16_t)(
        (uint16_t)currentLeft -
        (uint16_t)controller->previousLeftEncoderCount);

    int16_t deltaRight = (int16_t)(
        (uint16_t)currentRight -
        (uint16_t)controller->previousRightEncoderCount);

    controller->previousLeftEncoderCount = currentLeft;
    controller->previousRightEncoderCount = currentRight;

    float mmPerCount =
        MotionController_GetMmPerCount(controller);

    float deltaLeftMm =
        (float)deltaLeft * mmPerCount;

    float deltaRightMm =
        (float)deltaRight * mmPerCount;

    /*
     * Signed displacement of the rear-axle centre.
     */
    float deltaCentreMm =
        0.5f * (deltaLeftMm + deltaRightMm);

    /*
     * The encoder increments measured here correspond
     * approximately to motion performed under the steering
     * command from the previous controller update.
     *
     * SteeringController retains the effective-angle estimate
     * associated with that command.
     */
    float curvaturePerMm =
        MotionController_GetWheelReferenceCurvaturePerMm(
            controller);

    /*
     * Bicycle-model rear-wheel relationship:
     *
     *   dR - dL = W * kappa * ds
     *
     * Accumulate the relationship expected from the
     * commanded path curvature.
     */
    controller->desiredWheelTravelDifferenceMm +=
        controller->kinematics->rearTrackWidthMm *
        curvaturePerMm *
        deltaCentreMm;

    controller->leftTravelMm += deltaLeftMm;
    controller->rightTravelMm += deltaRightMm;

    controller->travelledDistanceMm =
        0.5f *
        (controller->leftTravelMm +
            controller->rightTravelMm);

    /*
     * Compare actual rear-wheel relationship with
     * the relationship required by the commanded path.
     *
     * Positive error:
     *     right traveled farther than required.
     *
     * Negative error:
     *     right traveled less far than required.
     */
    controller->wheelSyncErrorMm =
        (controller->rightTravelMm -
            controller->leftTravelMm)
        - controller->desiredWheelTravelDifferenceMm;
}


static void MotionController_BeginBraking(
    MotionController *controller)
{
    WheelSpeedController_SetTarget(controller->leftWheel, 0.0f);
    WheelSpeedController_SetTarget(controller->rightWheel, 0.0f);

    /*
     * Abort active profile
     */
    MotionProfile_Stop(&controller->motionProfile);
    controller->targetSpeedCps = 0.0f;

    /*
     * Wheel synchronisation no longer commands speed
     * corrections once braking begins.
     */
    controller->wheelSyncCorrectionCps = 0.0f;
    // We preserve first the wheelSyncErrorMm for diagnostics purpose.

    SteeringController_Centre(controller->steering);

    controller->stationarySamples = 0U;

    controller->mode = MOTIONCONTROLLER_BRAKING;
}


/**
 * Stops wheel speed controller and allow the robot
 * to coast once it is stationary.
 */
static void MotionController_FinishBraking(
    MotionController *controller)
{
    /*
     * Resynchronise the wheel speed controller encoder references
     * once more at the final stationary position.
     */
    WheelSpeedController_Stop(controller->leftWheel);
    WheelSpeedController_Stop(controller->rightWheel);

    SteeringController_Centre(controller->steering);

    controller->stationarySamples = 0U;
    controller->mode = MOTIONCONTROLLER_IDLE;
}

bool MotionController_Init(
    MotionController *controller,
    WheelSpeedController *leftWheel,
    WheelSpeedController *rightWheel,
    SteeringController *steering,
    ICM20948 *imu,
    const RobotKinematics *kinematics,
    float headingKp,
    float headingKi,
    float headingKd,
    float maxHeadingSteeringAngleRad,
    float wheelSyncKpCpsPerMm,
    float maxWheelSyncCorrectionCps,
	float motionAccelerationMmps2,
	float motionDecelerationMmps2)
{
	if (controller == NULL ||
	    leftWheel == NULL ||
	    rightWheel == NULL ||
	    leftWheel->motor == NULL ||
	    rightWheel->motor == NULL ||
	    steering == NULL ||
	    imu == NULL ||
	    kinematics == NULL)
    {
        return false;
    }

    if (kinematics->rearEncoderCountsPerRev == 0U ||
        kinematics->rearWheelDiameterMm <= 0.0f ||
        kinematics->wheelbaseMm <= 0.0f ||
        kinematics->rearTrackWidthMm <= 0.0f) {
        return false;
    }

    float minEffectiveAngleRad =
        SteeringController_GetMinEffectiveAngleRad(
            steering);

    float maxEffectiveAngleRad =
        SteeringController_GetMaxEffectiveAngleRad(
            steering);

    /*
     * Straight-line heading control must be able to correct
     * in either direction, so use only the range available
     * symmetrically about zero.
     */
    if (minEffectiveAngleRad >= 0.0f ||
        maxEffectiveAngleRad <= 0.0f)
    {
        return false;
    }

    float maxSymmetricSteeringAngleRad =
        fminf(
            -minEffectiveAngleRad,
            maxEffectiveAngleRad);

    if (maxHeadingSteeringAngleRad <= 0.0f ||
        maxHeadingSteeringAngleRad >
            maxSymmetricSteeringAngleRad)
    {
        return false;
    }

    if (wheelSyncKpCpsPerMm < 0.0f ||
        maxWheelSyncCorrectionCps < 0.0f)
    {
        return false;
    }

    *controller = (MotionController){0};

    controller->leftWheel = leftWheel;
    controller->rightWheel = rightWheel;
    controller->steering = steering;
    controller->imu = imu;
    controller->kinematics = kinematics;

    controller->wheelSyncKpCpsPerMm =
        wheelSyncKpCpsPerMm;

    controller->maxWheelSyncCorrectionCps =
        maxWheelSyncCorrectionCps;

    controller->mode = MOTIONCONTROLLER_IDLE;

    if (!MotionProfile_Init(
            &controller->motionProfile,
            motionAccelerationMmps2,
            motionDecelerationMmps2))
    {
        return false;
    }


    if (!PIDController_Init(
        &controller->headingPID,
        headingKp,
        headingKi,
        headingKd,
        -maxHeadingSteeringAngleRad,
		maxHeadingSteeringAngleRad))
    {
        return false;
    }

    return true;
}

/**
 * Common helper for the primitives. Motions
 * share majority of initial state configurations,
 * including the motion profile.
 */
static bool MotionController_BeginMotion(
	MotionController *controller,
	float distanceMm,
	float speedCps) {
    if (controller == NULL)
        return false;

    if (controller->mode != MOTIONCONTROLLER_IDLE)
        return false;

    if (speedCps <= 0.0f)
        return false;

    if (distanceMm == 0.0f)
        return true;

    controller->motionDirection = distanceMm > 0.0f ? 1 : -1;

    controller->targetDistanceMm = fabsf(distanceMm);

    /*
     * MotionProfile now responsible for determining the speed
     * across each control interval.
     *
     * Now the robot shall start at 0Cps
     */
    controller->maxSpeedCps = speedCps;
    controller->targetSpeedCps = 0.0f;

    /*
     * Enforcing uniform state representation for curved and
     * straight motions
     */
    controller->targetCurvaturePerMm = 0.0f;
    controller->targetSteeringAngleRad = 0.0f;

    float mmPerCount =
        MotionController_GetMmPerCount(controller);

    float maxSpeedMmps =
        speedCps * mmPerCount;

    if (!MotionProfile_Start(
            &controller->motionProfile,
            controller->targetDistanceMm,
            maxSpeedMmps))
    {
        return false;
    }

    /*
     * Straight-line heading is relative to the orientation
     * at the start of this command.
     */
    controller->yawDeg = 0.0f;

    /*
     * The desired wheel relationship starts at zero.
     * It may subsequently become non-zero while the
     * heading controller steers the robot back toward
     * the requested straight path.
     */
    controller->desiredWheelTravelDifferenceMm = 0.0f;
    controller->wheelSyncErrorMm = 0.0f;
    controller->wheelSyncCorrectionCps = 0.0f;

    PIDController_Reset(&controller->headingPID);

    MotionController_ResetOdometry(controller);

    WheelSpeedController_SetTarget(
        controller->leftWheel,
        controller->targetSpeedCps);

    WheelSpeedController_SetTarget(
        controller->rightWheel,
        controller->targetSpeedCps);

    return true;
}

bool MotionController_MoveStraight(
    MotionController *controller,
    float distanceMm,
    float speedCps)
{
    if (controller == NULL)
        return false;

    if (!MotionController_BeginMotion(
    		controller,
		distanceMm,
		speedCps))
    {
    		return false;
    }


    SteeringController_Centre(controller->steering);

    controller->mode = MOTIONCONTROLLER_STRAIGHT;

    return true;
}

bool MotionController_MoveArc(
    MotionController *controller,
    float distanceMm,
    float radiusMm,
    float speedCps)
{
    if (controller == NULL)
    {
        return false;
    }

    if (radiusMm == 0.0f)
    {
        return false;
    }

    float curvaturePerMm =
        1.0f / radiusMm;

    float steeringAngleRad =
        RobotKinematics_GetSteeringAngleRad(
            controller->kinematics,
            curvaturePerMm);

    /*
     * Reject paths outside the experimentally calibrated
     * effective steering range rather than allowing
     * SteeringController to silently clamp them.
     */
    float minSteeringAngleRad =
        SteeringController_GetMinEffectiveAngleRad(
            controller->steering);

    float maxSteeringAngleRad =
        SteeringController_GetMaxEffectiveAngleRad(
            controller->steering);

    if (steeringAngleRad < minSteeringAngleRad ||
        steeringAngleRad > maxSteeringAngleRad)
    {
        return false;
    }

    /*
     * Use the existing profiled-motion setup.
     */
    if (!MotionController_BeginMotion(
        controller,
        distanceMm,
        speedCps))
    {
        return false;
    }

    /*
     * Replace the straight-path geometry with the
     * requested constant-curvature geometry.
     */
    controller->targetCurvaturePerMm =
        curvaturePerMm;

    controller->targetSteeringAngleRad =
        steeringAngleRad;

    controller->mode = MOTIONCONTROLLER_ARC;

    return true;
}

bool MotionController_Brake(MotionController *controller) {
	if (controller == NULL)
	{
        return false;
    }

    if (controller->mode == MOTIONCONTROLLER_BRAKING) {
        return true;		// no-op
    }

    MotionController_BeginBraking(controller);

    return true;
}

static bool MotionController_UpdateYawEstimate(MotionController *controller, float dt) {
    ICM20948Measurement measurement;
    if (!ICM20948_ReadMeasurement(controller->imu, &measurement)) {
        return false;
    }
    /*
     * Relative yaw:
     *
     *     yaw[k+1] = yaw[k] + gyroZ * dt
     */
    controller->yawDeg += measurement.gyroDps.z * dt;

    return true;
}

static float MotionController_Clamp(
    float value,
    float min,
    float max)
{
    if (value > max)
        return max;

    if (value < min)
        return min;

    return value;
}


static void MotionController_UpdateWheelSynchronisation(
    MotionController *controller)
{
	float curvaturePerMm =
	    MotionController_GetWheelReferenceCurvaturePerMm(
	        controller);

    float leftBaseTargetCps;
    float rightBaseTargetCps;

    RobotKinematics_GetRearWheelSpeedTargets(
        controller->kinematics,
        controller->targetSpeedCps,
        curvaturePerMm,
        &leftBaseTargetCps,
        &rightBaseTargetCps);

    /*
     * --------------------------------------------------------
     * RELATIONSHIP FEEDBACK
     * --------------------------------------------------------
     *
     * e_sync =
     *     actual(R-L) - desired(R-L)
     */
    controller->wheelSyncErrorMm =
        (controller->rightTravelMm -
            controller->leftTravelMm)
        - controller->desiredWheelTravelDifferenceMm;

    /*
     * P-only outer synchronization controller.
     *
     * Units:
     *     [CPS/mm] * [mm] = [CPS]
     */
    float correctionCps =
        controller->wheelSyncKpCpsPerMm *
        controller->wheelSyncErrorMm;

    /*
     * Prevent the synchronizer from overwhelming the
     * geometric base targets or reversing one wheel.
     *
     * Once steering makes the two base targets unequal,
     * the smaller base-target magnitude is the limiting one.
     */
    float smallestBaseTargetMagnitudeCps =
        fminf(
            fabsf(leftBaseTargetCps),
            fabsf(rightBaseTargetCps));

    float correctionLimitCps =
        fminf(
            controller->maxWheelSyncCorrectionCps,
            smallestBaseTargetMagnitudeCps);

    correctionCps =
        MotionController_Clamp(
            correctionCps,
            -correctionLimitCps,
            correctionLimitCps);

    controller->wheelSyncCorrectionCps =
        correctionCps;

    /*
     * Geometry determines the nominal left/right relationship.
     * The synchronizer only corrects deviations from it.
     */
    WheelSpeedController_SetTarget(
        controller->leftWheel,
        leftBaseTargetCps + correctionCps);

    WheelSpeedController_SetTarget(
        controller->rightWheel,
        rightBaseTargetCps - correctionCps);
}

bool MotionController_Update(
    MotionController *controller,
    float dt)
{
    if (controller == NULL || dt <= 0.0f)
        return false;

    if (controller->mode == MOTIONCONTROLLER_IDLE)
        return true;

    /*
     * Keep odometry running during both normal motion and
     * braking, so endpoint overshoot is captured.
     */
    MotionController_UpdateOdometry(controller);

    /*
     * --------------------------------------------------------
     * STOPPING
     * --------------------------------------------------------
     */
    if (controller->mode == MOTIONCONTROLLER_BRAKING)
    {
        /*
         * Target is already zero. Update() still measures
         * encoder velocity before handling the zero target,
         * which lets us determine when the chassis has
         * actually stopped.
         */
        WheelSpeedController_Update(
            controller->leftWheel,
            dt);

        WheelSpeedController_Update(
            controller->rightWheel,
            dt);

        /**
         * Continue to measure yaw,
         * Could be useful for inspecting yaw
         * caused by asymmetric braking.
         */
        MotionController_UpdateYawEstimate(controller, dt);

        bool leftStationary = WheelSpeedController_IsStationary(
            controller->leftWheel);

        bool rightStationary = WheelSpeedController_IsStationary(
            controller->rightWheel);

        if (leftStationary && rightStationary)
        {
            controller->stationarySamples++;

            if (controller->stationarySamples >=
                MOTIONCONTROLLER_STOP_STABLE_SAMPLES)
            {
                MotionController_FinishBraking(controller);
            }
        }
        else
        {
            controller->stationarySamples = 0U;
        }

        return true;
    }

    /*
     * --------------------------------------------------------
     * STRAIGHT
     * --------------------------------------------------------
     */

    float progressMm =
        (float)controller->motionDirection *
        controller->travelledDistanceMm;

    float targetSpeedMmps =
        MotionProfile_Update(
            &controller->motionProfile,
            progressMm,
            dt);

    /*
     * MotionProfile becomes inactive once the requested
     * distance has been reached.
     *
     * Enter the existing braking state so that zero velocity
     * is actively enforced until both wheels are stationary.
     */
    if (!MotionProfile_IsActive(
            &controller->motionProfile))
    {
        MotionController_BeginBraking(controller);
        return true;
    }

    /*
     * Convert vehicle-speed magnitude back to encoder CPS
     * and restore the commanded motion direction.
     */
    float mmPerCount =
        MotionController_GetMmPerCount(controller);

    controller->targetSpeedCps =
        (float)controller->motionDirection *
        targetSpeedMmps /
        mmPerCount;

    if (!MotionController_UpdateYawEstimate(controller, dt)) {
        /*
         * Heading feedback has failed.
         * Stop rather than continuing open-loop.
         */
        MotionController_Stop(controller);
        return false;
    }

    float targetSteeringAngleRad;

    if (controller->mode == MOTIONCONTROLLER_STRAIGHT)
    {
        /*
         * Straight-line heading feedback.
         * Desired relative yaw = 0.
         */
        float headingErrorRad =
            -controller->yawDeg *
            (MOTION_PI / 180.0f);

        float steeringAngleCorrectionRad =
            PIDController_Update(
                &controller->headingPID,
                headingErrorRad,
                dt);

        /*
         * Reverse motion reverses the yaw response
         * produced by a given physical steering angle.
         */
        targetSteeringAngleRad =
            steeringAngleCorrectionRad *
            (float)controller->motionDirection;
    }
    else if (controller->mode == MOTIONCONTROLLER_ARC)
    {
        /*
         * Desired relative heading for a constant-curvature path:
         *
         *     psi_d = kappa * s
         *
         * travelledDistanceMm is signed, so reverse motion
         * naturally produces the corresponding opposite yaw.
         */
        float desiredYawRad =
            controller->targetCurvaturePerMm *
            controller->travelledDistanceMm;

        float measuredYawRad =
            controller->yawDeg *
            (MOTION_PI / 180.0f);

        float headingErrorRad =
            desiredYawRad -
            measuredYawRad;

        /*
         * Heading feedback is a correction around the
         * geometric feedforward steering angle.
         */
        float steeringAngleCorrectionRad =
            PIDController_Update(
                &controller->headingPID,
                headingErrorRad,
                dt);

        /*
         * As for straight motion, reversing the vehicle reverses
         * the yaw response of a given steering correction.
         */
        steeringAngleCorrectionRad *=
            (float)controller->motionDirection;

        targetSteeringAngleRad =
            controller->targetSteeringAngleRad +
            steeringAngleCorrectionRad;

        /*
         * The PID output itself is limited, but feedforward +
         * feedback can still exceed the available effective
         * steering envelope.
         */
        targetSteeringAngleRad =
            MotionController_Clamp(
                targetSteeringAngleRad,
                SteeringController_GetMinEffectiveAngleRad(
                    controller->steering),
                SteeringController_GetMaxEffectiveAngleRad(
                    controller->steering));
    }
    else
    {
        MotionController_Stop(controller);
        return false;
    }

    SteeringController_SetEffectiveAngleRad(
        controller->steering,
        targetSteeringAngleRad,
        dt);

    /*
     * --------------------------------------------------------
     * REAR-WHEEL SYNCHRONISATION
     * --------------------------------------------------------
     *
     * Convert accumulated wheel relationship error into
     * corrected left/right speed targets.
     */
    MotionController_UpdateWheelSynchronisation(controller);

    /*
     * Low-level speed loops remain responsible for motor PWM.
     */
    WheelSpeedController_Update(
        controller->leftWheel,
        dt);

    WheelSpeedController_Update(
        controller->rightWheel,
        dt);

    return true;
}

void MotionController_Stop(
    MotionController *controller)
{
    if (controller == NULL)
        return;

    WheelSpeedController_Stop(controller->leftWheel);
    WheelSpeedController_Stop(controller->rightWheel);

    SteeringController_Centre(controller->steering);

    PIDController_Reset(&controller->headingPID);

    controller->mode = MOTIONCONTROLLER_IDLE;
}


bool MotionController_IsBusy(
    const MotionController *controller)
{
    if (controller == NULL)
        return false;

    return controller->mode != MOTIONCONTROLLER_IDLE;
}
