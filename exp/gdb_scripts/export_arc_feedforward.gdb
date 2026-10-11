# Source with the target suspended at MotionControllerArcTestFinished.
# Choose a fresh filename: logging appends if it already exists.
set print elements unlimited
set print repeats unlimited
init-if-undefined $arc_ff_battery_v = 0.0

define arc-ff-export
    set logging file $arg0
    set logging overwrite off
    set logging enabled on
    echo === OPEN-LOOP ARC FEEDFORWARD CALIBRATION ===\n
    p $arc_ff_battery_v
    echo Distance completion; use matched pre-braking exit yaw and travel.\n
    p motionControllerArcTestRequestedDistanceMm
    p motionControllerArcTestRequestedRadiusMm
    p motionControllerArcTestRequestedSpeedCps
    p motionControllerArcTestPreparedRawCommand
    p arcTestMotionConfig
    p arcMotionConfig
    p motionControllerArcTestCommandAccepted
    p motionControllerArcTestTimedOut
    echo === Matched pre-braking exit measurements ===\n
    p motionControllerArcTestArcExitCaptured
    p motionControllerArcTestArcExitTimeMs
    p motionControllerArcTestArcExitDistanceMm
    p motionControllerArcTestArcExitYawDeg
    p motionControllerArcTestArcExitIdealYawDeg
    p motionControllerArcTestArcExitLeftDistanceMm
    p motionControllerArcTestArcExitRightDistanceMm
    echo === Final stopped measurements ===\n
    p motionControllerArcTestFinalDistanceMm
    p motionControllerArcTestFinalYawDeg
    p motionControllerArcTestFinalLeftDistanceMm
    p motionControllerArcTestFinalRightDistanceMm
    echo === Timeout diagnostics ===\n
    p motionControllerArcTestTimeoutMode
    p motionControllerArcTestTimeoutDistanceMm
    p motionControllerArcTestTimeoutTargetDistanceMm
    p motionControllerArcTestTimeoutYawDeg
    echo === Samples ===\n
    p motionControllerArcTestLogCount
    set $arc_ff_capacity = sizeof(motionControllerArcTestLog) / sizeof(motionControllerArcTestLog[0])
    if motionControllerArcTestLogCount > 0 && motionControllerArcTestLogCount <= $arc_ff_capacity
        p motionControllerArcTestLog[0] @ motionControllerArcTestLogCount
    else
        echo Empty or invalid log count: sample array was not read.\n
    end
    echo === EXPORT COMPLETE ===\n
    set logging enabled off
end
