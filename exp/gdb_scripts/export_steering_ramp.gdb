# Source with the target suspended and GDB cwd at the project root.
# Use a fresh filename: export appends if the file already exists.
set print elements unlimited
set print repeats unlimited
init-if-undefined $steering_ramp_battery_v = 0.0

define steering-ramp-export
    set logging file $arg0
    set logging overwrite off
    set logging enabled on
    echo === OPEN-LOOP STEERING RAMP BATCH ===\n
    p $steering_ramp_battery_v
    p steeringRampFinished
    p steeringRampBatchComplete
    p steeringRampLogPeriodMs
    p steeringRampTrials
    p steeringRampProfiles
    p steeringRampResultCount
    set $ramp_results_capacity = sizeof(steeringRampResults) / sizeof(steeringRampResults[0])
    if steeringRampResultCount > 0 && steeringRampResultCount <= $ramp_results_capacity
        p steeringRampResults[0] @ steeringRampResultCount
    else
        echo No results, or invalid count: result array was not read.\n
    end
    echo === ACTUAL CONFIGURATION AND CALIBRATION ===\n
    p steeringRampMotionConfig
    # File qualification selects this harness's live fixture. An unused C
    # pointer exported only for GDB can be removed by --gc-sections.
    set $ramp_controller = &'SteeringRampResponseTest.c'::fixture.motionController
    if $ramp_controller->initialized
        p *$ramp_controller->config->kinematics
        p *$ramp_controller->config->arcConfig
        set $ramp_arc = $ramp_controller->config->arcConfig
        if $ramp_arc->positivePointCount > 0 && $ramp_arc->positivePointCount <= 16
            p $ramp_arc->positivePoints[0] @ $ramp_arc->positivePointCount
        end
        if $ramp_arc->negativePointCount > 0 && $ramp_arc->negativePointCount <= 16
            p $ramp_arc->negativePoints[0] @ $ramp_arc->negativePointCount
        end
        p $ramp_controller->leftWheel->pid
        p $ramp_controller->rightWheel->pid
        p $ramp_controller->leftWheel->calibration
        p $ramp_controller->rightWheel->calibration
        p $ramp_controller->pathProfile
    end
    echo === SAMPLES (TIME RESTARTS AT EACH TRIAL) ===\n
    p steeringRampLogCount
    set $ramp_log_capacity = sizeof(steeringRampLog) / sizeof(steeringRampLog[0])
    if steeringRampLogCount > 0 && steeringRampLogCount <= $ramp_log_capacity
        p steeringRampLog[0] @ steeringRampLogCount
    else
        echo No samples, or invalid count: sample array was not read.\n
    end
    echo === EXPORT COMPLETE ===\n
    set logging enabled off
end

document steering-ramp-export
Export the suspended three-trial batch to the supplied filename.
Set $steering_ramp_battery_v to the measured battery voltage first.
Example: steering-ramp-export steering_ramp_run01.txt
No shell commands, target resume, or motion. Use a new filename each time.
end
