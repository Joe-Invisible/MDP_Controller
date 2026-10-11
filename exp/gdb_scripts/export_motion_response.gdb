# Source while suspended at MotionResponseTestFinished; cwd = project root.
set print elements unlimited
set print repeats unlimited
set max-value-size unlimited
init-if-undefined $motion_response_battery_v = 0.0

define motion-response-export
    set logging file $arg0
    set logging overwrite off
    set logging enabled on
    echo === RESPONSE-BASED ENDPOINT VALIDATION ===\n
    p $motion_response_battery_v
    p motionResponseFinished
    p motionResponseResult
    p motionResponsePlan
    echo === ACTUAL CONFIGURATION ===\n
    p motionResponseConfig
    set $response_controller = &'MotionResponseTest.c'::fixture.motionController
    if $response_controller->initialized
        p *$response_controller->config->kinematics
        p $response_controller->leftWheel->pid
        p $response_controller->rightWheel->pid
        p $response_controller->leftWheel->calibration
        p $response_controller->rightWheel->calibration
        p $response_controller->pathProfile
    end
    echo === SAMPLES (10 MS; ONE RUN) ===\n
    p motionResponseLogCount
    set $response_capacity = sizeof(motionResponseLog) / sizeof(motionResponseLog[0])
    if motionResponseLogCount > 0 && motionResponseLogCount <= $response_capacity
        p motionResponseLog[0] @ motionResponseLogCount
    else
        echo No samples, or invalid count: sample array was not read.\n
    end
    echo === EXPORT COMPLETE ===\n
    set logging enabled off
end

document motion-response-export
Export this suspended endpoint validation to a fresh filename.
Set $motion_response_battery_v to the measured battery voltage first.
Example: motion-response-export motion_response_run01.txt
No target resume or motion. Existing files are appended; choose a new filename.
end
