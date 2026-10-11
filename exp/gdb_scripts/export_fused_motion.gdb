# Source with the target suspended and GDB cwd at the project root.
# Usage: mctrl-fusion-export fusion_run01.txt
# Choose a fresh filename for each run (logging appends if it already exists).

set print elements unlimited
set print repeats unlimited
init-if-undefined $mctrl_fusion_battery_v = 0.0

define mctrl-fusion-export
    set logging file $arg0
    set logging overwrite off
    set logging enabled on
    echo === FUSED MOTION EXPERIMENT ===\n
    p $mctrl_fusion_battery_v
    p motionSequenceFusionTestPassed
    p motionSequenceFusionTestStatus
    p motionSequenceFusionTestTimedOut
    p motionSequenceFusionTestCancelled
    p motionSequenceFusionTestLogTruncated
    p motionSequenceFusionTestElapsedMs
    echo === Selected experiment and sampling ===\n
    p motionSequenceFusionTestExperiment
    p motionSequenceFusionTestLogPeriodMs
    if motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_SINGLE_TUNING
        echo === Single tuning run (index 0; matched and fused) ===\n
    else
        if motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK1_GAINS
            echo === A/B tuning comparison (0=reduced, 1=Task1; both matched and fused) ===\n
        else
            if motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_REFERENCE_FILTER
                echo === A/B reference filter comparison (0=existing, 1=matched; both fused) ===\n
            else
                echo === A/B Task 2 comparison (0=overlap fusion, 1=standalone stopped; yaw-priority arc exits) ===\n
            end
        end
    end
    p motionSequenceFusionTestResultCount
    set $fusion_result_capacity = sizeof(motionSequenceFusionTestResults) / sizeof(motionSequenceFusionTestResults[0])
    if motionSequenceFusionTestResultCount > 0 && motionSequenceFusionTestResultCount <= $fusion_result_capacity
        p motionSequenceFusionTestResults[0] @ motionSequenceFusionTestResultCount
    else
        echo No results, or invalid count: result array was not read.\n
    end
    p motionSequenceFusionTestComparisonValid
    p motionSequenceFusionTestSavedTimeMs
    p motionSequenceFusionTestSavedPercent
    if motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK2_TIMING
        echo === A prepared fused plan (actual blends and speed caps) ===\n
        p motionSequenceFusionTestFusedPlan
    end
    echo === Standalone stopped arc exits (including braking) ===\n
    p motionSequenceFusionTestArcExitCount
    set $fusion_arc_exit_capacity = sizeof(motionSequenceFusionTestArcExits) / sizeof(motionSequenceFusionTestArcExits[0])
    if motionSequenceFusionTestArcExitCount > 0 && motionSequenceFusionTestArcExitCount <= $fusion_arc_exit_capacity
        p motionSequenceFusionTestArcExits[0] @ motionSequenceFusionTestArcExitCount
    else
        echo No arc exits, or invalid count: arc exit array was not read.\n
    end
    echo === Plan and final sequence state ===\n
    p motionSequenceFusionTestSequence
    if motionSequenceFusionTestSequence.controller != 0
        p *motionSequenceFusionTestSequence.controller->config
        if motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_TASK2_TIMING
            echo === Frozen arc feedforward calibration ===\n
            p *motionSequenceFusionTestSequence.controller->config->arcConfig
            p motionSequenceFusionTestSequence.controller->config->arcConfig->positivePoints[0] @ motionSequenceFusionTestSequence.controller->config->arcConfig->positivePointCount
            p motionSequenceFusionTestSequence.controller->config->arcConfig->negativePoints[0] @ motionSequenceFusionTestSequence.controller->config->arcConfig->negativePointCount
        end
        p *motionSequenceFusionTestSequence.controller->steering->calibration
        if motionSequenceFusionTestSequence.controller->leftWheel != 0
            p motionSequenceFusionTestSequence.controller->leftWheel->pid
        end
        if motionSequenceFusionTestSequence.controller->rightWheel != 0
            p motionSequenceFusionTestSequence.controller->rightWheel->pid
        end
        echo === Terminal heading policy and result ===\n
        p motionSequenceFusionTestSequence.controller->pathProfile
        p motionSequenceFusionTestSequence.controller->pathFinalSample
        p motionSequenceFusionTestSequence.controller->terminalYawPredictedReached
        p motionSequenceFusionTestSequence.controller->terminalDistanceLimitReached
    end
    echo === Samples ===\n
    p motionSequenceFusionTestLogCount
    set $fusion_capacity = sizeof(motionSequenceFusionTestLog) / sizeof(motionSequenceFusionTestLog[0])
    if motionSequenceFusionTestLogCount > 0 && motionSequenceFusionTestLogCount <= $fusion_capacity
        p motionSequenceFusionTestLog[0] @ motionSequenceFusionTestLogCount
    else
        echo No samples, or invalid count: sample array was not read.\n
    end
    echo === EXPORT COMPLETE ===\n
    set logging enabled off
end

document mctrl-fusion-export
Export the suspended fusion harness to the supplied filename.
Example: mctrl-fusion-export fusion_run01.txt
Set $mctrl_fusion_battery_v to the manually measured battery voltage first.
Use a new filename for each trial. No shell commands, target resume or motion.
end
