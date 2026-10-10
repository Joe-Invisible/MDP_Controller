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
    if motionSequenceFusionTestExperiment == MOTION_SEQUENCE_FUSION_TEST_REFERENCE_FILTER
        echo === A/B reference filter comparison (0=existing, 1=matched; both fused) ===\n
    else
        echo === A/B timing comparison (0=fused, 1=stopped) ===\n
    end
    p motionSequenceFusionTestResultCount
    p motionSequenceFusionTestResults
    p motionSequenceFusionTestComparisonValid
    p motionSequenceFusionTestSavedTimeMs
    p motionSequenceFusionTestSavedPercent
    echo === Plan and final sequence state ===\n
    p motionSequenceFusionTestSequence
    if motionSequenceFusionTestSequence.controller != 0
        p *motionSequenceFusionTestSequence.controller->config
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
