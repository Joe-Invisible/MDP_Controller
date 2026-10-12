# Source with the robot suspended; choose a fresh export filename.
set print elements unlimited
set print repeats unlimited
init-if-undefined $task2_battery_v = 0.0

define task2-motion-export
    set logging file $arg0
    set logging overwrite off
    set logging enabled on
    echo === TASK 2 STANDALONE MOTION ===\n
    p $task2_battery_v
    p task2MotionTestConfig
    p task2MotionTestStartupRange
    p task2MotionTestState
    p task2MotionTestResult
    p task2MotionTestRoute
    p task2MotionTestControllerConfig
    if task2MotionTestControllerConfig.arcConfig != 0
        p *task2MotionTestControllerConfig.arcConfig
        p task2MotionTestControllerConfig.arcConfig->positivePoints[0] @ task2MotionTestControllerConfig.arcConfig->positivePointCount
        p task2MotionTestControllerConfig.arcConfig->negativePoints[0] @ task2MotionTestControllerConfig.arcConfig->negativePointCount
    end
    echo Steps: Approach O1 -> O1 depart -> O1 bypass -> O1 centre -> Approach O2 -> O2 depart -> O2 back -> Return park\n
    p task2MotionTestStepResults
    p task2MotionTestLogCount
    set $task2_sample_capacity = sizeof(task2MotionTestLog) / sizeof(task2MotionTestLog[0])
    if task2MotionTestLogCount > 0 && task2MotionTestLogCount <= $task2_sample_capacity
        p task2MotionTestLog[0] @ task2MotionTestLogCount
    else
        echo No samples, or invalid count: sample array was not read.\n
    end
    echo === EXPORT COMPLETE ===\n
    set logging enabled off
end

document task2-motion-export
Export the suspended standalone Task 2 test, including route, range and samples.
Example: task2-motion-export task2_run01.txt
end
