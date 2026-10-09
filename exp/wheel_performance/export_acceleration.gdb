# Halt at WheelPerformanceTest_Complete after the entire sweep.
if wheelPerformanceRunConfig.experiment == 2 && wheelPerformanceStatus != 1
set pagination off
set print pretty off
set print elements unlimited
set print repeats 0
set max-value-size unlimited
set logging file wheel_acceleration.txt
set logging overwrite on
set logging enabled on
printf "WHEEL_PERFORMANCE_V1\n"
printf "CONFIG_BEGIN\n"
p wheelPerformanceRunConfig
printf "CONFIG_END\n"
printf "METADATA_BEGIN\n"
p wheelPerformanceStatus
p wheelPerformanceMmPerCount
p wheelPerformanceTrialCount
p wheelPerformanceTraceCount
printf "METADATA_END\n"
printf "TRIALS_BEGIN\n"
if wheelPerformanceTrialCount > 0
  p wheelPerformanceTrials[0] @ wheelPerformanceTrialCount
end
printf "TRIALS_END\n"
printf "TRACE_BEGIN\n"
if wheelPerformanceTraceCount > 0
  p wheelPerformanceTrace[0] @ wheelPerformanceTraceCount
end
printf "TRACE_END\n"
set logging enabled off
else
  echo Cannot export: wrong experiment or sweep still running.\n
end
