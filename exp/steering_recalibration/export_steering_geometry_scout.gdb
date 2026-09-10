# export_steering_geometry_scout.gdb
#
# Run this after the 26 SCOUT runs, while the firmware is paused at:
#     "SCOUT COMPLETE / SW1 -> REFINED"
#
# Output:
#     steer_geometry_scout.txt
#
# Use from the STM32CubeIDE GDB console:
#     source export_steering_geometry_scout.gdb

set pagination off
set print elements 0
set print repeats 0
set print pretty on

set logging file steer_geometry_scout.txt
set logging overwrite on
set logging redirect on
set logging enabled on

printf "=== STEERING GEOMETRY CALIBRATION: SCOUT CHECKPOINT ===\n\n"

printf "--- Experiment info ---\n"
x/s g_steeringGeometryCalibrationInfo

printf "\n--- Result count ---\n"
p g_steeringGeometryCalibrationResultCount

printf "\n--- Scout curvature table [sweep][commandIndex] ---\n"
printf "[0] = increasing branch, [1] = decreasing branch\n"
printf "commandIndex 0..12 = raw command -12,-10,-8,-6,-4,-2,0,2,4,6,8,10,12\n"
p g_steeringGeometryCalibrationScoutCurvaturePerMm

printf "\n--- Chronological results collected so far ---\n"
p g_steeringGeometryCalibrationResults

printf "\n=== END SCOUT EXPORT ===\n"

set logging enabled off
set logging redirect off

printf "Export complete: steer_geometry_scout.txt\n"
