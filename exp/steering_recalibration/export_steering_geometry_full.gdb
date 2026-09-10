# export_steering_geometry_full.gdb
#
# Run this after all 52 runs have completed and the test is sitting at:
#     "STEER CAL DONE / EXPORT"
#
# Output:
#     steer_geometry_full.txt
#
# Use from the STM32CubeIDE GDB console:
#     source export_steering_geometry_full.gdb

set pagination off
set print elements 0
set print repeats 0
set print pretty on

set logging file steer_geometry_full.txt
set logging overwrite on
set logging redirect on
set logging enabled on

printf "=== STEERING GEOMETRY CALIBRATION: FULL EXPORT ===\n\n"

printf "--- Experiment info ---\n"
x/s g_steeringGeometryCalibrationInfo

printf "\n--- Result count ---\n"
p g_steeringGeometryCalibrationResultCount

printf "\n--- Scout curvature table [sweep][commandIndex] ---\n"
printf "[0] = increasing branch, [1] = decreasing branch\n"
printf "commandIndex 0..12 = raw command -12,-10,-8,-6,-4,-2,0,2,4,6,8,10,12\n"
p g_steeringGeometryCalibrationScoutCurvaturePerMm

printf "\n--- Refined effective-angle table [sweep][commandIndex] ---\n"
printf "[0] = increasing branch, [1] = decreasing branch\n"
printf "commandIndex 0..12 = raw command -12,-10,-8,-6,-4,-2,0,2,4,6,8,10,12\n"
p g_steeringGeometryCalibrationRefinedAngleRad

printf "\n--- All chronological calibration results ---\n"
p g_steeringGeometryCalibrationResults

printf "\n=== END FULL EXPORT ===\n"

set logging enabled off
set logging redirect off

printf "Export complete: steer_geometry_full.txt\n"
