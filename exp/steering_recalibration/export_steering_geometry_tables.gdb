# export_steering_geometry_tables.gdb
#
# Compact export containing only the result count and the two final tables.
#
# Output:
#     steer_geometry_tables.txt

set pagination off
set print elements 0
set print repeats 0
set print pretty on

set logging file steer_geometry_tables.txt
set logging overwrite on
set logging redirect on
set logging enabled on

printf "=== STEERING GEOMETRY CALIBRATION TABLES ===\n\n"

x/s g_steeringGeometryCalibrationInfo

printf "\nResult count:\n"
p g_steeringGeometryCalibrationResultCount

printf "\nScout curvature [0=INC,1=DEC], commandIndex -12..+12 step 2:\n"
p g_steeringGeometryCalibrationScoutCurvaturePerMm

printf "\nRefined effective angle [0=INC,1=DEC], commandIndex -12..+12 step 2:\n"
p g_steeringGeometryCalibrationRefinedAngleRad

set logging enabled off
set logging redirect off

printf "Export complete: steer_geometry_tables.txt\n"
