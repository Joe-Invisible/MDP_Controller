# export_steering_geometry_iteration.gdb
#
# Works after any completed continuation round or at final completion.
#
# Output:
#     steer_geometry_iteration.txt
#
# STM32CubeIDE GDB console:
#     source export_steering_geometry_iteration.gdb

set pagination off
set print elements 0
set print repeats 0
set print pretty on

set logging file steer_geometry_iteration.txt
set logging overwrite on
set logging redirect on
set logging enabled on

printf "=== STEERING GEOMETRY ITERATIVE CALIBRATION ===\n\n"

printf "--- Experiment info ---\n"
x/s g_steeringGeometryIterationInfo

printf "\n--- Progress ---\n"
p g_steeringGeometryIterationCompletedRounds
p g_steeringGeometryIterationResultCount
p g_steeringGeometryIterationUnresolvedCount

printf "\n--- Point solver state [0=INC,1=DEC][commandIndex] ---\n"
printf "commandIndex 0..12 = -12,-10,-8,-6,-4,-2,0,2,4,6,8,10,12\n"
p g_steeringGeometryIterationPointState

printf "\n--- Converged mask [0=INC,1=DEC] ---\n"
p g_steeringGeometryIterationConverged

printf "\n--- Continuation iteration count per point ---\n"
p g_steeringGeometryIterationCount

printf "\n--- Candidate curvature table ---\n"
p g_steeringGeometryIterationCurvaturePerMm

printf "\n--- Candidate effective-angle table ---\n"
p g_steeringGeometryIterationAngleRad

printf "\n--- Chronological continuation runs ---\n"
p g_steeringGeometryIterationResults

printf "\n=== END ITERATIVE CALIBRATION EXPORT ===\n"

set logging enabled off
set logging redirect off

printf "Export complete: steer_geometry_iteration.txt\n"
