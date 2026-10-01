# Load after ELF symbols, with GDB's working directory set to the project root.
# Windows PowerShell generates a fresh filename; GDB exports the target data.

set $mctrl_straight_ff_battery_v = 0.0
set $mctrl_straight_ff_export_ok = 0

define mctrl-straight-ff-export
    set $mctrl_straight_ff_export_ok = 0
    shell powershell.exe -NoProfile -ExecutionPolicy Bypass -File "exp/gdb_scripts/prepare_straight_feedforward_export.ps1"
    if $_shell_exitcode == 0
        source exp/logs/straight_feedforward/export-current.gdb
    else
        echo ERROR: export preparation failed; no successful export was reported.\n
    end
end

document mctrl-straight-ff-export
Export the current straight feedforward test results and log to a new timestamped file.
The target must be suspended. This also works for a manually interrupted run.
Success is reported only after the entire export finishes.
end

define mctrl-straight-ff-battery
    set $mctrl_straight_ff_battery_v = $arg0
end

document mctrl-straight-ff-battery
Record a manually measured battery voltage in subsequent exports.
Usage: mctrl-straight-ff-battery 11.95
Zero (the default) means unknown.
end

# Re-sourcing replaces only our breakpoint, preserving the user's breakpoints.
init-if-undefined $mctrl_straight_ff_export_bp = 0
if $mctrl_straight_ff_export_bp > 0
    delete $mctrl_straight_ff_export_bp
end

# All final globals are set and motion/braking has finished at this function.
# Normal completion, command rejection, and motion timeout reach this point.
hbreak MotionControllerStraightFeedforwardTest_ShowFinal
set $mctrl_straight_ff_export_bp = $bpnum
commands
    silent
    mctrl-straight-ff-export
    echo Straight feedforward finished; target remains suspended for review.\n
end

echo Straight feedforward auto-export armed. Output folder: exp/logs/straight_feedforward/\n
