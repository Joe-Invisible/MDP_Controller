# Load after ELF symbols, with GDB's working directory set to the project root.
# Windows PowerShell generates a fresh filename; GDB exports the target data.

set $mctrl_seq_battery_v = 0.0
set $mctrl_seq_export_ok = 0

define mctrl-seq-export
    set $mctrl_seq_export_ok = 0
    shell powershell.exe -NoProfile -ExecutionPolicy Bypass -File "exp/gdb_scripts/prepare_sequence_export.ps1"
    if $_shell_exitcode == 0
        source exp/logs/sequence/export-current.gdb
    else
        echo ERROR: export preparation failed; no successful export was reported.\n
    end
end

document mctrl-seq-export
Export the current sequence results and log to a new timestamped file.
The target must be suspended. This also works for a manually interrupted run.
Success is reported only after the entire export finishes.
end

define mctrl-seq-battery
    set $mctrl_seq_battery_v = $arg0
end

document mctrl-seq-battery
Record a manually measured battery voltage in subsequent exports.
Usage: mctrl-seq-battery 11.95
Zero (the default) means unknown.
end

# Re-sourcing replaces only our breakpoint, preserving the user's breakpoints.
init-if-undefined $mctrl_seq_export_bp = 0
if $mctrl_seq_export_bp > 0
    delete $mctrl_seq_export_bp
end

# All final globals are set and motion/braking has finished at this function.
# Normal completion, command rejection, and motion timeout reach this point.
hbreak MotionControllerSequenceTest_ShowFinal
set $mctrl_seq_export_bp = $bpnum
commands
    silent
    mctrl-seq-export
    echo Sequence finished; target remains suspended for review.\n
end

echo Sequence auto-export armed. Output folder: exp/logs/sequence/\n
