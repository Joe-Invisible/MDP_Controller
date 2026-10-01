# Automatic sequence-test exports in CubeIDE on Windows

The shared `MDP_Controller Debug.launch` loads `auto_export_sequence.gdb` after
loading the ELF. Run a sequence normally using SW1. When the sequence finishes,
GDB suspends at `MotionControllerSequenceTest_ShowFinal`, exports all results and
samples, and prints `EXPORTED: <absolute path>`. The target stays suspended so
you can review the run or end the session. Resume if you want the final OLED
screen to appear. No motion-control or firmware changes are required.

Exports are written under the project at `exp/logs/sequence/`, for example:

```text
mctrl_seq_20261001_183017_125_a81c207d.txt
```

Each export gets a new filename, including repeated exports of the same run.
The console is not flooded with the sample array. Logs include the sequence
commands, controller configuration and feedforward tables, loaded ELF details,
all original result/sample fields, export time with timezone, and checkout
revision/change status. Export time is not the data-collection start time, and
the checkout revision does not establish which revision was flashed. Keep
recording acquisition time and mechanical changes separately.

## One-time setup

Use the shared launch configuration from this checkout. In **Run > Debug
Configurations > STM32 C/C++ Application > MDP_Controller Debug > Startup**,
the post-load **Run Commands** should contain:

```gdb
cd ${workspace_loc:/MDP_Controller}
source exp/gdb_scripts/auto_export_sequence.gdb
```

These lines are already saved in the shared `.launch` file. If your CubeIDE uses
a separate/private launch configuration, copy them there once. Keep the source
command in the post-load Run Commands, rather than before ELF symbols load.
If the workspace project has a different name, adjust the workspace variable.
At startup the console must show `Sequence auto-export armed` before running
the test. GDB uses built-in commands and Windows PowerShell 5.1; no Codex app,
Python-enabled GDB, or separately installed Python is needed.

The completion breakpoint uses one hardware breakpoint. Re-loading the script
replaces its own breakpoint; it does not remove your other breakpoints. Keep
the completion breakpoint enabled. Use the Debug build with the sequence test
included; its static final-display function must be present in the ELF. Remove
the `source` line when debugging firmware that excludes this test.

## Optional console commands

Set measured battery voltage once per session, if available:

```gdb
mctrl-seq-battery 11.95
```

For a run you interrupted early, suspend the target and export its current
buffers with:

```gdb
mctrl-seq-export
```

Zero result/sample counts are supported. A partial export retains current
flags and buffers; it does not imply that the sequence completed. Automatic
capture covers normal completion, command rejection, and motion timeout. It
cannot capture a crash or a session terminated before the completion point.

Only `EXPORTED:` reports success. A successful file ends with
`=== EXPORT COMPLETE ===`. If an export fails, the target remains suspended;
fix the reported error and run `mctrl-seq-export` before ending the session.
An interrupted dump may leave a partial text file without the completion
marker. The convenience variable `$mctrl_seq_export_ok` is 1 only after a full
successful dump. Generated logs/scripts are excluded from Git.

The exported command count is calculated from the debugger-visible command
array's type. GCC can omit the separate static constant
`motionControllerSequenceTestCommandCount` from the ELF, even in a Debug build;
the exporter does not require that symbol.

An offline regression check can run the real exporter with GDB, an archived ELF,
and synthetic RAM, without connecting to ST-LINK or starting motion:

```powershell
python Tests/Host/test_sequence_export.py --gdb 'C:/path/to/arm-none-eabi-gdb.exe' --elf 'exp/logs/sequence/session_example/firmware.elf'
```

The test puts its synthetic export in a temporary directory, using the export
preparation script's optional `-OutputDirectory` parameter. Normal automatic
exports continue to use `exp/logs/sequence/`.

## Build, flash and repeat after code changes

With CubeIDE's debug session closed, run this from a Windows PowerShell console
in the project directory:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File exp/gdb_scripts/run_sequence_test.ps1
```

The runner detects a single standard `C:/ST/STM32CubeIDE*` installation. If yours
is elsewhere, or several versions are installed, pass the folder containing
`headless-build.bat`:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File exp/gdb_scripts/run_sequence_test.ps1 -CubeIdeRoot 'C:/ST/STM32CubeIDE_1.x.x/STM32CubeIDE' -BatteryVoltage 11.95
```

Tool discovery probes the installation's bin folders and each plugin's
`tools/bin` or `bin` folder. It does not recursively scan GNU C++ headers or
multilib directories, whose deep paths can fail on Windows before a build
even starts. The runner prints the selected executable paths before building.

Check discovery without CubeIDE or a probe with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File Tests/Host/test_sequence_tool_discovery.ps1
```

It builds the current `MDP_Controller/Debug` configuration in a stable,
checkout-specific headless workspace under Windows Temp. The workspace must be
outside the project tree; Eclipse rejects importing a project that contains its
workspace. The old `exp/logs/cubeide-workspace` is no longer used.
The runner refuses to flash if the build fails, starts ST-LINK/GDB,
flashes and resets, waits for your SW1 start, and exports at sequence completion.
It saves the exact flashed ELF, its SHA-256/build timestamp, and build and
debugger/server records in a new `session_*` subdirectory.
The exported experiment remains in `exp/logs/sequence/` with its unique name.
It closes its own debugger/server after export; it never closes CubeIDE's
session. `-StLinkSerial` selects a particular probe if more than one is attached.
`-Port` selects a port other than 61234.

Validate the build and archive the ELF without connecting to the probe or
flashing with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File exp/gdb_scripts/run_sequence_test.ps1 -BuildOnly
```

This is a build check, not an acquired run; it creates build records but no
debugger session or experiment export.

The runner's zero exit status means a full export succeeded; inspect the test
flags to determine whether the sequence itself passed. Timeout and rejection
logs are useful experiments too. The command waits for SW1, so an agent must
allow enough execution time and continue waiting while you prepare the robot.

Local Codex can now edit the code, call this runner, read the path reported by
`EXPORTED:`, analyse the saved data, then edit and call it again. Specify the
experimental objective and which code/settings it may change. For example:

> Verify the unified controller using isolated +1000 mm and -1000 mm commands
> at 2000 CPS, three runs each. Use the sequence runner for every run and analyse
> each exported file. Keep controller gains and feedforward tables unchanged.
> I will reposition the robot and press SW1 for each run. Record every code
> change and stop if a run is rejected or times out.

Each new invocation resets the test, so repeat runs need no new firmware loop
or debugger-console paste. Automatic code decisions come from the local Codex
session; the runner performs the repeatable build/run/export operation. It
does not edit controller code or start physical motion by itself.
