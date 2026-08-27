# Development Workflow: nvim + STM32CubeIDE

Edit in nvim, build/flash/debug in STM32CubeIDE.

## Why this works

STM32CubeIDE is Eclipse. It does **not** own your files — there is no database,
no import step, no lock. `.project` and `.cproject` are XML manifests in the
repo; the files on disk are the single source of truth. Both editors can be open
on the same tree simultaneously.

Only two pieces of friction need removing:

1. Eclipse caches file state and must be told to notice external edits.
2. clangd needs to know the ARM cross-compile flags to parse the code.

Both are handled below.

## One-time setup

### 1. CubeIDE: enable auto-refresh

**Required.** Without it, CubeIDE silently builds a stale copy of any file you
edited in nvim.

> **STM32CubeIDE → Settings (⌘,) → General → Workspace**
> - [x] Refresh using native hooks or polling
> - [x] Refresh on access

### 2. clangd: generate the compilation database

```sh
./gen-compile-commands.sh
```

### 3. nvim

Nothing to configure. The existing clangd setup in
`~/.config/nvim/lua/plugins/lsp.lua` works as-is — clangd locates newlib's
headers from the compiler path recorded in `compile_commands.json`, so
`--query-driver` does not need `arm-none-eabi-gcc` added to it.

## Daily loop

```
nvim         edit, with clangd completion / diagnostics / goto-definition
CubeIDE ⌘B   build
CubeIDE      Run → Debug As → STM32 C/C++ Application
```

## Adding a new source file

Eclipse generates `Debug/*/subdir.mk` from `.cproject`. A `.c` file created in
nvim is not in the build until CubeIDE regenerates those makefiles.

1. Create the file in nvim (e.g. `PeripheralDrivers/Src/fwdriver.c`)
2. Build in CubeIDE — it detects the new file and adds it
3. `./gen-compile-commands.sh` — so clangd sees it too

Adding a whole new **include directory** additionally requires
Project Properties → C/C++ Build → Settings → Include paths in CubeIDE,
*and* a matching entry in `gen-compile-commands.sh`.

Existing source roots (`Apps/`, `PeripheralDrivers/`, `Tests/`) and their `Inc/`
directories are already registered in both places.

## Board connections (WHEELTEC C30D-V2.1)

Three USB-C ports along the top edge. Identified empirically with `ls /dev/cu.*`:

| Position | Function | Identifier |
|---|---|---|
| **Left** | Power only — no data lines | nothing enumerates |
| **Middle** | Serial — `U3`, USART3 (`PD8`/`PD9`). Comms port | `/dev/cu.usbserial-5A6C0673351` |
| **Right** | Serial — `U13`, USART1 (`PA9`/`PA10`). **Flashing port** | `/dev/cu.usbserial-5A6C0673281` |

Each CH9102F has a fixed USB serial number, so the middle/right ports are
always distinguishable by their `/dev/cu.*` name.

The right port is the flashing one because the STM32 ROM bootloader listens on
USART1, and `U13`'s `RTS`/`DTR` lines are wired to `BOOT0`/`NRST` through
transistors Q6/Q5 (the auto-download circuit).

Other landmarks: `USER` and `RESET` tactile buttons (bottom right), a
`Motor OFF/ON` slide switch, a main `OFF/ON` slide switch, and a screw terminal
for the battery. `MotorB`/`MotorC`/`MotorD` are the JST connectors on the edge.

### Servo / PWM headers

A bank of six 3-pin servo headers, silkscreened `J1`, `J2`, `J4`, `J5`, `J6`,
`J7`, with the signal pin of each labelled above it:

| Header | Signal pin |
|---|---|
| `J1` | `PC6` — **steering servo** (`TIM8_CH1`) |
| `J2` | `PC7` |
| `J4` | `PC8` |
| `J5` | `PC9` |
| `J6` | `PB14` |
| `J7` | `PB15` |

Row order on every header is `G` / `5V` / `PWM`, matching the standard hobby
servo connector: brown to `G`, red to `5V`, orange/yellow/white to `PWM`. The
`PWM` row is the one adjacent to the signal-name silkscreen.

The `5V` rail is a buck converter fed from the 12V input — fuse `F1` is marked
`5V3A` — so it can supply the TD-8120MG's 2.5A stall current. USB 5V cannot;
the servo only works with the battery connected.

**There is no onboard debugger.** No ST-LINK chip exists on this board. SWD is
broken out to a 4-pin header (`PA13` = SWDIO, `PA14` = SWCLK, `3V3`, `GND`) and
requires an external ST-LINK adapter. Without one you can flash over UART but
cannot use breakpoints, the SFRs view, or Live Expressions.

### Flashing over UART (no ST-LINK)

There is **no manual BOOT0 switch** on this board. `BOOT0` is driven only by the
CH9102F's `RTS` line through transistor Q6, so the flashing tool must toggle
`DTR`/`RTS` itself to enter the bootloader.

The course notes specify **FlyMcu v0.2188** for C30D 2.0/2.1 boards. FlyMcu is
**Windows-only** — on macOS use `stm32flash`, which drives the same circuit:

```sh
brew install stm32flash
```

**The entry sequence needs a priming run first.** The auto-download circuit is
cross-coupled and only acts when `DTR` and `RTS` differ, so the entry sequence
is not self-contained — it depends on the line state left behind by the
previous run. Run a throwaway command first to establish that state:

```sh
PORT=/dev/cu.usbserial-5A6C0673281

# 1. priming run - EXPECTED TO FAIL, this only sets the DTR/RTS state
stm32flash -i 'rts,-dtr,dtr' "$PORT"

# 2. the real entry, which now succeeds
stm32flash -i '-rts,dtr,-dtr' "$PORT"
```

Confirmed response from step 2: `Device ID: 0x0413 (STM32F40xxx/41xxx)`.

Without step 1, step 2 reports `Failed to init device, timeout` and the board
does not even reset. If you see that, you have skipped the priming run — it is
not a hardware fault, and retrying step 2 alone will never work.

Note the sequence is inverted relative to FlyMcu's nominal description
("DTR low resets, RTS high enters BootLoader") because the USB-serial driver
inverts the logical levels. Do not "correct" it.

To flash, with entry and exit sequences so the board runs firmware afterwards.
Both sequences are verified — after flashing, the board boots into the new
firmware on a cold power cycle:

```sh
PORT=/dev/cu.usbserial-5A6C0673281

# generate a raw binary from the ELF
arm-none-eabi-objcopy -O binary Debug/MDP_Controller.elf MDP_Controller.bin

# priming run (see above) - expected to fail
stm32flash -i 'rts,-dtr,dtr' "$PORT" >/dev/null 2>&1

stm32flash -i '-rts,dtr,-dtr:rts,dtr,-dtr' \
           -w MDP_Controller.bin -v -g 0x08000000 \
           "$PORT"
```

`arm-none-eabi-objcopy` ships with CubeIDE, under
`/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin/`.

### Flashing safety procedure

From the course hardware notes — follow these every time:

1. **Disconnect the 12 V supply and switch off the main switch** before flashing.
2. **Disconnect the Raspberry Pi supply** if connected.
3. Connect USB-C to **port 1** — the port nearest the potentiometer (the
   right-hand port, `/dev/cu.usbserial-5A6C0673281`).
4. Flash.
5. **Disconnect the USB cable** from port 1.
6. Reconnect and switch on the 12 V supply.
7. Feel the CPU and IMU chips — touch the **plastic package only, never the
   pins**. If either is very hot, something is wrong with the program.

Handling rules:

- **Never rest the board on an anti-static bag while powered or flashing.** The
  bags are surface-conductive and are for storage only. Use an insulated surface
  — acrylic or a laminated tabletop.
- Hold the board **by its edges**. Avoid touching pins or solder joints.
- Store in an anti-static bag when not in use.

## Debugging in CubeIDE

**Requires an external ST-LINK** wired to the SWD header — see above. Without
one, none of this is available; use OLED output or `printf` over USART3 instead.

Two views worth knowing, both hard to replicate over plain GDB:

- **Window → Show View → SFRs** — live peripheral registers by name. Expand
  `TIM8 → CCR1` to read the servo pulse width the hardware actually received,
  or `TIM3 → CNT` for a live encoder count.
- **Window → Show View → Live Expressions** — reads target memory over SWD
  *while the target is running*, no breakpoint needed. Watch a driver struct's
  fields update during a motion test.

> **Caution when halting on motor/servo code.** Timer peripherals keep running
> in hardware when the CPU halts at a breakpoint. PWM output does not stop, so
> motors keep driving and the servo keeps holding its last commanded position.
> If that position is against a mechanical stop, the servo stays stalled (up to
> ~2.5 A) for as long as you stay paused. Use `Servo_Disable()` / brake before
> long inspection pauses.

## Files

| File | Purpose | Tracked |
|---|---|---|
| `gen-compile-commands.sh` | Regenerates the compilation DB. Auto-locates CubeIDE's bundled toolchain | yes |
| `.clangd` | Strips GCC-only flags clang rejects (`--specs=`, `-fcyclomatic-complexity`, `-fstack-usage`) | yes |
| `compile_commands.json` | clangd's flag database — contains absolute machine paths | no |
| `.cache/` | clangd's `--background-index` store | no |
| `Debug/` | CubeIDE build output and generated makefiles | no |

The flags in `gen-compile-commands.sh` mirror the real compiler invocation
CubeIDE writes into `Debug/Core/Src/subdir.mk`. If CubeIDE's build settings
change, re-check them against that file.

## Notes

- **CubeIDE cannot generate `compile_commands.json` for this project.** It
  bundles Eclipse CDT 9.3 with no compilation-database plugin, and CDT's
  managed-build system has no exporter. Only CubeIDE's CMake project type would
  produce one natively, which would mean abandoning the `.cproject` managed
  build. Hence `gen-compile-commands.sh`.
- **`.settings/language.settings.xml` showing as modified** is normal — CubeIDE
  rewrites an `env-hash` field when it re-fingerprints the toolchain
  environment. Harmless.
- **clangd config is per-directory and inherited from ancestors.** A stray
  `.clangd` or a user-level config at `~/Library/Preferences/clangd/config.yaml`
  (macOS) applies here too. Note that a user-level `Add:` is applied *after* a
  project-level `Remove:`, so this repo's `.clangd` cannot override one — check
  there first if flags appear from nowhere.

## Troubleshooting

**clangd reports errors on every file, e.g. an unexpected `-std=`**
A user-level config is injecting flags. Check
`~/Library/Preferences/clangd/config.yaml`, then any `.clangd` in a parent
directory. Diagnose with:

```sh
clangd --check=Core/Src/main.c 2>&1 | grep "Compile command from CDB"
```

**`'xyz.h' file not found` after adding a file or include path**
Re-run `./gen-compile-commands.sh`, then `:LspRestart` in nvim.

**CubeIDE builds an old version of a file edited in nvim**
Auto-refresh is off — see step 1. As a one-off, select the project and press F5.
