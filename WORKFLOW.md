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

## Debugging in CubeIDE

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
