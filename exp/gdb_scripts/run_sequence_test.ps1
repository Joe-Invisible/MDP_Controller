# Build -> flash -> SW1 start -> automatic export. Run on the Windows PC
# connected to ST-LINK, with CubeIDE's debug session closed.
[CmdletBinding()]
param(
    [string] $CubeIdeRoot,
    [ValidateRange(1, 65535)] [int] $Port = 61234,
    [string] $StLinkSerial,
    [ValidateRange(0, 30)] [double] $BatteryVoltage = 0,
    [switch] $BuildOnly
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$serverProcess = $null

function Get-CubeBuildWorkspace([string] $projectDirectory) {
    # Eclipse rejects an imported project that contains its own workspace.
    # Keep a stable, checkout-specific workspace outside the source tree.
    $projectPath = [System.IO.Path]::GetFullPath($projectDirectory).TrimEnd('\', '/')
    $hasher = [System.Security.Cryptography.SHA256]::Create()
    try {
        $bytes = [System.Text.Encoding]::UTF8.GetBytes($projectPath.ToLowerInvariant())
        $key = ([System.BitConverter]::ToString($hasher.ComputeHash($bytes))).Replace('-', '').Substring(0, 16)
    } finally { $hasher.Dispose() }
    $workspace = Join-Path ([System.IO.Path]::GetTempPath()) ('MDP_Controller-headless-' + $key)
    $workspace = [System.IO.Path]::GetFullPath($workspace).TrimEnd('\', '/')
    if ($workspace.Equals($projectPath, [System.StringComparison]::OrdinalIgnoreCase) -or
        $workspace.StartsWith($projectPath + '\', [System.StringComparison]::OrdinalIgnoreCase) -or
        $projectPath.StartsWith($workspace + '\', [System.StringComparison]::OrdinalIgnoreCase)) {
        throw 'The headless workspace must be outside the project tree. Set TEMP to an external directory.'
    }
    return $workspace
}

function Find-CubeTool([string] $name) {
    # Enumerate only the plugin directory itself, then probe known bin paths.
    # Recursing through GNU C++ multilib headers can hit Windows path limits
    # even when all required executables are installed and working.
    $binDirectories = @(
        $CubeIdeRoot
        (Join-Path $CubeIdeRoot 'bin')
        (Join-Path $CubeIdeRoot 'tools/bin')
    )
    $pluginDirectory = Join-Path $CubeIdeRoot 'plugins'
    if (Test-Path -LiteralPath $pluginDirectory -PathType Container) {
        foreach ($plugin in Get-ChildItem -LiteralPath $pluginDirectory -Directory) {
            $binDirectories += Join-Path $plugin.FullName 'tools/bin'
            $binDirectories += Join-Path $plugin.FullName 'bin'
        }
    }
    $matches = @(
        foreach ($directory in $binDirectories) {
            $candidate = Join-Path $directory $name
            if (Test-Path -LiteralPath $candidate -PathType Leaf) {
                Get-Item -LiteralPath $candidate
            }
        }
    )
    if ($matches.Count -ne 1) {
        $foundPaths = ($matches | ForEach-Object { $_.FullName }) -join '; '
        throw "Expected one $name in CubeIDE/plugin bin directories under $CubeIdeRoot; found $($matches.Count). Found paths: $foundPaths"
    }
    return $matches[0].FullName
}

try {
    if (-not $CubeIdeRoot) {
        $installs = @(Get-ChildItem -Path 'C:/ST/STM32CubeIDE*' -Directory -ErrorAction SilentlyContinue |
            Where-Object { Test-Path (Join-Path $_.FullName 'STM32CubeIDE/headless-build.bat') })
        if ($installs.Count -ne 1) {
            throw 'Pass -CubeIdeRoot with the folder containing headless-build.bat, e.g. C:/ST/STM32CubeIDE_1.x.x/STM32CubeIDE.'
        }
        $CubeIdeRoot = Join-Path $installs[0].FullName 'STM32CubeIDE'
    }
    $CubeIdeRoot = (Resolve-Path -LiteralPath $CubeIdeRoot).Path
    $headlessBuild = Join-Path $CubeIdeRoot 'headless-build.bat'
    if (-not (Test-Path -LiteralPath $headlessBuild)) {
        throw "Missing $headlessBuild"
    }
    $gdb = Find-CubeTool 'arm-none-eabi-gdb.exe'
    $server = Find-CubeTool 'ST-LINK_gdbserver.exe'
    $programmerDirectory = Split-Path (Find-CubeTool 'STM32_Programmer_CLI.exe') -Parent
    Write-Host "GDB: $gdb"
    Write-Host "ST-LINK server: $server"
    Write-Host "CubeProgrammer bin: $programmerDirectory"

    # Binding checks ownership without connecting to/disrupting another server.
    if (-not $BuildOnly) {
        $portCheck = New-Object System.Net.Sockets.TcpListener([System.Net.IPAddress]::Loopback, $Port)
        try { $portCheck.Start() } catch {
            throw "Port $Port is in use. Close the CubeIDE debug session before running this command, or choose -Port."
        } finally { $portCheck.Stop() }
    }

    $runId = '{0}_{1}' -f (Get-Date -Format 'yyyyMMdd_HHmmss_fff'), [Guid]::NewGuid().ToString('N').Substring(0, 8)
    $sessionDirectory = Join-Path $projectRoot "exp/logs/sequence/session_$runId"
    New-Item -ItemType Directory -Path $sessionDirectory -Force | Out-Null
    $buildWorkspace = Get-CubeBuildWorkspace $projectRoot
    $buildLog = Join-Path $sessionDirectory 'build.txt'
    $elf = Join-Path $projectRoot 'Debug/MDP_Controller.elf'

    Write-Host "Headless workspace: $buildWorkspace"
    Write-Host 'Building MDP_Controller/Debug...'
    # Remove the previous ELF so a failed build can never flash a stale image.
    Remove-Item -LiteralPath $elf -Force -ErrorAction SilentlyContinue
    $savedErrorPreference = $ErrorActionPreference
    $ErrorActionPreference = 'Continue' # Windows PowerShell treats native stderr as error records.
    try {
        & $headlessBuild -data $buildWorkspace -import $projectRoot -build 'MDP_Controller/Debug' 2>&1 |
            Tee-Object -FilePath $buildLog
        $buildExitCode = $LASTEXITCODE
    } finally { $ErrorActionPreference = $savedErrorPreference }
    if ($buildExitCode -ne 0 -or -not (Test-Path -LiteralPath $elf)) {
        throw "Build failed or produced no ELF; nothing flashed. See $buildLog"
    }

    # Archive and flash this exact image, even if the working build changes later.
    $sessionElf = Join-Path $sessionDirectory 'firmware.elf'
    Copy-Item -LiteralPath $elf -Destination $sessionElf
    $imageRecord = [ordered]@{
        builtAt = [DateTimeOffset]::Now.ToString('o')
        elfSha256 = (Get-FileHash -LiteralPath $sessionElf -Algorithm SHA256).Hash
        batteryVoltage = $BatteryVoltage
        cubeIdeRoot = $CubeIdeRoot
    }
    $imageRecord | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $sessionDirectory 'firmware.json') -Encoding UTF8

    if ($BuildOnly) {
        Write-Host "Build-only validation complete; nothing flashed. Build records: $sessionDirectory"
        return
    }

    $serverArguments = '-d -e -k -p {0} -cp "{1}"' -f $Port, $programmerDirectory
    if ($StLinkSerial) {
        if ($StLinkSerial -notmatch '^[A-Za-z0-9]+$') { throw 'ST-LINK serial must be alphanumeric.' }
        $serverArguments += ' -i ' + $StLinkSerial
    }
    $serverProcess = Start-Process -FilePath $server -ArgumentList $serverArguments -PassThru -NoNewWindow `
        -RedirectStandardOutput (Join-Path $sessionDirectory 'server.txt') `
        -RedirectStandardError (Join-Path $sessionDirectory 'server-error.txt')
    Start-Sleep -Milliseconds 500
    if ($serverProcess.HasExited) {
        throw "ST-LINK GDB server failed to start; see $sessionDirectory"
    }

    $battery = $BatteryVoltage.ToString([System.Globalization.CultureInfo]::InvariantCulture)
    $gdbCommands = @(
        'set confirm off'
        'set pagination off'
        'set tcp connect-timeout 15'
        "target extended-remote localhost:$Port"
        'load'
        'monitor reset'
        'source exp/gdb_scripts/auto_export_sequence.gdb'
        ('set $mctrl_seq_battery_v = ' + $battery)
        'continue'
        'if $mctrl_seq_export_ok != 1'
        '    echo ERROR: run stopped without a complete export.\n'
        '    quit 1'
        'end'
        'quit 0'
    )
    $gdbCommandFile = Join-Path $sessionDirectory 'run.gdb'
    $encoding = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllLines($gdbCommandFile, [string[]]$gdbCommands, $encoding)

    Write-Host 'Flashing and running. Position the robot, then press SW1 when ready.'
    Push-Location $projectRoot
    $savedErrorPreference = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        & $gdb --batch --nx $sessionElf -x $gdbCommandFile 2>&1 |
            Tee-Object -FilePath (Join-Path $sessionDirectory 'debugger.txt')
        $gdbExitCode = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $savedErrorPreference
        Pop-Location
    }
    if ($gdbExitCode -ne 0) {
        throw "Debugger run/export failed; inspect $sessionDirectory before retrying."
    }
    Write-Host "Run exported. Build/debugger records: $sessionDirectory"
    Write-Host 'Check the exported Passed/TimedOut/CommandRejected fields to assess the test outcome.'
} catch {
    Write-Error $_ -ErrorAction Continue
    exit 1
} finally {
    if ($serverProcess -and -not $serverProcess.HasExited) {
        Stop-Process -Id $serverProcess.Id -ErrorAction SilentlyContinue
    }
}
