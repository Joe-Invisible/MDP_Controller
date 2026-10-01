# Compatible with Windows PowerShell 5.1. No Python-enabled GDB is required.
# Called by auto_export_straight_feedforward.gdb while the target is suspended.
[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$outputDirectory = Join-Path $projectRoot 'exp/logs/straight_feedforward'
$commandFile = Join-Path $outputDirectory 'export-current.gdb'

function ConvertTo-GdbEcho([string] $value) {
    # GDB echo interprets backslash escapes. Never let metadata add commands.
    return $value.Replace('\', '\\').Replace("`r", ' ').Replace("`n", ' ')
}

try {
    New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null
    # A failed preparation must not leave a stale script available for reuse.
    Remove-Item -LiteralPath $commandFile -Force -ErrorAction SilentlyContinue

    $exportTime = [DateTimeOffset]::Now
    $uniqueSuffix = [Guid]::NewGuid().ToString('N').Substring(0, 8)
    $fileName = 'mctrl_straight_ff_{0}_{1}.txt' -f $exportTime.ToString('yyyyMMdd_HHmmss_fff'), $uniqueSuffix
    $logPath = (Join-Path $outputDirectory $fileName).Replace('\', '/')

    $revision = 'unknown (Git unavailable)'
    $trackedChanges = 'unknown'
    if (Get-Command git -ErrorAction SilentlyContinue) {
        $revisionOutput = & git -C $projectRoot rev-parse HEAD 2>$null
        if ($LASTEXITCODE -eq 0) {
            $revision = ($revisionOutput -join ' ')
            $statusOutput = & git -C $projectRoot status --porcelain --untracked-files=no 2>$null
            if ($LASTEXITCODE -eq 0) {
                $trackedChanges = if ($statusOutput) { 'yes' } else { 'no' }
            }
        }
    }

    $commands = @(
        'set logging enabled off'
        'set print elements unlimited'
        'set print repeats unlimited'
        'set print pretty off'
        'set pagination off'
        'set width 0'
        ('set logging file ' + $logPath)
        'set logging overwrite off'
        'set logging redirect on'
        'set logging enabled on'
        'echo === MotionController Straight Feedforward Test ===\n'
        ('echo Exported at (host clock, not run start): ' + $exportTime.ToString('o') + '\n')
        ('echo Checkout revision (not proof of flashed revision): ' + (ConvertTo-GdbEcho $revision) + '\n')
        ('echo Checkout has tracked changes: ' + $trackedChanges + '\n')
        'printf "Battery voltage (user supplied; 0 = unknown): %.3f V\n", $mctrl_straight_ff_battery_v'
        'echo Controller states: 0=IDLE, 1=STRAIGHT, 2=ARC, 3=BRAKING, 4=ARC_PREPARING, 5=STRAIGHT_PREPARING.\n'
        'echo === Loaded ELF ===\n'
        'info files'
        'echo === Commands and controller configuration ===\n'
        'p motionControllerStraightFeedforwardTestRequestedDistanceMm'
        'p motionControllerStraightFeedforwardTestRequestedSpeedCps'
        'p motionControllerStraightFeedforwardTestRawCommand'
        'p straightFeedforwardTestMotionConfig'
        'p *straightFeedforwardTestMotionConfig.kinematics'
        'echo === Results ===\n'
        'p motionControllerStraightFeedforwardTestCommandAccepted'
        'p motionControllerStraightFeedforwardTestTimedOut'
        'p motionControllerStraightFeedforwardTestUpdateStatus'
        'p motionControllerStraightFeedforwardTestMotionExitCaptured'
        'p motionControllerStraightFeedforwardTestMotionExitTimeMs'
        'p motionControllerStraightFeedforwardTestMotionExitDistanceMm'
        'p motionControllerStraightFeedforwardTestMotionExitYawDeg'
        'p motionControllerStraightFeedforwardTestMotionExitIdealYawDeg'
        'p motionControllerStraightFeedforwardTestMotionExitLeftDistanceMm'
        'p motionControllerStraightFeedforwardTestMotionExitRightDistanceMm'
        'p motionControllerStraightFeedforwardTestFinalDistanceMm'
        'p motionControllerStraightFeedforwardTestFinalYawDeg'
        'p motionControllerStraightFeedforwardTestFinalLeftDistanceMm'
        'p motionControllerStraightFeedforwardTestFinalRightDistanceMm'
        'p motionControllerStraightFeedforwardTestTimeoutMode'
        'p motionControllerStraightFeedforwardTestTimeoutDistanceMm'
        'p motionControllerStraightFeedforwardTestTimeoutTargetDistanceMm'
        'p motionControllerStraightFeedforwardTestTimeoutProfileActive'
        'p motionControllerStraightFeedforwardTestTimeoutStationarySamples'
        'p motionControllerStraightFeedforwardTestTimeoutLeftCps'
        'p motionControllerStraightFeedforwardTestTimeoutRightCps'
        'p motionControllerStraightFeedforwardTestTimeoutYawDeg'
        'p motionControllerStraightFeedforwardTestTimeoutMotionExitCaptured'
        'echo === Samples ===\n'
        'p motionControllerStraightFeedforwardTestLogCount'
        'if motionControllerStraightFeedforwardTestLogCount > 0'
        '    p motionControllerStraightFeedforwardTestLog[0] @ motionControllerStraightFeedforwardTestLogCount'
        'end'
        'echo === EXPORT COMPLETE ===\n'
        'set logging enabled off'
        'set logging redirect off'
        'set $mctrl_straight_ff_export_ok = 1'
        ('echo EXPORTED: ' + (ConvertTo-GdbEcho $logPath) + '\n')
    )

    # UTF-8 without BOM: GDB must see "set", rather than a BOM-prefixed command.
    $encoding = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllLines($commandFile, [string[]]$commands, $encoding)
} catch {
    Remove-Item -LiteralPath $commandFile -Force -ErrorAction SilentlyContinue
    Write-Error $_ -ErrorAction Continue
    exit 1
}
