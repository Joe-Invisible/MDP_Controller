# Run with Windows PowerShell 5.1 or pwsh. No CubeIDE/probe is needed.
# Load only the discovery function, never the build/flash runner itself.
$ErrorActionPreference = 'Stop'
$runner = Join-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) 'exp/gdb_scripts/run_sequence_test.ps1'
$tokens = $null
$parseErrors = $null
$ast = [System.Management.Automation.Language.Parser]::ParseFile($runner, [ref]$tokens, [ref]$parseErrors)
if ($parseErrors) { throw ($parseErrors | Out-String) }
$functionAst = $ast.Find({
    param($node)
    $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq 'Find-CubeTool'
}, $true)
if (-not $functionAst) { throw 'Discovery function not found.' }
Invoke-Expression $functionAst.Extent.Text

# Reject accidental recursive scans at runtime while retaining real filesystem
# enumeration. In particular, discovery must not enter toolchain/include trees.
function Get-ChildItem {
    param([string] $LiteralPath, [switch] $Directory, [switch] $Recurse)
    if ($Recurse) { throw 'Regression: recursive toolchain traversal.' }
    if ((Split-Path $LiteralPath -Leaf) -ne 'plugins') { throw "Unexpected traversal: $LiteralPath" }
    Microsoft.PowerShell.Management\Get-ChildItem -LiteralPath $LiteralPath -Directory:$Directory
}

$CubeIdeRoot = Join-Path ([System.IO.Path]::GetTempPath()) ('CubeIDE discovery with spaces ' + [Guid]::NewGuid().ToString('N'))
try {
    $tools = @{
        'arm-none-eabi-gdb.exe' = 'plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.win32_1.0/tools/bin'
        'ST-LINK_gdbserver.exe' = 'plugins/com.st.stm32cube.ide.mcu.externaltools.stlink-gdb-server.win32_1.0/tools/bin'
        'STM32_Programmer_CLI.exe' = 'plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_1.0/tools/bin'
    }
    foreach ($name in $tools.Keys) {
        $directory = Join-Path $CubeIdeRoot $tools[$name]
        New-Item -ItemType Directory -Path $directory -Force | Out-Null
        New-Item -ItemType File -Path (Join-Path $directory $name) | Out-Null
    }
    foreach ($name in $tools.Keys) {
        $expected = (Get-Item -LiteralPath (Join-Path (Join-Path $CubeIdeRoot $tools[$name]) $name)).FullName
        if ((Find-CubeTool $name) -ne $expected) { throw "Wrong path for $name" }
    }

    $missingRejected = $false
    try { Find-CubeTool 'missing.exe' | Out-Null } catch {
        if ($_.Exception.Message -notlike '*found 0*') { throw }
        $missingRejected = $true
    }
    if (-not $missingRejected) { throw 'Missing tool accepted.' }

    New-Item -ItemType File -Path (Join-Path $CubeIdeRoot 'arm-none-eabi-gdb.exe') | Out-Null
    $duplicateRejected = $false
    try { Find-CubeTool 'arm-none-eabi-gdb.exe' | Out-Null } catch {
        if ($_.Exception.Message -notlike '*found 2*') { throw }
        $duplicateRejected = $true
    }
    if (-not $duplicateRejected) { throw 'Ambiguous tool accepted.' }
    Write-Host 'PASS: CubeIDE plugin discovery, spaces, no recursive scans, missing and duplicate tools.'
} finally {
    Remove-Item -LiteralPath $CubeIdeRoot -Recurse -Force -ErrorAction SilentlyContinue
}
