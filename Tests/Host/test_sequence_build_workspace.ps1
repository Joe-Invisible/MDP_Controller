# No CubeIDE or probe is needed. Load the helper without executing the runner.
$ErrorActionPreference = 'Stop'
$runner = Join-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) 'exp/gdb_scripts/run_sequence_test.ps1'
$tokens = $null
$parseErrors = $null
$ast = [System.Management.Automation.Language.Parser]::ParseFile($runner, [ref]$tokens, [ref]$parseErrors)
if ($parseErrors) { throw ($parseErrors | Out-String) }
$functionAst = $ast.Find({
    param($node)
    $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq 'Get-CubeBuildWorkspace'
}, $true)
if (-not $functionAst) { throw 'Workspace helper not found.' }
Invoke-Expression $functionAst.Extent.Text

$project = 'C:\sequence workspace test\checkout with spaces'
$workspace = Get-CubeBuildWorkspace $project
if ($workspace.StartsWith($project + '\', [System.StringComparison]::OrdinalIgnoreCase)) {
    throw 'Regression: workspace is inside the imported project.'
}
if ($workspace -ne (Get-CubeBuildWorkspace ($project.ToUpperInvariant() + '\'))) {
    throw 'Equivalent Windows checkout paths must reuse the same workspace.'
}
if ($workspace -eq (Get-CubeBuildWorkspace ($project + '-another'))) {
    throw 'Different checkouts must not share workspace project registrations.'
}

# A project containing Temp would contain its workspace too; refuse this layout.
$overlapRejected = $false
try { Get-CubeBuildWorkspace ([System.IO.Path]::GetPathRoot($workspace)) | Out-Null } catch {
    if ($_.Exception.Message -notlike '*outside the project tree*') { throw }
    $overlapRejected = $true
}
if (-not $overlapRejected) { throw 'Overlapping project/workspace accepted.' }
Write-Host 'PASS: external workspace, stable Windows paths, isolated checkouts, overlap rejection.'
