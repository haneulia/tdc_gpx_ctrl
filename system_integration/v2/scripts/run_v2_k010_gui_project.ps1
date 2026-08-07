param(
    [string]$Vivado = "",
    [string]$ProjectDir = "",
    [ValidateRange(1, 16)]
    [int]$Jobs = 4
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$v2Dir = Split-Path -Parent $scriptDir
$hdlRoot = (Resolve-Path (Join-Path $v2Dir "..\..")).Path

if ([string]::IsNullOrWhiteSpace($Vivado)) {
    if ($env:XILINX_VIVADO) {
        $Vivado = Join-Path $env:XILINX_VIVADO "bin\vivado.bat"
    } else {
        $Vivado = "C:\AMDDesignTools\2025.2.1\Vivado\bin\vivado.bat"
    }
}
if (-not (Test-Path -LiteralPath $Vivado)) {
    throw "Vivado executable is missing: $Vivado"
}

if ([string]::IsNullOrWhiteSpace($ProjectDir)) {
    $ProjectDir = Join-Path $hdlRoot ".work\tdc_gpx_lidar_ctrl_v2_gui"
} elseif (-not [System.IO.Path]::IsPathRooted($ProjectDir)) {
    $ProjectDir = Join-Path $hdlRoot $ProjectDir
}
$ProjectDir = [System.IO.Path]::GetFullPath($ProjectDir)

$verifyExisting = $false
if (Test-Path -LiteralPath $ProjectDir) {
    $existingSummary = Join-Path $ProjectDir `
        "gui_signoff_reports\GUI_PROJECT_SUMMARY.txt"
    if (Test-Path -LiteralPath $existingSummary) {
        $verifyExisting = $true
        Write-Host "Verifying the existing GUI project: $ProjectDir"
    } else {
        Write-Host "Resuming an incomplete GUI project directory: $ProjectDir"
    }
}

$stamp = Get-Date -Format "yyMMdd_HHmmss"
$sessionKind = if ($verifyExisting) {
    "k010_gui_verify"
} else {
    "k010_gui_project"
}
$session = Join-Path $hdlRoot `
    "signoff_results\sessions\${stamp}_${sessionKind}"
New-Item -ItemType Directory -Force -Path $session | Out-Null

$action = if ($verifyExisting) { "verify" } else { "create" }
$log = Join-Path $session "${action}_gui_project.log"
$journal = Join-Path $session "${action}_gui_project.jou"
$tclName = if ($verifyExisting) {
    "verify_v2_k010_gui_project.tcl"
} else {
    "create_v2_k010_gui_project.tcl"
}
$tcl = Join-Path $scriptDir $tclName

& $Vivado `
    -mode batch `
    -notrace `
    -log $log `
    -journal $journal `
    -source $tcl `
    -tclargs $ProjectDir $Jobs | Out-Host
if ($LASTEXITCODE -ne 0) {
    throw "GUI project $action failed. See $log"
}
$passMarker = if ($verifyExisting) {
    "LIDAR_V2_K010_GUI_PROJECT_VERIFY_PASS"
} else {
    "LIDAR_V2_K010_GUI_PROJECT_PASS"
}
if (-not (Select-String -LiteralPath $log -SimpleMatch `
        $passMarker -Quiet)) {
    throw "GUI project PASS marker is missing: $log"
}

$xpr = Join-Path $ProjectDir "tdc_gpx_lidar_ctrl_v2_gui.xpr"
$projectSummary = Join-Path $ProjectDir `
    "gui_signoff_reports\GUI_PROJECT_SUMMARY.txt"
foreach ($required in @($xpr, $projectSummary)) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "Required GUI project artifact is missing: $required"
    }
}

$guide = Join-Path $hdlRoot `
    "system_integration\v2_architecture\V2_K010_GUI_PROJECT_GUIDE_KO.md"
Copy-Item -LiteralPath $guide `
    -Destination (Join-Path $ProjectDir "GUI_CHECK_GUIDE_KO.md")

$summary = @(
    "LIDAR_V2_K010_GUI_PROJECT_SIGNOFF_PASS",
    "action=$action",
    "project=$xpr",
    "block_designs=bd_async32_tdc_faster,bd_async128_proc_faster,bd_sync64_equal",
    "synthesis_runs=bd_async32_tdc_faster_u_lidar_ctrl_0_synth_1,bd_async128_proc_faster_u_lidar_ctrl_0_synth_1,bd_sync64_equal_u_lidar_ctrl_0_synth_1",
    "session=$session"
)
Set-Content -LiteralPath (Join-Path $session "SUMMARY.txt") `
    -Value $summary -Encoding UTF8

Write-Host "LIDAR_V2_K010_GUI_PROJECT_SIGNOFF_PASS project=$xpr"
