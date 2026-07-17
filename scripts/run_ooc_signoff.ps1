# Reproducible OOC synthesis/implementation wrapper for tdc_gpx_top.
param(
    [ValidateSet(32, 64, 128)]
    [int]$Width = 32,

    [ValidateSet(50, 100, 125, 150, 200)]
    [int]$AxisMhz = 150,

    [ValidateSet(50, 100, 125, 150, 200)]
    [int]$TdcMhz = 200,

    [ValidateSet("DEDICATED_2X2", "SHARED_DUAL_EDGE")]
    [string]$SlopeMode = "DEDICATED_2X2",

    [ValidateSet("ASYNC", "SYNC")]
    [string]$StreamMode = "ASYNC",

    [string]$Label = "baseline",
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$Implement,
    [switch]$AllowDirty
)

$ErrorActionPreference = "Stop"

if ($AxisMhz -gt $TdcMhz) {
    throw "g_AXIS_CLK_MHZ must not exceed g_TDC_CLK_MHZ"
}
if ($StreamMode -eq "SYNC" -and $AxisMhz -ne $TdcMhz) {
    throw "SYNC mode requires equal AXIS and TDC clock metadata"
}

$Hdl = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL"
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin/vivado.bat"
$Mode = if ($Implement) { "impl" } else { "synth" }
$SafeSlope = $SlopeMode.ToLowerInvariant()
$Session = "${Stamp}_${Label}_w${Width}_a${AxisMhz}_t${TdcMhz}_${SafeSlope}_${Mode}"
$OutDir = "$Hdl/signoff_results/sessions/$Session"
$Log = "$OutDir/vivado.log"

$GitHead = (git -C $Hdl rev-parse HEAD).Trim()
$GitChanges = @(git -C $Hdl status --porcelain --untracked-files=normal)
$GitState = if ($GitChanges.Count -eq 0) { "clean" } else { "dirty" }

if ($GitState -eq "dirty" -and -not $AllowDirty) {
    throw "OOC sign-off requires a clean Git worktree. Commit first or use -AllowDirty for development-only runs."
}
if (Test-Path -LiteralPath $OutDir) {
    throw "OOC session already exists and is immutable: $OutDir"
}

New-Item -ItemType Directory -Path $OutDir | Out-Null

@(
    "stamp=$Stamp"
    "label=$Label"
    "git_head=$GitHead"
    "git_state=$GitState"
    "width=$Width"
    "axis_mhz=$AxisMhz"
    "tdc_mhz=$TdcMhz"
    "slope_mode=$SlopeMode"
    "stream_mode=$StreamMode"
    "implement=$($Implement.IsPresent)"
) | Set-Content -Encoding ASCII "$OutDir/session.properties"

$DoImpl = if ($Implement) { "1" } else { "0" }
& $Vivado -mode batch -nojournal -log $Log `
    -source "$Hdl/scripts/run_ooc_signoff.tcl" `
    -tclargs $OutDir $Width $AxisMhz $TdcMhz $SlopeMode $StreamMode $DoImpl

if ($LASTEXITCODE -ne 0) {
    throw "Vivado OOC sign-off failed with exit code $LASTEXITCODE. See $Log"
}
if (-not (Select-String -Path $Log -Pattern "OOC_SIGNOFF_SYNTH_PASS" -SimpleMatch -Quiet)) {
    throw "Vivado log is missing OOC_SIGNOFF_SYNTH_PASS: $Log"
}
if ($Implement -and -not (Select-String -Path $Log -Pattern "OOC_SIGNOFF_IMPL_PASS" -SimpleMatch -Quiet)) {
    throw "Vivado log is missing OOC_SIGNOFF_IMPL_PASS: $Log"
}

Write-Host "OOC sign-off PASS: $OutDir"
