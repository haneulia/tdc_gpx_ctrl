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

    [ValidateSet("DEFAULT", "TIMING_EXPLORE")]
    [string]$ImplStrategy = "DEFAULT",

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

function Assert-StageClosed {
    param(
        [string]$Prefix,
        [string]$Stage
    )

    $timing = "$OutDir/${Prefix}_timing_summary.rpt"
    $checkTiming = "$OutDir/${Prefix}_check_timing.rpt"
    $cdcData = "$OutDir/${Prefix}_cdc_data.rpt"

    foreach ($report in @($timing, $checkTiming, $cdcData)) {
        if (-not (Test-Path -LiteralPath $report)) {
            throw "$Stage report is missing: $report"
        }
    }

    if (-not (Select-String -LiteralPath $timing `
            -Pattern "All user specified timing constraints are met." `
            -SimpleMatch -Quiet)) {
        throw "$Stage timing constraints are not met: $timing"
    }
    if (Select-String -LiteralPath $timing -Pattern 'Slack \(VIOLATED\)' -Quiet) {
        throw "$Stage timing report contains a violated path: $timing"
    }
    if (-not (Select-String -LiteralPath $checkTiming `
            -Pattern "checking no_clock (0)" -SimpleMatch -Quiet)) {
        throw "$Stage has registers without a clock: $checkTiming"
    }
    if (-not (Select-String -LiteralPath $checkTiming `
            -Pattern "checking unconstrained_internal_endpoints (0)" `
            -SimpleMatch -Quiet)) {
        throw "$Stage has unconstrained internal endpoints: $checkTiming"
    }

    $unsafeCdcRows = @(Select-String -LiteralPath $cdcData `
        -Pattern '^\s*CDC-(4|10)\s')
    if ($unsafeCdcRows.Count -ne 0) {
        throw "$Stage contains unsafe data CDC rows (CDC-4/CDC-10): $cdcData"
    }
}

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
    "impl_strategy=$ImplStrategy"
    "implement=$($Implement.IsPresent)"
) | Set-Content -Encoding ASCII "$OutDir/session.properties"

$DoImpl = if ($Implement) { "1" } else { "0" }
& $Vivado -mode batch -nojournal -log $Log `
    -source "$Hdl/scripts/run_ooc_signoff.tcl" `
    -tclargs $OutDir $Width $AxisMhz $TdcMhz $SlopeMode $StreamMode $DoImpl $ImplStrategy

if ($LASTEXITCODE -ne 0) {
    throw "Vivado OOC sign-off failed with exit code $LASTEXITCODE. See $Log"
}
if (-not (Select-String -Path $Log -Pattern "OOC_SIGNOFF_SYNTH_PASS" -SimpleMatch -Quiet)) {
    throw "Vivado log is missing OOC_SIGNOFF_SYNTH_PASS: $Log"
}
Assert-StageClosed -Prefix "post_synth" -Stage "Post-synthesis"

if ($Implement -and -not (Select-String -Path $Log -Pattern "OOC_SIGNOFF_IMPL_PASS" -SimpleMatch -Quiet)) {
    throw "Vivado log is missing OOC_SIGNOFF_IMPL_PASS: $Log"
}
if ($Implement) {
    Assert-StageClosed -Prefix "post_route" -Stage "Post-route"
}

Write-Host "OOC sign-off PASS: $OutDir"
