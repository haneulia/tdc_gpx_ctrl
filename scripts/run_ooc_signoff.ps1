# Reproducible OOC synthesis/implementation wrapper for tdc_gpx_top.
param(
    [ValidateSet(32, 64, 128)]
    [int]$Width = 32,

    [ValidateSet(50, 100, 125, 150, 200)]
    [int]$AxisMhz = 150,

    [ValidateSet(50, 100, 125, 150, 200)]
    [int]$TdcMhz = 200,

    [ValidatePattern('^[01]{4}$')]
    [string]$PresentChipMask = "1111",

    [ValidatePattern('^[01]{4}$')]
    [string]$RiseChipMask = "0011",

    [ValidatePattern('^[01]{4}$')]
    [string]$FallChipMask = "1100",

    [ValidateRange(2, 8)]
    [int]$MaxStopsPerChip = 8,

    [ValidateRange(1, 7)]
    [int]$MaxHitsPerStop = 7,

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

$presentBits = [Convert]::ToInt32($PresentChipMask, 2)
$riseBits = [Convert]::ToInt32($RiseChipMask, 2)
$fallBits = [Convert]::ToInt32($FallChipMask, 2)
$activeRiseBits = $riseBits -band $presentBits
$activeFallBits = $fallBits -band $presentBits
$roleCoverage = ($activeRiseBits -bor $activeFallBits)
$presentCount = ($PresentChipMask.ToCharArray() | Where-Object { $_ -eq '1' }).Count
$riseActiveCount = ([Convert]::ToString($activeRiseBits, 2).ToCharArray() | Where-Object { $_ -eq '1' }).Count
$fallActiveCount = ([Convert]::ToString($activeFallBits, 2).ToCharArray() | Where-Object { $_ -eq '1' }).Count

if ($presentBits -eq 0) {
    throw "g_PRESENT_CHIP_MASK must enable at least one chip"
}
if ($roleCoverage -ne $presentBits) {
    throw "Every present chip must have a rising and/or falling role"
}
if ($riseActiveCount -lt $fallActiveCount) {
    throw "Rising-capable chip count must be greater than or equal to falling-capable chip count"
}

$Hdl = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL"
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin/vivado.bat"
$Mode = if ($Implement) { "impl" } else { "synth" }
$Topology = "p${PresentChipMask}_r${RiseChipMask}_f${FallChipMask}"
$Session = "${Stamp}_${Label}_w${Width}_a${AxisMhz}_t${TdcMhz}_${Topology}_${Mode}"
$OutDir = "$Hdl/signoff_results/sessions/$Session"
$Log = "$OutDir/vivado.log"

function Assert-StageClosed {
    param(
        [string]$Prefix,
        [string]$Stage,
        [bool]$RequireTiming = $true
    )

    $timing = "$OutDir/${Prefix}_timing_summary.rpt"
    $checkTiming = "$OutDir/${Prefix}_check_timing.rpt"
    $cdcData = "$OutDir/${Prefix}_cdc_data.rpt"

    foreach ($report in @($timing, $checkTiming, $cdcData)) {
        if (-not (Test-Path -LiteralPath $report)) {
            throw "$Stage report is missing: $report"
        }
    }

    $timingMet = Select-String -LiteralPath $timing `
        -Pattern "All user specified timing constraints are met." `
        -SimpleMatch -Quiet
    $hasViolatedPath = Select-String -LiteralPath $timing `
        -Pattern 'Slack \(VIOLATED\)' -Quiet

    if ($RequireTiming -and (-not $timingMet -or $hasViolatedPath)) {
        throw "$Stage timing constraints are not met: $timing"
    }
    if (-not $RequireTiming -and (-not $timingMet -or $hasViolatedPath)) {
        Write-Warning "$Stage timing is advisory for an implementation run; post-route timing remains the hard gate: $timing"
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

function Assert-GeneratedTopology {
    param(
        [string]$UtilizationReport,
        [int]$ExpectedRiseBuilders,
        [int]$ExpectedFallBuilders
    )

    $riseBuilders = @(Select-String -LiteralPath $UtilizationReport `
        -Pattern '^\|\s+gen_chip\[\d+\]\.gen_rise_builder\.u_cell_bld_rise\s+\|').Count
    $fallBuilders = @(Select-String -LiteralPath $UtilizationReport `
        -Pattern '^\|\s+gen_chip\[\d+\]\.gen_fall_builder\.u_cell_bld_fall\s+\|').Count

    if ($riseBuilders -ne $ExpectedRiseBuilders) {
        throw "Expected $ExpectedRiseBuilders rising builders, found $riseBuilders in $UtilizationReport"
    }
    if ($fallBuilders -ne $ExpectedFallBuilders) {
        throw "Expected $ExpectedFallBuilders falling builders, found $fallBuilders in $UtilizationReport"
    }

    $fallStagePatterns = @(
        '^\|\s+gen_fall_face_assembler\.u_face_asm_fall\s+\|'
        '^\|\s+gen_fall_fifo\.u_face_fall_fifo\s+\|'
        '^\|\s+gen_fall_line_packer\.u_line_packer_fall\s+\|'
        '^\|\s+gen_fall_header\.u_header_fall\s+\|'
    )
    $expectedStageRows = if ($ExpectedFallBuilders -eq 0) { 0 } else { $fallStagePatterns.Count }
    $fallStageRows = 0
    foreach ($pattern in $fallStagePatterns) {
        $fallStageRows += @(Select-String -LiteralPath $UtilizationReport -Pattern $pattern).Count
    }
    if ($fallStageRows -ne $expectedStageRows) {
        throw "Expected $expectedStageRows falling output-stage rows, found $fallStageRows in $UtilizationReport"
    }

    @(
        "rise_builders=$riseBuilders"
        "fall_builders=$fallBuilders"
        "fall_output_stage_rows=$fallStageRows"
    ) | Set-Content -Encoding ASCII "$OutDir/generated_topology.txt"
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
    "num_chips=$presentCount"
    "present_chip_mask=$PresentChipMask"
    "rise_chip_mask=$RiseChipMask"
    "fall_chip_mask=$FallChipMask"
    "max_stops_per_chip=$MaxStopsPerChip"
    "max_hits_per_stop=$MaxHitsPerStop"
    "stream_mode=$StreamMode"
    "impl_strategy=$ImplStrategy"
    "implement=$($Implement.IsPresent)"
) | Set-Content -Encoding ASCII "$OutDir/session.properties"

# The local TclStore catalog is damaged on this workstation. Match the parent
# reference flow by exposing every installation pkgIndex directory explicitly.
$TclStore = 'C:/AMDDesignTools/2025.2.1/Vivado/data/XilinxTclStore'
$PackageDirs = Get-ChildItem -LiteralPath $TclStore -Recurse `
    -File -Filter 'pkgIndex.tcl' | ForEach-Object {
        $_.DirectoryName.Replace('\', '/')
    } | Sort-Object -Unique
$TclLibPath = $PackageDirs -join ' '
$env:TCLLIBPATH = if ([string]::IsNullOrWhiteSpace($env:TCLLIBPATH)) {
    $TclLibPath
} else {
    "$TclLibPath $env:TCLLIBPATH"
}

$DoImpl = if ($Implement) { "1" } else { "0" }
& $Vivado -mode batch -nojournal -log $Log `
    -source "$Hdl/scripts/run_ooc_signoff.tcl" `
    -tclargs $OutDir $Width $AxisMhz $TdcMhz `
    $PresentChipMask $RiseChipMask $FallChipMask `
    $MaxStopsPerChip $MaxHitsPerStop $StreamMode $DoImpl $ImplStrategy

if ($LASTEXITCODE -ne 0) {
    throw "Vivado OOC sign-off failed with exit code $LASTEXITCODE. See $Log"
}
if (-not (Select-String -Path $Log -Pattern "OOC_SIGNOFF_SYNTH_PASS" -SimpleMatch -Quiet)) {
    throw "Vivado log is missing OOC_SIGNOFF_SYNTH_PASS: $Log"
}
$RequireSynthTiming = -not $Implement.IsPresent
Assert-StageClosed -Prefix "post_synth" -Stage "Post-synthesis" `
    -RequireTiming $RequireSynthTiming
Assert-GeneratedTopology `
    -UtilizationReport "$OutDir/post_synth_utilization_hier.rpt" `
    -ExpectedRiseBuilders $presentCount `
    -ExpectedFallBuilders $fallActiveCount

if ($Implement -and -not (Select-String -Path $Log -Pattern "OOC_SIGNOFF_IMPL_PASS" -SimpleMatch -Quiet)) {
    throw "Vivado log is missing OOC_SIGNOFF_IMPL_PASS: $Log"
}
if ($Implement) {
    Assert-StageClosed -Prefix "post_route" -Stage "Post-route"
}

Write-Host "OOC sign-off PASS: $OutDir"
