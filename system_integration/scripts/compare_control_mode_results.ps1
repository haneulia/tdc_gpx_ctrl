param(
    [Parameter(Mandatory = $true)]
    [string]$LocalResult,
    [Parameter(Mandatory = $true)]
    [string]$UnifiedResult
)

$ErrorActionPreference = "Stop"

$LocalPath = (Resolve-Path $LocalResult).Path
$UnifiedPath = (Resolve-Path $UnifiedResult).Path
$Local = Get-Content -Raw -LiteralPath $LocalPath | ConvertFrom-Json
$Unified = Get-Content -Raw -LiteralPath $UnifiedPath | ConvertFrom-Json

if ($Local.verdict -ne "PASS" -or $Unified.verdict -ne "PASS") {
    throw "Both control-mode regressions must pass before equivalence comparison"
}
if ($Local.control_mode -ne "local" -or $Unified.control_mode -ne "unified") {
    throw "Expected one local result and one unified result"
}
if ($Local.scenario.scenario_id -ne $Unified.scenario.scenario_id) {
    throw "Control-mode results use different scenarios"
}

# Bring-up duration depends on the selected CSR transaction sequence. These
# counters observe initialization activity, not the steady-state data contract.
$IgnoredMetrics = @(
    "control_mode",
    "md_active_cycles",
    "md_cfg_busy_cycles",
    "md_dec_changes",
    "md_virt_changes",
    # This includes inactive-face and shutdown gaps. The exact steady-state
    # angular lattice is checked by measured_shot_interval_min_clks instead.
    "measured_shot_interval_max_clks"
)

# A local AXI-Lite bank and the unified adapter can release the same committed
# setting one clock apart. Permit that registered-boundary difference only on
# end-to-end completion timing and its directly derived margin.
$OneClockToleranceMetrics = @(
    "shot_to_rise_tlast_min_clks",
    "shot_to_rise_tlast_max_clks",
    "shot_to_fall_tlast_min_clks",
    "shot_to_fall_tlast_max_clks",
    "fire_to_output_max_clks",
    "point_budget_margin_clks"
)

$Failures = [System.Collections.Generic.List[string]]::new()
$MetricNames = @($Local.metrics.PSObject.Properties.Name | Sort-Object)
foreach ($Name in $MetricNames) {
    if ($Name -in $IgnoredMetrics) {
        continue
    }
    if ($null -eq $Unified.metrics.PSObject.Properties[$Name]) {
        $Failures.Add("missing unified metric: $Name")
        continue
    }

    $LocalValue = $Local.metrics.$Name
    $UnifiedValue = $Unified.metrics.$Name
    if ($Name -in $OneClockToleranceMetrics) {
        if ([math]::Abs([int64]$LocalValue - [int64]$UnifiedValue) -gt 1) {
            $Failures.Add("$Name local=$LocalValue unified=$UnifiedValue (tolerance=1)")
        }
    }
    elseif ([string]$LocalValue -ne [string]$UnifiedValue) {
        $Failures.Add("$Name local=$LocalValue unified=$UnifiedValue (exact)")
    }
}

$UnifiedOnly = @($Unified.metrics.PSObject.Properties.Name |
    Where-Object { $_ -notin $MetricNames -and $_ -notin $IgnoredMetrics })
foreach ($Name in $UnifiedOnly) {
    $Failures.Add("missing local metric: $Name")
}

if ($Failures.Count -ne 0) {
    throw "CONTROL_MODE_EQUIVALENCE_FAIL`n$($Failures -join "`n")"
}

Write-Output "CONTROL_MODE_EQUIVALENCE_PASS"
Write-Output "scenario=$($Local.scenario.scenario_id)"
Write-Output "exact_metrics=$($MetricNames.Count - $IgnoredMetrics.Count - $OneClockToleranceMetrics.Count)"
Write-Output "one_clock_tolerance_metrics=$($OneClockToleranceMetrics.Count)"
