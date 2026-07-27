param(
    [string]$Scenario = "",
    [switch]$FullCrossProduct,
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$RunSmoke = Join-Path $PSScriptRoot "run_smoke.ps1"
if ([string]::IsNullOrWhiteSpace($Scenario)) {
    $Scenario = Join-Path $Hdl `
        "system_integration/scenarios/return7_external_axis150_tdc200_v001.json"
}
$Scenario = (Resolve-Path $Scenario).Path
$ScenarioCfg = Get-Content -Raw -LiteralPath $Scenario | ConvertFrom-Json

# The boundary matrix proves the independent dimensions without repeating all
# equivalent combinations: 1..7 Returns use the slowest 32-bit output, then
# the maximum 7-Return payload is repeated at 64 and 128 bits. Use
# -FullCrossProduct when every Return/width pair is required for release audit.
$Cases = [System.Collections.Generic.List[object]]::new()
if ($FullCrossProduct) {
    foreach ($Width in @(32, 64, 128)) {
        foreach ($ReturnCount in 1..7) {
            $Cases.Add([pscustomobject]@{
                output_width_bits = $Width
                returns_per_stop = $ReturnCount
            })
        }
    }
}
else {
    foreach ($ReturnCount in 1..7) {
        $Cases.Add([pscustomobject]@{
            output_width_bits = 32
            returns_per_stop = $ReturnCount
        })
    }
    foreach ($Width in @(64, 128)) {
        $Cases.Add([pscustomobject]@{
            output_width_bits = $Width
            returns_per_stop = 7
        })
    }
}

$Archive = Join-Path $Hdl `
    "sim_results/vivado_xsim/sessions/${Stamp}_return_feasibility_matrix"
New-Item -ItemType Directory -Force -Path $Archive | Out-Null

$Rows = [System.Collections.Generic.List[object]]::new()
foreach ($Case in $Cases) {
    $Width = [int]$Case.output_width_bits
    $ReturnCount = [int]$Case.returns_per_stop
    $RunStamp = "${Stamp}_r${ReturnCount}_w${Width}"

    & $RunSmoke `
        -Scenario $Scenario `
        -OutputWidth $Width `
        -ReturnsPerStop $ReturnCount `
        -MaxHits 7 `
        -Stamp $RunStamp
    if ($LASTEXITCODE -ne 0) {
        throw "Return feasibility run failed: returns=$ReturnCount width=$Width"
    }

    $ResultPath = Join-Path $Hdl `
        "sim_results/vivado_xsim/sessions/${RunStamp}_system_integration_smoke/rtl_result.json"
    $Result = Get-Content -Raw -LiteralPath $ResultPath | ConvertFrom-Json
    $M = $Result.metrics
    $ExpectedRawWords = 4 * 8 * $ReturnCount

    if ($Result.verdict -ne "PASS" -or
        $M.stat5 -ne "00000001" -or
        $M.stat7 -ne "00000000" -or
        [int]$M.schedule_overrun -ne 0 -or
        [int]$M.point_budget_pass -ne 1 -or
        [int]$M.i_mode_words_per_shot -ne $ExpectedRawWords) {
        throw "Return feasibility contract mismatch: returns=$ReturnCount width=$Width"
    }

    $Rows.Add([pscustomobject][ordered]@{
        returns_per_stop = $ReturnCount
        output_width_bits = $Width
        verdict = $Result.verdict
        max_range_5ns_ticks = [int64]$M.max_range_5ns_ticks
        operating_motor_rpm = [int64]$M.operating_motor_rpm
        horizontal_resolution_mdeg = [int64]$M.horizontal_resolution_mdeg
        operating_point_interval_clks = [int64]$M.operating_point_interval_clks
        revolution_period_ns = [int64]$M.revolution_period_ns
        planned_shot_interval_clks = [int64]$M.planned_shot_interval_clks
        measured_shot_interval_min_clks = [int64]$M.measured_shot_interval_min_clks
        fire_done_delay_clks = [int64]$M.fire_done_delay_clks
        range_wait_max_clks = [int64]$M.range_wait_max_clks
        shot_to_rise_tlast_max_clks = [int64]$M.shot_to_rise_tlast_max_clks
        shot_to_fall_tlast_max_clks = [int64]$M.shot_to_fall_tlast_max_clks
        fire_to_output_max_clks = [int64]$M.fire_to_output_max_clks
        point_budget_margin_clks = [int64]$M.point_budget_margin_clks
        point_budget_pass = [int64]$M.point_budget_pass
        tdc_drain_margin_ns = [int64]$M.tdc_drain_margin_ns
        raw_words_per_shot = [int64]$M.i_mode_words_per_shot
        raw_bus_checks = [int64]$M.i_mode_bus_checks
        rise_hit_checks = [int64]$M.i_mode_rise_checks
        fall_hit_checks = [int64]$M.i_mode_fall_checks
        stat5 = [string]$M.stat5
        stat6 = [string]$M.stat6
        stat7 = [string]$M.stat7
        schedule_overrun = [int64]$M.schedule_overrun
        result_path = $ResultPath.Replace('\', '/')
    })
}

$MaxVerified = ($Rows | Measure-Object -Property returns_per_stop -Maximum).Maximum
$Summary = [ordered]@{
    schema_version = 1
    matrix_kind = $(if ($FullCrossProduct) { "full_cross_product" } else { "boundary" })
    axis_clock_mhz = 150
    tdc_clock_mhz = 200
    channels = 32
    apd_channels = 16
    max_verified_returns_per_stop = $MaxVerified
    operating_motor_rpm = [double]$ScenarioCfg.operating_motor_rpm
    horizontal_resolution_deg = [double]$ScenarioCfg.horizontal_resolution_deg
    max_range_m = [double]$ScenarioCfg.max_range_m
    target_distance_m = [double]$ScenarioCfg.target_distance_m
    backpressure_gap_clocks = [int]$ScenarioCfg.backpressure_gap_clocks
    qualification = "Only for the recorded operating RPM, horizontal resolution, maximum range, clocks, topology, and backpressure policy"
    result_count = $Rows.Count
    verdict = "PASS"
    rows = $Rows
}

$Summary | ConvertTo-Json -Depth 6 | Set-Content -Encoding UTF8 `
    -LiteralPath (Join-Path $Archive "return_feasibility_summary.json")
$Rows | Export-Csv -NoTypeInformation -Encoding UTF8 `
    -LiteralPath (Join-Path $Archive "return_feasibility_summary.csv")

Write-Host "RETURN_FEASIBILITY_MATRIX_PASS max_returns=$MaxVerified cases=$($Rows.Count)"
Write-Host "Result: $Archive"
