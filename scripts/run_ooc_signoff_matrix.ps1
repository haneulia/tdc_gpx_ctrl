# Representative OOC synthesis matrix for the supported clock/width contract.
param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Hdl = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL"
$Runner = "$Hdl/scripts/run_ooc_signoff.ps1"
$MatrixDir = "$Hdl/signoff_results/matrices"
$SummaryPath = "$MatrixDir/${Stamp}_representative_matrix.csv"

$gitChanges = @(git -C $Hdl status --porcelain --untracked-files=normal)
if ($gitChanges.Count -ne 0) {
    throw "Representative sign-off matrix requires a clean Git worktree."
}
$gitHead = (git -C $Hdl rev-parse HEAD).Trim()

$cases = @(
    [pscustomobject]@{ Label = "matrix_clk50_equal_dedicated_async";  Width = 32;  Axis = 50;  Tdc = 50;  Slope = "DEDICATED_2X2";    Stream = "ASYNC"; Implement = $false; ImplStrategy = "DEFAULT" },
    [pscustomobject]@{ Label = "matrix_clk100_equal_shared_async";    Width = 64;  Axis = 100; Tdc = 100; Slope = "SHARED_DUAL_EDGE"; Stream = "ASYNC"; Implement = $false; ImplStrategy = "DEFAULT" },
    [pscustomobject]@{ Label = "matrix_clk125_equal_dedicated_async"; Width = 128; Axis = 125; Tdc = 125; Slope = "DEDICATED_2X2";    Stream = "ASYNC"; Implement = $false; ImplStrategy = "DEFAULT" },
    [pscustomobject]@{ Label = "matrix_clk150_equal_shared_async";    Width = 32;  Axis = 150; Tdc = 150; Slope = "SHARED_DUAL_EDGE"; Stream = "ASYNC"; Implement = $false; ImplStrategy = "DEFAULT" },
    [pscustomobject]@{ Label = "matrix_w32_200_dedicated_async";      Width = 32;  Axis = 200; Tdc = 200; Slope = "DEDICATED_2X2";    Stream = "ASYNC"; Implement = $true;  ImplStrategy = "TIMING_EXPLORE" },
    [pscustomobject]@{ Label = "matrix_w64_200_dedicated_async";      Width = 64;  Axis = 200; Tdc = 200; Slope = "DEDICATED_2X2";    Stream = "ASYNC"; Implement = $true;  ImplStrategy = "TIMING_EXPLORE" },
    [pscustomobject]@{ Label = "matrix_w128_200_dedicated_async";     Width = 128; Axis = 200; Tdc = 200; Slope = "DEDICATED_2X2";    Stream = "ASYNC"; Implement = $true;  ImplStrategy = "TIMING_EXPLORE" },
    [pscustomobject]@{ Label = "matrix_w128_200_shared_async";        Width = 128; Axis = 200; Tdc = 200; Slope = "SHARED_DUAL_EDGE"; Stream = "ASYNC"; Implement = $true;  ImplStrategy = "TIMING_EXPLORE" },
    [pscustomobject]@{ Label = "matrix_w128_50_200_shared_async";     Width = 128; Axis = 50;  Tdc = 200; Slope = "SHARED_DUAL_EDGE"; Stream = "ASYNC"; Implement = $false; ImplStrategy = "DEFAULT" },
    [pscustomobject]@{ Label = "matrix_w128_200_dedicated_sync";      Width = 128; Axis = 200; Tdc = 200; Slope = "DEDICATED_2X2";    Stream = "SYNC";  Implement = $true;  ImplStrategy = "TIMING_EXPLORE" }
)

function Get-IntraClockWns {
    param(
        [string]$Report,
        [string]$Clock
    )
    foreach ($line in Get-Content -LiteralPath $Report) {
        if ($line -match "^\s*$Clock\s+(-?\d+\.\d+)\s+") {
            return [double]$Matches[1]
        }
    }
    throw "Clock '$Clock' not found in timing report: $Report"
}

function Get-Utilization {
    param([string]$Report)
    foreach ($line in Get-Content -LiteralPath $Report) {
        if ($line -match '^\| tdc_gpx_top\s+\|\s+\(top\)\s+\|\s+(\d+)\s+\|\s+(\d+)\s+\|\s+(\d+)\s+\|\s+\d+\s+\|\s+(\d+)\s+\|') {
            return [pscustomobject]@{
                TotalLut = [int]$Matches[1]
                LogicLut = [int]$Matches[2]
                Lutram   = [int]$Matches[3]
                Ff       = [int]$Matches[4]
            }
        }
    }
    throw "Top utilization row not found: $Report"
}

New-Item -ItemType Directory -Force -Path $MatrixDir | Out-Null
$results = @()

foreach ($case in $cases) {
    $safeSlope = $case.Slope.ToLowerInvariant()
    $stageSuffix = if ($case.Implement) { "impl" } else { "synth" }
    $reportPrefix = if ($case.Implement) { "post_route" } else { "post_synth" }
    $sessionName = "${Stamp}_$($case.Label)_w$($case.Width)_a$($case.Axis)_t$($case.Tdc)_${safeSlope}_${stageSuffix}"
    $session = "$Hdl/signoff_results/sessions/$sessionName"

    Write-Host "=== OOC matrix: $($case.Label) ==="
    if (Test-Path -LiteralPath $session) {
        $properties = Get-Content -LiteralPath "$session/session.properties"
        $expectedProperties = @(
            "git_head=$gitHead"
            "git_state=clean"
            "width=$($case.Width)"
            "axis_mhz=$($case.Axis)"
            "tdc_mhz=$($case.Tdc)"
            "slope_mode=$($case.Slope)"
            "stream_mode=$($case.Stream)"
            "impl_strategy=$($case.ImplStrategy)"
            "implement=$($case.Implement)"
        )
        foreach ($property in $expectedProperties) {
            if ($properties -notcontains $property) {
                throw "Existing matrix session property mismatch '$property': $session"
            }
        }
        Write-Host "Reusing verified immutable session: $session"
    }
    else {
        $runnerArgs = @{
            Width        = $case.Width
            AxisMhz      = $case.Axis
            TdcMhz       = $case.Tdc
            SlopeMode    = $case.Slope
            StreamMode   = $case.Stream
            ImplStrategy = $case.ImplStrategy
            Label        = $case.Label
            Stamp        = $Stamp
        }
        if ($case.Implement) {
            $runnerArgs.Implement = $true
        }
        & $Runner @runnerArgs
    }

    $timing = "$session/${reportPrefix}_timing_summary.rpt"
    $utilReport = "$session/${reportPrefix}_utilization_hier.rpt"
    $checkTiming = "$session/${reportPrefix}_check_timing.rpt"
    $cdcData = "$session/${reportPrefix}_cdc_data.rpt"

    if (-not (Select-String -LiteralPath $timing -Pattern "All user specified timing constraints are met." -SimpleMatch -Quiet)) {
        throw "Timing constraints are not met: $timing"
    }
    if (-not (Select-String -LiteralPath $checkTiming -Pattern "checking unconstrained_internal_endpoints (0)" -SimpleMatch -Quiet)) {
        throw "Unconstrained internal endpoints found: $checkTiming"
    }
    if (-not (Select-String -LiteralPath $checkTiming -Pattern "checking no_clock (0)" -SimpleMatch -Quiet)) {
        throw "Registers without clocks found: $checkTiming"
    }

    $util = Get-Utilization $utilReport
    $builderCount = @(Select-String -LiteralPath $utilReport -Pattern 'u_cell_bld_(rise|fall)').Count
    $cdc4Count = @(Select-String -LiteralPath $cdcData -Pattern '^\s*CDC-4\s').Count
    $cdc10Count = @(Select-String -LiteralPath $cdcData -Pattern '^\s*CDC-10\s').Count

    $results += [pscustomobject]@{
        GitHead        = $gitHead
        Label          = $case.Label
        Width          = $case.Width
        AxisMhz        = $case.Axis
        TdcMhz         = $case.Tdc
        SlopeMode      = $case.Slope
        StreamMode     = $case.Stream
        SignoffStage   = $reportPrefix
        ImplStrategy   = $case.ImplStrategy
        AxiWnsNs       = Get-IntraClockWns $timing "axi_clk"
        AxisWnsNs      = Get-IntraClockWns $timing "axis_clk"
        TdcWnsNs       = Get-IntraClockWns $timing "tdc_clk"
        TotalLut       = $util.TotalLut
        LogicLut       = $util.LogicLut
        Lutram         = $util.Lutram
        Ff             = $util.Ff
        CellBuilders   = $builderCount
        Cdc4Rows       = $cdc4Count
        Cdc10Rows      = $cdc10Count
        InternalUnconstrained = 0
        Session        = $sessionName
    }

    $results | Export-Csv -LiteralPath $SummaryPath -NoTypeInformation -Encoding ASCII
}

$results | Format-Table Label, Width, AxisMhz, TdcMhz, SlopeMode, StreamMode, SignoffStage, AxisWnsNs, TdcWnsNs, TotalLut, Ff, CellBuilders -AutoSize
Write-Host "Representative OOC matrix PASS: $SummaryPath"
