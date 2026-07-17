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
    [pscustomobject]@{ Label = "matrix_clk50_split_async"; Width = 32;  Axis = 50;  Tdc = 200; Stream = "ASYNC" },
    [pscustomobject]@{ Label = "matrix_clk125_150_async"; Width = 32;  Axis = 125; Tdc = 150; Stream = "ASYNC" },
    [pscustomobject]@{ Label = "matrix_w32_200_async"; Width = 32;  Axis = 200; Tdc = 200; Stream = "ASYNC" },
    [pscustomobject]@{ Label = "matrix_w64_200_async"; Width = 64;  Axis = 200; Tdc = 200; Stream = "ASYNC" },
    [pscustomobject]@{ Label = "matrix_w128_200_async"; Width = 128; Axis = 200; Tdc = 200; Stream = "ASYNC" },
    [pscustomobject]@{ Label = "matrix_w128_200_sync";  Width = 128; Axis = 200; Tdc = 200; Stream = "SYNC" }
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
    $sessionName = "${Stamp}_$($case.Label)_w$($case.Width)_a$($case.Axis)_t$($case.Tdc)_dedicated_2x2_synth"
    $session = "$Hdl/signoff_results/sessions/$sessionName"

    Write-Host "=== OOC matrix: $($case.Label) ==="
    if (Test-Path -LiteralPath $session) {
        $properties = Get-Content -LiteralPath "$session/session.properties"
        if ($properties -notcontains "git_head=$gitHead" -or
            $properties -notcontains "git_state=clean") {
            throw "Existing matrix session does not match the current clean commit: $session"
        }
        Write-Host "Reusing verified immutable session: $session"
    }
    else {
        & $Runner `
            -Width $case.Width `
            -AxisMhz $case.Axis `
            -TdcMhz $case.Tdc `
            -SlopeMode "DEDICATED_2X2" `
            -StreamMode $case.Stream `
            -Label $case.Label `
            -Stamp $Stamp
    }

    $timing = "$session/post_synth_timing_summary.rpt"
    $utilReport = "$session/post_synth_utilization_hier.rpt"
    $checkTiming = "$session/post_synth_check_timing.rpt"
    $cdcData = "$session/post_synth_cdc_data.rpt"

    if (-not (Select-String -LiteralPath $timing -Pattern "All user specified timing constraints are met." -SimpleMatch -Quiet)) {
        throw "Timing constraints are not met: $timing"
    }
    if (-not (Select-String -LiteralPath $checkTiming -Pattern "checking unconstrained_internal_endpoints (0)" -SimpleMatch -Quiet)) {
        throw "Unconstrained internal endpoints found: $checkTiming"
    }

    $util = Get-Utilization $utilReport
    $builderCount = @(Select-String -LiteralPath $utilReport -Pattern 'u_cell_bld_(rise|fall)').Count
    $cdc4Count = @(Select-String -LiteralPath $cdcData -Pattern '^CDC-4\s').Count
    $cdc10Count = @(Select-String -LiteralPath $cdcData -Pattern '^CDC-10\s').Count

    $results += [pscustomobject]@{
        Label          = $case.Label
        Width          = $case.Width
        AxisMhz        = $case.Axis
        TdcMhz         = $case.Tdc
        StreamMode     = $case.Stream
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

$results | Format-Table Label, Width, AxisMhz, TdcMhz, StreamMode, AxisWnsNs, TdcWnsNs, TotalLut, Ff, CellBuilders -AutoSize
Write-Host "Representative OOC matrix PASS: $SummaryPath"
