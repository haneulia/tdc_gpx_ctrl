param(
    [string]$Scenario = "",
    [ValidateSet("local", "unified")]
    [string]$ControlMode = "local",
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$Runner = Join-Path $PSScriptRoot "run_smoke.ps1"
if ([string]::IsNullOrWhiteSpace($Scenario)) {
    $Scenario = Join-Path $Hdl "system_integration/scenarios/smoke_external_axis150_tdc200_v001.json"
}
$Scenario = (Resolve-Path $Scenario).Path

$Rows = foreach ($Width in @(32, 64, 128)) {
    $CaseStamp = "${Stamp}_w${Width}"
    Write-Host "=== TDC-GPX output-width integration: ${Width} bit ==="
    & $Runner -Scenario $Scenario -ControlMode $ControlMode `
        -OutputWidth $Width -Stamp $CaseStamp |
        Out-Host

    $ResultPath = Join-Path $Hdl "sim_results/vivado_xsim/sessions/${CaseStamp}_system_integration_smoke/rtl_result.json"
    if (-not (Test-Path -LiteralPath $ResultPath -PathType Leaf)) {
        throw "Missing width-${Width} result: $ResultPath"
    }
    $Result = Get-Content -Raw -LiteralPath $ResultPath | ConvertFrom-Json
    if ($Result.verdict -ne "PASS" -or
        [int]$Result.scenario.output_width_bits -ne $Width) {
        throw "Width-${Width} integration result is inconsistent"
    }

    [pscustomobject]@{
        WidthBits = $Width
        ControlMode = $ControlMode
        AxisMhz   = [int]$Result.scenario.axis_clock_mhz
        TdcMhz    = [int]$Result.scenario.tdc_clock_mhz
        RiseBeats = [int64]$Result.metrics.rise_beats
        FallBeats = [int64]$Result.metrics.fall_beats
        Verdict   = $Result.verdict
        Result    = $ResultPath.Replace('\', '/')
    }
}

$MatrixDir = Join-Path $Hdl "sim_results/vivado_xsim/matrices"
New-Item -ItemType Directory -Force -Path $MatrixDir | Out-Null
$SummaryPath = Join-Path $MatrixDir "${Stamp}_output_width_matrix.csv"
$Rows | Export-Csv -NoTypeInformation -Encoding UTF8 -LiteralPath $SummaryPath
$Rows | Format-Table -AutoSize
Write-Host "TDC_GPX_OUTPUT_WIDTH_MATRIX_PASS"
Write-Host "Summary: $SummaryPath"
