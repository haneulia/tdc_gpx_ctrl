param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$SkipGoldenCompare
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Glbl = (Resolve-Path -LiteralPath (
    "C:/AMDDesignTools/2025.2.1/Vivado/data/verilog/src/glbl.v")).Path
$OrderFile = Join-Path $PSScriptRoot "v2_rtl_compile_order.txt"
$WorkRoot = Join-Path $Hdl "tmp/v2_k13_operating_matrix"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl (
    "signoff_results/sessions/${Stamp}_v2_k13_operating_matrix")
$Html = Join-Path $Hdl (
    "Doc/cluster_analysis/C08_HDL_HTML_Alignment/" +
    "C08_HDL_HTML_Alignment_260808_V2_Operating_Matrix_Simulator_v027.html")
$Golden = Join-Path $Hdl (
    "system_integration/v2/golden/v2_k13_operating_matrix_golden.json")
$Verifier = Join-Path $PSScriptRoot "verify_v2_k13_operating_matrix.mjs"

function Invoke-Checked {
    param([string]$Exe, [string[]]$ArgList)
    & $Exe @ArgList | Out-Null
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Exe $($ArgList -join ' ')"
    }
}

function Invoke-BatchChecked {
    param([string]$Exe, [string[]]$ArgList)
    $QuotedArgs = foreach ($Arg in $ArgList) {
        if ($Arg -match '[\s=]') {
            '"' + $Arg.Replace('"', '\"') + '"'
        }
        else {
            $Arg
        }
    }
    $CommandLine = '"' + $Exe + '" ' + ($QuotedArgs -join ' ')
    & $env:ComSpec /d /s /c $CommandLine | Out-Null
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $CommandLine"
    }
}

function Get-MetricRecord {
    param(
        [string]$Text,
        [string]$Marker,
        [string]$ScenarioName
    )
    $Line = @($Text -split "`r?`n" | Where-Object {
        $_ -match [regex]::Escape($Marker)
    })
    if ($Line.Count -ne 1) {
        throw "$ScenarioName expected one $Marker line, got $($Line.Count)"
    }

    $Result = [ordered]@{}
    foreach ($Match in [regex]::Matches(
            $Line[0], '([a-z][a-z0-9_]*)=([0-9]+)')) {
        $Result[$Match.Groups[1].Value] =
            [int64]$Match.Groups[2].Value
    }
    return [pscustomobject]$Result
}

function Assert-CleanSimulation {
    param([string]$Text, [string]$ScenarioName)
    if ($Text -match "Failure:|Fatal:") {
        throw "$ScenarioName reported a simulation failure"
    }
}

New-Item -ItemType Directory -Force -Path $Work | Out-Null

$ProductionSourceFiles = foreach ($Line in Get-Content -LiteralPath $OrderFile) {
    $Entry = $Line.Trim()
    if ($Entry.Length -gt 0 -and -not $Entry.StartsWith("#")) {
        (Resolve-Path -LiteralPath (Join-Path $Hdl $Entry)).Path
    }
}
$SourceFiles = @($ProductionSourceFiles)
$SourceFiles += @(
    (Resolve-Path -LiteralPath (Join-Path $Hdl `
        "system_integration/v2/pkg/lidar_config_reference_pkg.vhd")).Path,
    (Resolve-Path -LiteralPath (Join-Path $Hdl `
        "system_integration/v2/tb/tb_tdc_gpx_lidar_ctrl_v2_k05.vhd")).Path,
    (Resolve-Path -LiteralPath (Join-Path $Hdl `
        "system_integration/v2/tb/tb_lidar_gpx_axis_output_subsystem.vhd")).Path
)

$VhdlProject = Join-Path $Work "v2_k13_operating_matrix_vhdl.prj"
$VerilogProject = Join-Path $Work "v2_k13_operating_matrix_verilog.prj"
$ProjectLines = foreach ($File in $SourceFiles) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $VhdlProject
@(
    "verilog xil_defaultlib `"$($Glbl.Replace('\', '/'))`"",
    "nosort"
) | Set-Content -Encoding ASCII -LiteralPath $VerilogProject

$RunTcl = Join-Path $Work "run.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

$ClockProfiles = @(
    [pscustomobject][ordered]@{
        name = "proc150_tdc200"; proc_mhz = 150; tdc_mhz = 200
    },
    [pscustomobject][ordered]@{
        name = "proc200_tdc150"; proc_mhz = 200; tdc_mhz = 150
    }
)

# Pairwise/boundary matrix: every Return is measured at 32 bit, every width at
# Return 7, and three representative round-trip windows at the worst Return.
$AcquisitionMap = [ordered]@{}
foreach ($Profile in $ClockProfiles) {
    foreach ($ReturnCount in 1..7) {
        $Key = "$($Profile.name)_w32_r${ReturnCount}_t1335"
        $AcquisitionMap[$Key] = [pscustomobject][ordered]@{
            name = $Key
            proc_mhz = $Profile.proc_mhz
            tdc_mhz = $Profile.tdc_mhz
            width = 32
            returns = $ReturnCount
            target_5ns = 1335
        }
    }
    foreach ($Width in @(64, 128)) {
        $Key = "$($Profile.name)_w${Width}_r7_t1335"
        $AcquisitionMap[$Key] = [pscustomobject][ordered]@{
            name = $Key
            proc_mhz = $Profile.proc_mhz
            tdc_mhz = $Profile.tdc_mhz
            width = $Width
            returns = 7
            target_5ns = 1335
        }
    }
    foreach ($Target in @(288, 668)) {
        $Key = "$($Profile.name)_w32_r7_t${Target}"
        $AcquisitionMap[$Key] = [pscustomobject][ordered]@{
            name = $Key
            proc_mhz = $Profile.proc_mhz
            tdc_mhz = $Profile.tdc_mhz
            width = 32
            returns = 7
            target_5ns = $Target
        }
    }
}
$AcquisitionScenarios = @($AcquisitionMap.Values)

# Maximum payload intersection for every supported lane topology and width.
$TopologyScenarios = @()
foreach ($ProcMhz in @(150, 200)) {
    foreach ($Width in @(32, 64, 128)) {
        foreach ($Topology in 0..2) {
            $TopologyScenarios += [pscustomobject][ordered]@{
                name = "proc${ProcMhz}_w${Width}_topology${Topology}"
                proc_mhz = $ProcMhz
                width = $Width
                topology = $Topology
                stops = 8
                returns = 7
            }
        }
    }
}

$AcquisitionResults = @()
$TopologyResults = @()
$SummaryLines = [System.Collections.Generic.List[string]]::new()

Push-Location $Work
try {
    Invoke-Checked "$Vivado/xvlog.bat" @(
        "--relax", "-prj", $VerilogProject,
        "-log", (Join-Path $Work "xvlog.log"))
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--2008", "--relax", "-prj", $VhdlProject,
        "-log", (Join-Path $Work "xvhdl.log"))

    foreach ($Scenario in $AcquisitionScenarios) {
        $Snapshot = "$($Scenario.name)_${Stamp}_snap"
        $ElabLog = Join-Path $Work "xelab_$($Scenario.name).log"
        $SimLog = Join-Path $Work "xsim_$($Scenario.name).log"
        $Text = if (Test-Path -LiteralPath $SimLog) {
            Get-Content -Raw -LiteralPath $SimLog
        }
        else {
            ""
        }
        if ($Text -match "Failure:|Fatal:" -or
            $Text -notmatch "LIDAR_V2_K13_METRIC") {
            Invoke-BatchChecked "$Vivado/xelab.bat" @(
                "--debug", "off", "--relax", "--mt", "2",
                "--snapshot", $Snapshot,
                "--generic_top", "G_PROC_CLK_MHZ=$($Scenario.proc_mhz)",
                "--generic_top", "G_TDC_CLK_MHZ=$($Scenario.tdc_mhz)",
                "--generic_top", "G_OUTPUT_WIDTH=$($Scenario.width)",
                "--generic_top", "G_AXIS_STALL_CLKS=0",
                "--generic_top", "G_ACTIVE_RETURNS=$($Scenario.returns)",
                "--generic_top", "G_TARGET_RANGE_5NS=$($Scenario.target_5ns)",
                "xil_defaultlib.tb_tdc_gpx_lidar_ctrl_v2_k06",
                "xil_defaultlib.glbl", "-log", $ElabLog)
            Invoke-Checked "$Vivado/xsim.bat" @(
                $Snapshot, "-tclbatch", $RunTcl.Replace('\', '/'),
                "-log", $SimLog.Replace('\', '/'))
            $Text = Get-Content -Raw -LiteralPath $SimLog
        }
        Assert-CleanSimulation $Text $Scenario.name
        $Metric = Get-MetricRecord $Text "LIDAR_V2_K13_METRIC" `
            $Scenario.name
        $AcquisitionResults += $Metric
        $MetricLine = @($Text -split "`r?`n" | Where-Object {
            $_ -match "LIDAR_V2_K13_METRIC"
        })[0]
        $SummaryLines.Add($MetricLine)
    }

    foreach ($Scenario in $TopologyScenarios) {
        $Snapshot = "$($Scenario.name)_${Stamp}_snap"
        $ElabLog = Join-Path $Work "xelab_$($Scenario.name).log"
        $SimLog = Join-Path $Work "xsim_$($Scenario.name).log"
        $Text = if (Test-Path -LiteralPath $SimLog) {
            Get-Content -Raw -LiteralPath $SimLog
        }
        else {
            ""
        }
        if ($Text -match "Failure:|Fatal:" -or
            $Text -notmatch "LIDAR_V2_K13_TOPOLOGY_METRIC") {
            Invoke-BatchChecked "$Vivado/xelab.bat" @(
                "--debug", "off", "--relax", "--mt", "2",
                "--snapshot", $Snapshot,
                "--generic_top", "G_PROC_MHZ=$($Scenario.proc_mhz)",
                "--generic_top", "G_OUTPUT_WIDTH=$($Scenario.width)",
                "--generic_top", "G_TOPOLOGY=$($Scenario.topology)",
                "--generic_top", "G_ACTIVE_STOPS=$($Scenario.stops)",
                "--generic_top", "G_ACTIVE_RETURNS=$($Scenario.returns)",
                "--generic_top", "G_FOOTER_STALL_CLKS=11",
                "xil_defaultlib.tb_lidar_gpx_axis_output_subsystem",
                "-log", $ElabLog)
            Invoke-Checked "$Vivado/xsim.bat" @(
                $Snapshot, "-tclbatch", $RunTcl.Replace('\', '/'),
                "-log", $SimLog.Replace('\', '/'))
            $Text = Get-Content -Raw -LiteralPath $SimLog
        }
        Assert-CleanSimulation $Text $Scenario.name
        $Metric = Get-MetricRecord $Text `
            "LIDAR_V2_K13_TOPOLOGY_METRIC" $Scenario.name
        $TopologyResults += $Metric
        $MetricLine = @($Text -split "`r?`n" | Where-Object {
            $_ -match "LIDAR_V2_K13_TOPOLOGY_METRIC"
        })[0]
        $SummaryLines.Add($MetricLine)
    }
}
finally {
    Pop-Location
}

$Measurements = [ordered]@{
    schema = "tdc-gpx-lidar-v2-k13-rtl-measurements-v1"
    generated_by = "run_v2_k13_operating_matrix.ps1"
    acquisition_profiles = $AcquisitionResults
    topology_profiles = $TopologyResults
}
$MeasurementPath = Join-Path $Work "rtl_measurements.json"
$Measurements | ConvertTo-Json -Depth 5 |
    Set-Content -Encoding ASCII -LiteralPath $MeasurementPath
$SummaryLines | Set-Content -Encoding ASCII `
    -LiteralPath (Join-Path $Work "rtl_metric_summary.log")

$ComparisonPath = Join-Path $Work "comparison.json"
if (-not $SkipGoldenCompare) {
    foreach ($RequiredPath in @($Html, $Golden, $Verifier)) {
        if (-not (Test-Path -LiteralPath $RequiredPath)) {
            throw "K1-3 comparison input is missing: $RequiredPath"
        }
    }
    Invoke-Checked "node" @(
        $Verifier, $Html, $Golden, $MeasurementPath, $ComparisonPath)
}

New-Item -ItemType Directory -Force -Path $Archive | Out-Null
$ScenarioSummary = [ordered]@{
    checkpoint = "K1-3"
    purpose = "RTL-measured operating matrix versus executable HTML model"
    clock_relationships = @(
        "Processing 150 / TDC 200 MHz",
        "Processing 200 / TDC 150 MHz"
    )
    acquisition_profile_count = $AcquisitionResults.Count
    topology_profile_count = $TopologyResults.Count
    matrix_policy = @(
        "Return 1..7 at 32-bit and both clock relationships",
        "64/128-bit at Return 7 and both clock relationships",
        "target round-trip 288/668/1335 ticks at Return 7",
        "dedicated 2R2F, one-chip dual-edge and four-chip dual-edge",
        "topology intersection uses eight STOPs and seven Returns"
    )
    golden_compare_skipped = [bool]$SkipGoldenCompare
}
$ScenarioSummary | ConvertTo-Json -Depth 5 |
    Set-Content -Encoding ASCII -LiteralPath (
        Join-Path $Archive "scenario.json")
Copy-Item -Force -LiteralPath $MeasurementPath -Destination $Archive
Copy-Item -Force -LiteralPath (Join-Path $Work "rtl_metric_summary.log") `
    -Destination $Archive
if (Test-Path -LiteralPath $ComparisonPath) {
    Copy-Item -Force -LiteralPath $ComparisonPath -Destination $Archive
}

$Manifest = foreach ($File in $SourceFiles) {
    $Item = Get-Item -LiteralPath $File
    $Hash = Get-FileHash -LiteralPath $File -Algorithm SHA256
    [ordered]@{
        path = $Item.FullName.Replace('\', '/')
        bytes = $Item.Length
        sha256 = $Hash.Hash
    }
}
$Manifest | ConvertTo-Json -Depth 3 |
    Set-Content -Encoding ASCII -LiteralPath (
        Join-Path $Archive "source_manifest.json")

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith($ResolvedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "LIDAR_V2_K13_OPERATING_MATRIX_PASS"
Write-Output "Acquisition profiles: $($AcquisitionResults.Count)"
Write-Output "Topology profiles: $($TopologyResults.Count)"
Write-Output "Result: $Archive"
