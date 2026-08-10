param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$SkipHlsSynthesis,
    [string]$VivadoRoot = "C:\AMDDesignTools\2025.2.1\Vivado"
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$RepoRoot = (Resolve-Path (Join-Path $V3Root "..\..")).Path
$V2Root = Join-Path $RepoRoot "system_integration\v2"
$Work = Join-Path $RepoRoot ".work\v3_gpx_cell_collector_impl\$Stamp"
$ToolHome = Join-Path $RepoRoot ".work\v3_cell_impl_tool_home"
$HlsRunner = Join-Path $ScriptDir "run_v3_hls_cell_collector.ps1"
$HlsRtl = Join-Path $RepoRoot (
    ".work\v3_hls_cell_collector_component\hls\syn\verilog")
$Vivado = Join-Path $VivadoRoot "bin\vivado.bat"

if (-not $SkipHlsSynthesis) {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass `
        -File $HlsRunner -Step csynth
    if ($LASTEXITCODE -ne 0) {
        throw "V3 HLS Cell collector synthesis failed"
    }
}

if (-not (Test-Path -LiteralPath $HlsRtl)) {
    throw "Generated HLS RTL directory is missing: $HlsRtl"
}
$HlsFiles = @(Get-ChildItem -File -LiteralPath $HlsRtl -Filter "*.v" |
    Sort-Object -Property Name |
    ForEach-Object { $_.FullName })
if ($HlsFiles.Count -eq 0) {
    throw "Generated HLS RTL files are missing: $HlsRtl"
}

$VhdlFiles = @(
    (Join-Path $V2Root "pkg\lidar_build_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_config_types_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_event_types_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_event_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_data_pkg.vhd"),
    (Join-Path $V3Root "pkg\lidar_v3_hls_contract_pkg.vhd"),
    (Join-Path $V3Root (
        "rtl\bridges\lidar_gpx_cell_collector_hls_adapter.vhd")),
    (Join-Path $V3Root "tb\lidar_gpx_cell_collector_hls_impl.vhd")
)
foreach ($File in $HlsFiles + $VhdlFiles) {
    if (-not (Test-Path -LiteralPath $File)) {
        throw "Required implementation source is missing: $File"
    }
}

New-Item -ItemType Directory -Force -Path $Work | Out-Null
Get-ChildItem -File -LiteralPath $HlsRtl -Filter "*.dat" |
    Copy-Item -Destination $Work -Force

$Profiles = @(
    [ordered]@{ Name = "150mhz"; Mhz = 150; Period = "6.667" },
    [ordered]@{ Name = "200mhz"; Mhz = 200; Period = "5.000" }
)

New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null
$SavedEnvironment = @{
    HOME = $env:HOME
    USERPROFILE = $env:USERPROFILE
    APPDATA = $env:APPDATA
    LOCALAPPDATA = $env:LOCALAPPDATA
}
$env:HOME = $ToolHome
$env:USERPROFILE = $ToolHome
$env:APPDATA = Join-Path $ToolHome "AppData\Roaming"
$env:LOCALAPPDATA = Join-Path $ToolHome "AppData\Local"
New-Item -ItemType Directory -Force -Path $env:APPDATA | Out-Null
New-Item -ItemType Directory -Force -Path $env:LOCALAPPDATA | Out-Null

try {
    foreach ($Profile in $Profiles) {
        $Name = $Profile.Name
        $Tcl = Join-Path $Work "implement_${Name}.tcl"
        $Timing = Join-Path $Work "timing_${Name}.rpt"
        $Utilization = Join-Path $Work "utilization_${Name}.rpt"
        $Drc = Join-Path $Work "drc_${Name}.rpt"
        $Metrics = Join-Path $Work "metrics_${Name}.txt"
        $ReadVerilog = ($HlsFiles | ForEach-Object {
            "{$($_.Replace('\', '/'))}"
        }) -join " "
        $ReadVhdl = ($VhdlFiles | ForEach-Object {
            "{$($_.Replace('\', '/'))}"
        }) -join " "

        @(
            "set_param general.maxThreads 2",
            "set_msg_config -id {Route 35-198} -suppress",
            "cd {$($Work.Replace('\', '/'))}",
            "read_verilog [list $ReadVerilog]",
            "read_vhdl -vhdl2008 [list $ReadVhdl]",
            "synth_design -top lidar_gpx_cell_collector_hls_impl -part xc7z020clg484-2 -mode out_of_context -flatten_hierarchy none -generic G_PROC_CLK_MHZ=$($Profile.Mhz)",
            "create_clock -name proc_clk -period $($Profile.Period) [get_ports i_clk]",
            "set_property HD.CLK_SRC BUFGCTRL_X0Y0 [get_ports i_clk]",
            "opt_design",
            "place_design",
            "phys_opt_design",
            "route_design",
            "report_timing_summary -delay_type max -max_paths 20 -file {$($Timing.Replace('\', '/'))}",
            "report_utilization -hierarchical -file {$($Utilization.Replace('\', '/'))}",
            "report_drc -file {$($Drc.Replace('\', '/'))}",
            "set latches [get_cells -quiet -hier -filter {REF_NAME == LDCE || REF_NAME == LDPE}]",
            "set paths [get_timing_paths -delay_type max -max_paths 1]",
            "set wns_value 0.0",
            "set wns {NA}",
            "if {[llength `$paths] > 0} { set wns_value [get_property SLACK [lindex `$paths 0]]; set wns [format {%.3f} `$wns_value] }",
            "set fp [open {$($Metrics.Replace('\', '/'))} w]",
            "puts `$fp [format {PROFILE=%s} {$Name}]",
            "puts `$fp [format {PROC_MHZ=%d} $($Profile.Mhz)]",
            "puts `$fp [format {LATCH_COUNT=%d} [llength `$latches]]",
            "puts `$fp [format {WNS_NS=%s} `$wns]",
            "close `$fp",
            "if {[llength `$latches] != 0} { error {Inferred latch detected} }",
            "if {[llength `$paths] == 0} { error {No timed path found} }",
            "if {`$wns_value < 0.0} { error [format {Negative WNS: %.3f ns} `$wns_value] }",
            "exit"
        ) | Set-Content -Encoding ASCII -LiteralPath $Tcl

        & $Vivado -mode batch -notrace -source $Tcl `
            -log (Join-Path $Work "vivado_${Name}.log") `
            -journal (Join-Path $Work "vivado_${Name}.jou")
        if ($LASTEXITCODE -ne 0) {
            throw "V3 HLS Cell collector implementation failed: $Name"
        }

        $DrcText = Get-Content -Raw -LiteralPath $Drc
        $BlockingDrcCount = [regex]::Matches(
            $DrcText,
            '(?m)^\|\s*\S+\s*\|\s*(Critical Warning|Error)\s*\|').Count
        Add-Content -Encoding ASCII -LiteralPath $Metrics -Value (
            "DRC_BLOCKING_COUNT=$BlockingDrcCount")
        if ($BlockingDrcCount -ne 0) {
            throw "$Name has $BlockingDrcCount blocking DRC categories"
        }

        Write-Host (
            "LIDAR_V3_GPX_CELL_COLLECTOR_IMPL_PROFILE_PASS profile=$Name")
        Get-Content -LiteralPath $Metrics
    }
}
finally {
    foreach ($Name in $SavedEnvironment.Keys) {
        $Value = $SavedEnvironment[$Name]
        if ($null -eq $Value) {
            Remove-Item "Env:$Name" -ErrorAction SilentlyContinue
        }
        else {
            Set-Item "Env:$Name" $Value
        }
    }
}

Write-Host "LIDAR_V3_GPX_CELL_COLLECTOR_IMPL_PASS"
Write-Host "Result: $Work"
