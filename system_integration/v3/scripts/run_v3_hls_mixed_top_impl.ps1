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
$Work = Join-Path $RepoRoot ".work\v3_hls_mixed_top_impl\$Stamp"
$ToolHome = Join-Path $RepoRoot ".work\v3_h5_impl_tool_home"
$Vivado = Join-Path $VivadoRoot "bin\vivado.bat"

function Invoke-Checked {
    param(
        [Parameter(Mandatory = $true)][string]$Executable,
        [Parameter(Mandatory = $true)][string[]]$ToolArguments
    )
    & $Executable @ToolArguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Executable"
    }
}

$HlsComponents = @(
    [ordered]@{
        Runner = "run_v3_hls_hit_decoder.ps1"
        Rtl = ".work\v3_hls_hit_decoder_component\hls\syn\verilog"
    },
    [ordered]@{
        Runner = "run_v3_hls_cell_collector.ps1"
        Rtl = ".work\v3_hls_cell_collector_component\hls\syn\verilog"
    },
    [ordered]@{
        Runner = "run_v3_hls_frame_assembler.ps1"
        Rtl = ".work\v3_hls_frame_assembler_component\hls\syn\verilog"
    },
    [ordered]@{
        Runner = "run_v3_hls_lane_word_formatter.ps1"
        Rtl = ".work\v3_hls_lane_word_formatter_component\hls\syn\verilog"
    }
)

if (-not $SkipHlsSynthesis) {
    foreach ($Component in $HlsComponents) {
        Invoke-Checked -Executable "powershell.exe" -ToolArguments @(
            "-NoProfile", "-ExecutionPolicy", "Bypass",
            "-File", (Join-Path $ScriptDir $Component.Runner),
            "-Step", "csynth"
        )
    }
}

$HlsFiles = @()
$HlsDataFiles = @()
foreach ($Component in $HlsComponents) {
    $Directory = Join-Path $RepoRoot $Component.Rtl
    if (-not (Test-Path -LiteralPath $Directory)) {
        throw "Generated HLS RTL directory is missing: $Directory"
    }
    $Files = @(Get-ChildItem -File -LiteralPath $Directory -Filter "*.v" |
        Sort-Object -Property Name | ForEach-Object { $_.FullName })
    if ($Files.Count -eq 0) {
        throw "Generated HLS RTL files are missing: $Directory"
    }
    $HlsFiles += $Files
    $HlsDataFiles += @(Get-ChildItem -File -LiteralPath $Directory `
        -Filter "*.dat" | ForEach-Object { $_.FullName })
}
$HlsFiles = @($HlsFiles | Sort-Object -Unique)

$VhdlFiles = @(
    (Join-Path $V2Root "pkg\lidar_build_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_config_types_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_config_reference_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_event_types_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_event_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_data_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_vdma_pkg.vhd"),
    (Join-Path $V3Root "pkg\lidar_v3_hls_contract_pkg.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_skid_buffer.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_sync_fifo.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_axis_word_packer.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_frame_close_fork.vhd"),
    (Join-Path $V3Root "rtl\bridges\lidar_gpx_hit_decoder_hls_adapter.vhd"),
    (Join-Path $V3Root "rtl\bridges\lidar_gpx_cell_collector_hls_adapter.vhd"),
    (Join-Path $V3Root "rtl\bridges\lidar_gpx_frame_lane_assembler_hls_adapter.vhd"),
    (Join-Path $V3Root "rtl\bridges\lidar_gpx_lane_word_formatter_hls_adapter.vhd"),
    (Join-Path $V3Root "rtl\top\lidar_gpx_hls_hit_cell_frame_pipeline.vhd"),
    (Join-Path $V3Root "rtl\top\lidar_gpx_hls_axis_output_subsystem.vhd"),
    (Join-Path $V3Root "rtl\top\lidar_gpx_hls_mixed_data_top.vhd"),
    (Join-Path $V3Root "tb\lidar_gpx_hls_mixed_data_top_impl.vhd")
)
foreach ($File in $HlsFiles + $VhdlFiles) {
    if (-not (Test-Path -LiteralPath $File)) {
        throw "Required implementation source is missing: $File"
    }
}

# V3 Sign-off 출력 폭과 Processing clock을 직교 검증한다.
$Profiles = @(
    [ordered]@{
        Name = "150mhz_32bit"; Mhz = 150; Period = "6.667"; Width = 32
    },
    [ordered]@{
        Name = "200mhz_32bit"; Mhz = 200; Period = "5.000"; Width = 32
    },
    [ordered]@{
        Name = "150mhz_64bit"; Mhz = 150; Period = "6.667"; Width = 64
    },
    [ordered]@{
        Name = "200mhz_64bit"; Mhz = 200; Period = "5.000"; Width = 64
    }
)

New-Item -ItemType Directory -Force -Path $Work | Out-Null
New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null
foreach ($DataFile in $HlsDataFiles) {
    Copy-Item -LiteralPath $DataFile -Destination $Work -Force
}

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
        $RouteStatus = Join-Path $Work "route_status_${Name}.rpt"
        $Checkpoint = Join-Path $Work "post_route_${Name}.dcp"
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
            "synth_design -top lidar_gpx_hls_mixed_data_top_impl -part xc7z020clg484-2 -mode out_of_context -flatten_hierarchy none -generic G_PROC_CLK_MHZ=$($Profile.Mhz) -generic G_OUTPUT_WIDTH=$($Profile.Width)",
            "create_clock -name proc_clk -period $($Profile.Period) [get_ports i_clk]",
            "set_property HD.CLK_SRC BUFGCTRL_X0Y0 [get_ports i_clk]",
            "opt_design",
            "place_design",
            "phys_opt_design",
            "route_design",
            "report_timing_summary -delay_type min_max -max_paths 20 -file {$($Timing.Replace('\', '/'))}",
            "report_utilization -hierarchical -file {$($Utilization.Replace('\', '/'))}",
            "report_route_status -file {$($RouteStatus.Replace('\', '/'))}",
            "report_drc -file {$($Drc.Replace('\', '/'))}",
            "write_checkpoint -force {$($Checkpoint.Replace('\', '/'))}",
            "set latches [get_cells -quiet -hier -filter {REF_NAME == LDCE || REF_NAME == LDPE}]",
            "set setup_paths [get_timing_paths -delay_type max -max_paths 1]",
            "set hold_paths [get_timing_paths -delay_type min -max_paths 1]",
            "set wns_value 0.0",
            "set whs_value 0.0",
            "set wns {NA}",
            "set whs {NA}",
            "if {[llength `$setup_paths] > 0} { set wns_value [get_property SLACK [lindex `$setup_paths 0]]; set wns [format {%.3f} `$wns_value] }",
            "if {[llength `$hold_paths] > 0} { set whs_value [get_property SLACK [lindex `$hold_paths 0]]; set whs [format {%.3f} `$whs_value] }",
            "set route_errors [get_drc_violations -quiet -filter {SEVERITY == Error}]",
            "set fp [open {$($Metrics.Replace('\', '/'))} w]",
            "puts `$fp [format {PROFILE=%s} {$Name}]",
            "puts `$fp [format {PROC_MHZ=%d} $($Profile.Mhz)]",
            "puts `$fp [format {OUTPUT_WIDTH=%d} $($Profile.Width)]",
            "puts `$fp [format {LATCH_COUNT=%d} [llength `$latches]]",
            "puts `$fp [format {WNS_NS=%s} `$wns]",
            "puts `$fp [format {WHS_NS=%s} `$whs]",
            "puts `$fp [format {DRC_ERROR_COUNT=%d} [llength `$route_errors]]",
            "close `$fp",
            "if {[llength `$latches] != 0} { error {Inferred latch detected} }",
            "if {[llength `$setup_paths] == 0 || [llength `$hold_paths] == 0} { error {Timed setup or hold path is missing} }",
            "if {`$wns_value < 0.0 || `$whs_value < 0.0} { error [format {Negative timing slack: WNS=%.3f WHS=%.3f ns} `$wns_value `$whs_value] }",
            "if {[llength `$route_errors] != 0} { error [format {Routed DRC errors: %s} `$route_errors] }",
            "exit"
        ) | Set-Content -Encoding ASCII -LiteralPath $Tcl

        & $Vivado -mode batch -notrace -source $Tcl `
            -log (Join-Path $Work "vivado_${Name}.log") `
            -journal (Join-Path $Work "vivado_${Name}.jou")
        if ($LASTEXITCODE -ne 0) {
            throw "V3 H5 mixed Top implementation failed: $Name"
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

        $RouteText = Get-Content -Raw -LiteralPath $RouteStatus
        $RoutingErrorMatch = [regex]::Match(
            $RouteText,
            '(?m)^\s*# of nets with routing errors\.+\s*:\s*(\d+)\s*:')
        if (-not $RoutingErrorMatch.Success) {
            throw "$Name route report has no routing-error metric"
        }
        $RoutingErrorCount = [int]$RoutingErrorMatch.Groups[1].Value
        Add-Content -Encoding ASCII -LiteralPath $Metrics -Value (
            "ROUTING_ERROR_NET_COUNT=$RoutingErrorCount")
        if ($RoutingErrorCount -ne 0) {
            throw "$Name has $RoutingErrorCount nets with routing errors"
        }

        Write-Host (
            "LIDAR_V3_H5_MIXED_DATA_TOP_IMPL_PROFILE_PASS " +
            "profile=$Name")
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

Write-Host "LIDAR_V3_H5_MIXED_DATA_TOP_IMPL_PASS"
Write-Host "Result: $Work"
