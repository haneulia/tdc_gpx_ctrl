param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$SkipHlsSynthesis,
    [ValidateSet("", "proc150_tdc200_w32", "proc200_tdc150_w64")]
    [string]$ProfileName = "",
    [ValidateSet("default", "timing_explore")]
    [string]$ImplementationStrategy = "timing_explore",
    [string]$VivadoRoot = "C:\AMDDesignTools\2025.2.1\Vivado"
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$RepoRoot = (Resolve-Path (Join-Path $V3Root "..\..")).Path
$V2Root = Join-Path $RepoRoot "system_integration\v2"
$OrderFile = Join-Path $V2Root "scripts\v2_rtl_compile_order.txt"
$Work = Join-Path $RepoRoot ".work\v3_h6b_integrated_top_impl\$Stamp"
$ToolHome = Join-Path $RepoRoot ".work\v3_h6b_impl_tool_home"
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

$V2Files = foreach ($Line in Get-Content -LiteralPath $OrderFile) {
    $Entry = $Line.Trim()
    if ($Entry.Length -gt 0 -and -not $Entry.StartsWith("#")) {
        (Resolve-Path -LiteralPath (Join-Path $RepoRoot $Entry)).Path
    }
}
$V3Files = @(
    (Join-Path $RepoRoot "px_utility_pkg.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_sync_fifo.vhd"),
    (Join-Path $V3Root "pkg\lidar_v3_hls_contract_pkg.vhd"),
    (Join-Path $V3Root "pkg\lidar_v3_status_pkg.vhd"),
    (Join-Path $V3Root "rtl\status\lidar_v3_processing_status_source.vhd"),
    (Join-Path $V3Root "rtl\status\lidar_v3_status_snapshot_subsystem.vhd"),
    (Join-Path $V3Root "rtl\bridges\lidar_gpx_hit_decoder_hls_adapter.vhd"),
    (Join-Path $V3Root "rtl\bridges\lidar_gpx_cell_collector_hls_adapter.vhd"),
    (Join-Path $V3Root "rtl\bridges\lidar_gpx_frame_lane_assembler_hls_adapter.vhd"),
    (Join-Path $V3Root "rtl\bridges\lidar_gpx_lane_word_formatter_hls_adapter.vhd"),
    (Join-Path $V3Root "rtl\top\lidar_gpx_hls_hit_cell_frame_pipeline.vhd"),
    (Join-Path $V3Root "rtl\top\lidar_gpx_hls_axis_output_subsystem.vhd"),
    (Join-Path $V3Root "rtl\top\lidar_gpx_hls_mixed_data_top.vhd"),
    (Join-Path $V3Root "rtl\top\lidar_gpx_hls_parent_data_subsystem.vhd"),
    (Join-Path $V3Root "rtl\top\tdc_gpx_lidar_ctrl_v3_top.vhd")
)
$VhdlFiles = @($V2Files + $V3Files)

foreach ($File in $HlsFiles + $VhdlFiles) {
    if (-not (Test-Path -LiteralPath $File)) {
        throw "Required implementation source is missing: $File"
    }
}

# 사용자가 정한 제품 회귀 범위: Processing/TDC 150/200 MHz 두 교차 조합.
$Profiles = @(
    [ordered]@{
        Name = "proc150_tdc200_w32"
        ProcMhz = 150; ProcPeriod = "6.667"
        TdcMhz = 200; TdcPeriod = "5.000"
        Width = 32
    },
    [ordered]@{
        Name = "proc200_tdc150_w64"
        ProcMhz = 200; ProcPeriod = "5.000"
        TdcMhz = 150; TdcPeriod = "6.667"
        Width = 64
    }
)
if ($ProfileName -ne "") {
    $Profiles = @($Profiles | Where-Object { $_.Name -eq $ProfileName })
}

New-Item -ItemType Directory -Force -Path $Work | Out-Null
New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null
foreach ($DataFile in $HlsDataFiles) {
    Copy-Item -LiteralPath $DataFile -Destination $Work -Force
}

$ReadVerilog = ($HlsFiles | ForEach-Object {
    "{$($_.Replace('\', '/'))}"
}) -join " "
$ReadVhdl = ($VhdlFiles | ForEach-Object {
    "{$($_.Replace('\', '/'))}"
}) -join " "

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
        $Cdc = Join-Path $Work "cdc_${Name}.rpt"
        $RouteStatus = Join-Path $Work "route_status_${Name}.rpt"
        $Checkpoint = Join-Path $Work "post_route_${Name}.dcp"
        $Metrics = Join-Path $Work "metrics_${Name}.txt"

        if ($ImplementationStrategy -eq "timing_explore") {
            $OptCommand = "opt_design -directive ExploreWithRemap"
            $PlaceCommand = "place_design -directive Explore"
            $PhysOptCommand = "phys_opt_design -directive AggressiveExplore"
            $RouteCommand = "route_design -directive AggressiveExplore -tns_cleanup"
        }
        else {
            $OptCommand = "opt_design"
            $PlaceCommand = "place_design"
            $PhysOptCommand = "phys_opt_design"
            $RouteCommand = "route_design"
        }

        @(
            "set_param general.maxThreads 2",
            "set_msg_config -id {Route 35-198} -suppress",
            "cd {$($Work.Replace('\', '/'))}",
            "read_verilog [list $ReadVerilog]",
            "read_vhdl -vhdl2008 [list $ReadVhdl]",
            "synth_design -top tdc_gpx_lidar_ctrl_v3_top -part xc7z020clg484-2 -mode out_of_context -flatten_hierarchy rebuilt -generic G_NUM_CHIPS=4 -generic G_STOPS_PER_CHIP=8 -generic G_MAX_RETURNS_PER_STOP=7 -generic G_PROC_CLK_MHZ=$($Profile.ProcMhz) -generic G_TDC_CLK_MHZ=$($Profile.TdcMhz) -generic G_OUTPUT_WIDTH=$($Profile.Width) -generic G_ENABLE_ECHO_RECEIVER=false -generic G_ENABLE_ECHO_SIMULATION=false",
            # 실제 Parent XDC가 없는 OOC에서만 GPX I/O register의 IOB 배치를
            # 해제한다. 실보드 Parent는 해당 속성과 package pin을 보존해야 한다.
            "set gpx_iob_cells [get_cells -quiet -hier -filter {NAME =~ *u_proven_bus_phy* && IOB == TRUE}]",
            "if {[llength `$gpx_iob_cells] == 0} { error {No GPX IOB-attributed cell found before OOC relaxation} }",
            "set_property IOB FALSE `$gpx_iob_cells",
            "create_clock -name csr_clk -period 10.000 [get_ports s_axi_csr_aclk]",
            "create_clock -name proc_clk -period $($Profile.ProcPeriod) [get_ports proc_aclk]",
            "create_clock -name tdc_clk -period $($Profile.TdcPeriod) [get_ports i_tdc_clk]",
            "set_clock_groups -asynchronous -group [get_clocks csr_clk] -group [get_clocks proc_clk] -group [get_clocks tdc_clk]",
            "set_property HD.CLK_SRC BUFGCTRL_X0Y2 [get_ports s_axi_csr_aclk]",
            "set_property HD.CLK_SRC BUFGCTRL_X0Y0 [get_ports proc_aclk]",
            "set_property HD.CLK_SRC BUFGCTRL_X0Y1 [get_ports i_tdc_clk]",
            $OptCommand,
            $PlaceCommand,
            $PhysOptCommand,
            $RouteCommand,
            "report_timing_summary -delay_type min_max -max_paths 30 -file {$($Timing.Replace('\', '/'))}",
            "report_utilization -hierarchical -file {$($Utilization.Replace('\', '/'))}",
            "report_route_status -file {$($RouteStatus.Replace('\', '/'))}",
            "report_drc -file {$($Drc.Replace('\', '/'))}",
            "report_cdc -details -file {$($Cdc.Replace('\', '/'))}",
            "write_checkpoint -force {$($Checkpoint.Replace('\', '/'))}",
            "set latches [get_cells -quiet -hier -filter {REF_NAME == LDCE || REF_NAME == LDPE}]",
            "set black_boxes [get_cells -quiet -hier -filter {IS_BLACKBOX == 1}]",
            "set csr_paths [get_timing_paths -from [get_clocks csr_clk] -to [get_clocks csr_clk] -delay_type max -max_paths 1]",
            "set proc_paths [get_timing_paths -from [get_clocks proc_clk] -to [get_clocks proc_clk] -delay_type max -max_paths 1]",
            "set tdc_paths [get_timing_paths -from [get_clocks tdc_clk] -to [get_clocks tdc_clk] -delay_type max -max_paths 1]",
            "set hold_paths [get_timing_paths -delay_type min -max_paths 1]",
            "if {[llength `$csr_paths] == 0 || [llength `$proc_paths] == 0 || [llength `$tdc_paths] == 0 || [llength `$hold_paths] == 0} { error {Timed setup or hold path is missing} }",
            "set csr_wns [get_property SLACK [lindex `$csr_paths 0]]",
            "set proc_wns [get_property SLACK [lindex `$proc_paths 0]]",
            "set tdc_wns [get_property SLACK [lindex `$tdc_paths 0]]",
            "set whs [get_property SLACK [lindex `$hold_paths 0]]",
            "set route_errors [get_drc_violations -quiet -filter {SEVERITY == Error}]",
            "set fp [open {$($Metrics.Replace('\', '/'))} w]",
            "puts `$fp [format {PROFILE=%s} {$Name}]",
            "puts `$fp [format {NUM_CHIPS=%d} 4]",
            "puts `$fp [format {STOPS_PER_CHIP=%d} 8]",
            "puts `$fp [format {MAX_RETURNS_PER_STOP=%d} 7]",
            "puts `$fp [format {OUTPUT_WIDTH=%d} $($Profile.Width)]",
            "puts `$fp [format {CSR_WNS_NS=%.3f} `$csr_wns]",
            "puts `$fp [format {PROC_WNS_NS=%.3f} `$proc_wns]",
            "puts `$fp [format {TDC_WNS_NS=%.3f} `$tdc_wns]",
            "puts `$fp [format {WHS_NS=%.3f} `$whs]",
            "puts `$fp [format {LATCH_COUNT=%d} [llength `$latches]]",
            "puts `$fp [format {BLACK_BOX_COUNT=%d} [llength `$black_boxes]]",
            "puts `$fp [format {GPX_IOB_RELAXED_COUNT=%d} [llength `$gpx_iob_cells]]",
            "puts `$fp [format {DRC_ERROR_COUNT=%d} [llength `$route_errors]]",
            "close `$fp",
            "if {`$csr_wns < 0.0 || `$proc_wns < 0.0 || `$tdc_wns < 0.0 || `$whs < 0.0} { error [format {Negative timing slack: CSR=%.3f PROC=%.3f TDC=%.3f WHS=%.3f ns} `$csr_wns `$proc_wns `$tdc_wns `$whs] }",
            "if {[llength `$latches] != 0} { error {Inferred latch detected} }",
            "if {[llength `$black_boxes] != 0} { error {Black box detected} }",
            "if {[llength `$route_errors] != 0} { error [format {Routed DRC errors: %s} `$route_errors] }",
            "exit"
        ) | Set-Content -Encoding ASCII -LiteralPath $Tcl

        Invoke-Checked -Executable $Vivado -ToolArguments @(
            "-mode", "batch", "-notrace", "-source", $Tcl,
            "-log", (Join-Path $Work "vivado_${Name}.log"),
            "-journal", (Join-Path $Work "vivado_${Name}.jou")
        )

        $CdcText = Get-Content -Raw -LiteralPath $Cdc
        $CriticalCdcCount = 0
        foreach ($Match in [regex]::Matches(
                $CdcText, '(?m)^CDC-\d+\s+Critical\s+(\d+)')) {
            $CriticalCdcCount += [int]$Match.Groups[1].Value
        }

        $DrcText = Get-Content -Raw -LiteralPath $Drc
        $BlockingDrcNames = @(
            [regex]::Matches($DrcText,
                '(?m)^\|\s*(\S+)\s*\|\s*(Critical Warning|Error)\s*\|') |
                ForEach-Object { $_.Groups[1].Value }
        )
        $ParentXdcExclusions = @("IOSTDTYPE-1", "NSTD-1", "UCIO-1")
        $UnexpectedDrcNames = @($BlockingDrcNames | Where-Object {
            $_ -notin $ParentXdcExclusions
        })

        $RouteText = Get-Content -Raw -LiteralPath $RouteStatus
        $RoutingErrorMatch = [regex]::Match($RouteText,
            '(?m)^\s*# of nets with routing errors\.+\s*:\s*(\d+)\s*:')
        if (-not $RoutingErrorMatch.Success) {
            throw "$Name route report has no routing-error metric"
        }
        $RoutingErrorCount = [int]$RoutingErrorMatch.Groups[1].Value

        Add-Content -Encoding ASCII -LiteralPath $Metrics -Value @(
            "CDC_CRITICAL_COUNT=$CriticalCdcCount",
            "DRC_UNEXPECTED_BLOCKING_COUNT=$($UnexpectedDrcNames.Count)",
            "DRC_UNEXPECTED_NAMES=$($UnexpectedDrcNames -join ',')",
            "ROUTING_ERROR_NET_COUNT=$RoutingErrorCount"
        )
        if ($CriticalCdcCount -ne 0) {
            throw "$Name has $CriticalCdcCount critical CDC paths"
        }
        if ($UnexpectedDrcNames.Count -ne 0) {
            throw "$Name has blocking DRC categories: $($UnexpectedDrcNames -join ', ')"
        }
        if ($RoutingErrorCount -ne 0) {
            throw "$Name has $RoutingErrorCount nets with routing errors"
        }

        Write-Host "LIDAR_V3_H6B_INTEGRATED_IMPL_PROFILE_PASS $Name"
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

Write-Host "LIDAR_V3_H6B_INTEGRATED_TOP_IMPL_PASS"
Write-Host "Result: $Work"
