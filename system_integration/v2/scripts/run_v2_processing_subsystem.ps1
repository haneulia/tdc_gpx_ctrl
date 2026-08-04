param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$WorkRoot = Join-Path $Hdl "tmp/v2_processing_subsystem"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl `
    "signoff_results/sessions/${Stamp}_v2_processing_subsystem"

New-Item -ItemType Directory -Force -Path $Work | Out-Null

function Invoke-Checked {
    param(
        [string]$Exe,
        [string[]]$ArgList
    )

    & $Exe @ArgList
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Exe $($ArgList -join ' ')"
    }
}

$SynthFiles = @(
    "$Hdl/system_integration/v2/pkg/lidar_build_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_config_types_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_event_types_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_processing_pkg.vhd",
    "$Hdl/system_integration/v2/rtl/proc/motor_virtual_index.vhd",
    "$Hdl/system_integration/v2/rtl/proc/motor_virtual_source.vhd",
    "$Hdl/system_integration/v2/rtl/proc/motor_position_core.vhd",
    "$Hdl/system_integration/v2/rtl/proc/face_tracker.vhd",
    "$Hdl/system_integration/v2/rtl/proc/shot_scheduler.vhd",
    "$Hdl/system_integration/v2/rtl/proc/laser_fire_done_bridge.vhd",
    "$Hdl/system_integration/v2/rtl/proc/laser_registered_pulses.vhd",
    "$Hdl/system_integration/v2/rtl/proc/laser_diagnostics_counter.vhd",
    "$Hdl/system_integration/v2/rtl/proc/laser_executor_core.vhd",
    "$Hdl/system_integration/v2/rtl/proc/laser_executor.vhd",
    "$Hdl/system_integration/v2/rtl/proc/lidar_processing_axis_monitor.vhd",
    "$Hdl/system_integration/v2/rtl/proc/lidar_processing_subsystem.vhd",
    "$Hdl/system_integration/v2/tb/lidar_processing_subsystem_impl.vhd"
)
$SimFiles = @(
    $SynthFiles[0],
    $SynthFiles[1],
    "$Hdl/system_integration/v2/pkg/lidar_config_reference_pkg.vhd",
    $SynthFiles[2],
    $SynthFiles[3],
    $SynthFiles[4],
    $SynthFiles[5],
    $SynthFiles[6],
    $SynthFiles[7],
    $SynthFiles[8],
    $SynthFiles[9],
    $SynthFiles[10],
    $SynthFiles[11],
    $SynthFiles[12],
    $SynthFiles[13],
    $SynthFiles[14],
    $SynthFiles[15],
    "$Hdl/system_integration/v2/rtl/proc/lidar_operation_manager.vhd",
    "$Hdl/system_integration/v2/tb/tb_lidar_processing_subsystem.vhd",
    "$Hdl/system_integration/v2/tb/tb_lidar_processing_subsystem_profiles.vhd"
)

$Project = Join-Path $Work "v2_processing_subsystem.prj"
$ProjectLines = foreach ($File in $SimFiles) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $Project
$RunTcl = Join-Path $Work "run.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl
$CompileLog = Join-Path $Work "xvhdl.log"

$Profiles = @(
    [ordered]@{
        name = "proc150_tdc200"
        proc_mhz = 150
        tdc_mhz = 200
        period_ns = "6.667"
        top = "tb_lidar_processing_subsystem_150_200"
    },
    [ordered]@{
        name = "proc200_tdc150"
        proc_mhz = 200
        tdc_mhz = 150
        period_ns = "5.000"
        top = "tb_lidar_processing_subsystem_200_150"
    }
)

Push-Location $Work
try {
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--relax", "-prj", $Project, "-log", $CompileLog
    )

    foreach ($Profile in $Profiles) {
        $Name = $Profile.name
        $Snapshot = "v2_proc_${Name}_${Stamp}_snap"
        $ElabLog = Join-Path $Work "xelab_${Name}.log"
        $SimLog = Join-Path $Work "xsim_${Name}.log"

        Invoke-Checked "$Vivado/xelab.bat" @(
            "--debug", "off", "--relax", "--mt", "2",
            "--snapshot", $Snapshot,
            "xil_defaultlib.$($Profile.top)",
            "-log", $ElabLog
        )
        Invoke-Checked "$Vivado/xsim.bat" @(
            $Snapshot,
            "-tclbatch", $RunTcl.Replace('\', '/'),
            "-log", $SimLog.Replace('\', '/')
        )

        $SimText = Get-Content -Raw -LiteralPath $SimLog
        foreach ($Scenario in "P50", "P51", "P52", "P53") {
            if ($SimText -notmatch "V2-PROC-$Scenario.*PASS") {
                throw "$Name did not pass $Scenario"
            }
        }
        $Marker = "LIDAR_V2_PROCESSING_SUBSYSTEM_PASS proc_mhz=" +
            "$($Profile.proc_mhz) tdc_mhz=$($Profile.tdc_mhz)"
        if ($SimText -notmatch [regex]::Escape($Marker)) {
            throw "$Name did not report the final PASS marker"
        }
    }
}
finally {
    Pop-Location
}

foreach ($Profile in $Profiles) {
    $Name = $Profile.name
    $Tcl = Join-Path $Work "implement_${Name}.tcl"
    $Timing = Join-Path $Work "timing_${Name}.rpt"
    $Util = Join-Path $Work "utilization_${Name}.rpt"
    $Cdc = Join-Path $Work "cdc_${Name}.rpt"
    $Drc = Join-Path $Work "drc_${Name}.rpt"
    $Metrics = Join-Path $Work "metrics_${Name}.txt"
    $VivadoLog = Join-Path $Work "vivado_${Name}.log"
    $VivadoJournal = Join-Path $Work "vivado_${Name}.jou"
    $ReadFiles = ($SynthFiles | ForEach-Object {
        "{$($_.Replace('\', '/'))}"
    }) -join " "

    @(
        "set_param general.maxThreads 2",
        "lappend ::auto_path {C:/AMDDesignTools/2025.2.1/Vivado/data/XilinxTclStore/support/appinit}",
        "package require ::tclapp::support::appinit 1.2",
        "lappend ::auto_path {C:/AMDDesignTools/2025.2.1/Vivado/data/XilinxTclStore/tclapp/xilinx/xsim}",
        "package require ::tclapp::xilinx::xsim 2.520",
        "lappend ::auto_path {C:/AMDDesignTools/2025.2.1/Vivado/data/XilinxTclStore/tclapp/xilinx/modelsim}",
        "package require ::tclapp::xilinx::modelsim 2.375",
        "lappend ::auto_path {C:/AMDDesignTools/2025.2.1/Vivado/data/XilinxTclStore/tclapp/xilinx/questa}",
        "package require ::tclapp::xilinx::questa 2.339",
        "lappend ::auto_path {C:/AMDDesignTools/2025.2.1/Vivado/data/XilinxTclStore/tclapp/xilinx/ies}",
        "package require ::tclapp::xilinx::ies 4.79",
        "lappend ::auto_path {C:/AMDDesignTools/2025.2.1/Vivado/data/XilinxTclStore/tclapp/xilinx/xcelium}",
        "package require ::tclapp::xilinx::xcelium 11.194",
        "lappend ::auto_path {C:/AMDDesignTools/2025.2.1/Vivado/data/XilinxTclStore/tclapp/xilinx/vcs}",
        "package require ::tclapp::xilinx::vcs 17.7",
        "lappend ::auto_path {C:/AMDDesignTools/2025.2.1/Vivado/data/XilinxTclStore/tclapp/aldec/riviera}",
        "package require ::tclapp::aldec::riviera 1.42",
        "lappend ::auto_path {C:/AMDDesignTools/2025.2.1/Vivado/data/XilinxTclStore/tclapp/aldec/activehdl}",
        "package require ::tclapp::aldec::activehdl 1.42",
        "read_vhdl -vhdl2008 [list $ReadFiles]",
        "synth_design -top lidar_processing_subsystem_impl -part xc7z020clg484-2 -mode out_of_context -flatten_hierarchy rebuilt -generic G_PROC_CLK_MHZ=$($Profile.proc_mhz) -generic G_TDC_CLK_MHZ=$($Profile.tdc_mhz)",
        "create_clock -name proc_clk -period $($Profile.period_ns) [get_ports i_clk]",
        "opt_design",
        "place_design",
        "phys_opt_design",
        "route_design",
        "report_timing_summary -delay_type max -max_paths 20 -file {$($Timing.Replace('\', '/'))}",
        "report_utilization -hierarchical -file {$($Util.Replace('\', '/'))}",
        "report_cdc -details -file {$($Cdc.Replace('\', '/'))}",
        "report_drc -file {$($Drc.Replace('\', '/'))}",
        "set p [get_timing_paths -delay_type max -max_paths 1]",
        "if {[llength `$p] == 0} { error {No setup timing path found} }",
        "set wns [get_property SLACK [lindex `$p 0]]",
        "set latches [get_cells -quiet -hier -filter {REF_NAME == LDCE || REF_NAME == LDPE}]",
        "set async_regs [get_cells -quiet -hier -filter {ASYNC_REG == TRUE}]",
        "set capture_regs [get_cells -quiet -hier -filter {REF_NAME == FDPE && NAME =~ *start_capture_r_reg}]",
        "set capture_pre [get_pins -quiet -of_objects `$capture_regs -filter {REF_PIN_NAME == PRE}]",
        "set preset_sources [all_fanin -flat -startpoints_only -to `$capture_pre]",
        "set preset_cells [all_fanin -flat -only_cells -to `$capture_pre]",
        "set raw_meta_d [get_pins -quiet -hier -filter {NAME =~ *raw_meta_r_reg/D}]",
        "set raw_endpoints [all_fanout -flat -endpoints_only -from [get_ports i_fire_done_raw]]",
        "set raw_direct [expr {[llength `$preset_sources] == 1 && [get_property NAME `$preset_sources] eq {i_fire_done_raw} && [llength `$preset_cells] == 1 && [get_property NAME `$preset_cells] eq [get_property NAME `$capture_regs]}]",
        "set raw_endpoint_ok [expr {[llength `$raw_endpoints] == 2 && [llength `$raw_meta_d] == 1 && [lsearch -exact [get_property NAME `$raw_endpoints] [get_property NAME `$capture_pre]] >= 0 && [lsearch -exact [get_property NAME `$raw_endpoints] [get_property NAME `$raw_meta_d]] >= 0}]",
        "set control_starts [all_fanin -flat -startpoints_only -to [get_ports {o_fire_pulse o_start_tdc o_stop_tdc}]]",
        "set ready_in_control [expr {[lsearch -exact [get_property NAME `$control_starts] {m_mon_axis_tready}] >= 0}]",
        "set fp [open {$($Metrics.Replace('\', '/'))} w]",
        "puts `$fp [format {PROFILE_PROC_MHZ=%d} $($Profile.proc_mhz)]",
        "puts `$fp [format {PROFILE_TDC_MHZ=%d} $($Profile.tdc_mhz)]",
        "puts `$fp [format {WNS_NS=%.3f} `$wns]",
        "puts `$fp [format {LATCH_COUNT=%d} [llength `$latches]]",
        "puts `$fp [format {ASYNC_REG_COUNT=%d} [llength `$async_regs]]",
        "puts `$fp [format {CAPTURE_FDPE_COUNT=%d} [llength `$capture_regs]]",
        "puts `$fp [format {PRESET_RAW_DIRECT=%d} `$raw_direct]",
        "puts `$fp [format {RAW_ASYNC_ENDPOINT_COUNT=%d} [llength `$raw_endpoints]]",
        "puts `$fp [format {RAW_ASYNC_ENDPOINTS_OK=%d} `$raw_endpoint_ok]",
        "puts `$fp [format {MONITOR_READY_IN_CONTROL=%d} `$ready_in_control]",
        "close `$fp",
        "if {`$wns < 0.0} { error [format {Negative WNS: %.3f ns} `$wns] }",
        "if {[llength `$latches] != 0} { error {Inferred latch detected} }",
        "if {[llength `$async_regs] != 16} { error {Expected sixteen ASYNC_REG cells} }",
        "if {[llength `$capture_regs] != 1 || [llength `$capture_pre] != 1} { error {Physical START capture is not one FDPE} }",
        "if {!`$raw_direct} { error {fire_done does not drive the FDPE preset directly} }",
        "if {!`$raw_endpoint_ok} { error {fire_done has unexpected endpoints} }",
        "if {`$ready_in_control} { error {monitor TREADY reached laser control outputs} }",
        "exit"
    ) | Set-Content -Encoding ASCII -LiteralPath $Tcl

    Invoke-Checked "$Vivado/vivado.bat" @(
        "-mode", "batch", "-notrace",
        "-source", $Tcl,
        "-log", $VivadoLog,
        "-journal", $VivadoJournal
    )

    $CdcText = Get-Content -Raw -LiteralPath $Cdc
    $CriticalCount = 0
    foreach ($Match in [regex]::Matches($CdcText,
            '(?m)^CDC-\d+\s+Critical\s+(\d+)')) {
        $CriticalCount += [int]$Match.Groups[1].Value
    }
    Add-Content -Encoding ASCII -LiteralPath $Metrics -Value `
        "CDC_CRITICAL_COUNT=$CriticalCount"
    if ($CriticalCount -ne 0) {
        throw "$Name has $CriticalCount critical CDC paths"
    }
}

New-Item -ItemType Directory -Force -Path $Archive | Out-Null
$Manifest = foreach ($File in $SimFiles) {
    $Item = Get-Item -LiteralPath $File
    $Hash = Get-FileHash -LiteralPath $File -Algorithm SHA256
    [ordered]@{
        path = $Item.FullName.Replace('\', '/')
        bytes = $Item.Length
        sha256 = $Hash.Hash
    }
}
$Manifest | ConvertTo-Json -Depth 3 | Set-Content -Encoding ASCII `
    -LiteralPath (Join-Path $Archive "source_manifest.json")

$Artifacts = @($CompileLog, $Project, $RunTcl)
$Artifacts += Get-ChildItem -File -LiteralPath $Work | Where-Object {
    $_.Name -notmatch '\.backup\.log$' -and
    $_.Name -match '^(xelab|xsim).*\.log$|^implement.*\.tcl$|^(timing|utilization|cdc|drc).*\.rpt$|^metrics.*\.txt$|^vivado_.*\.log$'
} | ForEach-Object { $_.FullName }
Copy-Item -Force -LiteralPath $Artifacts -Destination $Archive

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith($ResolvedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "LIDAR_V2_PROCESSING_SUBSYSTEM_PASS"
Write-Output "Result: $Archive"
