param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$SkipSimulation,
    [switch]$SkipImplementation
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Glbl = (Resolve-Path -LiteralPath (
    "C:/AMDDesignTools/2025.2.1/Vivado/data/verilog/src/glbl.v")).Path
$WorkRoot = Join-Path $Hdl "tmp/v2_gpx_b5_b8_subsystem"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl (
    "signoff_results/sessions/${Stamp}_v2_gpx_b5_b8_subsystem")

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

$CommonFiles = @(
    "$Hdl/px_utility_pkg.vhd",
    "$Hdl/tdc_gpx_pkg.vhd",
    "$Hdl/tdc_gpx_cfg_pkg.vhd",
    "$Hdl/tdc_gpx_bus_phy.vhd",
    "$Hdl/tdc_gpx_skid_buffer.vhd",
    "$Hdl/tdc_gpx_chip_init.vhd",
    "$Hdl/tdc_gpx_chip_run.vhd",
    "$Hdl/tdc_gpx_chip_reg.vhd",
    "$Hdl/tdc_gpx_chip_ctrl.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_build_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_config_types_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_config_reference_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_event_types_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_gpx_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_gpx_image_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_gpx_event_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_gpx_data_pkg.vhd",
    "$Hdl/system_integration/v2/rtl/common/lidar_stream_gateway.vhd",
    "$Hdl/system_integration/v2/rtl/tdc/lidar_gpx_bus_engine.vhd",
    "$Hdl/system_integration/v2/rtl/tdc/lidar_gpx_acquisition_lane.vhd",
    "$Hdl/system_integration/v2/rtl/tdc/lidar_gpx_event_merge.vhd",
    "$Hdl/system_integration/v2/rtl/tdc/lidar_gpx_acquisition_coordinator.vhd",
    "$Hdl/system_integration/v2/rtl/tdc/lidar_gpx_shot_gateway.vhd",
    "$Hdl/system_integration/v2/rtl/tdc/lidar_gpx_result_gateway.vhd",
    "$Hdl/system_integration/v2/rtl/tdc/lidar_gpx_stop_gateway.vhd",
    "$Hdl/system_integration/v2/rtl/tdc/lidar_gpx_acquisition_subsystem.vhd",
    "$Hdl/system_integration/v2/rtl/proc/lidar_gpx_hit_decoder.vhd",
    "$Hdl/system_integration/v2/rtl/proc/lidar_gpx_cell_collector.vhd",
    "$Hdl/system_integration/v2/rtl/proc/lidar_gpx_frame_lane_assembler.vhd",
    "$Hdl/system_integration/v2/rtl/proc/lidar_face_close_owner.vhd",
    "$Hdl/system_integration/v2/rtl/proc/lidar_gpx_hit_cell_frame_pipeline.vhd",
    "$Hdl/system_integration/v2/rtl/proc/lidar_gpx_b5_b8_subsystem.vhd"
)
$SimFiles = $CommonFiles + @(
    "$Hdl/system_integration/v2/tb/tb_lidar_face_close_owner.vhd",
    "$Hdl/system_integration/v2/tb/tb_lidar_gpx_b5_b8_subsystem.vhd"
)
$SynthFiles = $CommonFiles + @(
    "$Hdl/system_integration/v2/tb/lidar_gpx_b5_b8_subsystem_impl.vhd"
)

$Project = Join-Path $Work "v2_gpx_b5_b8_subsystem_vhdl.prj"
$VerilogProject = Join-Path $Work "v2_gpx_b5_b8_subsystem_verilog.prj"
$ProjectLines = foreach ($File in $SimFiles) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $Project
@(
    "verilog xil_defaultlib `"$($Glbl.Replace('\', '/'))`"",
    "nosort"
) | Set-Content -Encoding ASCII -LiteralPath $VerilogProject

$RunTcl = Join-Path $Work "run.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

$SimulationProfiles = @(
    "tb_lidar_face_close_owner",
    "tb_lidar_gpx_b5_b8_subsystem_150_200",
    "tb_lidar_gpx_b5_b8_subsystem_200_150",
    "tb_lidar_gpx_b5_b8_subsystem_150_200_all_dual",
    "tb_lidar_gpx_b5_b8_subsystem_200_150_all_dual"
)

if (-not $SkipSimulation) {
    Push-Location $Work
    try {
        Invoke-Checked "$Vivado/xvlog.bat" @(
            "--relax", "-prj", $VerilogProject,
            "-log", (Join-Path $Work "xvlog.log")
        )
        Invoke-Checked "$Vivado/xvhdl.bat" @(
            "--2008", "--relax", "-prj", $Project,
            "-log", (Join-Path $Work "xvhdl.log")
        )

        foreach ($Top in $SimulationProfiles) {
            $Snapshot = "${Top}_${Stamp}_snap"
            $ElabLog = Join-Path $Work "xelab_${Top}.log"
            $SimLog = Join-Path $Work "xsim_${Top}.log"
            Invoke-Checked "$Vivado/xelab.bat" @(
                "--debug", "off", "--relax", "--mt", "2",
                "--snapshot", $Snapshot,
                "xil_defaultlib.$Top", "xil_defaultlib.glbl",
                "-log", $ElabLog
            )
            Invoke-Checked "$Vivado/xsim.bat" @(
                $Snapshot,
                "-tclbatch", $RunTcl.Replace('\', '/'),
                "-log", $SimLog.Replace('\', '/')
            )

            $SimText = Get-Content -Raw -LiteralPath $SimLog
            if ($SimText -notmatch
                "LIDAR_V2_(GPX_B5_B8_SUBSYSTEM|FACE_CLOSE_OWNER)_PASS") {
                throw "$Top did not report the final PASS marker"
            }
            if ($SimText -match "Failure:|Fatal:") {
                throw "$Top reported a simulation failure"
            }
        }
    }
    finally {
        Pop-Location
    }
}

$ImplementationProfiles = @(
    [ordered]@{
        name = "proc150_tdc200"
        proc_mhz = 150
        tdc_mhz = 200
        proc_period = "6.667"
        tdc_period = "5.000"
    },
    [ordered]@{
        name = "proc200_tdc150"
        proc_mhz = 200
        tdc_mhz = 150
        proc_period = "5.000"
        tdc_period = "6.667"
    }
)

if (-not $SkipImplementation) {
    foreach ($Profile in $ImplementationProfiles) {
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
            "synth_design -top lidar_gpx_b5_b8_subsystem_impl -part xc7z020clg484-2 -mode out_of_context -flatten_hierarchy none -generic G_PROC_CLK_MHZ=$($Profile.proc_mhz) -generic G_TDC_CLK_MHZ=$($Profile.tdc_mhz)",
            "create_clock -name proc_clk -period $($Profile.proc_period) [get_ports i_proc_clk]",
            "create_clock -name tdc_clk -period $($Profile.tdc_period) [get_ports i_tdc_clk]",
            "set_clock_groups -asynchronous -group [get_clocks proc_clk] -group [get_clocks tdc_clk]",
            "opt_design",
            "place_design",
            "phys_opt_design",
            "route_design",
            "report_timing_summary -delay_type max -max_paths 20 -file {$($Timing.Replace('\', '/'))}",
            "report_utilization -hierarchical -file {$($Util.Replace('\', '/'))}",
            "report_cdc -details -file {$($Cdc.Replace('\', '/'))}",
            "report_drc -file {$($Drc.Replace('\', '/'))}",
            "set fifo_instances [get_cells -quiet -hier -filter {REF_NAME =~ xpm_fifo_async*}]",
            "set async_regs [get_cells -quiet -hier -filter {ASYNC_REG == TRUE}]",
            "set latches [get_cells -quiet -hier -filter {REF_NAME == LDCE || REF_NAME == LDPE}]",
            "set paths [get_timing_paths -delay_type max -max_paths 1]",
            "if {[llength `$paths] == 0} { error {No setup timing path found} }",
            "set wns [get_property SLACK [lindex `$paths 0]]",
            "set fp [open {$($Metrics.Replace('\', '/'))} w]",
            "puts `$fp [format {PROFILE=%s} {$Name}]",
            "puts `$fp [format {PROC_MHZ=%d} $($Profile.proc_mhz)]",
            "puts `$fp [format {TDC_MHZ=%d} $($Profile.tdc_mhz)]",
            "puts `$fp [format {ASYNC_FIFO_INSTANCE_COUNT=%d} [llength `$fifo_instances]]",
            "puts `$fp [format {ASYNC_REG_COUNT=%d} [llength `$async_regs]]",
            "puts `$fp [format {LATCH_COUNT=%d} [llength `$latches]]",
            "puts `$fp [format {WNS_NS=%.3f} `$wns]",
            "close `$fp",
            "if {[llength `$fifo_instances] != 3} { error {Expected three asynchronous FIFOs} }",
            "if {[llength `$async_regs] == 0} { error {CDC attributes missing} }",
            "if {[llength `$latches] != 0} { error {Inferred latch detected} }",
            "if {`$wns < 0.0} { error [format {Negative WNS: %.3f ns} `$wns] }",
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
        $DrcText = Get-Content -Raw -LiteralPath $Drc
        $BlockingDrcCount = [regex]::Matches($DrcText,
            '(?m)^\|\s*\S+\s*\|\s*(Critical Warning|Error)\s*\|').Count
        Add-Content -Encoding ASCII -LiteralPath $Metrics -Value @(
            "CDC_CRITICAL_COUNT=$CriticalCount",
            "DRC_BLOCKING_COUNT=$BlockingDrcCount"
        )
        if ($CriticalCount -ne 0) {
            throw "$Name has $CriticalCount critical CDC paths"
        }
        if ($BlockingDrcCount -ne 0) {
            throw "$Name has $BlockingDrcCount blocking DRC categories"
        }
    }
}

New-Item -ItemType Directory -Force -Path $Archive | Out-Null
$Scenario = [ordered]@{
    checkpoint = "I4/B5-B8"
    topology = "four GPX Chips; dedicated 2-rise/2-fall functional test"
    timing_topology = "maximum four-Chip dual-edge"
    clocks = @("Processing 150 / TDC 200 MHz", "Processing 200 / TDC 150 MHz")
    data_contract = "28-bit I-Mode raw word; low 17-bit Hit preserved"
    face_close = @(
        "wait for all accepted Shots",
        "registered close transfer",
        "wait for downstream close consumption before scheduler acknowledgement",
        "trailing and all-hole Face closure"
    )
    cursor_optimization = "slot and Chip/STOP feedback cones decoupled"
    simulation_profiles = $SimulationProfiles
    implementation_profiles = @($ImplementationProfiles)
}
$Scenario | ConvertTo-Json -Depth 5 | Set-Content -Encoding ASCII `
    -LiteralPath (Join-Path $Archive "scenario.json")

$ManifestFiles = @($SimFiles + $SynthFiles + $PSCommandPath) |
    Sort-Object -Unique
$Manifest = foreach ($File in $ManifestFiles) {
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

$Artifacts = Get-ChildItem -File -LiteralPath $Work | Where-Object {
    $_.Name -notmatch '\.backup\.log$' -and
    $_.Name -match '^(xvhdl|xvlog|xelab|xsim).*\.log$|^implement.*\.tcl$|^(timing|utilization|cdc|drc).*\.rpt$|^metrics.*\.txt$|^vivado_.*\.log$|\.prj$|^run\.tcl$'
} | ForEach-Object { $_.FullName }
Copy-Item -Force -LiteralPath $Artifacts -Destination $Archive

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith($ResolvedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "LIDAR_V2_GPX_B5_B8_SUBSYSTEM_PASS"
Write-Output "Result: $Archive"
