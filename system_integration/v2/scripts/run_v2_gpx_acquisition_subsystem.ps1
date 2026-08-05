param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$SkipSimulation,
    [switch]$SkipImplementation
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Glbl = (Resolve-Path "C:/AMDDesignTools/2025.2.1/Vivado/data/verilog/src/glbl.v").Path
$WorkRoot = Join-Path $Hdl "tmp/v2_gpx_acquisition_subsystem"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl "signoff_results/sessions/${Stamp}_v2_gpx_acquisition_subsystem"

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
    "$Hdl/system_integration/v2/rtl/common/lidar_stream_gateway.vhd",
    "$Hdl/system_integration/v2/rtl/tdc/lidar_gpx_bus_engine.vhd",
    "$Hdl/system_integration/v2/rtl/tdc/lidar_gpx_acquisition_lane.vhd",
    "$Hdl/system_integration/v2/rtl/tdc/lidar_gpx_event_merge.vhd",
    "$Hdl/system_integration/v2/rtl/tdc/lidar_gpx_acquisition_coordinator.vhd",
    "$Hdl/system_integration/v2/rtl/tdc/lidar_gpx_shot_gateway.vhd",
    "$Hdl/system_integration/v2/rtl/tdc/lidar_gpx_result_gateway.vhd",
    "$Hdl/system_integration/v2/rtl/tdc/lidar_gpx_stop_gateway.vhd",
    "$Hdl/system_integration/v2/rtl/tdc/lidar_gpx_acquisition_subsystem.vhd"
)
$SimFiles = $CommonFiles + @(
    "$Hdl/system_integration/v2/tb/tb_lidar_gpx_acquisition_subsystem.vhd"
)
$SynthFiles = $CommonFiles + @(
    "$Hdl/system_integration/v2/tb/lidar_gpx_acquisition_subsystem_impl.vhd"
)

$Project = Join-Path $Work "v2_gpx_acquisition_subsystem_vhdl.prj"
$VerilogProject = Join-Path $Work "v2_gpx_acquisition_subsystem_verilog.prj"
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
$CompileLog = Join-Path $Work "xvhdl.log"
$VerilogCompileLog = Join-Path $Work "xvlog.log"

$SimulationProfiles = @(
    "tb_lidar_gpx_acquisition_subsystem_150_200",
    "tb_lidar_gpx_acquisition_subsystem_200_150"
)

if (-not $SkipSimulation) {
    Push-Location $Work
    try {
        Invoke-Checked "$Vivado/xvlog.bat" @(
            "--relax", "-prj", $VerilogProject,
            "-log", $VerilogCompileLog
        )
        Invoke-Checked "$Vivado/xvhdl.bat" @(
            "--2008", "--relax", "-prj", $Project,
            "-log", $CompileLog
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
                "LIDAR_V2_GPX_ACQUISITION_SUBSYSTEM_PASS") {
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
            "synth_design -top lidar_gpx_acquisition_subsystem_impl -part xc7z020clg484-2 -mode out_of_context -flatten_hierarchy none -generic G_PROC_CLK_MHZ=$($Profile.proc_mhz) -generic G_TDC_CLK_MHZ=$($Profile.tdc_mhz)",
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
            "if {[llength `$fifo_instances] != 3} { error {Expected exactly Shot, STOP, and result asynchronous FIFOs} }",
            "if {[llength `$async_regs] == 0} { error {CDC synchronizer attributes missing} }",
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
        $SyncMatch = [regex]::Match($CdcText,
            '(?m)^CDC-3\s+Info\s+(\d+)')
        $SyncCount = if ($SyncMatch.Success) {
            [int]$SyncMatch.Groups[1].Value
        } else {
            0
        }
        $Cdc15Match = [regex]::Match($CdcText,
            '(?m)^CDC-15\s+Warning\s+(\d+)')
        $Cdc15Count = if ($Cdc15Match.Success) {
            [int]$Cdc15Match.Groups[1].Value
        } else {
            0
        }
        $DrcText = Get-Content -Raw -LiteralPath $Drc
        $BlockingDrcCount = [regex]::Matches($DrcText,
            '(?m)^\|\s*\S+\s*\|\s*(Critical Warning|Error)\s*\|').Count
        Add-Content -Encoding ASCII -LiteralPath $Metrics -Value @(
            "CDC_CRITICAL_COUNT=$CriticalCount",
            "CDC_SYNC_PATH_COUNT=$SyncCount",
            "CDC15_WARNING_COUNT=$Cdc15Count",
            "DRC_BLOCKING_COUNT=$BlockingDrcCount"
        )
        if ($CriticalCount -ne 0) {
            throw "$Name has $CriticalCount critical CDC paths"
        }
        if ($SyncCount -eq 0) {
            throw "$Name did not recognize synchronized CDC paths"
        }
        if ($BlockingDrcCount -ne 0) {
            throw "$Name has $BlockingDrcCount blocking DRC categories"
        }
    }
}

New-Item -ItemType Directory -Force -Path $Archive | Out-Null
$Scenario = [ordered]@{
    checkpoint = "H3"
    chip_count = 4
    stops_per_chip = 4
    observable_channels = 16
    max_returns_per_stop = 7
    words_per_ififo_capacity = 28
    events_per_chip_capacity = 58
    events_per_shot_capacity = 232
    result_fifo_depth = 256
    echo_delay_profile = [ordered]@{
        register = "CTL20"
        channel_0 = "bits 15:0, 5 ns ticks"
        channel_step = "bits 31:16, 5 ns ticks/channel"
        equation = "delay[channel] = channel_0 + channel * channel_step"
        per_channel_table = $false
    }
    echo_build_options = @(
        "receiver=false: no physical or synthetic Echo path",
        "receiver=true, simulation=false: physical Echo path",
        "receiver=true, simulation=true: physical and synthetic Echo paths"
    )
    simulation_profiles = $SimulationProfiles
    implementation_profiles = @($ImplementationProfiles)
    contracts = @(
        "Processing Shot and STOP cross through named asynchronous gateways",
        "raw 28-bit words and Shot identity remain atomic under backpressure",
        "one full-capacity Shot fits in the result FIFO",
        "physical drain cap is build-derived, not runtime max_hits",
        "malformed FIFO tails are purged after bounded observable data"
    )
}
$Scenario | ConvertTo-Json -Depth 6 | Set-Content -Encoding ASCII `
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

$Artifacts = @($Project, $VerilogProject, $RunTcl)
if (Test-Path -LiteralPath $CompileLog) {
    $Artifacts += $CompileLog
}
if (Test-Path -LiteralPath $VerilogCompileLog) {
    $Artifacts += $VerilogCompileLog
}
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

Write-Output "LIDAR_V2_GPX_ACQUISITION_SUBSYSTEM_PASS"
Write-Output "Result: $Archive"
