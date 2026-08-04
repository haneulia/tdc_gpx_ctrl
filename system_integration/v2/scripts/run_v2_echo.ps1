param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$SkipSimulation,
    [string]$OnlyImplementation = ""
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$WorkRoot = Join-Path $Hdl "tmp/v2_echo"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl "signoff_results/sessions/${Stamp}_v2_echo"

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
    "$Hdl/system_integration/v2/pkg/lidar_echo_pkg.vhd",
    "$Hdl/system_integration/v2/rtl/proc/echo_delay_profile.vhd",
    "$Hdl/system_integration/v2/rtl/proc/echo_sim_source.vhd",
    "$Hdl/system_integration/v2/rtl/proc/echo_stop_frontend.vhd",
    "$Hdl/system_integration/v2/rtl/proc/echo_diagnostics.vhd",
    "$Hdl/system_integration/v2/rtl/proc/lidar_echo_subsystem.vhd",
    "$Hdl/system_integration/v2/tb/lidar_echo_subsystem_impl.vhd"
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
    "$Hdl/system_integration/v2/tb/tb_lidar_echo_subsystem.vhd"
)

$Project = Join-Path $Work "v2_echo.prj"
$ProjectLines = foreach ($File in $SimFiles) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $Project
$RunTcl = Join-Path $Work "run.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl
$CompileLog = Join-Path $Work "xvhdl.log"

$SimulationProfiles = @(
    "tb_lidar_echo_physical_16_150_200",
    "tb_lidar_echo_physical_32_200_150",
    "tb_lidar_echo_sim_16_150_200",
    "tb_lidar_echo_sim_16_200_150",
    "tb_lidar_echo_disabled_16_150_200"
)

if (-not $SkipSimulation) {
    Push-Location $Work
    try {
        Invoke-Checked "$Vivado/xvhdl.bat" @(
            "--relax", "-prj", $Project, "-log", $CompileLog
        )

        foreach ($Top in $SimulationProfiles) {
            $Snapshot = "${Top}_${Stamp}_snap"
            $ElabLog = Join-Path $Work "xelab_${Top}.log"
            $SimLog = Join-Path $Work "xsim_${Top}.log"

            Invoke-Checked "$Vivado/xelab.bat" @(
                "--debug", "off", "--relax", "--mt", "2",
                "--snapshot", $Snapshot,
                "xil_defaultlib.$Top",
                "-log", $ElabLog
            )
            Invoke-Checked "$Vivado/xsim.bat" @(
                $Snapshot,
                "-tclbatch", $RunTcl.Replace('\', '/'),
                "-log", $SimLog.Replace('\', '/')
            )

            $SimText = Get-Content -Raw -LiteralPath $SimLog
            if ($SimText -notmatch "LIDAR_V2_ECHO_SUBSYSTEM_PASS") {
                throw "$Top did not report the final PASS marker"
            }
        }
    }
    finally {
        Pop-Location
    }
}

$ImplementationProfiles = @(
    [ordered]@{
        name = "physical16_proc150_tdc200"
        proc_mhz = 150
        tdc_mhz = 200
        chips = 2
        receiver = "true"
        simulation = "false"
        expected_ibufds = 16
        expected_profile = 0
        period_ns = "6.667"
    },
    [ordered]@{
        name = "physical32_proc200_tdc150"
        proc_mhz = 200
        tdc_mhz = 150
        chips = 4
        receiver = "true"
        simulation = "false"
        expected_ibufds = 32
        expected_profile = 0
        period_ns = "5.000"
    },
    [ordered]@{
        name = "simulation16_proc150_tdc200"
        proc_mhz = 150
        tdc_mhz = 200
        chips = 2
        receiver = "true"
        simulation = "true"
        expected_ibufds = 16
        expected_profile = 1
        period_ns = "6.667"
    },
    [ordered]@{
        name = "disabled16_proc150_tdc200"
        proc_mhz = 150
        tdc_mhz = 200
        chips = 2
        receiver = "false"
        simulation = "false"
        expected_ibufds = 0
        expected_profile = 0
        period_ns = "6.667"
    }
)

if ($OnlyImplementation -ne "") {
    $ImplementationProfiles = @($ImplementationProfiles | Where-Object {
        $_.name -eq $OnlyImplementation
    })
    if ($ImplementationProfiles.Count -ne 1) {
        throw "Unknown implementation profile: $OnlyImplementation"
    }
}

foreach ($Profile in $ImplementationProfiles) {
    $Name = $Profile.name
    $Tcl = Join-Path $Work "implement_${Name}.tcl"
    $Timing = Join-Path $Work "timing_${Name}.rpt"
    $Util = Join-Path $Work "utilization_${Name}.rpt"
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
        "synth_design -top lidar_echo_subsystem_impl -part xc7z020clg484-2 -mode out_of_context -flatten_hierarchy none -generic G_PROC_CLK_MHZ=$($Profile.proc_mhz) -generic G_TDC_CLK_MHZ=$($Profile.tdc_mhz) -generic G_NUM_CHIPS=$($Profile.chips) -generic G_ENABLE_RECEIVER=$($Profile.receiver) -generic G_ENABLE_SIMULATION=$($Profile.simulation)",
        "create_clock -name proc_clk -period $($Profile.period_ns) [get_ports i_clk]",
        "opt_design",
        "place_design",
        "phys_opt_design",
        "route_design",
        "report_timing_summary -delay_type max -max_paths 20 -file {$($Timing.Replace('\', '/'))}",
        "report_utilization -hierarchical -file {$($Util.Replace('\', '/'))}",
        "set ibufds [get_cells -quiet -hier -filter {REF_NAME == IBUFDS}]",
        "set profile_cells [get_cells -quiet -hier -filter {NAME =~ *u_profile*}]",
        "set latches [get_cells -quiet -hier -filter {REF_NAME == LDCE || REF_NAME == LDPE}]",
        "set stop_fanin [all_fanin -flat -only_cells -to [get_ports -quiet o_tdc_stop*]]",
        "set stop_regs [filter `$stop_fanin {REF_NAME =~ FD* || REF_NAME =~ LD*}]",
        "set paths [get_timing_paths -delay_type max -max_paths 1]",
        "set wns {NA}",
        "set wns_value 0.0",
        "if {[llength `$paths] > 0} { set wns_value [get_property SLACK [lindex `$paths 0]]; set wns [format {%.3f} `$wns_value] }",
        "set fp [open {$($Metrics.Replace('\', '/'))} w]",
        "puts `$fp [format {PROFILE=%s} {$Name}]",
        "puts `$fp [format {PROC_MHZ=%d} $($Profile.proc_mhz)]",
        "puts `$fp [format {TDC_MHZ=%d} $($Profile.tdc_mhz)]",
        "puts `$fp [format {ECHO_RECEIVER=%s} {$($Profile.receiver)}]",
        "puts `$fp [format {ECHO_SIMULATION=%s} {$($Profile.simulation)}]",
        "puts `$fp [format {IBUFDS_COUNT=%d} [llength `$ibufds]]",
        "puts `$fp [format {PROFILE_CELL_COUNT=%d} [llength `$profile_cells]]",
        "puts `$fp [format {STOP_SEQUENTIAL_FANIN_COUNT=%d} [llength `$stop_regs]]",
        "puts `$fp [format {LATCH_COUNT=%d} [llength `$latches]]",
        "puts `$fp [format {WNS_NS=%s} `$wns]",
        "close `$fp",
        "if {[llength `$ibufds] != $($Profile.expected_ibufds)} { error {Unexpected IBUFDS count} }",
        "if {$($Profile.expected_profile) == 0 && [llength `$profile_cells] != 0} { error {Simulation profile logic was not removed} }",
        "if {$($Profile.expected_profile) == 1 && [llength `$profile_cells] == 0} { error {Simulation profile logic is missing} }",
        "if {[llength `$latches] != 0} { error {Inferred latch detected} }",
        "if {$($Profile.receiver) && !$($Profile.simulation) && [llength `$stop_regs] != 0} { error {Registered cell reached physical STOP path} }",
        "if {[llength `$paths] > 0 && `$wns_value < 0.0} { error [format {Negative WNS: %.3f ns} `$wns_value] }",
        "exit"
    ) | Set-Content -Encoding ASCII -LiteralPath $Tcl

    Invoke-Checked "$Vivado/vivado.bat" @(
        "-mode", "batch", "-notrace",
        "-source", $Tcl,
        "-log", $VivadoLog,
        "-journal", $VivadoJournal
    )
}

New-Item -ItemType Directory -Force -Path $Archive | Out-Null
$Manifest = foreach ($File in $SimFiles + $SynthFiles[-1]) {
    $Item = Get-Item -LiteralPath $File
    $Hash = Get-FileHash -LiteralPath $File -Algorithm SHA256
    [ordered]@{
        path = $Item.FullName.Replace('\', '/')
        bytes = $Item.Length
        sha256 = $Hash.Hash
    }
}
$Manifest | Sort-Object path -Unique | ConvertTo-Json -Depth 3 |
    Set-Content -Encoding ASCII -LiteralPath (
        Join-Path $Archive "source_manifest.json")

$Artifacts = @($Project, $RunTcl)
if (Test-Path -LiteralPath $CompileLog) {
    $Artifacts += $CompileLog
}
$Artifacts += Get-ChildItem -File -LiteralPath $Work | Where-Object {
    $_.Name -notmatch '\.backup\.log$' -and
    $_.Name -match '^(xelab|xsim).*\.log$|^implement.*\.tcl$|^(timing|utilization).*\.rpt$|^metrics.*\.txt$|^vivado_.*\.log$'
} | ForEach-Object { $_.FullName }
Copy-Item -Force -LiteralPath $Artifacts -Destination $Archive

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith($ResolvedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "LIDAR_V2_ECHO_PASS"
Write-Output "Result: $Archive"
