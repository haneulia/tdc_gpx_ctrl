param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$SkipSimulation,
    [switch]$SkipImplementation
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$WorkRoot = Join-Path $Hdl "tmp/v2_gpx_bus"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl "signoff_results/sessions/${Stamp}_v2_gpx_bus"

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
    "$Hdl/tdc_gpx_pkg.vhd",
    "$Hdl/tdc_gpx_cfg_pkg.vhd",
    "$Hdl/tdc_gpx_bus_phy.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_build_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_gpx_pkg.vhd",
    "$Hdl/system_integration/v2/rtl/tdc/lidar_gpx_bus_engine.vhd"
)
$SimFiles = $CommonFiles + @(
    "$Hdl/system_integration/v2/tb/tb_lidar_gpx_bus_engine.vhd"
)
$SynthFiles = $CommonFiles + @(
    "$Hdl/system_integration/v2/tb/lidar_gpx_bus_engine_impl.vhd"
)

$Project = Join-Path $Work "v2_gpx_bus.prj"
$ProjectLines = foreach ($File in $SimFiles) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $Project

$RunTcl = Join-Path $Work "run.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl
$CompileLog = Join-Path $Work "xvhdl.log"

$SimulationProfiles = @(
    "tb_lidar_gpx_bus_engine_200",
    "tb_lidar_gpx_bus_engine_150",
    "tb_lidar_gpx_bus_engine_200_pullup",
    "tb_lidar_gpx_bus_engine_150_pullup"
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
            if ($SimText -notmatch "LIDAR_V2_GPX_BUS_ENGINE_PASS") {
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
        name = "dynamic_tdc200"
        tdc_mhz = 200
        oen_mode = "DYNAMIC_CONNECTED"
        period_ns = "5.000"
    },
    [ordered]@{
        name = "pullup_tdc150"
        tdc_mhz = 150
        oen_mode = "PULLUP_OR_NOT_CONNECTED"
        period_ns = "6.667"
    }
)

if (-not $SkipImplementation) {
    foreach ($Profile in $ImplementationProfiles) {
        $Name = $Profile.name
        $Tcl = Join-Path $Work "implement_${Name}.tcl"
        $Timing = Join-Path $Work "timing_${Name}.rpt"
        $Util = Join-Path $Work "utilization_${Name}.rpt"
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
            "synth_design -top lidar_gpx_bus_engine_impl -part xc7z020clg484-2 -mode out_of_context -flatten_hierarchy none -generic G_TDC_CLK_MHZ=$($Profile.tdc_mhz) -generic G_OEN_MODE=$($Profile.oen_mode)",
            "create_clock -name tdc_clk -period $($Profile.period_ns) [get_ports i_clk]",
            "opt_design",
            "place_design",
            "phys_opt_design",
            "route_design",
            "report_timing_summary -delay_type max -max_paths 20 -file {$($Timing.Replace('\', '/'))}",
            "report_utilization -hierarchical -file {$($Util.Replace('\', '/'))}",
            "report_drc -file {$($Drc.Replace('\', '/'))}",
            "set proven_cells [get_cells -quiet -hier -filter {NAME =~ *u_proven_bus_phy*}]",
            "set latches [get_cells -quiet -hier -filter {REF_NAME == LDCE || REF_NAME == LDPE}]",
            "set paths [get_timing_paths -delay_type max -max_paths 1]",
            "set wns {NA}",
            "set wns_value 0.0",
            "if {[llength `$paths] > 0} { set wns_value [get_property SLACK [lindex `$paths 0]]; set wns [format {%.3f} `$wns_value] }",
            "set fp [open {$($Metrics.Replace('\', '/'))} w]",
            "puts `$fp [format {PROFILE=%s} {$Name}]",
            "puts `$fp [format {TDC_MHZ=%d} $($Profile.tdc_mhz)]",
            "puts `$fp [format {OEN_MODE=%s} {$($Profile.oen_mode)}]",
            "puts `$fp [format {PROVEN_BUS_CELL_COUNT=%d} [llength `$proven_cells]]",
            "puts `$fp [format {LATCH_COUNT=%d} [llength `$latches]]",
            "puts `$fp [format {WNS_NS=%s} `$wns]",
            "close `$fp",
            "if {[llength `$proven_cells] == 0} { error {Proven GPX bus hierarchy is missing} }",
            "if {[llength `$latches] != 0} { error {Inferred latch detected} }",
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
}

New-Item -ItemType Directory -Force -Path $Archive | Out-Null

$Scenario = [ordered]@{
    checkpoint = "H1"
    clock_profiles_mhz = @(200, 150)
    oen_modes = @("DYNAMIC_CONNECTED", "PULLUP_OR_NOT_CONNECTED")
    simulation_profiles = $SimulationProfiles
    implementation_profiles = @($ImplementationProfiles)
}
$Scenario | ConvertTo-Json -Depth 4 | Set-Content -Encoding ASCII -LiteralPath (
    Join-Path $Archive "scenario.json")

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
$Manifest | ConvertTo-Json -Depth 3 | Set-Content -Encoding ASCII -LiteralPath (
    Join-Path $Archive "source_manifest.json")

$Artifacts = @($Project, $RunTcl)
if (Test-Path -LiteralPath $CompileLog) {
    $Artifacts += $CompileLog
}
$Artifacts += Get-ChildItem -File -LiteralPath $Work | Where-Object {
    $_.Name -notmatch '\.backup\.log$' -and
    $_.Name -match '^(xelab|xsim).*\.log$|^implement.*\.tcl$|^(timing|utilization|drc).*\.rpt$|^metrics.*\.txt$|^vivado_.*\.log$'
} | ForEach-Object { $_.FullName }
Copy-Item -Force -LiteralPath $Artifacts -Destination $Archive

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith($ResolvedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "LIDAR_V2_GPX_BUS_PASS"
Write-Output "Result: $Archive"
