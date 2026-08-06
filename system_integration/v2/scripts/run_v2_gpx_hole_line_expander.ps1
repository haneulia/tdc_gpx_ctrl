param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$SkipSimulation,
    [switch]$SkipImplementation
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$WorkRoot = Join-Path $Hdl "tmp/v2_gpx_hole_line_expander"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl (
    "signoff_results/sessions/${Stamp}_v2_gpx_hole_line_expander")

New-Item -ItemType Directory -Force -Path $Work | Out-Null

function Invoke-Checked {
    param([string]$Exe, [string[]]$ArgList)
    & $Exe @ArgList
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Exe $($ArgList -join ' ')"
    }
}

$CommonFiles = @(
    "$Hdl/system_integration/v2/pkg/lidar_build_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_config_types_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_event_types_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_gpx_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_gpx_event_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_gpx_data_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_gpx_vdma_pkg.vhd",
    "$Hdl/system_integration/v2/rtl/proc/lidar_gpx_hole_line_expander.vhd"
)
$SimFiles = $CommonFiles + @(
    "$Hdl/system_integration/v2/tb/tb_lidar_gpx_hole_line_expander.vhd"
)

$Project = Join-Path $Work "v2_gpx_hole_line_expander_vhdl.prj"
$ProjectLines = foreach ($File in $SimFiles) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $Project
$RunTcl = Join-Path $Work "run.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

$SimulationProfiles = @(
    "tb_lidar_gpx_hole_line_expander_150",
    "tb_lidar_gpx_hole_line_expander_200"
)

if (-not $SkipSimulation) {
    Push-Location $Work
    try {
        Invoke-Checked "$Vivado/xvhdl.bat" @(
            "--relax", "-prj", $Project,
            "-log", (Join-Path $Work "xvhdl.log")
        )
        foreach ($Top in $SimulationProfiles) {
            $Snapshot = "${Top}_${Stamp}_snap"
            $SimLog = Join-Path $Work "xsim_${Top}.log"
            Invoke-Checked "$Vivado/xelab.bat" @(
                "--debug", "off", "--relax", "--mt", "2",
                "--snapshot", $Snapshot, "xil_defaultlib.$Top",
                "-log", (Join-Path $Work "xelab_${Top}.log")
            )
            Invoke-Checked "$Vivado/xsim.bat" @(
                $Snapshot, "-tclbatch", $RunTcl.Replace('\', '/'),
                "-log", $SimLog.Replace('\', '/')
            )
            if ((Get-Content -Raw -LiteralPath $SimLog) -notmatch
                "LIDAR_V2_GPX_HOLE_LINE_EXPANDER_PASS") {
                throw "$Top did not report the final PASS marker"
            }
        }
    }
    finally {
        Pop-Location
    }
}

$ImplementationProfiles = @(
    [ordered]@{ name = "150mhz"; mhz = 150; period = "6.667" },
    [ordered]@{ name = "200mhz"; mhz = 200; period = "5.000" }
)

if (-not $SkipImplementation) {
    foreach ($Profile in $ImplementationProfiles) {
        $Name = $Profile.name
        $Tcl = Join-Path $Work "implement_${Name}.tcl"
        $Timing = Join-Path $Work "timing_${Name}.rpt"
        $Util = Join-Path $Work "utilization_${Name}.rpt"
        $Drc = Join-Path $Work "drc_${Name}.rpt"
        $Metrics = Join-Path $Work "metrics_${Name}.txt"
        $ReadFiles = ($CommonFiles | ForEach-Object {
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
            "synth_design -top lidar_gpx_hole_line_expander -part xc7z020clg484-2 -mode out_of_context -flatten_hierarchy none",
            "create_clock -name proc_clk -period $($Profile.period) [get_ports i_clk]",
            "opt_design",
            "place_design",
            "phys_opt_design",
            "route_design",
            "report_timing_summary -delay_type max -max_paths 20 -file {$($Timing.Replace('\', '/'))}",
            "report_utilization -hierarchical -file {$($Util.Replace('\', '/'))}",
            "report_drc -file {$($Drc.Replace('\', '/'))}",
            "set latches [get_cells -quiet -hier -filter {REF_NAME == LDCE || REF_NAME == LDPE}]",
            "set paths [get_timing_paths -delay_type max -max_paths 1]",
            "set wns {NA}",
            "set wns_value 0.0",
            "if {[llength `$paths] > 0} { set wns_value [get_property SLACK [lindex `$paths 0]]; set wns [format {%.3f} `$wns_value] }",
            "set fp [open {$($Metrics.Replace('\', '/'))} w]",
            "puts `$fp [format {PROFILE=%s} {$Name}]",
            "puts `$fp [format {PROC_MHZ=%d} $($Profile.mhz)]",
            "puts `$fp [format {LATCH_COUNT=%d} [llength `$latches]]",
            "puts `$fp [format {WNS_NS=%s} `$wns]",
            "close `$fp",
            "if {[llength `$latches] != 0} { error {Inferred latch detected} }",
            "if {[llength `$paths] > 0 && `$wns_value < 0.0} { error [format {Negative WNS: %.3f ns} `$wns_value] }",
            "exit"
        ) | Set-Content -Encoding ASCII -LiteralPath $Tcl

        Invoke-Checked "$Vivado/vivado.bat" @(
            "-mode", "batch", "-notrace", "-source", $Tcl,
            "-log", (Join-Path $Work "vivado_${Name}.log"),
            "-journal", (Join-Path $Work "vivado_${Name}.jou")
        )

        $DrcText = Get-Content -Raw -LiteralPath $Drc
        $BlockingDrcCount = [regex]::Matches($DrcText,
            '(?m)^\|\s*\S+\s*\|\s*(Critical Warning|Error)\s*\|').Count
        Add-Content -Encoding ASCII -LiteralPath $Metrics -Value (
            "DRC_BLOCKING_COUNT=$BlockingDrcCount")
        if ($BlockingDrcCount -ne 0) {
            throw "$Name has $BlockingDrcCount blocking DRC categories"
        }
    }
}

New-Item -ItemType Directory -Force -Path $Archive | Out-Null
$Scenario = [ordered]@{
    checkpoint = "J5B explicit geometric Hole Lines"
    processing_clocks_mhz = @(150, 200)
    cases = @("leading", "interior", "trailing", "all-hole", "abort")
    hole_position = "0xFFFF"
    hole_timestamp_valid = $false
    output_word_bits = 32
}
$Scenario | ConvertTo-Json -Depth 5 | Set-Content -Encoding ASCII -LiteralPath (
    Join-Path $Archive "scenario.json")

$Manifest = foreach ($File in @($SimFiles + $PSCommandPath) |
        Sort-Object -Unique) {
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

$Artifacts = Get-ChildItem -File -LiteralPath $Work | Where-Object {
    $_.Name -notmatch '\.backup\.log$' -and
    $_.Name -match '^(xvhdl|xelab|xsim).*\.log$|^implement.*\.tcl$|^(timing|utilization|drc).*\.rpt$|^metrics.*\.txt$|^vivado_.*\.log$|\.prj$|^run\.tcl$'
} | ForEach-Object { $_.FullName }
if ($Artifacts) {
    Copy-Item -Force -LiteralPath $Artifacts -Destination $Archive
}

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith($ResolvedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "LIDAR_V2_GPX_HOLE_LINE_EXPANDER_PASS"
Write-Output "Result: $Archive"
