param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$WorkRoot = Join-Path $Hdl "tmp/v2_commit_calculator"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl `
    "signoff_results/sessions/${Stamp}_v2_commit_calculator"

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
    "$Hdl/system_integration/v2/rtl/config/lidar_u32_u16_multiplier_seq.vhd",
    "$Hdl/system_integration/v2/rtl/config/lidar_u64_u32_divider_seq.vhd",
    "$Hdl/system_integration/v2/rtl/config/lidar_config_validator_seq.vhd",
    "$Hdl/system_integration/v2/rtl/config/lidar_config_deriver_seq.vhd",
    "$Hdl/system_integration/v2/rtl/config/lidar_commit_calculator.vhd"
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
    "$Hdl/system_integration/v2/tb/tb_lidar_commit_calculator.vhd",
    "$Hdl/system_integration/v2/tb/tb_lidar_commit_calculator_profiles.vhd"
)

$Project = Join-Path $Work "v2_commit_calculator.prj"
$ProjectLines = foreach ($File in $SimFiles) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $Project

$CompileLog = Join-Path $Work "xvhdl.log"
$RunTcl = Join-Path $Work "run.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

$SimProfiles = @(
    [ordered]@{
        name = "150mhz"
        top = "tb_lidar_commit_calculator_150"
        marker = "half_period_ps=3333"
    },
    [ordered]@{
        name = "200mhz"
        top = "tb_lidar_commit_calculator_200"
        marker = "half_period_ps=2500"
    }
)

Push-Location $Work
try {
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--relax", "-prj", $Project, "-log", $CompileLog
    )

    foreach ($Profile in $SimProfiles) {
        $Snapshot = "v2_commit_$($Profile.name)_${Stamp}_snap"
        $ElabLog = Join-Path $Work "xelab_$($Profile.name).log"
        $SimLog = Join-Path $Work "xsim_$($Profile.name).log"

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
        if (($SimText -notmatch "LIDAR_V2_COMMIT_CALCULATOR_PASS") -or
            ($SimText -notmatch [regex]::Escape($Profile.marker))) {
            throw "Commit calculator $($Profile.name) did not report PASS"
        }
    }
}
finally {
    Pop-Location
}

$SynthProfiles = @(
    [ordered]@{ name = "150mhz"; period_ns = "6.667" },
    [ordered]@{ name = "200mhz"; period_ns = "5.000" }
)

foreach ($Profile in $SynthProfiles) {
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
        "synth_design -top lidar_commit_calculator -part xc7z020clg484-2 -mode out_of_context -flatten_hierarchy rebuilt",
        "create_clock -name cfg_clk -period $($Profile.period_ns) [get_ports i_clk]",
        "opt_design",
        "place_design",
        "phys_opt_design",
        "route_design",
        "report_timing_summary -delay_type max -max_paths 10 -file {$($Timing.Replace('\', '/'))}",
        "report_utilization -hierarchical -file {$($Util.Replace('\', '/'))}",
        "report_drc -file {$($Drc.Replace('\', '/'))}",
        "set p [get_timing_paths -delay_type max -max_paths 1]",
        "if {[llength `$p] == 0} { error {No setup timing path found} }",
        "set wns [get_property SLACK [lindex `$p 0]]",
        "set latches [get_cells -quiet -hier -filter {REF_NAME == LDCE || REF_NAME == LDPE}]",
        "set fp [open {$($Metrics.Replace('\', '/'))} w]",
        "puts `$fp [format {WNS_NS=%.3f} `$wns]",
        "puts `$fp [format {LATCH_COUNT=%d} [llength `$latches]]",
        "close `$fp",
        "if {`$wns < 0.0} { error [format {Negative WNS: %.3f ns} `$wns] }",
        "if {[llength `$latches] != 0} { error {Inferred latch detected} }",
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

$Artifacts = @(
    $CompileLog,
    $Project,
    $RunTcl
)
$Artifacts += Get-ChildItem -File -LiteralPath $Work | Where-Object {
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

Write-Output "LIDAR_V2_COMMIT_CALCULATOR_PASS"
Write-Output "Result: $Archive"
