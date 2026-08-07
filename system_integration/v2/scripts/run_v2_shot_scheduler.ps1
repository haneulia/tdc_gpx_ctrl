param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$WorkRoot = Join-Path $Hdl "tmp/v2_shot_scheduler"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl `
    "signoff_results/sessions/${Stamp}_v2_shot_scheduler"

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
    "$Hdl/system_integration/v2/rtl/proc/shot_scheduler.vhd"
)
$SimFiles = @(
    $SynthFiles[0],
    $SynthFiles[1],
    "$Hdl/system_integration/v2/pkg/lidar_config_reference_pkg.vhd",
    $SynthFiles[2],
    "$Hdl/system_integration/v2/rtl/proc/lidar_operation_manager.vhd",
    "$Hdl/system_integration/v2/rtl/proc/face_tracker.vhd",
    $SynthFiles[3],
    "$Hdl/system_integration/v2/tb/tb_shot_scheduler.vhd",
    "$Hdl/system_integration/v2/tb/tb_shot_scheduler_profiles.vhd",
    "$Hdl/system_integration/v2/tb/tb_v2_processing_control_chain.vhd",
    "$Hdl/system_integration/v2/tb/tb_v2_processing_control_chain_profiles.vhd"
)

$Project = Join-Path $Work "v2_shot_scheduler.prj"
$ProjectLines = foreach ($File in $SimFiles) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $Project

$RunTcl = Join-Path $Work "run.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl
$CompileLog = Join-Path $Work "xvhdl.log"

Push-Location $Work
try {
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--relax", "-prj", $Project, "-log", $CompileLog
    )

    foreach ($Mhz in 150, 200) {
        $UnitTop = "tb_shot_scheduler_$Mhz"
        $UnitSnapshot = "v2_sched_${Mhz}_${Stamp}_snap"
        $UnitElabLog = Join-Path $Work "xelab_unit_${Mhz}.log"
        $UnitSimLog = Join-Path $Work "xsim_unit_${Mhz}.log"

        Invoke-Checked "$Vivado/xelab.bat" @(
            "--debug", "off", "--relax", "--mt", "2",
            "--snapshot", $UnitSnapshot,
            "xil_defaultlib.$UnitTop",
            "-log", $UnitElabLog
        )
        Invoke-Checked "$Vivado/xsim.bat" @(
            $UnitSnapshot,
            "-tclbatch", $RunTcl.Replace('\', '/'),
            "-log", $UnitSimLog.Replace('\', '/')
        )

        $UnitText = Get-Content -Raw -LiteralPath $UnitSimLog
        foreach ($Scenario in "P20", "P21", "P22", "P23", "P24", "P25") {
            if ($UnitText -notmatch "V2-SCHED-$Scenario.*PASS") {
                throw "${Mhz} MHz unit test did not pass $Scenario"
            }
        }
        $UnitMarker = "LIDAR_V2_SHOT_SCHEDULER_PASS proc_mhz=$Mhz"
        if ($UnitText -notmatch [regex]::Escape($UnitMarker)) {
            throw "${Mhz} MHz unit test did not report PASS"
        }

        $ChainTop = "tb_v2_processing_control_chain_$Mhz"
        $ChainSnapshot = "v2_sched_chain_${Mhz}_${Stamp}_snap"
        $ChainElabLog = Join-Path $Work "xelab_chain_${Mhz}.log"
        $ChainSimLog = Join-Path $Work "xsim_chain_${Mhz}.log"

        Invoke-Checked "$Vivado/xelab.bat" @(
            "--debug", "off", "--relax", "--mt", "2",
            "--snapshot", $ChainSnapshot,
            "xil_defaultlib.$ChainTop",
            "-log", $ChainElabLog
        )
        Invoke-Checked "$Vivado/xsim.bat" @(
            $ChainSnapshot,
            "-tclbatch", $RunTcl.Replace('\', '/'),
            "-log", $ChainSimLog.Replace('\', '/')
        )

        $ChainText = Get-Content -Raw -LiteralPath $ChainSimLog
        if ($ChainText -notmatch "V2-SCHED-P25.*PASS") {
            throw "${Mhz} MHz chain test did not pass P25"
        }
        $ChainMarker = `
            "LIDAR_V2_PROCESSING_CONTROL_CHAIN_PASS proc_mhz=$Mhz"
        if ($ChainText -notmatch [regex]::Escape($ChainMarker)) {
            throw "${Mhz} MHz chain test did not report PASS"
        }
    }
}
finally {
    Pop-Location
}

$ImplProfiles = @(
    [ordered]@{ name = "150mhz"; period_ns = "6.667" },
    [ordered]@{ name = "200mhz"; period_ns = "5.000" }
)

foreach ($Profile in $ImplProfiles) {
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
        "synth_design -top shot_scheduler -part xc7z020clg484-2 -mode out_of_context -flatten_hierarchy rebuilt",
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
        "set fp [open {$($Metrics.Replace('\', '/'))} w]",
        "puts `$fp [format {WNS_NS=%.3f} `$wns]",
        "puts `$fp [format {LATCH_COUNT=%d} [llength `$latches]]",
        "puts `$fp [format {ASYNC_REG_COUNT=%d} [llength `$async_regs]]",
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

Write-Output "LIDAR_V2_SHOT_SCHEDULER_PASS"
Write-Output "Result: $Archive"
