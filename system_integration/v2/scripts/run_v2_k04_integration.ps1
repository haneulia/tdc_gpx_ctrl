param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$SkipSimulation,
    [switch]$SkipImplementation,
    [string]$OnlyImplementation = ""
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Glbl = (Resolve-Path -LiteralPath (
    "C:/AMDDesignTools/2025.2.1/Vivado/data/verilog/src/glbl.v")).Path
$OrderFile = Join-Path $PSScriptRoot "v2_rtl_compile_order.txt"
$WorkRoot = Join-Path $Hdl "tmp/v2_k04_integration"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl (
    "signoff_results/sessions/${Stamp}_v2_k04_integration")

function Invoke-Checked {
    param([string]$Exe, [string[]]$ArgList)
    & $Exe @ArgList
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Exe $($ArgList -join ' ')"
    }
}

function Invoke-BatchChecked {
    param([string]$Exe, [string[]]$ArgList)
    $QuotedArgs = foreach ($Arg in $ArgList) {
        if ($Arg -match '[\s=]') {
            '"' + $Arg.Replace('"', '\"') + '"'
        }
        else {
            $Arg
        }
    }
    $CommandLine = '"' + $Exe + '" ' + ($QuotedArgs -join ' ')
    & $env:ComSpec /d /s /c $CommandLine
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $CommandLine"
    }
}

New-Item -ItemType Directory -Force -Path $Work | Out-Null

$ProductionSourceFiles = foreach ($Line in Get-Content -LiteralPath $OrderFile) {
    $Entry = $Line.Trim()
    if ($Entry.Length -gt 0 -and -not $Entry.StartsWith("#")) {
        (Resolve-Path -LiteralPath (Join-Path $Hdl $Entry)).Path
    }
}
$SourceFiles = @($ProductionSourceFiles)
$SourceFiles += @(
    (Resolve-Path -LiteralPath (Join-Path $Hdl `
        "system_integration/v2/pkg/lidar_config_reference_pkg.vhd")).Path,
    (Resolve-Path -LiteralPath (Join-Path $Hdl `
        "system_integration/v2/tb/tb_lidar_processing_activation_barrier.vhd")).Path,
    (Resolve-Path -LiteralPath (Join-Path $Hdl `
        "system_integration/v2/tb/tb_tdc_gpx_lidar_ctrl_v2_k04.vhd")).Path
)

$Project = Join-Path $Work "v2_k04_integration.prj"
$VerilogProject = Join-Path $Work "v2_k04_integration_verilog.prj"
$ProjectLines = foreach ($File in $SourceFiles) {
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

$Profiles = @(
    [ordered]@{
        name = "proc150_tdc200"
        proc = 150
        tdc = 200
        proc_period = "6.667"
        tdc_period = "5.000"
    },
    [ordered]@{
        name = "proc200_tdc150"
        proc = 200
        tdc = 150
        proc_period = "5.000"
        tdc_period = "6.667"
    }
)
$Modes = @(
    [ordered]@{ name = "physical"; simulation = "false" },
    [ordered]@{ name = "simulation"; simulation = "true" }
)
$ImplementationNames = foreach ($Profile in $Profiles) {
    foreach ($Mode in $Modes) {
        "top_k04_$($Mode.name)_$($Profile.name)"
    }
}
if ($SkipImplementation -and $OnlyImplementation -ne "") {
    throw "-SkipImplementation and -OnlyImplementation cannot be combined"
}
if ($OnlyImplementation -ne "" -and
    $OnlyImplementation -notin $ImplementationNames) {
    throw "Unknown implementation profile '$OnlyImplementation'. Valid values: $($ImplementationNames -join ', ')"
}

if (-not $SkipSimulation) {
    Push-Location $Work
    try {
        Invoke-Checked "$Vivado/xvlog.bat" @(
            "--relax", "-prj", $VerilogProject,
            "-log", (Join-Path $Work "xvlog.log"))
        Invoke-Checked "$Vivado/xvhdl.bat" @(
            "--2008", "--relax", "-prj", $Project,
            "-log", (Join-Path $Work "xvhdl.log"))

        $BarrierSnapshot = "barrier_${Stamp}_snap"
        $BarrierElab = Join-Path $Work "xelab_barrier.log"
        $BarrierSim = Join-Path $Work "xsim_barrier.log"
        Invoke-BatchChecked "$Vivado/xelab.bat" @(
            "--debug", "off", "--relax", "--mt", "2",
            "--snapshot", $BarrierSnapshot,
            "xil_defaultlib.tb_lidar_processing_activation_barrier",
            "xil_defaultlib.glbl",
            "-log", $BarrierElab)
        Invoke-Checked "$Vivado/xsim.bat" @(
            $BarrierSnapshot, "-tclbatch", $RunTcl.Replace('\', '/'),
            "-log", $BarrierSim.Replace('\', '/'))
        $BarrierText = Get-Content -Raw -LiteralPath $BarrierSim
        if ($BarrierText -notmatch "LIDAR_V2_PROCESSING_ACTIVATION_BARRIER_PASS" -or
            $BarrierText -match "Failure:|Fatal:") {
            throw "Processing activation barrier regression failed"
        }

        foreach ($Profile in $Profiles) {
            foreach ($Mode in $Modes) {
                $Stem = "top_k04_$($Mode.name)_$($Profile.name)"
                $Snapshot = "${Stem}_${Stamp}_snap"
                $ElabLog = Join-Path $Work "xelab_${Stem}.log"
                $SimLog = Join-Path $Work "xsim_${Stem}.log"
                Invoke-BatchChecked "$Vivado/xelab.bat" @(
                    "--debug", "off", "--relax", "--mt", "2",
                    "--snapshot", $Snapshot,
                    "--generic_top", "G_PROC_CLK_MHZ=$($Profile.proc)",
                    "--generic_top", "G_TDC_CLK_MHZ=$($Profile.tdc)",
                    "--generic_top", "G_SIMULATION_MODE=$($Mode.simulation)",
                    "xil_defaultlib.tb_tdc_gpx_lidar_ctrl_v2_k04",
                    "xil_defaultlib.glbl",
                    "-log", $ElabLog)
                Invoke-Checked "$Vivado/xsim.bat" @(
                    $Snapshot, "-tclbatch", $RunTcl.Replace('\', '/'),
                    "-log", $SimLog.Replace('\', '/'))

                $Text = Get-Content -Raw -LiteralPath $SimLog
                $Marker = "LIDAR_V2_TOP_K04_INTEGRATION_PASS " +
                    "mode=$($Mode.name) proc_mhz=$($Profile.proc) " +
                    "tdc_mhz=$($Profile.tdc)"
                if ($Text -notmatch [regex]::Escape($Marker)) {
                    throw "$Stem did not report its PASS marker"
                }
                if ($Text -match "Failure:|Fatal:") {
                    throw "$Stem reported a failure"
                }
            }
        }
    }
    finally {
        Pop-Location
    }
}

$ReadFiles = ($ProductionSourceFiles | ForEach-Object {
    "{$($_.Replace('\', '/'))}"
}) -join " "

if (-not $SkipImplementation) {
    foreach ($Profile in $Profiles) {
        foreach ($Mode in $Modes) {
            $Name = "top_k04_$($Mode.name)_$($Profile.name)"
            if ($OnlyImplementation -ne "" -and
                $Name -ne $OnlyImplementation) {
                continue
            }
        $Tcl = Join-Path $Work "implement_${Name}.tcl"
        $Timing = Join-Path $Work "timing_${Name}.rpt"
        $Util = Join-Path $Work "utilization_${Name}.rpt"
        $Cdc = Join-Path $Work "cdc_${Name}.rpt"
        $Drc = Join-Path $Work "drc_${Name}.rpt"
        $Metrics = Join-Path $Work "metrics_${Name}.txt"
        $VivadoLog = Join-Path $Work "vivado_${Name}.log"
        $VivadoJournal = Join-Path $Work "vivado_${Name}.jou"

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
            "synth_design -top tdc_gpx_lidar_ctrl_v2_top -part xc7z020clg484-2 -mode out_of_context -flatten_hierarchy rebuilt -generic G_PROC_CLK_MHZ=$($Profile.proc) -generic G_TDC_CLK_MHZ=$($Profile.tdc) -generic G_ENABLE_ECHO_SIMULATION=$($Mode.simulation)",
            "create_clock -name csr_clk -period 10.000 [get_ports s_axi_csr_aclk]",
            "create_clock -name proc_clk -period $($Profile.proc_period) [get_ports proc_aclk]",
            "create_clock -name tdc_clk -period $($Profile.tdc_period) [get_ports i_tdc_clk]",
            "set_clock_groups -asynchronous -group [get_clocks csr_clk] -group [get_clocks proc_clk] -group [get_clocks tdc_clk]",
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
            "set ibufds [get_cells -quiet -hier -filter {REF_NAME == IBUFDS}]",
            "set fp [open {$($Metrics.Replace('\', '/'))} w]",
            "puts `$fp [format {PROFILE_MODE=%s} {$($Mode.name)}]",
            "puts `$fp [format {PROFILE_PROC_MHZ=%d} $($Profile.proc)]",
            "puts `$fp [format {PROFILE_TDC_MHZ=%d} $($Profile.tdc)]",
            "puts `$fp [format {WNS_NS=%.3f} `$wns]",
            "puts `$fp [format {LATCH_COUNT=%d} [llength `$latches]]",
            "puts `$fp [format {ASYNC_REG_COUNT=%d} [llength `$async_regs]]",
            "puts `$fp [format {CAPTURE_FDPE_COUNT=%d} [llength `$capture_regs]]",
            "puts `$fp [format {ECHO_IBUFDS_COUNT=%d} [llength `$ibufds]]",
            "close `$fp",
            "if {`$wns < 0.0} { error [format {Negative WNS: %.3f ns} `$wns] }",
            "if {[llength `$latches] != 0} { error {Inferred latch detected} }",
            "if {[llength `$capture_regs] != 1} { error {Physical START capture is not one FDPE} }",
            "if {[llength `$ibufds] != 32} { error {Expected 32 physical Echo IBUFDS cells} }",
            "exit"
        ) | Set-Content -Encoding ASCII -LiteralPath $Tcl

            Invoke-Checked "$Vivado/vivado.bat" @(
                "-mode", "batch", "-notrace",
                "-source", $Tcl,
                "-log", $VivadoLog,
                "-journal", $VivadoJournal)

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
    }
}

New-Item -ItemType Directory -Force -Path $Archive | Out-Null
$Scenario = [ordered]@{
    checkpoint = "K0-4"
    purpose = "Processing and Echo production-owner integration"
    profiles = @(
        "Processing 150 / TDC 200 MHz",
        "Processing 200 / TDC 150 MHz"
    )
    modes = @("physical", "simulation")
    checks = @(
        "VDMA and matching Echo-version activation barrier",
        "physical encoder to fire ordering and raw fire_done low-latency START",
        "physical LVDS channel-preserving direct STOP",
        "simulation START and synthetic Echo with physical fire suppressed",
        "SOFT_RESET clears RUN/ARM while preserving Active Config"
    )
}
$Scenario | ConvertTo-Json -Depth 4 | Set-Content -Encoding ASCII `
    -LiteralPath (Join-Path $Archive "scenario.json")

$Manifest = foreach ($File in $SourceFiles) {
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
Copy-Item -Force -LiteralPath $OrderFile -Destination $Archive

$Artifacts = Get-ChildItem -File -LiteralPath $Work | Where-Object {
    $_.Name -match '^(xvhdl|xelab|xsim).*\.log$|\.prj$|^run\.tcl$|^implement.*\.tcl$|^(timing|utilization|cdc|drc).*\.rpt$|^metrics.*\.txt$|^vivado_.*\.log$'
} | ForEach-Object { $_.FullName }
Copy-Item -Force -LiteralPath $Artifacts -Destination $Archive

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith($ResolvedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "LIDAR_V2_K04_INTEGRATION_REGRESSION_PASS"
Write-Output "Result: $Archive"
