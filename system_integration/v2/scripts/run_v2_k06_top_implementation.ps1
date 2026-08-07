param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [int]$OnlyProcMhz = 0,
    [int]$OnlyWidth = 0
)

$ErrorActionPreference = "Stop"

if ($OnlyProcMhz -ne 0 -and $OnlyProcMhz -notin @(150, 200)) {
    throw "OnlyProcMhz must be zero, 150, or 200"
}
if ($OnlyWidth -ne 0 -and $OnlyWidth -notin @(32, 64, 128)) {
    throw "OnlyWidth must be zero, 32, 64, or 128"
}

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$OrderFile = Join-Path $PSScriptRoot "v2_rtl_compile_order.txt"
$WorkRoot = Join-Path $Hdl "tmp/v2_k06_top_implementation"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl (
    "signoff_results/sessions/${Stamp}_v2_k06_top_implementation")

function Invoke-Checked {
    param([string]$Exe, [string[]]$ArgList)
    & $Exe @ArgList
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Exe $($ArgList -join ' ')"
    }
}

New-Item -ItemType Directory -Force -Path $Work | Out-Null

$SourceFiles = foreach ($Line in Get-Content -LiteralPath $OrderFile) {
    $Entry = $Line.Trim()
    if ($Entry.Length -gt 0 -and -not $Entry.StartsWith("#")) {
        (Resolve-Path -LiteralPath (Join-Path $Hdl $Entry)).Path
    }
}
$ReadFiles = ($SourceFiles | ForEach-Object {
    "{$($_.Replace('\', '/'))}"
}) -join " "

$AllProfiles = @(
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
$Profiles = if ($OnlyProcMhz -eq 0) {
    $AllProfiles
}
else {
    @($AllProfiles | Where-Object { $_.proc -eq $OnlyProcMhz })
}
$Widths = if ($OnlyWidth -eq 0) { @(32, 64, 128) } else { @($OnlyWidth) }

foreach ($Profile in $Profiles) {
    foreach ($Width in $Widths) {
        $Name = "top_k06_$($Profile.name)_w${Width}"
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
            "synth_design -top tdc_gpx_lidar_ctrl_v2_top -part xc7z020clg484-2 -mode out_of_context -flatten_hierarchy rebuilt -generic G_PROC_CLK_MHZ=$($Profile.proc) -generic G_TDC_CLK_MHZ=$($Profile.tdc) -generic G_OUTPUT_WIDTH=$Width -generic G_ENABLE_ECHO_SIMULATION=false",
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
            "set paths [get_timing_paths -delay_type max -max_paths 1]",
            "if {[llength `$paths] == 0} { error {No setup timing path found} }",
            "set wns [get_property SLACK [lindex `$paths 0]]",
            "set latches [get_cells -quiet -hier -filter {REF_NAME == LDCE || REF_NAME == LDPE}]",
            "set black_boxes [get_cells -quiet -hier -filter {IS_BLACKBOX == 1}]",
            "set async_regs [get_cells -quiet -hier -filter {ASYNC_REG == TRUE}]",
            "set fp [open {$($Metrics.Replace('\', '/'))} w]",
            "puts `$fp [format {PROFILE=%s} {$($Profile.name)}]",
            "puts `$fp [format {PROC_MHZ=%d} $($Profile.proc)]",
            "puts `$fp [format {TDC_MHZ=%d} $($Profile.tdc)]",
            "puts `$fp [format {OUTPUT_WIDTH=%d} $Width]",
            "puts `$fp [format {WNS_NS=%.3f} `$wns]",
            "puts `$fp [format {LATCH_COUNT=%d} [llength `$latches]]",
            "puts `$fp [format {BLACK_BOX_COUNT=%d} [llength `$black_boxes]]",
            "puts `$fp [format {ASYNC_REG_COUNT=%d} [llength `$async_regs]]",
            "close `$fp",
            "if {`$wns < 0.100} { error [format {WNS below +0.100 ns: %.3f ns} `$wns] }",
            "if {[llength `$latches] != 0} { error {Inferred latch detected} }",
            "if {[llength `$black_boxes] != 0} { error {Black box detected} }",
            "exit"
        ) | Set-Content -Encoding ASCII -LiteralPath $Tcl

        Invoke-Checked "$Vivado/vivado.bat" @(
            "-mode", "batch", "-notrace",
            "-source", $Tcl,
            "-log", $VivadoLog,
            "-journal", $VivadoJournal
        )

        $CdcText = Get-Content -Raw -LiteralPath $Cdc
        $CriticalCdcCount = 0
        foreach ($Match in [regex]::Matches($CdcText,
                '(?m)^CDC-\d+\s+Critical\s+(\d+)')) {
            $CriticalCdcCount += [int]$Match.Groups[1].Value
        }

        $DrcText = Get-Content -Raw -LiteralPath $Drc
        $BlockingDrcNames = @(
            [regex]::Matches($DrcText,
                '(?m)^\|\s*(\S+)\s*\|\s*(Critical Warning|Error)\s*\|') |
                ForEach-Object { $_.Groups[1].Value }
        )
        # Pin standards and package locations belong to the parent board XDC.
        # Keep them visible in the archive, but do not classify them as an RTL
        # OOC blocker. Every other Critical Warning/Error remains blocking.
        $ParentXdcExclusions = @("IOSTDTYPE-1", "NSTD-1", "UCIO-1")
        $UnexpectedDrcNames = @($BlockingDrcNames | Where-Object {
            $_ -notin $ParentXdcExclusions
        })

        Add-Content -Encoding ASCII -LiteralPath $Metrics -Value @(
            "CDC_CRITICAL_COUNT=$CriticalCdcCount",
            "DRC_CRITICAL_OR_ERROR_COUNT=$($BlockingDrcNames.Count)",
            "DRC_PARENT_XDC_EXCLUDED_COUNT=$(($BlockingDrcNames | Where-Object { $_ -in $ParentXdcExclusions }).Count)",
            "DRC_UNEXPECTED_BLOCKING_COUNT=$($UnexpectedDrcNames.Count)",
            "DRC_UNEXPECTED_NAMES=$($UnexpectedDrcNames -join ',')"
        )
        if ($CriticalCdcCount -ne 0) {
            throw "$Name has $CriticalCdcCount critical CDC paths"
        }
        if ($UnexpectedDrcNames.Count -ne 0) {
            throw "$Name has blocking DRC categories: $($UnexpectedDrcNames -join ', ')"
        }
    }
}

New-Item -ItemType Directory -Force -Path $Archive | Out-Null
$Scenario = [ordered]@{
    checkpoint = "K0-6"
    purpose = "Top-level AXIS width implementation, timing and CDC closure"
    part = "xc7z020clg484-2"
    clock_profiles = @($Profiles | ForEach-Object {
        "Processing $($_.proc) / TDC $($_.tdc) MHz"
    })
    output_widths_bits = $Widths
    gates = @(
        "WNS is at least +0.100 ns after route",
        "zero inferred latches",
        "zero black boxes",
        "zero critical CDC paths",
        "zero unexpected blocking DRC categories"
    )
    parent_xdc_drc_exclusions = @(
        "IOSTDTYPE-1 board-specific I/O standard type",
        "NSTD-1 unspecified I/O standard",
        "UCIO-1 unconstrained logical port"
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
    $_.Name -match '^implement.*\.tcl$|^(timing|utilization|cdc|drc).*\.rpt$|^metrics.*\.txt$|^vivado_.*\.log$'
} | ForEach-Object { $_.FullName }
Copy-Item -Force -LiteralPath $Artifacts -Destination $Archive

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith($ResolvedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "LIDAR_V2_K06_TOP_IMPLEMENTATION_PASS"
Write-Output "Result: $Archive"
