param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [int]$StallClks = 13,
    [int]$WidthOnly = 0
)

$ErrorActionPreference = "Stop"

if ($StallClks -lt 1) {
    throw "StallClks must be at least one"
}
if ($WidthOnly -ne 0 -and $WidthOnly -notin @(32, 64, 128)) {
    throw "WidthOnly must be zero, 32, 64, or 128"
}

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Glbl = (Resolve-Path -LiteralPath (
    "C:/AMDDesignTools/2025.2.1/Vivado/data/verilog/src/glbl.v")).Path
$OrderFile = Join-Path $PSScriptRoot "v2_rtl_compile_order.txt"
$WorkRoot = Join-Path $Hdl "tmp/v2_k06_axis_integration"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl (
    "signoff_results/sessions/${Stamp}_v2_k06_axis_integration")

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
        "system_integration/v2/tb/tb_tdc_gpx_lidar_ctrl_v2_k05.vhd")).Path
)

$VhdlProject = Join-Path $Work "v2_k06_axis_integration_vhdl.prj"
$VerilogProject = Join-Path $Work "v2_k06_axis_integration_verilog.prj"
$ProjectLines = foreach ($File in $SourceFiles) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $VhdlProject
@(
    "verilog xil_defaultlib `"$($Glbl.Replace('\', '/'))`"",
    "nosort"
) | Set-Content -Encoding ASCII -LiteralPath $VerilogProject

$RunTcl = Join-Path $Work "run.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

$ClockProfiles = @(
    [ordered]@{ name = "proc150_tdc200"; proc = 150; tdc = 200 },
    [ordered]@{ name = "proc200_tdc150"; proc = 200; tdc = 150 }
)
$Widths = if ($WidthOnly -eq 0) { @(32, 64, 128) } else { @($WidthOnly) }

Push-Location $Work
try {
    Invoke-Checked "$Vivado/xvlog.bat" @(
        "--relax", "-prj", $VerilogProject,
        "-log", (Join-Path $Work "xvlog.log"))
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--2008", "--relax", "-prj", $VhdlProject,
        "-log", (Join-Path $Work "xvhdl.log"))

    foreach ($Profile in $ClockProfiles) {
        foreach ($Width in $Widths) {
            $Stem = "top_k06_$($Profile.name)_w${Width}"
            $Snapshot = "${Stem}_${Stamp}_snap"
            $ElabLog = Join-Path $Work "xelab_${Stem}.log"
            $SimLog = Join-Path $Work "xsim_${Stem}.log"
            Invoke-BatchChecked "$Vivado/xelab.bat" @(
                "--debug", "off", "--relax", "--mt", "2",
                "--snapshot", $Snapshot,
                "--generic_top", "G_PROC_CLK_MHZ=$($Profile.proc)",
                "--generic_top", "G_TDC_CLK_MHZ=$($Profile.tdc)",
                "--generic_top", "G_OUTPUT_WIDTH=$Width",
                "--generic_top", "G_AXIS_STALL_CLKS=$StallClks",
                "xil_defaultlib.tb_tdc_gpx_lidar_ctrl_v2_k06",
                "xil_defaultlib.glbl",
                "-log", $ElabLog)
            Invoke-Checked "$Vivado/xsim.bat" @(
                $Snapshot, "-tclbatch", $RunTcl.Replace('\', '/'),
                "-log", $SimLog.Replace('\', '/'))

            $Text = Get-Content -Raw -LiteralPath $SimLog
            $Marker = "LIDAR_V2_TOP_K06_AXIS_PASS " +
                "proc_mhz=$($Profile.proc) tdc_mhz=$($Profile.tdc) " +
                "output_width=$Width stall_clks=$StallClks"
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

New-Item -ItemType Directory -Force -Path $Archive | Out-Null
$Scenario = [ordered]@{
    checkpoint = "K0-6"
    purpose = "Top-level B8-to-AXIS width and Footer-backpressure closure"
    clock_profiles = @(
        "Processing 150 / TDC 200 MHz",
        "Processing 200 / TDC 150 MHz"
    )
    output_widths = $Widths
    footer_stall_clks = $StallClks
    checks = @(
        "exact accepted Beat, TLAST Line, SOF and Footer counts",
        "full TKEEP/TSTRB for aligned 32/64/128-bit Lines",
        "AXIS payload stability while TREADY is low",
        "Face-close acknowledgement waits for final Footer Beat",
        "overload Face skip and next clean Face recovery",
        "disabled Fall lane remains quiescent"
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
    $_.Name -match '^(xvlog|xvhdl|xelab|xsim).*\.log$|\.prj$|^run\.tcl$'
} | ForEach-Object { $_.FullName }
Copy-Item -Force -LiteralPath $Artifacts -Destination $Archive

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith($ResolvedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "LIDAR_V2_K06_AXIS_INTEGRATION_REGRESSION_PASS"
Write-Output "Result: $Archive"
