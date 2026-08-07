param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [int]$StallClks = 11,
    [int]$OnlyMhz = 0,
    [int]$OnlyWidth = 0,
    [int]$OnlyTopology = -1
)

$ErrorActionPreference = "Stop"

if ($StallClks -lt 2) {
    throw "StallClks must be at least two"
}
if ($OnlyMhz -ne 0 -and $OnlyMhz -notin @(150, 200)) {
    throw "OnlyMhz must be zero, 150, or 200"
}
if ($OnlyWidth -ne 0 -and $OnlyWidth -notin @(32, 64, 128)) {
    throw "OnlyWidth must be zero, 32, 64, or 128"
}
if ($OnlyTopology -lt -1 -or $OnlyTopology -gt 2) {
    throw "OnlyTopology must be -1, 0, 1, or 2"
}

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$WorkRoot = Join-Path $Hdl "tmp/v2_k06_axis_dual_lane"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl (
    "signoff_results/sessions/${Stamp}_v2_k06_axis_dual_lane")

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

$RelativeFiles = @(
    "system_integration/v2/pkg/lidar_build_pkg.vhd",
    "system_integration/v2/pkg/lidar_event_types_pkg.vhd",
    "system_integration/v2/pkg/lidar_gpx_pkg.vhd",
    "system_integration/v2/pkg/lidar_gpx_event_pkg.vhd",
    "system_integration/v2/pkg/lidar_gpx_data_pkg.vhd",
    "system_integration/v2/pkg/lidar_gpx_vdma_pkg.vhd",
    "system_integration/v2/rtl/proc/lidar_gpx_cell_word_serializer.vhd",
    "system_integration/v2/rtl/proc/lidar_gpx_shot_line_builder.vhd",
    "system_integration/v2/rtl/proc/lidar_gpx_hole_line_expander.vhd",
    "system_integration/v2/rtl/proc/lidar_gpx_frame_close_fork.vhd",
    "system_integration/v2/rtl/proc/lidar_gpx_face_footer_builder.vhd",
    "system_integration/v2/rtl/proc/lidar_gpx_axis_word_packer.vhd",
    "system_integration/v2/rtl/proc/lidar_gpx_axis_lane_pipeline.vhd",
    "system_integration/v2/rtl/proc/lidar_gpx_axis_output_subsystem.vhd",
    "system_integration/v2/tb/tb_lidar_gpx_axis_output_subsystem.vhd"
)
$SourceFiles = foreach ($RelativeFile in $RelativeFiles) {
    (Resolve-Path -LiteralPath (Join-Path $Hdl $RelativeFile)).Path
}

$Project = Join-Path $Work "v2_k06_axis_dual_lane_vhdl.prj"
$ProjectLines = foreach ($File in $SourceFiles) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $Project
$RunTcl = Join-Path $Work "run.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

$ClockValues = if ($OnlyMhz -eq 0) { @(150, 200) } else { @($OnlyMhz) }
$WidthValues = if ($OnlyWidth -eq 0) { @(32, 64, 128) } else { @($OnlyWidth) }
$TopologyNames = @("dedicated_2r2f", "dual_1chip", "dual_4chip")
$TopologyValues = if ($OnlyTopology -lt 0) {
    @(0, 1, 2)
}
else {
    @($OnlyTopology)
}

Push-Location $Work
try {
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--2008", "--relax", "-prj", $Project,
        "-log", (Join-Path $Work "xvhdl.log"))

    foreach ($Mhz in $ClockValues) {
        foreach ($Width in $WidthValues) {
            foreach ($Topology in $TopologyValues) {
                $Stem = "axis_dual_${Mhz}_${Width}_t${Topology}"
                $Snapshot = "${Stem}_${Stamp}_snap"
                $ElabLog = Join-Path $Work "xelab_${Stem}.log"
                $SimLog = Join-Path $Work "xsim_${Stem}.log"
                Invoke-BatchChecked "$Vivado/xelab.bat" @(
                    "--debug", "off", "--relax", "--mt", "2",
                    "--snapshot", $Snapshot,
                    "--generic_top", "G_PROC_MHZ=$Mhz",
                    "--generic_top", "G_OUTPUT_WIDTH=$Width",
                    "--generic_top", "G_TOPOLOGY=$Topology",
                    "--generic_top", "G_FOOTER_STALL_CLKS=$StallClks",
                    "xil_defaultlib.tb_lidar_gpx_axis_output_subsystem",
                    "-log", $ElabLog)
                Invoke-Checked "$Vivado/xsim.bat" @(
                    $Snapshot, "-tclbatch", $RunTcl.Replace('\', '/'),
                    "-log", $SimLog.Replace('\', '/'))

                $Text = Get-Content -Raw -LiteralPath $SimLog
                $Marker = "LIDAR_V2_GPX_AXIS_OUTPUT_DUAL_PASS " +
                    "proc_mhz=$Mhz width=$Width topology=$Topology"
                if ($Text -notmatch [regex]::Escape($Marker)) {
                    throw "$Stem did not report its PASS marker"
                }
                if ($Text -match "Failure:|Fatal:") {
                    throw "$Stem reported a failure"
                }
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
    purpose = "Rise/Fall AXIS lane ownership and final Footer completion"
    processing_clocks_mhz = $ClockValues
    output_widths_bits = $WidthValues
    topologies = @($TopologyValues | ForEach-Object {
        $TopologyNames[$_]
    })
    fall_footer_stall_clks = $StallClks
    checks = @(
        "dedicated two-Rise/two-Fall topology",
        "one-chip dual-slope topology",
        "four-chip dual-slope maximum topology",
        "independent Fall Footer backpressure",
        "global completion after both lane Footers",
        "exact Beat, TLAST, SOF, Footer Magic and Commit counts",
        "full TKEEP/TSTRB and AXIS stability under stall"
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

$Artifacts = Get-ChildItem -File -LiteralPath $Work | Where-Object {
    $_.Name -match '^(xvhdl|xelab|xsim).*\.log$|\.prj$|^run\.tcl$'
} | ForEach-Object { $_.FullName }
Copy-Item -Force -LiteralPath $Artifacts -Destination $Archive

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith($ResolvedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "LIDAR_V2_K06_AXIS_DUAL_LANE_REGRESSION_PASS"
Write-Output "Result: $Archive"
