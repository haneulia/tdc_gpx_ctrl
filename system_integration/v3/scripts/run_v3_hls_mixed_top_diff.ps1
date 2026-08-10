param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$SkipHlsSynthesis,
    [string]$VivadoRoot = "C:\AMDDesignTools\2025.2.1\Vivado"
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$RepoRoot = (Resolve-Path (Join-Path $V3Root "..\..")).Path
$V2Root = Join-Path $RepoRoot "system_integration\v2"
$Work = Join-Path $RepoRoot ".work\v3_hls_mixed_top_diff\$Stamp"
$ToolHome = Join-Path $RepoRoot ".work\v3_h5_diff_tool_home"
$VivadoBin = Join-Path $VivadoRoot "bin"
$Glbl = Join-Path $VivadoRoot "data\verilog\src\glbl.v"

function Invoke-Checked {
    param(
        [Parameter(Mandatory = $true)][string]$Executable,
        [Parameter(Mandatory = $true)][string[]]$ToolArguments
    )
    & $Executable @ToolArguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Executable"
    }
}

$HlsComponents = @(
    [ordered]@{
        Runner = "run_v3_hls_hit_decoder.ps1"
        Rtl = ".work\v3_hls_hit_decoder_component\hls\syn\verilog"
    },
    [ordered]@{
        Runner = "run_v3_hls_cell_collector.ps1"
        Rtl = ".work\v3_hls_cell_collector_component\hls\syn\verilog"
    },
    [ordered]@{
        Runner = "run_v3_hls_frame_assembler.ps1"
        Rtl = ".work\v3_hls_frame_assembler_component\hls\syn\verilog"
    },
    [ordered]@{
        Runner = "run_v3_hls_lane_word_formatter.ps1"
        Rtl = ".work\v3_hls_lane_word_formatter_component\hls\syn\verilog"
    }
)

if (-not $SkipHlsSynthesis) {
    foreach ($Component in $HlsComponents) {
        Invoke-Checked -Executable "powershell.exe" -ToolArguments @(
            "-NoProfile", "-ExecutionPolicy", "Bypass",
            "-File", (Join-Path $ScriptDir $Component.Runner),
            "-Step", "csynth"
        )
    }
}

$HlsFiles = @()
$HlsDataFiles = @()
foreach ($Component in $HlsComponents) {
    $Directory = Join-Path $RepoRoot $Component.Rtl
    if (-not (Test-Path -LiteralPath $Directory)) {
        throw "Generated HLS RTL directory is missing: $Directory"
    }
    $Files = @(Get-ChildItem -File -LiteralPath $Directory -Filter "*.v" |
        Sort-Object -Property Name | ForEach-Object { $_.FullName })
    if ($Files.Count -eq 0) {
        throw "Generated HLS RTL files are missing: $Directory"
    }
    $HlsFiles += $Files
    $HlsDataFiles += @(Get-ChildItem -File -LiteralPath $Directory `
        -Filter "*.dat" | ForEach-Object { $_.FullName })
}
$HlsFiles = @($HlsFiles | Sort-Object -Unique)

$VhdlFiles = @(
    (Join-Path $V2Root "pkg\lidar_build_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_config_types_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_config_reference_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_event_types_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_event_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_data_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_vdma_pkg.vhd"),
    (Join-Path $V3Root "pkg\lidar_v3_hls_contract_pkg.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_skid_buffer.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_sync_fifo.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_hit_decoder.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_cell_collector.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_frame_lane_assembler.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_hit_cell_frame_pipeline.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_cell_word_serializer.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_shot_line_builder.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_hole_line_expander.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_face_footer_builder.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_axis_word_packer.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_axis_lane_pipeline.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_frame_close_fork.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_axis_output_subsystem.vhd"),
    (Join-Path $V3Root "rtl\bridges\lidar_gpx_hit_decoder_hls_adapter.vhd"),
    (Join-Path $V3Root "rtl\bridges\lidar_gpx_cell_collector_hls_adapter.vhd"),
    (Join-Path $V3Root "rtl\bridges\lidar_gpx_frame_lane_assembler_hls_adapter.vhd"),
    (Join-Path $V3Root "rtl\bridges\lidar_gpx_lane_word_formatter_hls_adapter.vhd"),
    (Join-Path $V3Root "rtl\top\lidar_gpx_hls_hit_cell_frame_pipeline.vhd"),
    (Join-Path $V3Root "rtl\top\lidar_gpx_hls_axis_output_subsystem.vhd"),
    (Join-Path $V3Root "rtl\top\lidar_gpx_hls_mixed_data_top.vhd"),
    (Join-Path $V3Root "tb\tb_lidar_gpx_hls_mixed_data_top_diff.vhd")
)
foreach ($File in $VhdlFiles + $HlsFiles + @($Glbl)) {
    if (-not (Test-Path -LiteralPath $File)) {
        throw "Required source is missing: $File"
    }
}

$Profiles = @(
    "tb_lidar_gpx_hls_mixed_data_top_diff_dedicated_32_150",
    "tb_lidar_gpx_hls_mixed_data_top_diff_dedicated_64_200",
    "tb_lidar_gpx_hls_mixed_data_top_diff_all_dual_32_150",
    "tb_lidar_gpx_hls_mixed_data_top_diff_all_dual_64_200",
    "tb_lidar_gpx_hls_mixed_data_top_diff_rise_only_32_200"
)

New-Item -ItemType Directory -Force -Path $Work | Out-Null
New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null
foreach ($DataFile in $HlsDataFiles) {
    Copy-Item -LiteralPath $DataFile -Destination $Work -Force
}

$VerilogProject = Join-Path $Work "hls_verilog.prj"
$VhdlProject = Join-Path $Work "diff_vhdl.prj"
$RunTcl = Join-Path $Work "run.tcl"
$VerilogLines = foreach ($File in $HlsFiles + @($Glbl)) {
    "verilog xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$VerilogLines += "nosort"
$VerilogLines | Set-Content -Encoding ASCII -LiteralPath $VerilogProject
$VhdlLines = foreach ($File in $VhdlFiles) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$VhdlLines += "nosort"
$VhdlLines | Set-Content -Encoding ASCII -LiteralPath $VhdlProject
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

$SavedEnvironment = @{
    HOME = $env:HOME
    USERPROFILE = $env:USERPROFILE
    APPDATA = $env:APPDATA
    LOCALAPPDATA = $env:LOCALAPPDATA
}
$env:HOME = $ToolHome
$env:USERPROFILE = $ToolHome
$env:APPDATA = Join-Path $ToolHome "AppData\Roaming"
$env:LOCALAPPDATA = Join-Path $ToolHome "AppData\Local"
New-Item -ItemType Directory -Force -Path $env:APPDATA | Out-Null
New-Item -ItemType Directory -Force -Path $env:LOCALAPPDATA | Out-Null

Push-Location $Work
try {
    Invoke-Checked -Executable (Join-Path $VivadoBin "xvlog.bat") `
        -ToolArguments @(
            "--relax", "-prj", $VerilogProject,
            "-log", (Join-Path $Work "xvlog.log")
        )
    Invoke-Checked -Executable (Join-Path $VivadoBin "xvhdl.bat") `
        -ToolArguments @(
            "--relax", "-prj", $VhdlProject,
            "-log", (Join-Path $Work "xvhdl.log")
        )

    foreach ($Top in $Profiles) {
        $Snapshot = "${Top}_${Stamp}_snap"
        $ElabLog = Join-Path $Work "xelab_${Top}.log"
        $SimLog = Join-Path $Work "xsim_${Top}.log"
        Invoke-Checked -Executable (Join-Path $VivadoBin "xelab.bat") `
            -ToolArguments @(
                "--debug", "off", "--relax", "--mt", "2",
                "--snapshot", $Snapshot,
                "xil_defaultlib.$Top", "xil_defaultlib.glbl",
                "-log", $ElabLog
            )
        Invoke-Checked -Executable (Join-Path $VivadoBin "xsim.bat") `
            -ToolArguments @(
                $Snapshot,
                "-tclbatch", $RunTcl.Replace('\', '/'),
                "-log", $SimLog.Replace('\', '/')
            )
        $Text = Get-Content -Raw -LiteralPath $SimLog
        if ($Text -notmatch "LIDAR_V3_H5_MIXED_DATA_TOP_DIFF_PASS") {
            throw "$Top did not report the H5 PASS marker"
        }
        if ($Text -match "Failure:|Fatal:") {
            throw "$Top reported a simulation failure"
        }
        Write-Host "LIDAR_V3_H5_MIXED_DATA_TOP_DIFF_PROFILE_PASS top=$Top"
    }
}
finally {
    Pop-Location
    foreach ($Name in $SavedEnvironment.Keys) {
        $Value = $SavedEnvironment[$Name]
        if ($null -eq $Value) {
            Remove-Item "Env:$Name" -ErrorAction SilentlyContinue
        }
        else {
            Set-Item "Env:$Name" $Value
        }
    }
}

Write-Host "LIDAR_V3_H5_MIXED_DATA_TOP_DIFF_PASS"
Write-Host "Result: $Work"
