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
$Work = Join-Path $RepoRoot ".work\v3_h6_parent_data_diff\$Stamp"
$ToolHome = Join-Path $RepoRoot ".work\v3_h6_parent_diff_tool_home"
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
    (Join-Path $RepoRoot "px_utility_pkg.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_pkg.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_cfg_pkg.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_bus_phy.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_skid_buffer.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_sync_fifo.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_chip_init.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_chip_run.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_chip_reg.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_chip_ctrl.vhd"),
    (Join-Path $V2Root "pkg\lidar_build_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_config_types_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_config_reference_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_event_types_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_image_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_event_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_data_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_vdma_pkg.vhd"),
    (Join-Path $V3Root "pkg\lidar_v3_hls_contract_pkg.vhd"),
    (Join-Path $V2Root "rtl\common\lidar_stream_gateway.vhd"),
    (Join-Path $V2Root "rtl\tdc\lidar_gpx_bus_engine.vhd"),
    (Join-Path $V2Root "rtl\tdc\lidar_gpx_acquisition_lane.vhd"),
    (Join-Path $V2Root "rtl\tdc\lidar_gpx_event_merge.vhd"),
    (Join-Path $V2Root "rtl\tdc\lidar_gpx_acquisition_coordinator.vhd"),
    (Join-Path $V2Root "rtl\tdc\lidar_gpx_shot_gateway.vhd"),
    (Join-Path $V2Root "rtl\tdc\lidar_gpx_result_gateway.vhd"),
    (Join-Path $V2Root "rtl\tdc\lidar_gpx_stop_gateway.vhd"),
    (Join-Path $V2Root "rtl\tdc\lidar_gpx_acquisition_subsystem.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_hit_decoder.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_cell_collector.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_frame_lane_assembler.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_hit_cell_frame_pipeline.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_b5_b8_subsystem.vhd"),
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
    (Join-Path $V3Root "rtl\top\lidar_gpx_hls_parent_data_subsystem.vhd"),
    (Join-Path $V3Root "tb\tb_lidar_gpx_hls_parent_data_subsystem_diff.vhd")
)

foreach ($File in $VhdlFiles + $HlsFiles + @($Glbl)) {
    if (-not (Test-Path -LiteralPath $File)) {
        throw "Required source is missing: $File"
    }
}

$Pairs = @(
    [ordered]@{
        Name = "p50_t200_w32_dedicated_r1"
        V2 = "tb_lidar_gpx_h6_parent_v2_p50_t200_w32"
        V3 = "tb_lidar_gpx_h6_parent_v3_p50_t200_w32"
    },
    [ordered]@{
        Name = "p200_t50_w64_all_dual_r7"
        V2 = "tb_lidar_gpx_h6_parent_v2_p200_t50_w64"
        V3 = "tb_lidar_gpx_h6_parent_v3_p200_t50_w64"
    },
    [ordered]@{
        Name = "p150_t150_w32_all_dual_r7"
        V2 = "tb_lidar_gpx_h6_parent_v2_p150_t150_w32"
        V3 = "tb_lidar_gpx_h6_parent_v3_p150_t150_w32"
    },
    [ordered]@{
        Name = "p150_t200_w32_dedicated_r1"
        V2 = "tb_lidar_gpx_h6_parent_v2_p150_t200_w32"
        V3 = "tb_lidar_gpx_h6_parent_v3_p150_t200_w32"
    },
    [ordered]@{
        Name = "p200_t150_w64_all_dual_r7"
        V2 = "tb_lidar_gpx_h6_parent_v2_p200_t150_w64"
        V3 = "tb_lidar_gpx_h6_parent_v3_p200_t150_w64"
    }
)

New-Item -ItemType Directory -Force -Path $Work | Out-Null
New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null
foreach ($DataFile in $HlsDataFiles) {
    Copy-Item -LiteralPath $DataFile -Destination $Work -Force
}

$VerilogProject = Join-Path $Work "hls_verilog.prj"
$VhdlProject = Join-Path $Work "h6_vhdl.prj"
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

    foreach ($Pair in $Pairs) {
        foreach ($Capture in @(
                "v2_rise_capture.txt", "v2_fall_capture.txt",
                "v3_rise_capture.txt", "v3_fall_capture.txt")) {
            Remove-Item -LiteralPath (Join-Path $Work $Capture) `
                -Force -ErrorAction SilentlyContinue
        }

        foreach ($Top in @($Pair.V2, $Pair.V3)) {
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
            if ($Text -notmatch
                    "LIDAR_V3_H6_PARENT_DATA_DIFF_CAPTURE_PASS") {
                throw "$Top did not report the H6 capture PASS marker"
            }
            if ($Text -match "Failure:|Fatal:") {
                throw "$Top reported a simulation failure"
            }
        }

        $V2Rise = Get-Content -Raw -LiteralPath (
            Join-Path $Work "v2_rise_capture.txt")
        $V3Rise = Get-Content -Raw -LiteralPath (
            Join-Path $Work "v3_rise_capture.txt")
        $V2Fall = Get-Content -Raw -LiteralPath (
            Join-Path $Work "v2_fall_capture.txt")
        $V3Fall = Get-Content -Raw -LiteralPath (
            Join-Path $Work "v3_fall_capture.txt")
        if ($V2Rise -cne $V3Rise) {
            throw "$($Pair.Name) Rise AXI capture differs from V2 Golden"
        }
        if ($V2Fall -cne $V3Fall) {
            throw "$($Pair.Name) Fall AXI capture differs from V2 Golden"
        }

        $PairDirectory = Join-Path $Work $Pair.Name
        New-Item -ItemType Directory -Force -Path $PairDirectory | Out-Null
        foreach ($Capture in @(
                "v2_rise_capture.txt", "v2_fall_capture.txt",
                "v3_rise_capture.txt", "v3_fall_capture.txt")) {
            Copy-Item -LiteralPath (Join-Path $Work $Capture) `
                -Destination $PairDirectory -Force
        }
        Write-Host "LIDAR_V3_H6_PARENT_DATA_PROFILE_PASS $($Pair.Name)"
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

Write-Host "LIDAR_V3_H6_PARENT_DATA_DIFF_PASS"
Write-Host "Result: $Work"
