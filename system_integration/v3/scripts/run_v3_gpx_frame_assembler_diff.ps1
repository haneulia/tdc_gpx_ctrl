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
$Work = Join-Path $RepoRoot ".work\v3_gpx_frame_assembler_diff\$Stamp"
$ToolHome = Join-Path $RepoRoot ".work\v3_frame_diff_tool_home"
$HlsRunner = Join-Path $ScriptDir "run_v3_hls_frame_assembler.ps1"
$HlsRtl = Join-Path $RepoRoot (
    ".work\v3_hls_frame_assembler_component\hls\syn\verilog")
$VivadoBin = Join-Path $VivadoRoot "bin"
$Glbl = Join-Path $VivadoRoot "data\verilog\src\glbl.v"

function Invoke-Checked {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Executable,
        [Parameter(Mandatory = $true)]
        [string[]]$ToolArguments
    )
    & $Executable @ToolArguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Executable"
    }
}

if (-not $SkipHlsSynthesis) {
    Invoke-Checked -Executable "powershell.exe" -ToolArguments @(
        "-NoProfile", "-ExecutionPolicy", "Bypass",
        "-File", $HlsRunner, "-Step", "csynth"
    )
}

if (-not (Test-Path -LiteralPath $HlsRtl)) {
    throw "Generated HLS RTL directory is missing: $HlsRtl"
}
$HlsFiles = @(Get-ChildItem -File -LiteralPath $HlsRtl -Filter "*.v" |
    Sort-Object -Property Name | ForEach-Object { $_.FullName })
if ($HlsFiles.Count -eq 0) {
    throw "Generated HLS RTL files are missing: $HlsRtl"
}
if (-not (Test-Path -LiteralPath $Glbl)) {
    throw "Vivado glbl.v is missing: $Glbl"
}

$VhdlFiles = @(
    (Join-Path $V2Root "pkg\lidar_build_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_config_types_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_event_types_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_event_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_data_pkg.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_skid_buffer.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_sync_fifo.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_frame_lane_assembler.vhd"),
    (Join-Path $V3Root (
        "rtl\bridges\lidar_gpx_frame_lane_assembler_hls_adapter.vhd")),
    (Join-Path $V3Root (
        "tb\tb_lidar_gpx_frame_assembler_hls_diff.vhd"))
)
foreach ($File in $VhdlFiles) {
    if (-not (Test-Path -LiteralPath $File)) {
        throw "Required VHDL source is missing: $File"
    }
}

$SimulationProfiles = @(
    "tb_lidar_gpx_frame_assembler_hls_diff_dedicated_150",
    "tb_lidar_gpx_frame_assembler_hls_diff_dedicated_200",
    "tb_lidar_gpx_frame_assembler_hls_diff_one_dual_150",
    "tb_lidar_gpx_frame_assembler_hls_diff_one_dual_200",
    "tb_lidar_gpx_frame_assembler_hls_diff_fall_off_150",
    "tb_lidar_gpx_frame_assembler_hls_diff_fall_off_200",
    "tb_lidar_gpx_frame_assembler_hls_diff_reduced_150",
    "tb_lidar_gpx_frame_assembler_hls_diff_reduced_200",
    "tb_lidar_gpx_frame_assembler_hls_diff_all_dual_150",
    "tb_lidar_gpx_frame_assembler_hls_diff_all_dual_200"
)

New-Item -ItemType Directory -Force -Path $Work | Out-Null
New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null
Get-ChildItem -File -LiteralPath $HlsRtl -Filter "*.dat" |
    Copy-Item -Destination $Work -Force

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

    foreach ($Top in $SimulationProfiles) {
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
        $SimulationText = Get-Content -Raw -LiteralPath $SimLog
        if ($SimulationText -notmatch
                "LIDAR_V3_GPX_FRAME_ASSEMBLER_DIFF_PASS") {
            throw "$Top did not report the differential PASS marker"
        }
        Write-Host (
            "LIDAR_V3_GPX_FRAME_ASSEMBLER_DIFF_PROFILE_PASS top=$Top")
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

Write-Host "LIDAR_V3_GPX_FRAME_ASSEMBLER_DIFF_PASS"
Write-Host "Result: $Work"
