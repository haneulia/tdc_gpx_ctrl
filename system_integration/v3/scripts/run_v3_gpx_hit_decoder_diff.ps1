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
$Work = Join-Path $RepoRoot ".work\v3_gpx_hit_decoder_diff\$Stamp"
$ToolHome = Join-Path $RepoRoot ".work\v3_diff_tool_home"
$HlsRunner = Join-Path $ScriptDir "run_v3_hls_hit_decoder.ps1"
$HlsRtl = Join-Path $RepoRoot ".work\v3_hls_hit_decoder_component\hls\syn\verilog"
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

$HlsFiles = @(
    (Join-Path $HlsRtl "gpx_hit_decoder_hls_regslice_both.v"),
    (Join-Path $HlsRtl "gpx_hit_decoder_hls.v")
)
foreach ($File in $HlsFiles + @($Glbl)) {
    if (-not (Test-Path -LiteralPath $File)) {
        throw "Required generated RTL is missing: $File"
    }
}

$VhdlFiles = @(
    (Join-Path $V2Root "pkg\lidar_build_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_config_types_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_event_types_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_event_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_data_pkg.vhd"),
    (Join-Path $V3Root "pkg\lidar_v3_hls_contract_pkg.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_hit_decoder.vhd"),
    (Join-Path $V3Root "rtl\bridges\lidar_gpx_hit_decoder_hls_adapter.vhd"),
    (Join-Path $V3Root "tb\tb_lidar_gpx_hit_decoder_hls_diff.vhd")
)

$SimulationProfiles = @(
    "tb_lidar_gpx_hit_decoder_hls_diff_dedicated_150",
    "tb_lidar_gpx_hit_decoder_hls_diff_dedicated_200",
    "tb_lidar_gpx_hit_decoder_hls_diff_one_dual_150",
    "tb_lidar_gpx_hit_decoder_hls_diff_one_dual_200",
    "tb_lidar_gpx_hit_decoder_hls_diff_reduced_150",
    "tb_lidar_gpx_hit_decoder_hls_diff_reduced_200",
    "tb_lidar_gpx_hit_decoder_hls_diff_all_dual_150",
    "tb_lidar_gpx_hit_decoder_hls_diff_all_dual_200"
)

New-Item -ItemType Directory -Force -Path $Work | Out-Null
New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null

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
    Invoke-Checked -Executable (Join-Path $VivadoBin "xvlog.bat") -ToolArguments @(
        "--relax", "-prj", $VerilogProject,
        "-log", (Join-Path $Work "xvlog.log")
    )
    Invoke-Checked -Executable (Join-Path $VivadoBin "xvhdl.bat") -ToolArguments @(
        "--relax", "-prj", $VhdlProject,
        "-log", (Join-Path $Work "xvhdl.log")
    )

    foreach ($Top in $SimulationProfiles) {
        $Snapshot = "${Top}_${Stamp}_snap"
        $ElabLog = Join-Path $Work "xelab_${Top}.log"
        $SimLog = Join-Path $Work "xsim_${Top}.log"
        Invoke-Checked -Executable (Join-Path $VivadoBin "xelab.bat") -ToolArguments @(
            "--debug", "off", "--relax", "--mt", "2",
            "--snapshot", $Snapshot,
            "xil_defaultlib.$Top", "xil_defaultlib.glbl",
            "-log", $ElabLog
        )
        Invoke-Checked -Executable (Join-Path $VivadoBin "xsim.bat") -ToolArguments @(
            $Snapshot,
            "-tclbatch", $RunTcl.Replace('\', '/'),
            "-log", $SimLog.Replace('\', '/')
        )

        $SimulationText = Get-Content -Raw -LiteralPath $SimLog
        if ($SimulationText -notmatch "LIDAR_V3_GPX_HIT_DECODER_DIFF_PASS") {
            throw "$Top did not report the differential PASS marker"
        }
        Write-Host "LIDAR_V3_GPX_HIT_DECODER_DIFF_PROFILE_PASS top=$Top"
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

Write-Host "LIDAR_V3_GPX_HIT_DECODER_DIFF_PASS"
Write-Host "Result: $Work"
