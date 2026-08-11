param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [string]$VivadoRoot = "C:\AMDDesignTools\2025.2.1\Vivado"
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$RepoRoot = (Resolve-Path (Join-Path $V3Root "..\..")).Path
$V2Root = Join-Path $RepoRoot "system_integration\v2"
$Work = Join-Path $RepoRoot ".work\v3_h6b_formatter_status\$Stamp"
$ToolHome = Join-Path $RepoRoot ".work\v3_h6b_formatter_status_tool_home"
$VivadoBin = Join-Path $VivadoRoot "bin"

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

$Files = @(
    (Join-Path $RepoRoot "tdc_gpx_pkg.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_cfg_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_build_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_config_types_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_event_types_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_processing_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_image_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_event_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_data_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_vdma_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_echo_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_status_pkg.vhd"),
    (Join-Path $V3Root "pkg\lidar_v3_status_pkg.vhd"),
    (Join-Path $V3Root "rtl\status\lidar_v3_processing_status_source.vhd"),
    (Join-Path $V3Root "tb\tb_lidar_v3_processing_status_formatter_fault.vhd")
)
foreach ($File in $Files) {
    if (-not (Test-Path -LiteralPath $File)) {
        throw "Required source is missing: $File"
    }
}

New-Item -ItemType Directory -Force -Path $Work | Out-Null
New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null
$Project = Join-Path $Work "formatter_status_vhdl.prj"
$RunTcl = Join-Path $Work "run.tcl"
$Log = Join-Path $Work "xsim.log"
$Lines = foreach ($File in $Files) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$Lines += "nosort"
$Lines | Set-Content -Encoding ASCII -LiteralPath $Project
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

$SavedEnvironment = @{
    HOME = $env:HOME
    USERPROFILE = $env:USERPROFILE
    APPDATA = $env:APPDATA
    LOCALAPPDATA = $env:LOCALAPPDATA
}

try {
    $env:HOME = $ToolHome
    $env:USERPROFILE = $ToolHome
    $env:APPDATA = $ToolHome
    $env:LOCALAPPDATA = $ToolHome
    Push-Location $Work
    try {
        Invoke-Checked -Executable (Join-Path $VivadoBin "xvhdl.bat") `
            -ToolArguments @(
                "--2008", "--relax", "-prj", $Project,
                "-log", (Join-Path $Work "xvhdl.log"))
        $Snapshot = "v3_h6b_formatter_status_${Stamp}_snap"
        Invoke-Checked -Executable (Join-Path $VivadoBin "xelab.bat") `
            -ToolArguments @(
                "--debug", "off", "--relax", "--mt", "2",
                "--snapshot", $Snapshot,
                "xil_defaultlib.tb_lidar_v3_processing_status_formatter_fault",
                "-log", (Join-Path $Work "xelab.log"))
        Invoke-Checked -Executable (Join-Path $VivadoBin "xsim.bat") `
            -ToolArguments @(
                $Snapshot, "-tclbatch", $RunTcl.Replace('\', '/'),
                "-log", $Log.Replace('\', '/'))
    }
    finally {
        Pop-Location
    }
}
finally {
    foreach ($Key in $SavedEnvironment.Keys) {
        Set-Item -Path "Env:$Key" -Value $SavedEnvironment[$Key]
    }
}

$Text = Get-Content -Raw -LiteralPath $Log
if (-not $Text.Contains("LIDAR_V3_H6B_FORMATTER_STATUS_IRQ_PASS")) {
    throw "Formatter status/IRQ simulation did not emit the PASS marker"
}
if ($Text -match "Failure:|Fatal:|ERROR:") {
    throw "Formatter status/IRQ simulation reported a failure"
}

Write-Host "LIDAR_V3_H6B_FORMATTER_STATUS_IRQ_PASS"
Write-Host "Result: $Work"
