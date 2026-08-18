[CmdletBinding()]
param(
    [string]$VivadoRoot = ""
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V2Root = Split-Path -Parent $ScriptDir
$HdlRoot = (Resolve-Path (Join-Path $V2Root "..\..")).Path
$PackageTcl = Join-Path $ScriptDir "package_v2_ip.tcl"
$CheckTcl = Join-Path $ScriptDir "check_v2_ip_package.tcl"
$PackageDir = Join-Path $V2Root "ip_repo\tdc_gpx_lidar_ctrl_v2_2_0"
$ToolHome = Join-Path $HdlRoot ".work\v2_ip_package_tool_home"

function Resolve-VivadoRoot {
    $Candidates = @()
    if (-not [string]::IsNullOrWhiteSpace($VivadoRoot)) {
        $Candidates += $VivadoRoot
    }
    if (-not [string]::IsNullOrWhiteSpace($env:XILINX_VIVADO)) {
        $Candidates += $env:XILINX_VIVADO
    }
    $Candidates += @(
        "C:\AMDDesignTools\2025.2.1\Vivado",
        "C:\AMDDesignTools\2025.2\Vivado")
    foreach ($Candidate in $Candidates | Select-Object -Unique) {
        if (Test-Path -LiteralPath (Join-Path $Candidate "bin\vivado.bat")) {
            return (Resolve-Path -LiteralPath $Candidate).Path
        }
    }
    throw "Vivado installation was not found. Supply -VivadoRoot."
}

function Invoke-Checked {
    param(
        [string]$Vivado,
        [string[]]$Arguments,
        [string]$PassMarker
    )
    $Output = & $Vivado @Arguments 2>&1
    $Output | ForEach-Object { Write-Output $_ }
    if ($LASTEXITCODE -ne 0) {
        throw "Vivado failed with exit code $LASTEXITCODE."
    }
    if (-not ($Output -match [regex]::Escape($PassMarker))) {
        throw "Required PASS marker is missing: $PassMarker"
    }
    if ($Output -match "CRITICAL WARNING:") {
        throw "Critical Warning detected while expecting $PassMarker"
    }
}

$Vivado = Join-Path (Resolve-VivadoRoot) "bin\vivado.bat"
foreach ($Required in @($PackageTcl, $CheckTcl, $Vivado)) {
    if (-not (Test-Path -LiteralPath $Required -PathType Leaf)) {
        throw "Required V2 package input is missing: $Required"
    }
}

New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null
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

try {
    Invoke-Checked -Vivado $Vivado -Arguments @(
        "-mode", "batch", "-notrace", "-source", $PackageTcl,
        "-log", (Join-Path $ToolHome "package_v2_ip.log"),
        "-journal", (Join-Path $ToolHome "package_v2_ip.jou")) `
        -PassMarker "TDC_GPX_LIDAR_CTRL_V2_IP_PACKAGE_PASS"

    Invoke-Checked -Vivado $Vivado -Arguments @(
        "-mode", "batch", "-notrace", "-source", $CheckTcl,
        "-tclargs", $PackageDir,
        "-log", (Join-Path $ToolHome "check_v2_ip.log"),
        "-journal", (Join-Path $ToolHome "check_v2_ip.jou")) `
        -PassMarker "LIDAR_V2_K010_IP_PACKAGE_CHECK_PASS"
}
finally {
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

Write-Output "LIDAR_V2_IP_PACKAGE_RUNNER_PASS package=$PackageDir"
