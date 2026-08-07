param(
    [string]$Vivado = ""
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$v2Dir = Split-Path -Parent $scriptDir
$hdlRoot = (Resolve-Path (Join-Path $v2Dir "..\..")).Path
$packageDir = Join-Path $v2Dir "ip_repo\tdc_gpx_lidar_ctrl_v2_2_0"

if ([string]::IsNullOrWhiteSpace($Vivado)) {
    if ($env:XILINX_VIVADO) {
        $Vivado = Join-Path $env:XILINX_VIVADO "bin\vivado.bat"
    } else {
        $Vivado = "C:\AMDDesignTools\2025.2.1\Vivado\bin\vivado.bat"
    }
}
if (-not (Test-Path -LiteralPath $Vivado)) {
    throw "Vivado executable is missing: $Vivado"
}

$stamp = Get-Date -Format "yyMMdd_HHmmss"
$session = Join-Path $hdlRoot "signoff_results\sessions\${stamp}_k010_ip_package"
$oocDir = Join-Path $session "packaged_ooc"
New-Item -ItemType Directory -Force -Path $session, $oocDir | Out-Null

function Invoke-VivadoBatch {
    param(
        [string]$Name,
        [string]$Tcl,
        [string[]]$TclArgs = @()
    )

    $log = Join-Path $session "${Name}.log"
    $journal = Join-Path $session "${Name}.jou"
    $arguments = @(
        "-mode", "batch", "-notrace",
        "-log", $log,
        "-journal", $journal,
        "-source", $Tcl
    )
    if ($TclArgs.Count -gt 0) {
        $arguments += "-tclargs"
        $arguments += $TclArgs
    }

    Write-Host "[K0-10] $Name"
    & $Vivado @arguments | Out-Host
    if ($LASTEXITCODE -ne 0) {
        throw "$Name failed with exit code $LASTEXITCODE. See $log"
    }
    return $log
}

$packageLog = Invoke-VivadoBatch -Name "01_package" `
    -Tcl (Join-Path $scriptDir "package_v2_ip.tcl") `
    -TclArgs @($packageDir)
if (-not (Select-String -LiteralPath $packageLog -SimpleMatch `
        "TDC_GPX_LIDAR_CTRL_V2_IP_PACKAGE_PASS" -Quiet)) {
    throw "Package PASS marker is missing: $packageLog"
}

$checkLog = Invoke-VivadoBatch -Name "02_check" `
    -Tcl (Join-Path $scriptDir "check_v2_ip_package.tcl") `
    -TclArgs @($packageDir)
foreach ($marker in @(
    "LIDAR_V2_K010_SOURCE_SYNC_PASS",
    "LIDAR_V2_K010_COMPONENT_CONTRACT_PASS",
    "LIDAR_V2_K010_XGUI_CONTRACT_PASS",
    "LIDAR_V2_K010_V1_V2_CATALOG_COEXIST_PASS",
    "LIDAR_V2_K010_IP_PACKAGE_CHECK_PASS"
)) {
    if (-not (Select-String -LiteralPath $checkLog -SimpleMatch $marker -Quiet)) {
        throw "Package check marker '$marker' is missing: $checkLog"
    }
}

$oocLog = Invoke-VivadoBatch -Name "03_packaged_ooc" `
    -Tcl (Join-Path $scriptDir "run_v2_k010_packaged_ooc.tcl") `
    -TclArgs @($oocDir)
if (-not (Select-String -LiteralPath $oocLog -SimpleMatch `
        "LIDAR_V2_K010_PACKAGED_OOC_MATRIX_PASS profiles=3" -Quiet)) {
    throw "Packaged OOC matrix PASS marker is missing: $oocLog"
}

$summary = @(
    "LIDAR_V2_K010_IP_PACKAGE_SIGNOFF_PASS",
    "package=$packageDir",
    "source_sync=87 RTL + XGUI + 2 Korean guides",
    "catalog=v1 tdc_gpx_top:1.0 + v2 tdc_gpx_lidar_ctrl_v2:2.0",
    "ooc_profiles=async32(150/200), async128(200/150, echo off), sync64(150/150)",
    "session=$session"
)
Set-Content -LiteralPath (Join-Path $session "SUMMARY.txt") `
    -Value $summary -Encoding UTF8

Write-Host "LIDAR_V2_K010_IP_PACKAGE_SIGNOFF_PASS session=$session"
