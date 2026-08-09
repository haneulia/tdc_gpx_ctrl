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
$vivadoAppData = Join-Path $hdlRoot "tmp\vivado_k010_appdata"
New-Item -ItemType Directory -Force -Path `
    $session, $oocDir, $vivadoAppData | Out-Null

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
    $savedAppData = $env:APPDATA
    $savedHome = $env:HOME
    $savedUserProfile = $env:USERPROFILE
    $savedLocalAppData = $env:LOCALAPPDATA
    $locationPushed = $false
    try {
        # Vivado Tcl Store 상태가 사용자 GUI 환경이나 이전 실행에 의존하지
        # 않도록 이 회귀만의 격리된 사용자 캐시를 사용한다. Vivado 2025.2.1은
        # Tcl Store 위치를 정할 때 아래 네 경로를 함께 참조한다.
        $env:APPDATA = $vivadoAppData
        $env:HOME = $vivadoAppData
        $env:USERPROFILE = $vivadoAppData
        $env:LOCALAPPDATA = $vivadoAppData
        Push-Location $vivadoAppData
        $locationPushed = $true
        & $Vivado @arguments | Out-Host
        $vivadoExitCode = $LASTEXITCODE
    } finally {
        if ($locationPushed) {
            Pop-Location
        }
        $env:APPDATA = $savedAppData
        $env:HOME = $savedHome
        $env:USERPROFILE = $savedUserProfile
        $env:LOCALAPPDATA = $savedLocalAppData
    }
    if ($vivadoExitCode -ne 0) {
        throw "$Name failed with exit code $vivadoExitCode. See $log"
    }
    if (Select-String -LiteralPath $log -SimpleMatch `
            "CRITICAL WARNING:" -Quiet) {
        throw "$Name emitted a Vivado Critical Warning. See $log"
    }
    return $log
}

function Assert-VivadoWarningContract {
    param(
        [string]$Name,
        [string]$Log,
        [hashtable]$AllowedMaximum
    )

    $observed = @{}
    foreach ($line in Get-Content -LiteralPath $Log -Encoding UTF8) {
        if ($line -notmatch '^WARNING:') {
            continue
        }
        if ($line -notmatch '^WARNING:\s+\[([^\]]+)\]') {
            throw "$Name emitted an unclassified Vivado warning: $line"
        }

        $messageId = $matches[1]
        if (-not $AllowedMaximum.ContainsKey($messageId)) {
            throw "$Name emitted a new Vivado warning ID '$messageId'. See $Log"
        }
        if (-not $observed.ContainsKey($messageId)) {
            $observed[$messageId] = 0
        }
        $observed[$messageId]++
    }

    $audit = @()
    foreach ($messageId in ($AllowedMaximum.Keys | Sort-Object)) {
        $count = if ($observed.ContainsKey($messageId)) {
            $observed[$messageId]
        } else {
            0
        }
        $maximum = $AllowedMaximum[$messageId]
        if ($count -gt $maximum) {
            throw "$Name warning '$messageId' increased to $count (maximum $maximum). See $Log"
        }
        $audit += "${Name}: $messageId=$count/$maximum"
    }
    if ($AllowedMaximum.Count -eq 0) {
        $audit += "${Name}: no warnings"
    }

    Write-Host "[K0-10] warning contract PASS: $Name"
    return $audit
}

$warningAudit = @()

$packageLog = Invoke-VivadoBatch -Name "01_package" `
    -Tcl (Join-Path $scriptDir "package_v2_ip.tcl") `
    -TclArgs @($packageDir)
if (-not (Select-String -LiteralPath $packageLog -SimpleMatch `
        "TDC_GPX_LIDAR_CTRL_V2_IP_PACKAGE_PASS" -Quiet)) {
    throw "Package PASS marker is missing: $packageLog"
}
$warningAudit += Assert-VivadoWarningContract -Name "01_package" `
    -Log $packageLog -AllowedMaximum @{
        "IP_Flow 19-11770" = 3
        "IP_Flow 19-3158" = 3
        "IP_Flow 19-5661" = 3
        "IP_Flow 19-2187" = 1
        "IP_Flow 19-11888" = 1
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
$warningAudit += Assert-VivadoWarningContract -Name "02_check" `
    -Log $checkLog -AllowedMaximum @{}

$oocLog = Invoke-VivadoBatch -Name "03_packaged_ooc" `
    -Tcl (Join-Path $scriptDir "run_v2_k010_packaged_ooc.tcl") `
    -TclArgs @($oocDir)
if (-not (Select-String -LiteralPath $oocLog -SimpleMatch `
        "LIDAR_V2_K010_PACKAGED_OOC_MATRIX_PASS profiles=3" -Quiet)) {
    throw "Packaged OOC matrix PASS marker is missing: $oocLog"
}
$warningAudit += Assert-VivadoWarningContract -Name "03_packaged_ooc" `
    -Log $oocLog -AllowedMaximum @{
        "Synth 8-7129" = 300
        "Synth 8-6014" = 287
        "Synth 8-3917" = 123
        "Constraints 18-5572" = 15
        "Synth 8-3332" = 6
    }

Set-Content -LiteralPath (Join-Path $session "WARNING_AUDIT.txt") `
    -Value (@("LIDAR_V2_K010_WARNING_CONTRACT_PASS") + $warningAudit) `
    -Encoding UTF8

$summary = @(
    "LIDAR_V2_K010_IP_PACKAGE_SIGNOFF_PASS",
    "package=$packageDir",
    "source_sync=87 RTL + XGUI + 3 Korean guides",
    "catalog=v1 tdc_gpx_top:1.0 + v2 tdc_gpx_lidar_ctrl_v2:2.0",
    "ooc_profiles=async32(150/200), async128(200/150, echo off), sync64(150/150)",
    "warning_contract=new warning IDs or count increases fail sign-off",
    "session=$session"
)
Set-Content -LiteralPath (Join-Path $session "SUMMARY.txt") `
    -Value $summary -Encoding UTF8

Write-Host "LIDAR_V2_K010_IP_PACKAGE_SIGNOFF_PASS session=$session"
