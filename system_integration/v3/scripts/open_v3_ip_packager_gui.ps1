[CmdletBinding()]
param(
    [ValidateSet("EmbeddedRtl", "HlsIp")]
    [string]$Variant = "EmbeddedRtl",
    [switch]$Recreate,
    [switch]$ValidateOnly,
    [switch]$RefreshPackage,
    [switch]$SkipHlsSynthesis,
    [switch]$RefreshHlsChildIp,
    [string]$VivadoRoot = ""
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$HdlRoot = (Resolve-Path (Join-Path $V3Root "..\..")).Path
$ControllerRoot = Split-Path -Parent $HdlRoot
$CreateTcl = Join-Path $ScriptDir "create_v3_ip_packager_project.tcl"
$CheckTcl = Join-Path $ScriptDir "check_v3_ip_packager_project.tcl"
$OpenTcl = Join-Path $ScriptDir "open_v3_ip_packager_gui.tcl"
switch ($Variant) {
    "EmbeddedRtl" {
        $PackageProfile = "EMBEDDED_RTL"
        $WorkRoot = Join-Path $HdlRoot ".work\v3_ip_packager"
        $ProjectName = "tdc_gpx_lidar_ctrl_v3_ip_packager"
        $PackageFolder = "tdc_gpx_lidar_ctrl_v3_3_0"
        $ExpectedVlnv = "victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3:3.0"
        $PackageRunner = Join-Path $ScriptDir "run_v3_ip_package.ps1"
        $PackageCheckTcl = Join-Path $ScriptDir "check_v3_ip_package.tcl"
        $PackageCheckMarker = "TDC_GPX_LIDAR_CTRL_V3_IP_CHECK_PASS"
        $PackageCheckUsesRepo = $false
    }
    "HlsIp" {
        $PackageProfile = "HLS_IP"
        $WorkRoot = Join-Path $HdlRoot ".work\v3_hls_ip_packager"
        $ProjectName = "tdc_gpx_lidar_ctrl_v3_hls_ip_packager"
        $PackageFolder = "tdc_gpx_lidar_ctrl_v3_hls_ip_3_0"
        $ExpectedVlnv = "victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3_hls_ip:3.0"
        $PackageRunner = Join-Path $ScriptDir "run_v3_hls_ip_package.ps1"
        $PackageCheckTcl = Join-Path $ScriptDir "check_v3_hls_ip_package.tcl"
        $PackageCheckMarker = "TDC_GPX_LIDAR_CTRL_V3_HLS_IP_CHECK_PASS"
        $PackageCheckUsesRepo = $true
    }
}
$ProjectPath = Join-Path $WorkRoot "$ProjectName.xpr"
$Component = Join-Path $V3Root "ip_repo\$PackageFolder\component.xml"

function Assert-File {
    param([string]$Path, [string]$Purpose)
    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw "$Purpose is missing: $Path"
    }
}

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

foreach ($Required in @(
        $CreateTcl, $CheckTcl, $PackageCheckTcl, $OpenTcl,
        $PackageRunner)) {
    Assert-File -Path $Required -Purpose "V3 IP Packager input"
}
if ($RefreshPackage) {
    $PackageArguments = @(
        "-NoProfile", "-ExecutionPolicy", "Bypass",
        "-File", $PackageRunner)
    if ($SkipHlsSynthesis) {
        $PackageArguments += "-SkipHlsSynthesis"
    }
    if ($Variant -eq "HlsIp" -and $RefreshHlsChildIp) {
        $PackageArguments += "-RefreshHlsChildIp"
    }
    & powershell.exe @PackageArguments
    if ($LASTEXITCODE -ne 0) {
        throw "V3 package refresh failed with exit code $LASTEXITCODE."
    }
}
Assert-File -Path $Component -Purpose "V3 $Variant packaged component"

$SubstExe = "C:\Windows\System32\subst.exe"
$ExistingMaps = @{}
foreach ($Line in @(& $SubstExe)) {
    if ($Line -match '^([A-Z]:)\\: => (.+)$') {
        $ExistingMaps[$Matches[1]] = [System.IO.Path]::GetFullPath(
            $Matches[2].Trim())
    }
}
$ShortDrive = @($ExistingMaps.Keys) |
    Where-Object { $ExistingMaps[$_] -eq $ControllerRoot } |
    Select-Object -First 1
$MapCreated = $false
if ($null -eq $ShortDrive) {
    $ShortDrive = @("P:", "Q:", "R:", "S:", "T:", "U:") |
        Where-Object { -not $ExistingMaps.ContainsKey($_) -and
            -not (Test-Path -LiteralPath "$_\") } |
        Select-Object -First 1
    if ($null -eq $ShortDrive) {
        throw "No free P: through U: drive is available for V3 IP Packager."
    }
    & $SubstExe $ShortDrive $ControllerRoot
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to map $ShortDrive to $ControllerRoot"
    }
    $MapCreated = $true
}

$MappedHdlRoot = "$ShortDrive\HDL"
$MappedWorkRoot = if ($Variant -eq "EmbeddedRtl") {
    "$MappedHdlRoot\.work\v3_ip_packager"
} else {
    "$MappedHdlRoot\.work\v3_hls_ip_packager"
}
$MappedProjectPath = "$MappedWorkRoot\$ProjectName.xpr"
$MappedIpRepo = "$MappedHdlRoot\system_integration\v3\ip_repo"
$MappedComponent = "$MappedIpRepo\$PackageFolder\component.xml"
$MappedPackageDir = Split-Path -Parent $MappedComponent
$MappedPackageCheckArgument = if ($PackageCheckUsesRepo) {
    $MappedIpRepo
} else {
    $MappedPackageDir
}
$ResolvedVivadoRoot = Resolve-VivadoRoot
$Vivado = Join-Path $ResolvedVivadoRoot "bin\vivado.bat"

$AllowedRoot = [System.IO.Path]::GetFullPath(
    (Join-Path $HdlRoot ".work"))
$ResolvedWorkRoot = [System.IO.Path]::GetFullPath($WorkRoot)
if (-not $ResolvedWorkRoot.StartsWith(
        "$AllowedRoot\",
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to manage an IP Packager project outside $AllowedRoot"
}
if ($Recreate -and (Test-Path -LiteralPath $WorkRoot)) {
    Remove-Item -LiteralPath $WorkRoot -Recurse -Force
}
New-Item -ItemType Directory -Force -Path $WorkRoot | Out-Null

$LocationPushed = $false
$GuiStarted = $false
try {
    Push-Location $WorkRoot
    $LocationPushed = $true

    # Refuse to open a stale or externally referenced package. This is the
    # same source/XGUI/IP-XACT contract used by the batch release workflow.
    $PackageCheckOutput = & $Vivado @(
        "-mode", "batch", "-nolog", "-nojournal", "-notrace",
        "-source", $PackageCheckTcl, "-tclargs",
        $MappedPackageCheckArgument) 2>&1
    $PackageCheckOutput | ForEach-Object { Write-Output $_ }
    if ($LASTEXITCODE -ne 0 -or -not ($PackageCheckOutput -match
            $PackageCheckMarker)) {
        throw "Canonical V3 packaged IP validation failed."
    }

    if (-not (Test-Path -LiteralPath $ProjectPath -PathType Leaf)) {
        $CreateOutput = & $Vivado @(
            "-mode", "batch", "-nolog", "-nojournal", "-notrace",
            "-source", $CreateTcl, "-tclargs",
            $MappedComponent, $MappedWorkRoot, $ProjectName,
            $ExpectedVlnv) 2>&1
        $CreateOutput | ForEach-Object { Write-Output $_ }
        if ($LASTEXITCODE -ne 0 -or -not ($CreateOutput -match
                "TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGER_CREATE_PASS")) {
            throw "V3 IP Packager project creation failed."
        }
    }

    Assert-File -Path $MappedProjectPath -Purpose "V3 IP Packager project"
    $CheckOutput = & $Vivado @(
        "-mode", "batch", "-nolog", "-nojournal", "-notrace",
        "-source", $CheckTcl, "-tclargs",
        $MappedProjectPath, $MappedComponent, $ExpectedVlnv,
        $PackageProfile) 2>&1
    $CheckOutput | ForEach-Object { Write-Output $_ }
    if ($LASTEXITCODE -ne 0 -or -not ($CheckOutput -match
            "TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGER_CHECK_PASS")) {
        throw "V3 IP Packager project validation failed."
    }

    if ($ValidateOnly) {
        Write-Output (
            "TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGER_RUNNER_VALIDATE_PASS " +
            "project=$MappedProjectPath")
        exit 0
    }

    $Process = Start-Process -FilePath $Vivado -ArgumentList @(
        "-mode", "gui", "-nolog", "-nojournal",
        "-source", $OpenTcl, "-tclargs",
        $MappedProjectPath, $MappedComponent, $ExpectedVlnv) `
        -WorkingDirectory $WorkRoot -PassThru
    $GuiStarted = $true
    Write-Output (
        "TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGER_GUI_STARTED " +
        "variant=$Variant pid=$($Process.Id) project=$MappedProjectPath " +
        "component=$MappedComponent short_map=$ShortDrive=>$ControllerRoot")
    Write-Output (
        "Keep $ShortDrive mapped while IP Packager is open. Close every " +
        "dependent Vivado window before: subst $ShortDrive /D")
    Write-Output (
        "Package IP edits metadata. To inspect the actual end-user XGUI, run " +
        "open_v3_customize_ip_gui.ps1 -Variant $Variant")
}
finally {
    if ($LocationPushed) { Pop-Location }
    # A mapping created by this runner is required for the lifetime of the GUI
    # because the edit project deliberately stores short P:/... style paths.
    # Remove it automatically when validation finishes or startup fails.
    if (-not $GuiStarted -and $MapCreated) {
        & $SubstExe $ShortDrive /D | Out-Null
    }
}
