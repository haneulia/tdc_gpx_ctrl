[CmdletBinding()]
param(
    [switch]$Recreate,
    [switch]$ValidateOnly,
    [switch]$RefreshPackage,
    [string]$VivadoRoot = ""
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V2Root = Split-Path -Parent $ScriptDir
$HdlRoot = (Resolve-Path (Join-Path $V2Root "..\..")).Path
$ControllerRoot = Split-Path -Parent $HdlRoot
$WorkRoot = Join-Path $HdlRoot ".work\v2_ip_packager"
$ProjectName = "tdc_gpx_lidar_ctrl_v2_ip_packager"
$ProjectPath = Join-Path $WorkRoot "$ProjectName.xpr"
$PackageFolder = "tdc_gpx_lidar_ctrl_v2_2_0"
$ExpectedVlnv = "victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v2:2.0"
$Component = Join-Path $V2Root "ip_repo\$PackageFolder\component.xml"
$PackagedXgui = Join-Path $V2Root (
    "ip_repo\$PackageFolder\xgui\tdc_gpx_lidar_ctrl_v2_v2_0.tcl")
$CreateTcl = Join-Path $ScriptDir "create_v2_ip_packager_project.tcl"
$CheckTcl = Join-Path $ScriptDir "check_v2_ip_packager_project.tcl"
$OpenTcl = Join-Path $ScriptDir "open_v2_ip_packager_gui.tcl"
$PackageCheckTcl = Join-Path $ScriptDir "check_v2_ip_package.tcl"
$PackageRunner = Join-Path $ScriptDir "run_v2_ip_package.ps1"
$ToolHome = Join-Path $HdlRoot ".work\v2_ip_packager_gui_tool_home"

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

function Set-XguiPreviewFreshness {
    Assert-File $Component "V2 packaged component"
    Assert-File $PackagedXgui "V2 packaged XGUI"
    $ComponentTime = (Get-Item -LiteralPath $Component).LastWriteTimeUtc
    $XguiItem = Get-Item -LiteralPath $PackagedXgui
    if ($XguiItem.LastWriteTimeUtc -le $ComponentTime) {
        $XguiItem.LastWriteTimeUtc = $ComponentTime.AddSeconds(2)
    }
    if ($XguiItem.LastWriteTimeUtc -le $ComponentTime) {
        throw "Unable to establish V2 XGUI preview freshness."
    }
}

foreach ($Required in @(
        $CreateTcl, $CheckTcl, $OpenTcl, $PackageCheckTcl,
        $PackageRunner)) {
    Assert-File $Required "V2 IP Packager input"
}
$ResolvedVivadoRoot = Resolve-VivadoRoot
$Vivado = Join-Path $ResolvedVivadoRoot "bin\vivado.bat"
if ($RefreshPackage) {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
        $PackageRunner -VivadoRoot $ResolvedVivadoRoot
    if ($LASTEXITCODE -ne 0) {
        throw "V2 package refresh failed with exit code $LASTEXITCODE."
    }
}
Assert-File $Component "V2 packaged component"
Assert-File $PackagedXgui "V2 packaged XGUI"

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
        throw "No free P: through U: drive is available for V2 IP Packager."
    }
    & $SubstExe $ShortDrive $ControllerRoot
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to map $ShortDrive to $ControllerRoot"
    }
    $MapCreated = $true
}

$MappedHdlRoot = "$ShortDrive\HDL"
$MappedWorkRoot = "$MappedHdlRoot\.work\v2_ip_packager"
$MappedProjectPath = "$MappedWorkRoot\$ProjectName.xpr"
$MappedIpRepo = "$MappedHdlRoot\system_integration\v2\ip_repo"
$MappedComponent = "$MappedIpRepo\$PackageFolder\component.xml"
$MappedXgui = "$MappedIpRepo\$PackageFolder\xgui\tdc_gpx_lidar_ctrl_v2_v2_0.tcl"
$MappedPackageDir = Split-Path -Parent $MappedComponent

$AllowedRoot = [System.IO.Path]::GetFullPath((Join-Path $HdlRoot ".work"))
$ResolvedWorkRoot = [System.IO.Path]::GetFullPath($WorkRoot)
if (-not $ResolvedWorkRoot.StartsWith(
        "$AllowedRoot\", [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to manage a V2 IP Packager project outside $AllowedRoot"
}
if ($Recreate -and (Test-Path -LiteralPath $WorkRoot)) {
    Remove-Item -LiteralPath $WorkRoot -Recurse -Force
}
New-Item -ItemType Directory -Force -Path $WorkRoot | Out-Null

$LocationPushed = $false
$GuiStarted = $false
$SavedEnvironment = @{
    HOME = $env:HOME
    USERPROFILE = $env:USERPROFILE
    APPDATA = $env:APPDATA
    LOCALAPPDATA = $env:LOCALAPPDATA
}
New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null
$env:HOME = $ToolHome
$env:USERPROFILE = $ToolHome
$env:APPDATA = Join-Path $ToolHome "AppData\Roaming"
$env:LOCALAPPDATA = Join-Path $ToolHome "AppData\Local"
New-Item -ItemType Directory -Force -Path $env:APPDATA | Out-Null
New-Item -ItemType Directory -Force -Path $env:LOCALAPPDATA | Out-Null
try {
    Push-Location $WorkRoot
    $LocationPushed = $true
    $PackageCheckOutput = & $Vivado @(
        "-mode", "batch", "-nolog", "-nojournal", "-notrace",
        "-source", $PackageCheckTcl, "-tclargs", $MappedPackageDir) 2>&1
    $PackageCheckOutput | ForEach-Object { Write-Output $_ }
    if ($LASTEXITCODE -ne 0 -or -not ($PackageCheckOutput -match
            "LIDAR_V2_K010_IP_PACKAGE_CHECK_PASS")) {
        throw "Canonical V2 packaged IP validation failed."
    }
    if ($PackageCheckOutput -match "CRITICAL WARNING:") {
        throw "Critical Warning detected during V2 package validation."
    }

    if (-not (Test-Path -LiteralPath $ProjectPath -PathType Leaf)) {
        $CreateOutput = & $Vivado @(
            "-mode", "batch", "-nolog", "-nojournal", "-notrace",
            "-source", $CreateTcl, "-tclargs", $MappedComponent,
            $MappedWorkRoot, $ProjectName, $ExpectedVlnv) 2>&1
        $CreateOutput | ForEach-Object { Write-Output $_ }
        if ($LASTEXITCODE -ne 0 -or -not ($CreateOutput -match
                "TDC_GPX_LIDAR_CTRL_V2_IP_PACKAGER_CREATE_PASS")) {
            throw "V2 IP Packager project creation failed."
        }
        if ($CreateOutput -match "CRITICAL WARNING:") {
            throw "Critical Warning detected while creating V2 edit project."
        }
    }

    Assert-File $MappedProjectPath "V2 IP Packager project"
    $CheckOutput = & $Vivado @(
        "-mode", "batch", "-nolog", "-nojournal", "-notrace",
        "-source", $CheckTcl, "-tclargs", $MappedProjectPath,
        $MappedComponent, $ExpectedVlnv) 2>&1
    $CheckOutput | ForEach-Object { Write-Output $_ }
    if ($LASTEXITCODE -ne 0 -or -not ($CheckOutput -match
            "TDC_GPX_LIDAR_CTRL_V2_IP_PACKAGER_CHECK_PASS")) {
        throw "V2 IP Packager project validation failed."
    }
    if ($CheckOutput -match "CRITICAL WARNING:") {
        throw "Critical Warning detected while checking V2 edit project."
    }

    if ($ValidateOnly) {
        Write-Output (
            "TDC_GPX_LIDAR_CTRL_V2_IP_PACKAGER_RUNNER_VALIDATE_PASS " +
            "project=$MappedProjectPath")
        return
    }

    Set-XguiPreviewFreshness
    $Process = Start-Process -FilePath $Vivado -ArgumentList @(
        "-mode", "gui", "-nolog", "-nojournal", "-source", $OpenTcl,
        "-tclargs", $MappedProjectPath, $MappedComponent,
        $ExpectedVlnv, $MappedXgui) -WorkingDirectory $WorkRoot -PassThru
    $GuiStarted = $true
    Write-Output (
        "TDC_GPX_LIDAR_CTRL_V2_IP_PACKAGER_GUI_STARTED " +
        "pid=$($Process.Id) project=$MappedProjectPath " +
        "component=$MappedComponent short_map=$ShortDrive=>$ControllerRoot")
    Write-Output (
        "Keep $ShortDrive mapped while IP Packager is open. Close every " +
        "dependent Vivado window before: subst $ShortDrive /D")
}
finally {
    if ($LocationPushed) { Pop-Location }
    if (-not $GuiStarted -and $MapCreated) {
        & $SubstExe $ShortDrive /D | Out-Null
    }
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
