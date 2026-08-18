[CmdletBinding()]
param(
    [switch]$Recreate,
    [switch]$ValidateOnly,
    [switch]$RefreshPackage,
    [switch]$SkipHlsSynthesis,
    [string]$VivadoRoot = ""
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$HdlRoot = (Resolve-Path (Join-Path $V3Root "..\..")).Path
$WorkRoot = Join-Path $HdlRoot ".work\v3_ip_packager"
$ProjectName = "tdc_gpx_lidar_ctrl_v3_ip_packager"
$ProjectPath = Join-Path $WorkRoot "$ProjectName.xpr"
$Component = Join-Path $V3Root `
    "ip_repo\tdc_gpx_lidar_ctrl_v3_3_0\component.xml"
$CreateTcl = Join-Path $ScriptDir "create_v3_ip_packager_project.tcl"
$CheckTcl = Join-Path $ScriptDir "check_v3_ip_packager_project.tcl"
$PackageCheckTcl = Join-Path $ScriptDir "check_v3_ip_package.tcl"
$OpenTcl = Join-Path $ScriptDir "open_v3_ip_packager_gui.tcl"
$PackageRunner = Join-Path $ScriptDir "run_v3_ip_package.ps1"

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
        $Component, $CreateTcl, $CheckTcl, $PackageCheckTcl, $OpenTcl,
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
    & powershell.exe @PackageArguments
    if ($LASTEXITCODE -ne 0) {
        throw "V3 package refresh failed with exit code $LASTEXITCODE."
    }
}

$SubstExe = "C:\Windows\System32\subst.exe"
$ExistingMaps = @{}
foreach ($Line in @(& $SubstExe)) {
    if ($Line -match '^([A-Z]:)\\: => (.+)$') {
        $ExistingMaps[$Matches[1]] = [System.IO.Path]::GetFullPath(
            $Matches[2].Trim())
    }
}
$ShortDrive = @($ExistingMaps.Keys) |
    Where-Object { $ExistingMaps[$_] -eq $HdlRoot } |
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
    & $SubstExe $ShortDrive $HdlRoot
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to map $ShortDrive to $HdlRoot"
    }
    $MapCreated = $true
}

$MappedWorkRoot = "$ShortDrive\.work\v3_ip_packager"
$MappedProjectPath = "$MappedWorkRoot\$ProjectName.xpr"
$MappedComponent = "$ShortDrive\system_integration\v3\ip_repo" +
    "\tdc_gpx_lidar_ctrl_v3_3_0\component.xml"
$MappedPackageDir = Split-Path -Parent $MappedComponent
$ResolvedVivadoRoot = Resolve-VivadoRoot
$Vivado = Join-Path $ResolvedVivadoRoot "bin\vivado.bat"
$ToolHome = Join-Path $WorkRoot "tool_home"
New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null

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
    New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null
}

$SavedEnvironment = @{
    HOME = $env:HOME
    USERPROFILE = $env:USERPROFILE
    APPDATA = $env:APPDATA
    LOCALAPPDATA = $env:LOCALAPPDATA
}
$LocationPushed = $false
$GuiStarted = $false
try {
    $env:HOME = $ToolHome
    $env:USERPROFILE = $ToolHome
    $env:APPDATA = Join-Path $ToolHome "AppData\Roaming"
    $env:LOCALAPPDATA = Join-Path $ToolHome "AppData\Local"
    New-Item -ItemType Directory -Force -Path `
        $env:APPDATA, $env:LOCALAPPDATA | Out-Null
    Push-Location $ToolHome
    $LocationPushed = $true

    # Refuse to open a stale or externally referenced package. This is the
    # same source/XGUI/IP-XACT contract used by the batch release workflow.
    $PackageCheckOutput = & $Vivado @(
        "-mode", "batch", "-nolog", "-nojournal", "-notrace",
        "-source", $PackageCheckTcl, "-tclargs",
        $MappedPackageDir) 2>&1
    $PackageCheckOutput | ForEach-Object { Write-Output $_ }
    if ($LASTEXITCODE -ne 0 -or -not ($PackageCheckOutput -match
            "TDC_GPX_LIDAR_CTRL_V3_IP_CHECK_PASS")) {
        throw "Canonical V3 packaged IP validation failed."
    }

    if (-not (Test-Path -LiteralPath $ProjectPath -PathType Leaf)) {
        $CreateOutput = & $Vivado @(
            "-mode", "batch", "-nolog", "-nojournal", "-notrace",
            "-source", $CreateTcl, "-tclargs",
            $MappedComponent, $MappedWorkRoot, $ProjectName) 2>&1
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
        $MappedProjectPath, $MappedComponent) 2>&1
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
        $MappedProjectPath, $MappedComponent) `
        -WorkingDirectory $ToolHome -PassThru
    $GuiStarted = $true
    Write-Output (
        "TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGER_GUI_STARTED " +
        "pid=$($Process.Id) project=$MappedProjectPath " +
        "component=$MappedComponent short_map=$ShortDrive=>$HdlRoot")
    Write-Output (
        "Keep $ShortDrive mapped while IP Packager is open. Close every " +
        "dependent Vivado window before: subst $ShortDrive /D")
}
finally {
    if ($LocationPushed) { Pop-Location }
    foreach ($Name in $SavedEnvironment.Keys) {
        $Value = $SavedEnvironment[$Name]
        if ($null -eq $Value) {
            Remove-Item "Env:$Name" -ErrorAction SilentlyContinue
        }
        else {
            Set-Item "Env:$Name" $Value
        }
    }
    # A mapping created by this runner is required for the lifetime of the GUI
    # because the edit project deliberately stores short P:/... style paths.
    # Remove it automatically when validation finishes or startup fails.
    if (-not $GuiStarted -and $MapCreated) {
        & $SubstExe $ShortDrive /D | Out-Null
    }
}
