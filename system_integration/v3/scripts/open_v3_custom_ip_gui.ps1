[CmdletBinding()]
param(
    [ValidateSet(32, 64)]
    [int]$OutputWidth = 32,
    [string]$OutputRoot = "",
    [switch]$Recreate,
    [switch]$ValidateOnly,
    [string]$VivadoRoot = ""
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$RepoRoot = (Resolve-Path (Join-Path $V3Root "..\..")).Path
$AllowedRoot = [System.IO.Path]::GetFullPath((Join-Path $RepoRoot ".work"))
$ParentRunner = Join-Path $V3Root "parent_l0\run_v3_l0_parent.ps1"
$CheckTcl = Join-Path $ScriptDir "check_v3_custom_ip_gui.tcl"
$IpRepo = Join-Path $V3Root "ip_repo"

if ([string]::IsNullOrWhiteSpace($OutputRoot)) {
    # Keep the physical path short as a second line of defence. Vivado is also
    # launched through a SUBST drive below because generated AXI infrastructure
    # can add more than 190 characters below the project directory.
    $OutputRoot = Join-Path $RepoRoot ".work\v3gui\w$OutputWidth"
}
$OutputRoot = [System.IO.Path]::GetFullPath($OutputRoot)
if (-not $OutputRoot.StartsWith(
        $AllowedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Custom IP GUI projects must be created below $AllowedRoot"
}
if ($ValidateOnly -and $Recreate) {
    throw "-ValidateOnly and -Recreate cannot be used together."
}

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
        "C:\AMDDesignTools\2025.2\Vivado"
    )

    foreach ($Candidate in $Candidates | Select-Object -Unique) {
        if (Test-Path -LiteralPath (Join-Path $Candidate "bin\vivado.bat")) {
            return (Resolve-Path -LiteralPath $Candidate).Path
        }
    }
    throw "Vivado installation was not found. Supply -VivadoRoot."
}

Assert-File -Path $ParentRunner -Purpose "V3 Parent generator"
Assert-File -Path $CheckTcl -Purpose "Custom IP GUI validator"
Assert-File `
    -Path (Join-Path $IpRepo "tdc_gpx_lidar_ctrl_v3_3_0\component.xml") `
    -Purpose "V3 packaged IP"

$ProjectPath = Join-Path $OutputRoot "project_4_lidar_v3_l0.xpr"
$ProjectExists = Test-Path -LiteralPath $ProjectPath -PathType Leaf
if ($ValidateOnly -and -not $ProjectExists) {
    throw "Writable Parent project does not exist: $ProjectPath"
}

if (-not $ValidateOnly -and ($Recreate -or -not $ProjectExists)) {
    $RunnerArguments = @(
        "-NoProfile", "-ExecutionPolicy", "Bypass",
        "-File", $ParentRunner,
        "-OutputRoot", $OutputRoot,
        "-OutputWidth", ([string]$OutputWidth)
    )
    if ($Recreate -or (Test-Path -LiteralPath $OutputRoot)) {
        $RunnerArguments += "-Recreate"
    }

    & powershell.exe @RunnerArguments
    if ($LASTEXITCODE -ne 0) {
        throw "Writable V3 Parent generation failed with exit code $LASTEXITCODE."
    }
}

Assert-File -Path $ProjectPath -Purpose "Writable V3 Parent project"
$ResolvedVivadoRoot = Resolve-VivadoRoot
$Vivado = Join-Path $ResolvedVivadoRoot "bin\vivado.bat"
$CheckArguments = @(
    "-mode", "batch", "-nolog", "-nojournal", "-notrace",
    "-source", $CheckTcl, "-tclargs", $ProjectPath, $IpRepo
)
$CheckSessionRoot = Join-Path $RepoRoot `
    ".work\v3_gui_sessions\CUSTOM_W$OutputWidth\check"
$CheckToolHome = Join-Path $CheckSessionRoot "tool_home"
New-Item -ItemType Directory -Force -Path `
    $CheckSessionRoot, $CheckToolHome | Out-Null
$SavedEnvironment = @{
    HOME = $env:HOME
    USERPROFILE = $env:USERPROFILE
    APPDATA = $env:APPDATA
    LOCALAPPDATA = $env:LOCALAPPDATA
}
$LocationPushed = $false
try {
    $env:HOME = $CheckToolHome
    $env:USERPROFILE = $CheckToolHome
    $env:APPDATA = Join-Path $CheckToolHome "AppData\Roaming"
    $env:LOCALAPPDATA = Join-Path $CheckToolHome "AppData\Local"
    New-Item -ItemType Directory -Force -Path `
        $env:APPDATA, $env:LOCALAPPDATA | Out-Null
    Push-Location $CheckSessionRoot
    $LocationPushed = $true
    $CheckOutput = & $Vivado @CheckArguments 2>&1
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
}
$CheckOutput | ForEach-Object { Write-Output $_ }
if ($LASTEXITCODE -ne 0 -or
    -not ($CheckOutput -match "LIDAR_V3_CUSTOM_IP_GUI_VALIDATE_PASS")) {
    throw (
        "Writable V3 Parent is stale or locked. Re-run with -Recreate " +
        "after closing any Vivado window that uses it.")
}

if ($ValidateOnly) {
    Write-Output (
        "LIDAR_V3_CUSTOM_IP_GUI_RUNNER_VALIDATE_PASS " +
        "width=$OutputWidth project=$ProjectPath")
    exit 0
}

$SubstExe = "C:\Windows\System32\subst.exe"
$ShortMapRoot = [System.IO.Path]::GetFullPath(
    (Split-Path -Parent $OutputRoot))
$ExistingMaps = @{}
foreach ($Line in @(& $SubstExe)) {
    if ($Line -match '^([A-Z]:)\\: => (.+)$') {
        $ExistingMaps[$Matches[1]] = [System.IO.Path]::GetFullPath(
            $Matches[2].Trim())
    }
}

$ShortDrive = @($ExistingMaps.Keys) |
    Where-Object {
        $ExistingMaps[$_] -eq $ShortMapRoot
    } |
    Select-Object -First 1
if ($null -eq $ShortDrive) {
    $ShortDrive = @("V:", "W:", "X:", "Y:", "Z:") |
        Where-Object { -not $ExistingMaps.ContainsKey($_) -and
            -not (Test-Path -LiteralPath "$_\") } |
        Select-Object -First 1
    if ($null -eq $ShortDrive) {
        throw (
            "No free V:, W:, X:, Y:, or Z: drive is available for the " +
            "Vivado short-path mapping.")
    }
    & $SubstExe $ShortDrive $ShortMapRoot
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to map $ShortDrive to $ShortMapRoot"
    }
}

$RelativeProjectPath = $ProjectPath.Substring(
    $ShortMapRoot.Length).TrimStart('\')
$MappedProjectPath = "$ShortDrive\$RelativeProjectPath"
Assert-File -Path $MappedProjectPath -Purpose "Short-path V3 Parent project"

$SessionRoot = Join-Path $RepoRoot ".work\v3_gui_sessions\CUSTOM_W$OutputWidth"
New-Item -ItemType Directory -Force -Path $SessionRoot | Out-Null
$Process = Start-Process `
    -FilePath $Vivado `
    -ArgumentList @("-mode", "gui", "-nolog", "-nojournal", $MappedProjectPath) `
    -WorkingDirectory $SessionRoot `
    -PassThru

Write-Output (
    "LIDAR_V3_CUSTOM_IP_GUI_STARTED width=$OutputWidth " +
    "access=READ_WRITE pid=$($Process.Id) project=$MappedProjectPath " +
    "physical_project=$ProjectPath short_map=$ShortDrive=>$ShortMapRoot")
Write-Output (
    "Keep $ShortDrive mapped while Vivado is open. After every Vivado window " +
    "using it is closed, release it with: subst $ShortDrive /D")
