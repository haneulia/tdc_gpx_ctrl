[CmdletBinding()]
param(
    [switch]$Recreate,
    [string]$VivadoRoot = ""
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V2Root = Split-Path -Parent $ScriptDir
$HdlRoot = (Resolve-Path (Join-Path $V2Root "..\..")).Path
$ControllerRoot = Split-Path -Parent $HdlRoot
$WorkRoot = Join-Path $HdlRoot ".work\v2_customize_ip"
$ProjectName = "tdc_gpx_lidar_ctrl_v2_customize"
$ProjectPath = Join-Path $WorkRoot "$ProjectName.xpr"
$ExpectedVlnv = "victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v2:2.0"
$ModuleName = "tdc_gpx_lidar_ctrl_v2_preview"
$OpenTcl = Join-Path $ScriptDir "open_v2_customize_ip_gui.tcl"
$XguiChecker = Join-Path $ScriptDir "check_v2_xgui_source_contract.tcl"
$PackagedXgui = Join-Path $V2Root (
    "ip_repo\tdc_gpx_lidar_ctrl_v2_2_0\xgui\tdc_gpx_lidar_ctrl_v2_v2_0.tcl")
$ToolHome = Join-Path $HdlRoot ".work\v2_customize_gui_tool_home"

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

foreach ($Required in @($OpenTcl, $XguiChecker, $PackagedXgui)) {
    if (-not (Test-Path -LiteralPath $Required -PathType Leaf)) {
        throw "V2 Customize IP input is missing: $Required"
    }
}
$XguiCheck = & tclsh.exe $XguiChecker $PackagedXgui 2>&1
$XguiCheck | ForEach-Object { Write-Output $_ }
if ($LASTEXITCODE -ne 0 -or -not ($XguiCheck -match
        "LIDAR_V2_XGUI_NATIVE_VISUAL_CONTRACT_PASS")) {
    throw "V2 native XGUI contract failed before Customize IP launch."
}

$AllowedRoot = [System.IO.Path]::GetFullPath((Join-Path $HdlRoot ".work"))
$ResolvedWorkRoot = [System.IO.Path]::GetFullPath($WorkRoot)
if (-not $ResolvedWorkRoot.StartsWith(
        "$AllowedRoot\", [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to manage a V2 preview project outside $AllowedRoot"
}
if ($Recreate -and (Test-Path -LiteralPath $WorkRoot)) {
    Remove-Item -LiteralPath $WorkRoot -Recurse -Force
}
New-Item -ItemType Directory -Force -Path $WorkRoot | Out-Null

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
        throw "No free P: through U: drive is available for V2 Customize IP."
    }
    & $SubstExe $ShortDrive $ControllerRoot
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to map $ShortDrive to $ControllerRoot"
    }
    $MapCreated = $true
}

$MappedHdlRoot = "$ShortDrive\HDL"
$MappedWorkRoot = "$MappedHdlRoot\.work\v2_customize_ip"
$MappedIpRepo = "$MappedHdlRoot\system_integration\v2\ip_repo"
$MappedOpenTcl = "$MappedHdlRoot\system_integration\v2\scripts\open_v2_customize_ip_gui.tcl"
$Vivado = Join-Path (Resolve-VivadoRoot) "bin\vivado.bat"
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
    $Process = Start-Process -FilePath $Vivado -ArgumentList @(
        "-mode", "gui", "-nolog", "-nojournal", "-source", $MappedOpenTcl,
        "-tclargs", $MappedWorkRoot, $ProjectName, $MappedIpRepo,
        $ExpectedVlnv, $ModuleName) -WorkingDirectory $WorkRoot -PassThru
    $GuiStarted = $true
    Write-Output (
        "TDC_GPX_LIDAR_CTRL_V2_CUSTOMIZE_IP_GUI_STARTED " +
        "pid=$($Process.Id) project=$ProjectPath vlnv=$ExpectedVlnv " +
        "short_map=$ShortDrive=>$ControllerRoot")
    Write-Output (
        "Keep $ShortDrive mapped while Customize IP is open. Close every " +
        "dependent Vivado window before: subst $ShortDrive /D")
}
finally {
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
