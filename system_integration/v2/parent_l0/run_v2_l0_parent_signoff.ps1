param(
    [string]$ProjectRoot =
        'C:\Projects\my_sp\ALINX\Logic\project_4_lidar_v2_l0',
    [ValidateSet('SYNTH', 'IMPL')]
    [string]$Mode = 'IMPL',
    [string]$SessionTag = ''
)

$ErrorActionPreference = 'Stop'
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$HdlRoot = (Resolve-Path (Join-Path $ScriptDir '..\..\..')).Path
$Vivado = if ($env:XILINX_VIVADO) {
    Join-Path $env:XILINX_VIVADO 'bin\vivado.bat'
} else {
    'C:\AMDDesignTools\2025.2.1\Vivado\bin\vivado.bat'
}
$SignoffTcl = Join-Path $ScriptDir 'run_v2_l0_parent_signoff.tcl'
$ProjectRoot = [System.IO.Path]::GetFullPath($ProjectRoot)
$ProjectPath = Join-Path $ProjectRoot 'project_4_lidar_v2_l0.xpr'

foreach ($Required in @($Vivado, $SignoffTcl, $ProjectPath)) {
    if (-not (Test-Path -LiteralPath $Required)) {
        throw "Required L0 sign-off input is missing: $Required"
    }
}
if ([string]::IsNullOrWhiteSpace($SessionTag)) {
    $SessionTag = '{0}_l0_parent_{1}' -f (
        Get-Date -Format 'yyMMdd_HHmmss'), $Mode.ToLowerInvariant()
}
if ($SessionTag -notmatch '^[A-Za-z0-9_.-]+$') {
    throw "SessionTag contains unsupported characters: $SessionTag"
}

$ResultDir = Join-Path $HdlRoot "signoff_results\sessions\$SessionTag"
if (Test-Path -LiteralPath $ResultDir) {
    throw "Sign-off result directory already exists: $ResultDir"
}
New-Item -ItemType Directory -Path $ResultDir | Out-Null
$LogPath = Join-Path $ResultDir 'vivado.log'
$VivadoAppData = Join-Path $HdlRoot 'tmp\vivado_l0_appdata'
New-Item -ItemType Directory -Force -Path $VivadoAppData | Out-Null

$ShortPathRoot = [System.IO.Path]::GetFullPath('C:\Projects\my_sp')
if (-not $ProjectRoot.StartsWith(
        $ShortPathRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "ProjectRoot must be below $ShortPathRoot for the Vivado short-path mapping."
}
$SubstExe = 'C:\Windows\System32\subst.exe'
$ShortDrive = @('V:', 'W:', 'X:') |
    Where-Object { -not (Test-Path -LiteralPath "$_\") } |
    Select-Object -First 1
if ($null -eq $ShortDrive) {
    throw 'No free V:, W:, or X: drive is available for the Vivado short-path mapping.'
}
$RelativeProject = $ProjectRoot.Substring($ShortPathRoot.Length).TrimStart('\')
$TclProjectRoot = "$ShortDrive/$($RelativeProject.Replace('\', '/'))"
$TclResultDir = $ResultDir.Replace('\', '/')
$Arguments = @(
    '-mode', 'batch',
    '-nolog', '-nojournal', '-notrace',
    '-source', $SignoffTcl,
    '-tclargs', $TclProjectRoot, $TclResultDir, $Mode)
$Mapped = $false
$SavedAppData = $env:APPDATA
$SavedHome = $env:HOME
$SavedUserProfile = $env:USERPROFILE
$SavedLocalAppData = $env:LOCALAPPDATA
$LocationPushed = $false
try {
    $env:APPDATA = $VivadoAppData
    $env:HOME = $VivadoAppData
    $env:USERPROFILE = $VivadoAppData
    $env:LOCALAPPDATA = $VivadoAppData
    Push-Location $VivadoAppData
    $LocationPushed = $true
    & $SubstExe $ShortDrive $ShortPathRoot
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to map $ShortDrive to $ShortPathRoot"
    }
    $Mapped = $true
    $Output = & $Vivado @Arguments 2>&1
    $ExitCode = $LASTEXITCODE
} finally {
    if ($LocationPushed) {
        Pop-Location
    }
    $env:APPDATA = $SavedAppData
    $env:HOME = $SavedHome
    $env:USERPROFILE = $SavedUserProfile
    $env:LOCALAPPDATA = $SavedLocalAppData
    if ($Mapped) {
        & $SubstExe $ShortDrive /D
    }
}
$Output | Set-Content -Encoding UTF8 -LiteralPath $LogPath
$Output | ForEach-Object { Write-Output $_ }

if ($ExitCode -ne 0) {
    throw "Vivado L0 $Mode sign-off failed with exit code $ExitCode"
}
$PassMarker = "LIDAR_V2_L0_PARENT_${Mode}_SIGNOFF_PASS"
if (-not ($Output -match [regex]::Escape($PassMarker))) {
    throw "Vivado completed without the L0 $Mode PASS marker."
}
if ($Output -match 'CRITICAL WARNING:') {
    throw 'Vivado L0 sign-off emitted a Critical Warning.'
}

$Commit = (& git -C $HdlRoot rev-parse HEAD).Trim()
$Branch = (& git -C $HdlRoot branch --show-current).Trim()
$Manifest = @(
    "session=$SessionTag",
    "mode=$Mode",
    "created=$(Get-Date -Format 'yyyy-MM-ddTHH:mm:ssK')",
    "hdl_commit=$Commit",
    "hdl_branch=$Branch",
    "project=$ProjectRoot",
    "vivado_short_path_root=$ShortPathRoot",
    "vivado=$Vivado")
$Manifest | Set-Content -Encoding UTF8 -LiteralPath (
    Join-Path $ResultDir 'session_manifest.txt')

Write-Output "LIDAR_V2_L0_PARENT_SIGNOFF_RUNNER_PASS results=$ResultDir"
