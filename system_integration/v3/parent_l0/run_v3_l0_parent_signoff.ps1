param(
    [string]$ProjectRoot = '',
    [ValidateSet('SYNTH', 'IMPL')]
    [string]$Mode = 'IMPL',
    [string]$SessionTag = ''
)

$ErrorActionPreference = 'Stop'
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$HdlRoot = (Resolve-Path (Join-Path $ScriptDir '..\..\..')).Path
if ([string]::IsNullOrWhiteSpace($ProjectRoot)) {
    $ProjectRoot = Join-Path $HdlRoot '.work\v3_parent_l0\w32'
}
$Vivado = if ($env:XILINX_VIVADO) {
    Join-Path $env:XILINX_VIVADO 'bin\vivado.bat'
} else {
    'C:\AMDDesignTools\2025.2.1\Vivado\bin\vivado.bat'
}
$SignoffTcl = Join-Path $ScriptDir 'run_v3_l0_parent_signoff.tcl'
$ProjectRoot = [System.IO.Path]::GetFullPath($ProjectRoot)
$ProjectPath = Join-Path $ProjectRoot 'project_4_lidar_v3_l0.xpr'

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

$ResultDir = Join-Path $HdlRoot ".work\v3_parent_signoff\$SessionTag"
if (Test-Path -LiteralPath $ResultDir) {
    throw "Sign-off result directory already exists: $ResultDir"
}
New-Item -ItemType Directory -Path $ResultDir | Out-Null
$LogPath = Join-Path $ResultDir 'vivado.log'
$VivadoAppData = Join-Path $HdlRoot '.work\v3_parent_signoff\tool_home'
New-Item -ItemType Directory -Force -Path $VivadoAppData | Out-Null

$ShortPathRoot = [System.IO.Path]::GetFullPath(
    (Split-Path -Parent $ProjectRoot))
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
    # Vivado의 write_hw_platform은 Windows에서도 Linux 전용 CPU profiler
    # 안내를 stderr로 내보낼 수 있다. Windows PowerShell은 전역 Stop 정책에서
    # 이 비치명적 stderr 한 줄을 NativeCommandError 예외로 승격하므로, Vivado
    # 실행 구간만 Continue로 낮추고 실제 성공 여부는 종료 코드와 PASS 표식으로
    # 엄격하게 판정한다.
    $SavedNativeErrorAction = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        $Output = & $Vivado @Arguments 2>&1
        $ExitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $SavedNativeErrorAction
    }
    # Vivado 2025.2.1 emits this harmless platform probe on stderr even when
    # every Tcl command succeeds.  Keep all other stderr records in the log.
    $Output = @($Output | Where-Object {
            ([string]$_) -notmatch '^Google Cpuprofiler is only supported on Linux\.$'
        })
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
$PassMarker = "LIDAR_V3_L0_PARENT_${Mode}_SIGNOFF_PASS"
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

Write-Output "LIDAR_V3_L0_PARENT_SIGNOFF_RUNNER_PASS results=$ResultDir"
