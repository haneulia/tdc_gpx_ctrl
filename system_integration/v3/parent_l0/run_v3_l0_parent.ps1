param(
    [string]$OutputRoot = '',
    [string]$ReferenceBd =
        'C:\Projects\my_sp\ALINX\Logic\project_4\project_4.srcs\sources_1\bd\design_1_lidar_ctrl\design_1_lidar_ctrl.bd',
    [string]$BasePinXdc =
        'C:\Projects\my_sp\ALINX\Logic\project_4\project_4.srcs\constrs_1\imports\XDC\project4_vt_hrl2_tdc01_service.xdc',
    [string]$ExtensionPinXdc =
        'C:\Projects\my_sp\ALINX\Logic\project_4_tdc_gpx_4chip\project_4.srcs\constrs_1\imports\XDC\project4_vt_hrl2_tdc23_extension.xdc',
    [ValidateSet(32, 64)]
    [int]$OutputWidth = 32,
    [switch]$Recreate,
    [switch]$ValidateOnly
)

$ErrorActionPreference = 'Stop'
$RunnerMutex = [System.Threading.Mutex]::new(
    $false,
    'Local\Victek_TdcGpx_V3_Parent_Runner')
$RunnerLockAcquired = $false
try {
    try {
        $RunnerLockAcquired = $RunnerMutex.WaitOne(
            [TimeSpan]::FromMinutes(10))
    }
    catch [System.Threading.AbandonedMutexException] {
        # A terminated prior runner cannot leave a Windows named mutex locked.
        # The exception means this process now owns the recovered mutex.
        $RunnerLockAcquired = $true
    }
    if (-not $RunnerLockAcquired) {
        throw 'Timed out waiting for the V3 Parent runner lock.'
    }

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$HdlRoot = (Resolve-Path (Join-Path $ScriptDir '..\..\..')).Path
if ([string]::IsNullOrWhiteSpace($OutputRoot)) {
    $OutputRoot = Join-Path $HdlRoot ".work\v3_parent_l0\w$OutputWidth"
}
$Vivado = if ($env:XILINX_VIVADO) {
    Join-Path $env:XILINX_VIVADO 'bin\vivado.bat'
}
else {
    'C:\AMDDesignTools\2025.2.1\Vivado\bin\vivado.bat'
}
$CreateTcl = Join-Path $ScriptDir 'create_v3_l0_parent.tcl'
$ValidateTcl = Join-Path $ScriptDir 'validate_v3_l0_parent.tcl'
$TimingXdc = Join-Path $ScriptDir 'l0_v3_parent_timing.xdc'
$PackageComponent = Join-Path $HdlRoot `
    'system_integration\v3\ip_repo\tdc_gpx_lidar_ctrl_v3_3_0\component.xml'

foreach ($Required in @(
        $Vivado, $CreateTcl, $ValidateTcl, $ReferenceBd, $BasePinXdc,
        $ExtensionPinXdc, $TimingXdc, $PackageComponent)) {
    if (-not (Test-Path -LiteralPath $Required)) {
        throw "Required V3 L0 input is missing: $Required"
    }
}

$OutputRoot = [System.IO.Path]::GetFullPath($OutputRoot)
$AllowedRoot = [System.IO.Path]::GetFullPath((Join-Path $HdlRoot '.work'))
if ($ValidateOnly -and $Recreate) {
    throw '-ValidateOnly and -Recreate cannot be used together.'
}
if (Test-Path -LiteralPath $OutputRoot) {
    if (-not $Recreate -and -not $ValidateOnly) {
        throw "Output project already exists: $OutputRoot. Use -Recreate explicitly."
    }
    if ($Recreate -and -not $OutputRoot.StartsWith(
            $AllowedRoot,
            [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "V3 runner only recreates projects below $AllowedRoot"
    }
    if ($Recreate) {
        Remove-Item -LiteralPath $OutputRoot -Recurse -Force
    }
}
elseif ($ValidateOnly) {
    throw "Validate-only project does not exist: $OutputRoot"
}

function ConvertTo-TclBraceValue {
    param([AllowEmptyString()][string]$Value)
    return $Value.Replace('\', '\\').Replace('{', '\{').Replace('}', '\}')
}

$ReferenceJson = Get-Content -Raw -Encoding UTF8 -LiteralPath $ReferenceBd |
    ConvertFrom-Json
$ReferencePs = $ReferenceJson.design.components.processing_system7_0.parameters
if ($null -eq $ReferencePs) {
    throw 'processing_system7_0 parameters are missing from the reference BD.'
}
$PsProperties = @($ReferencePs.PSObject.Properties | Sort-Object Name)
$WorkRoot = Join-Path $HdlRoot '.work\v3_parent_l0\runner'
$ToolHome = Join-Path $WorkRoot 'tool_home'
New-Item -ItemType Directory -Force -Path $WorkRoot, $ToolHome | Out-Null
$PsConfigPath = Join-Path $WorkRoot ("ps7_config_{0}.tcl" -f $PID)
$PsConfigLines = @(
    '# Generated from the board-proven project_4 PS7 cell.',
    'set l0_ps_config [list]')
foreach ($Property in $PsProperties) {
    $Name = ConvertTo-TclBraceValue -Value ("CONFIG.{0}" -f $Property.Name)
    $Value = ConvertTo-TclBraceValue -Value ([string]$Property.Value.value)
    $PsConfigLines += "lappend l0_ps_config {$Name} {$Value}"
}
$PsConfigLines += "set l0_reference_ps_parameter_count $($PsProperties.Count)"
$PsConfigLines | Set-Content -Encoding ASCII -LiteralPath $PsConfigPath

$SavedEnvironment = @{
    HOME = $env:HOME
    USERPROFILE = $env:USERPROFILE
    APPDATA = $env:APPDATA
    LOCALAPPDATA = $env:LOCALAPPDATA
}
$LocationPushed = $false
$Mapped = $false
$SubstExe = 'C:\Windows\System32\subst.exe'
$ShortMapRoot = [System.IO.Path]::GetFullPath(
    (Split-Path -Parent $OutputRoot))
New-Item -ItemType Directory -Force -Path $ShortMapRoot | Out-Null
$ShortDrive = @('V:', 'W:', 'X:') |
    Where-Object { -not (Test-Path -LiteralPath "$_\") } |
    Select-Object -First 1
if ($null -eq $ShortDrive) {
    throw 'No free V:, W:, or X: drive is available for short-path mapping.'
}

function ConvertTo-ShortMappedPath {
    param([Parameter(Mandatory = $true)][string]$Path)
    $Full = [System.IO.Path]::GetFullPath($Path)
    if (-not $Full.StartsWith(
            $ShortMapRoot,
            [System.StringComparison]::OrdinalIgnoreCase)) {
        return $Full.Replace('\', '/')
    }
    $Relative = $Full.Substring($ShortMapRoot.Length).TrimStart('\')
    if ([string]::IsNullOrEmpty($Relative)) {
        return "$ShortDrive/"
    }
    return "$ShortDrive/$($Relative.Replace('\', '/'))"
}

try {
    $env:HOME = $ToolHome
    $env:USERPROFILE = $ToolHome
    $env:APPDATA = Join-Path $ToolHome 'AppData\Roaming'
    $env:LOCALAPPDATA = Join-Path $ToolHome 'AppData\Local'
    New-Item -ItemType Directory -Force -Path `
        $env:APPDATA, $env:LOCALAPPDATA | Out-Null
    Push-Location $ToolHome
    $LocationPushed = $true
    & $SubstExe $ShortDrive $ShortMapRoot
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to map $ShortDrive to $ShortMapRoot"
    }
    $Mapped = $true

    if (-not $ValidateOnly) {
        $CreateArguments = @(
            '-mode', 'batch', '-nolog', '-nojournal', '-notrace',
            '-source', $CreateTcl, '-tclargs',
            (ConvertTo-ShortMappedPath $OutputRoot),
            (ConvertTo-ShortMappedPath $PsConfigPath),
            $BasePinXdc.Replace('\', '/'), $ExtensionPinXdc.Replace('\', '/'),
            (ConvertTo-ShortMappedPath $TimingXdc), ([string]$OutputWidth),
            (ConvertTo-ShortMappedPath $HdlRoot))
        $CreateOutput = & $Vivado @CreateArguments 2>&1
        $CreateOutput | ForEach-Object { Write-Output $_ }
        if ($LASTEXITCODE -ne 0 -or
            -not ($CreateOutput -match 'LIDAR_V3_L0_PARENT_CREATE_PASS')) {
            throw 'V3 Parent creation failed or did not emit its PASS marker.'
        }
        if ($CreateOutput -match 'CRITICAL WARNING:') {
            throw 'V3 Parent creation emitted a Critical Warning.'
        }
    }

    $ValidateArguments = @(
        '-mode', 'batch', '-nolog', '-nojournal', '-notrace',
        '-source', $ValidateTcl, '-tclargs',
        (ConvertTo-ShortMappedPath $OutputRoot), ([string]$OutputWidth))
    $ValidateOutput = & $Vivado @ValidateArguments 2>&1
    $ValidateOutput | ForEach-Object { Write-Output $_ }
    if ($LASTEXITCODE -ne 0 -or
        -not ($ValidateOutput -match 'LIDAR_V3_L0_PARENT_VALIDATE_PASS')) {
        throw 'V3 Parent reopen validation failed or omitted its PASS marker.'
    }
    if (-not ($ValidateOutput -match 'LIDAR_V3_L0_VDMA_XCI_CONTRACT_PASS')) {
        throw 'V3 Parent VDMA XCI contract PASS marker is missing.'
    }
    if ($ValidateOutput -match 'CRITICAL WARNING:') {
        throw 'V3 Parent reopen validation emitted a Critical Warning.'
    }
}
finally {
    if ($LocationPushed) { Pop-Location }
    if ($Mapped) { & $SubstExe $ShortDrive /D | Out-Null }
    foreach ($Name in $SavedEnvironment.Keys) {
        $Value = $SavedEnvironment[$Name]
        if ($null -eq $Value) {
            Remove-Item "Env:$Name" -ErrorAction SilentlyContinue
        }
        else {
            Set-Item "Env:$Name" $Value
        }
    }
    if (Test-Path -LiteralPath $PsConfigPath) {
        Remove-Item -LiteralPath $PsConfigPath -Force
    }
}

Write-Output "LIDAR_V3_L0_PARENT_RUNNER_PASS output=$OutputRoot width=$OutputWidth"
}
finally {
    if ($RunnerLockAcquired) {
        $RunnerMutex.ReleaseMutex()
    }
    $RunnerMutex.Dispose()
}
