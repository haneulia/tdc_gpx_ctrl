param(
    [string]$OutputRoot =
        'C:\Projects\my_sp\ALINX\Logic\project_4_lidar_v2_l0',
    [string]$ReferenceBd =
        'C:\Projects\my_sp\ALINX\Logic\project_4\project_4.srcs\sources_1\bd\design_1_lidar_ctrl\design_1_lidar_ctrl.bd',
    [string]$ExtensionPinXdc =
        'C:\Projects\my_sp\ALINX\Logic\project_4_tdc_gpx_4chip\project_4.srcs\constrs_1\imports\XDC\project4_vt_hrl2_tdc23_extension.xdc',
    [ValidateSet(32, 64, 128)]
    [int]$OutputWidth = 32,
    [switch]$Recreate
)

$ErrorActionPreference = 'Stop'
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$HdlRoot = (Resolve-Path (Join-Path $ScriptDir '..\..\..')).Path
$Vivado = if ($env:XILINX_VIVADO) {
    Join-Path $env:XILINX_VIVADO 'bin\vivado.bat'
} else {
    'C:\AMDDesignTools\2025.2.1\Vivado\bin\vivado.bat'
}
$CreateTcl = Join-Path $ScriptDir 'create_v2_l0_parent.tcl'
$ValidateTcl = Join-Path $ScriptDir 'validate_v2_l0_parent.tcl'
$PinXdc =
    'C:\Projects\my_sp\ALINX\Logic\project_4\project_4.srcs\constrs_1\imports\XDC\project4_vt_hrl2_tdc01_service.xdc'
$TimingXdc = Join-Path $ScriptDir 'constraints\l0_parent_timing.xdc'

foreach ($Required in @(
        $Vivado, $CreateTcl, $ValidateTcl, $ReferenceBd, $PinXdc,
        $ExtensionPinXdc, $TimingXdc)) {
    if (-not (Test-Path -LiteralPath $Required)) {
        throw "Required L0 input is missing: $Required"
    }
}

$OutputRoot = [System.IO.Path]::GetFullPath($OutputRoot)
if (Test-Path -LiteralPath $OutputRoot) {
    if (-not $Recreate) {
        throw "Output project already exists: $OutputRoot. Use -Recreate explicitly."
    }

    $AllowedParent = [System.IO.Path]::GetFullPath(
        'C:\Projects\my_sp\ALINX\Logic')
    $AllowedLeaf = 'project_4_lidar_v2_l0'
    $OutputItem = Get-Item -LiteralPath $OutputRoot
    if (-not $OutputItem.FullName.StartsWith(
            $AllowedParent,
            [System.StringComparison]::OrdinalIgnoreCase) -or
        $OutputItem.Name -ne $AllowedLeaf) {
        throw "Refusing to recreate unexpected directory: $OutputRoot"
    }
    Remove-Item -LiteralPath $OutputRoot -Recurse -Force
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
$PsConfigDirectory = Join-Path $HdlRoot 'signoff_results\work'
New-Item -ItemType Directory -Force -Path $PsConfigDirectory | Out-Null
$VivadoAppData = Join-Path $HdlRoot 'tmp\vivado_l0_appdata'
New-Item -ItemType Directory -Force -Path $VivadoAppData | Out-Null
$PsConfigPath = Join-Path $PsConfigDirectory (
    "lidar_v2_l0_ps7_config_{0}.tcl" -f $PID)
$PsConfigLines = @(
    '# Generated from the board-proven design_1_lidar_ctrl PS7 cell.',
    'set l0_ps_config [list]')
foreach ($Property in $PsProperties) {
    $Name = ConvertTo-TclBraceValue -Value ("CONFIG.{0}" -f $Property.Name)
    $Value = ConvertTo-TclBraceValue -Value ([string]$Property.Value.value)
    $PsConfigLines += "lappend l0_ps_config {$Name} {$Value}"
}
$PsConfigLines += "set l0_reference_ps_parameter_count $($PsProperties.Count)"
$PsConfigLines | Set-Content -Encoding ASCII -LiteralPath $PsConfigPath

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
    $TclOutputRoot = $OutputRoot.Replace('\', '/')
    $TclPsConfigPath = $PsConfigPath.Replace('\', '/')
    $TclPinXdc = $PinXdc.Replace('\', '/')
    $TclExtensionPinXdc = $ExtensionPinXdc.Replace('\', '/')
    $TclTimingXdc = $TimingXdc.Replace('\', '/')
    $TclHdlRoot = $HdlRoot.Replace('\', '/')
    $Arguments = @(
        '-mode', 'batch',
        '-nolog', '-nojournal', '-notrace',
        '-source', $CreateTcl,
        '-tclargs', $TclOutputRoot, $TclPsConfigPath, $TclPinXdc,
        $TclExtensionPinXdc, $TclTimingXdc, ([string]$OutputWidth),
        $TclHdlRoot)
    $Output = & $Vivado @Arguments 2>&1
    $Output | ForEach-Object { Write-Output $_ }
    if ($LASTEXITCODE -ne 0) {
        throw "Vivado L0 project generation failed with exit code $LASTEXITCODE"
    }
    if (-not ($Output -match 'LIDAR_V2_L0_PARENT_CREATE_PASS')) {
        throw 'Vivado completed without the L0 parent PASS marker.'
    }
    if ($Output -match 'CRITICAL WARNING:') {
        throw 'Vivado L0 project creation emitted a Critical Warning.'
    }

    $ValidateArguments = @(
        '-mode', 'batch',
        '-nolog', '-nojournal', '-notrace',
        '-source', $ValidateTcl,
        '-tclargs', $TclOutputRoot, ([string]$OutputWidth))
    $ValidateOutput = & $Vivado @ValidateArguments 2>&1
    $ValidateOutput | ForEach-Object { Write-Output $_ }
    if ($LASTEXITCODE -ne 0) {
        throw "Vivado L0 project validation failed with exit code $LASTEXITCODE"
    }
    if (-not ($ValidateOutput -match 'LIDAR_V2_L0_PARENT_VALIDATE_PASS')) {
        throw 'Vivado completed without the L0 validation PASS marker.'
    }
    if ($ValidateOutput -match 'CRITICAL WARNING:') {
        throw 'Vivado L0 project validation emitted a Critical Warning.'
    }
} finally {
    if ($LocationPushed) {
        Pop-Location
    }
    $env:APPDATA = $SavedAppData
    $env:HOME = $SavedHome
    $env:USERPROFILE = $SavedUserProfile
    $env:LOCALAPPDATA = $SavedLocalAppData
    if (Test-Path -LiteralPath $PsConfigPath) {
        Remove-Item -LiteralPath $PsConfigPath -Force
    }
}

Write-Output "LIDAR_V2_L0_PARENT_RUNNER_PASS output=$OutputRoot width=$OutputWidth"
