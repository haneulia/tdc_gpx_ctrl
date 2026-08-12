param(
    [switch]$SkipHlsSynthesis,
    [string]$VivadoRoot = 'C:\AMDDesignTools\2025.2.1\Vivado'
)

$ErrorActionPreference = 'Stop'
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$HdlRoot = (Resolve-Path (Join-Path $V3Root '..\..')).Path
$Vivado = Join-Path $VivadoRoot 'bin\vivado.bat'
$PackageTcl = Join-Path $ScriptDir 'package_v3_ip.tcl'
$CheckTcl = Join-Path $ScriptDir 'check_v3_ip_package.tcl'
$PackageDir = Join-Path $V3Root 'ip_repo\tdc_gpx_lidar_ctrl_v3_3_0'
$ToolHome = Join-Path $HdlRoot '.work\v3_ip_package_tool_home'

function Invoke-Checked {
    param(
        [Parameter(Mandatory = $true)][string]$Executable,
        [Parameter(Mandatory = $true)][string[]]$ToolArguments,
        [Parameter(Mandatory = $true)][string]$PassMarker
    )
    $Output = & $Executable @ToolArguments 2>&1
    $Output | ForEach-Object { Write-Output $_ }
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Executable"
    }
    if (-not ($Output -match [regex]::Escape($PassMarker))) {
        throw "Required PASS marker is missing: $PassMarker"
    }
    if ($Output -match 'CRITICAL WARNING:') {
        throw "Critical Warning detected while expecting $PassMarker"
    }
}

foreach ($Required in @($Vivado, $PackageTcl, $CheckTcl)) {
    if (-not (Test-Path -LiteralPath $Required)) {
        throw "Required V3 package input is missing: $Required"
    }
}

$HlsRunners = @(
    'run_v3_hls_hit_decoder.ps1',
    'run_v3_hls_cell_collector.ps1',
    'run_v3_hls_frame_assembler.ps1',
    'run_v3_hls_lane_word_formatter.ps1')
if (-not $SkipHlsSynthesis) {
    foreach ($Runner in $HlsRunners) {
        & powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
            (Join-Path $ScriptDir $Runner) -Step csynth
        if ($LASTEXITCODE -ne 0) {
            throw "HLS synthesis failed: $Runner"
        }
    }
}

$GeneratedDirs = @(
    '.work\v3_hls_hit_decoder_component\hls\syn\verilog',
    '.work\v3_hls_cell_collector_component\hls\syn\verilog',
    '.work\v3_hls_frame_assembler_component\hls\syn\verilog',
    '.work\v3_hls_lane_word_formatter_component\hls\syn\verilog')
foreach ($Relative in $GeneratedDirs) {
    $Directory = Join-Path $HdlRoot $Relative
    if (-not (Test-Path -LiteralPath $Directory)) {
        throw "Generated HLS RTL directory is missing: $Directory"
    }
}

New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null
$SavedEnvironment = @{
    HOME = $env:HOME
    USERPROFILE = $env:USERPROFILE
    APPDATA = $env:APPDATA
    LOCALAPPDATA = $env:LOCALAPPDATA
}
$env:HOME = $ToolHome
$env:USERPROFILE = $ToolHome
$env:APPDATA = Join-Path $ToolHome 'AppData\Roaming'
$env:LOCALAPPDATA = Join-Path $ToolHome 'AppData\Local'
New-Item -ItemType Directory -Force -Path $env:APPDATA | Out-Null
New-Item -ItemType Directory -Force -Path $env:LOCALAPPDATA | Out-Null

try {
    Invoke-Checked -Executable $Vivado -ToolArguments @(
        '-mode', 'batch', '-notrace', '-source', $PackageTcl,
        '-log', (Join-Path $ToolHome 'package_v3_ip.log'),
        '-journal', (Join-Path $ToolHome 'package_v3_ip.jou')) `
        -PassMarker 'TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGE_PASS'

    $HlsPackageRoot = Join-Path $PackageDir 'src\hls_generated'
    $HashManifest = Join-Path $PackageDir 'doc\HLS_GENERATED_SHA256.txt'
    $HashLines = @(
        '# Vitis HLS generated RTL copied into this self-contained IP package.',
        "generated=$(Get-Date -Format 'yyyy-MM-ddTHH:mm:ssK')")
    Get-ChildItem -File -Recurse -LiteralPath $HlsPackageRoot |
        Sort-Object FullName | ForEach-Object {
            $Relative = $_.FullName.Substring(
                $PackageDir.Length + 1).Replace('\', '/')
            $Hash = (Get-FileHash -Algorithm SHA256 -LiteralPath $_.FullName).Hash
            $HashLines += "$Hash  $Relative"
        }
    $HashLines | Set-Content -Encoding ASCII -LiteralPath $HashManifest

    Invoke-Checked -Executable $Vivado -ToolArguments @(
        '-mode', 'batch', '-notrace', '-source', $CheckTcl,
        '-tclargs', $PackageDir,
        '-log', (Join-Path $ToolHome 'check_v3_ip.log'),
        '-journal', (Join-Path $ToolHome 'check_v3_ip.jou')) `
        -PassMarker 'TDC_GPX_LIDAR_CTRL_V3_IP_CHECK_PASS'
}
finally {
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

Write-Output "LIDAR_V3_IP_PACKAGE_RUNNER_PASS package=$PackageDir"
