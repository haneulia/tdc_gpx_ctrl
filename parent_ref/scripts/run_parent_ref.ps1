param(
    [ValidateSet('VALIDATE', 'SYNTH', 'IMPL')]
    [string]$Mode = 'VALIDATE',
    [string]$Stamp = (Get-Date -Format 'yyMMddHHmmss')
)

$ErrorActionPreference = 'Stop'

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ParentDir = Split-Path -Parent $ScriptDir
$HdlDir = Split-Path -Parent $ParentDir
$BuildDir = Join-Path $HdlDir ".tmp_parent/$Stamp"
$ResultDir = Join-Path $ParentDir "results/sessions/${Stamp}_ps_fclk_parent_ref"
$TclScript = Join-Path $ScriptDir 'create_parent_ref.tcl'
$VerifyScript = Join-Path $ScriptDir 'verify_parent_contract.ps1'
$SignoffScript = Join-Path $ScriptDir 'verify_parent_signoff.ps1'

$Vivado = 'C:/AMDDesignTools/2025.2.1/Vivado/bin/vivado.bat'
if (-not (Test-Path -LiteralPath $Vivado)) {
    throw "Vivado 2025.2.1 was not found at $Vivado"
}

# The local TclStore catalog is damaged. Its installation fallback contains
# all packages, but does not add leaf package directories to Tcl auto_path.
$TclStore = 'C:/AMDDesignTools/2025.2.1/Vivado/data/XilinxTclStore'
$PackageDirs = Get-ChildItem -LiteralPath $TclStore -Recurse `
    -File -Filter 'pkgIndex.tcl' | ForEach-Object {
        $_.DirectoryName.Replace('\', '/')
    } | Sort-Object -Unique
$TclLibPath = $PackageDirs -join ' '
$env:TCLLIBPATH = if ([string]::IsNullOrWhiteSpace($env:TCLLIBPATH)) {
    $TclLibPath
} else {
    "$TclLibPath $env:TCLLIBPATH"
}

New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
New-Item -ItemType Directory -Force -Path $ResultDir | Out-Null

& $Vivado -mode batch -nojournal -nolog -source $TclScript `
    -tclargs $BuildDir $ResultDir $Mode

if ($LASTEXITCODE -ne 0) {
    throw "Parent reference $Mode failed with exit code $LASTEXITCODE"
}

$BdPath = Join-Path $BuildDir `
    'tdc_gpx_parent_ref.srcs/sources_1/bd/tdc_gpx_parent/tdc_gpx_parent.bd'
$ContractReport = Join-Path $ResultDir 'parent_ref_contract_verified.txt'
& $VerifyScript -BdPath $BdPath -ReportPath $ContractReport

if ($Mode -in @('SYNTH', 'IMPL')) {
    & $SignoffScript -ResultDir $ResultDir -Stage $Mode
}

Write-Host "Parent reference $Mode PASS"
Write-Host "Project: $BuildDir/tdc_gpx_parent_ref.xpr"
Write-Host "Results: $ResultDir"
