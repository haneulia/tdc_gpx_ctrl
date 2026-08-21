[CmdletBinding(SupportsShouldProcess)]
param()

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V2Root = Split-Path -Parent $ScriptDir
$CanonicalXgui = Join-Path $V2Root (
    "ip_package\tdc_gpx_lidar_ctrl_v2_xgui.tcl")
$PackagedXgui = Join-Path $V2Root (
    "ip_repo\tdc_gpx_lidar_ctrl_v2_2_0\xgui\" +
    "tdc_gpx_lidar_ctrl_v2_v2_0.tcl")
$ContractChecker = Join-Path $ScriptDir "check_v2_xgui_source_contract.tcl"

foreach ($Required in @($PackagedXgui, $ContractChecker)) {
    if (-not (Test-Path -LiteralPath $Required -PathType Leaf)) {
        throw "Required V2 XGUI maintenance file is missing: $Required"
    }
}

$Tclsh = (Get-Command tclsh -ErrorAction Stop).Source
$CheckOutput = & $Tclsh $ContractChecker $PackagedXgui 2>&1
$CheckOutput | ForEach-Object { Write-Output $_ }
if ($LASTEXITCODE -ne 0 -or
    -not ($CheckOutput -match "LIDAR_V2_XGUI_NATIVE_VISUAL_CONTRACT_PASS")) {
    throw (
        "The V2 Package IP XGUI is not in Vivado-native visual-editor " +
        "form. Do not import it into the canonical source.")
}

$SourceText = [System.IO.File]::ReadAllText($PackagedXgui)
$SourceText = $SourceText.Replace("`r`n", "`n")
$SourceText = $SourceText.Replace("`r", "`n").TrimEnd()
$DestinationText = if (Test-Path -LiteralPath $CanonicalXgui) {
    $Text = [System.IO.File]::ReadAllText($CanonicalXgui)
    $Text.Replace("`r`n", "`n").Replace("`r", "`n").TrimEnd()
} else {
    ""
}
$Changed = $SourceText -cne $DestinationText

if ($Changed -and $PSCmdlet.ShouldProcess(
        $CanonicalXgui, "Import V2 Package IP Layout/Preview XGUI")) {
    Copy-Item -LiteralPath $PackagedXgui -Destination $CanonicalXgui -Force
}

Write-Output (
    "TDC_GPX_LIDAR_CTRL_V2_XGUI_IMPORT_PASS " +
    "changed=$Changed canonical=$CanonicalXgui")
Write-Output (
    "Repackage V2 so component.xml and the canonical Layout/Preview " +
    "remain synchronized.")
