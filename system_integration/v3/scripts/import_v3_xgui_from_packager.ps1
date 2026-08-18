[CmdletBinding(SupportsShouldProcess)]
param(
    [ValidateSet("EmbeddedRtl", "HlsIp")]
    [string]$Variant = "EmbeddedRtl"
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$CanonicalXgui = Join-Path $V3Root (
    "ip_package\tdc_gpx_lidar_ctrl_v3_xgui.tcl")
$ContractChecker = Join-Path $ScriptDir "check_v3_xgui_source_contract.tcl"

switch ($Variant) {
    "EmbeddedRtl" {
        $PackageFolder = "tdc_gpx_lidar_ctrl_v3_3_0"
        $XguiName = "tdc_gpx_lidar_ctrl_v3_v3_0.tcl"
    }
    "HlsIp" {
        $PackageFolder = "tdc_gpx_lidar_ctrl_v3_hls_ip_3_0"
        $XguiName = "tdc_gpx_lidar_ctrl_v3_hls_ip_v3_0.tcl"
    }
}
$PackagedXgui = Join-Path $V3Root (
    "ip_repo\$PackageFolder\xgui\$XguiName")

foreach ($Required in @($PackagedXgui, $ContractChecker)) {
    if (-not (Test-Path -LiteralPath $Required -PathType Leaf)) {
        throw "Required XGUI maintenance file is missing: $Required"
    }
}

$Tclsh = (Get-Command tclsh -ErrorAction Stop).Source
$CheckOutput = & $Tclsh $ContractChecker $PackagedXgui 2>&1
$CheckOutput | ForEach-Object { Write-Output $_ }
if ($LASTEXITCODE -ne 0 -or
    -not ($CheckOutput -match "LIDAR_V3_XGUI_NATIVE_VISUAL_CONTRACT_PASS")) {
    throw (
        "The Package IP XGUI is not in Vivado-native visual-editor form. " +
        "Do not import it into the canonical source.")
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
        $CanonicalXgui, "Import Package IP Layout/Preview XGUI")) {
    Copy-Item -LiteralPath $PackagedXgui -Destination $CanonicalXgui -Force
}

Write-Output (
    "TDC_GPX_LIDAR_CTRL_V3_XGUI_IMPORT_PASS variant=$Variant " +
    "changed=$Changed canonical=$CanonicalXgui")
Write-Output (
    "Repackage EmbeddedRtl and HlsIp so both VLNVs receive the same " +
    "canonical Layout/Preview.")
