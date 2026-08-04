param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$WorkRoot = Join-Path $Hdl "tmp/v2_config_pkg"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl `
    "signoff_results/sessions/${Stamp}_v2_config_pkg"

New-Item -ItemType Directory -Force -Path $Work, $Archive | Out-Null

function Invoke-Checked {
    param(
        [string]$Exe,
        [string[]]$ArgList
    )

    & $Exe @ArgList
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Exe $($ArgList -join ' ')"
    }
}

$Files = @(
    "$Hdl/system_integration/v2/pkg/lidar_build_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_config_types_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_config_reference_pkg.vhd",
    "$Hdl/system_integration/v2/tb/tb_lidar_config_types_pkg.vhd"
)

$Project = Join-Path $Work "v2_config_pkg.prj"
$ProjectLines = foreach ($File in $Files) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $Project

$CompileLog = Join-Path $Work "xvhdl.log"
$ElabLog = Join-Path $Work "xelab.log"
$SimLog = Join-Path $Work "xsim.log"
$Snapshot = "tb_lidar_config_types_pkg_${Stamp}_snap"
$RunTcl = Join-Path $Work "run.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

Push-Location $Work
try {
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--relax", "-prj", $Project, "-log", $CompileLog
    )
    Invoke-Checked "$Vivado/xelab.bat" @(
        "--debug", "off", "--relax", "--mt", "2",
        "--snapshot", $Snapshot,
        "xil_defaultlib.tb_lidar_config_types_pkg",
        "-log", $ElabLog
    )
    Invoke-Checked "$Vivado/xsim.bat" @(
        $Snapshot,
        "-tclbatch", $RunTcl.Replace('\', '/'),
        "-log", $SimLog.Replace('\', '/')
    )
}
finally {
    Pop-Location
}

$SimText = Get-Content -Raw -LiteralPath $SimLog
if ($SimText -notmatch "LIDAR_V2_CONFIG_TYPES_PASS") {
    throw "v2 configuration package did not report PASS. See $SimLog"
}

$Manifest = foreach ($File in $Files) {
    $Item = Get-Item -LiteralPath $File
    $Hash = Get-FileHash -LiteralPath $File -Algorithm SHA256
    [ordered]@{
        path = $Item.FullName.Replace('\', '/')
        bytes = $Item.Length
        sha256 = $Hash.Hash
    }
}
$Manifest | ConvertTo-Json -Depth 3 | Set-Content -Encoding ASCII `
    -LiteralPath (Join-Path $Archive "source_manifest.json")

Copy-Item -Force -LiteralPath `
    $CompileLog, $ElabLog, $SimLog, $Project, $RunTcl -Destination $Archive

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith($ResolvedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "LIDAR_V2_CONFIG_TYPES_PASS"
Write-Output "Result: $Archive"
