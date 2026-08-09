param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Glbl = (Resolve-Path "C:/AMDDesignTools/2025.2.1/Vivado/data/verilog/src/glbl.v").Path
$WorkRoot = Join-Path $Hdl "tmp/v2_stream_gateway_reset"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl "signoff_results/sessions/${Stamp}_v2_stream_gateway_reset"
$Top = "tb_lidar_stream_gateway_reset"
$Snapshot = "${Top}_${Stamp}_snap"

New-Item -ItemType Directory -Force -Path $Work | Out-Null

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

$VhdlFiles = @(
    "$Hdl/system_integration/v2/pkg/lidar_build_pkg.vhd",
    "$Hdl/system_integration/v2/rtl/common/lidar_stream_gateway.vhd",
    "$Hdl/system_integration/v2/tb/tb_lidar_stream_gateway_reset.vhd"
)
$VhdlProject = Join-Path $Work "vhdl.prj"
$VerilogProject = Join-Path $Work "verilog.prj"
$RunTcl = Join-Path $Work "run.tcl"

@($VhdlFiles | ForEach-Object {
    "vhdl2008 xil_defaultlib `"$($_.Replace('\', '/'))`""
}) + "nosort" | Set-Content -Encoding ASCII -LiteralPath $VhdlProject
@(
    "verilog xil_defaultlib `"$($Glbl.Replace('\', '/'))`"",
    "nosort"
) | Set-Content -Encoding ASCII -LiteralPath $VerilogProject
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

Push-Location $Work
try {
    Invoke-Checked "$Vivado/xvlog.bat" @(
        "--relax", "-prj", $VerilogProject, "-log", "xvlog.log"
    )
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--relax", "-prj", $VhdlProject, "-log", "xvhdl.log"
    )
    Invoke-Checked "$Vivado/xelab.bat" @(
        "--debug", "off", "--relax", "--mt", "2",
        "--snapshot", $Snapshot,
        "xil_defaultlib.$Top", "xil_defaultlib.glbl",
        "-log", "xelab.log"
    )
    Invoke-Checked "$Vivado/xsim.bat" @(
        $Snapshot, "-tclbatch", $RunTcl.Replace('\', '/'),
        "-log", "xsim.log"
    )
}
finally {
    Pop-Location
}

$SimulationText = Get-Content -Raw -LiteralPath (Join-Path $Work "xsim.log")
if ($SimulationText -notmatch "LIDAR_V2_STREAM_GATEWAY_RESET_200_TO_150_PASS") {
    throw "Stream gateway reset regression did not report its PASS marker"
}

New-Item -ItemType Directory -Force -Path $Archive | Out-Null
Copy-Item -Force -LiteralPath @(
    (Join-Path $Work "xvhdl.log"),
    (Join-Path $Work "xelab.log"),
    (Join-Path $Work "xsim.log")
) -Destination $Archive

$Manifest = foreach ($File in $VhdlFiles) {
    $Item = Get-Item -LiteralPath $File
    $Hash = Get-FileHash -LiteralPath $File -Algorithm SHA256
    [ordered]@{
        path = $Item.FullName.Replace('\', '/')
        bytes = $Item.Length
        sha256 = $Hash.Hash
    }
}
$Manifest | ConvertTo-Json -Depth 3 | Set-Content -Encoding ASCII -LiteralPath (
    Join-Path $Archive "source_manifest.json")

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith($ResolvedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "LIDAR_V2_STREAM_GATEWAY_RESET_200_TO_150_PASS"
Write-Output "Result: $Archive"
