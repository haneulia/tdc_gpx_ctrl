param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Work = Join-Path $Hdl "tmp/unified_csr/$Stamp"
$Archive = Join-Path $Hdl `
    "sim_results/vivado_xsim/sessions/${Stamp}_unified_csr_contract"

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
    "$Hdl/system_integration/rtl/lidar_unified_csr_pkg.vhd",
    "$Hdl/system_integration/tb/tb_lidar_unified_csr_pkg.vhd"
)
$Project = Join-Path $Work "unified_csr_vhdl.prj"
$ProjectLines = foreach ($File in $Files) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $Project

$CompileLog = Join-Path $Work "xvhdl.log"
$ElabLog = Join-Path $Work "xelab.log"
$SimLog = Join-Path $Work "xsim.log"
$Snapshot = "tb_lidar_unified_csr_pkg_${Stamp}_snap"
$RunTcl = Join-Path $Work "run.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

Push-Location $Work
try {
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--relax", "-prj", $Project, "-log", $CompileLog
    )
    Invoke-Checked "$Vivado/xelab.bat" @(
        "--debug", "typical", "--relax", "--mt", "2",
        "-L", "xil_defaultlib", "-L", "unisim", "-L", "xpm",
        "--snapshot", $Snapshot,
        "xil_defaultlib.tb_lidar_unified_csr_pkg",
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
if ($SimText -notmatch "UNIFIED_CSR_CONTRACT_PASS") {
    throw "Unified CSR contract did not report PASS. See $SimLog"
}

Copy-Item -Force -LiteralPath `
    $CompileLog, $ElabLog, $SimLog, $Project, $RunTcl -Destination $Archive

Write-Output "UNIFIED_CSR_CONTRACT_PASS"
Write-Output "Result: $Archive"
