param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$Csr32 = (Resolve-Path (Join-Path $Hdl "../../my_axil_csr32/HDL")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Work = Join-Path $Hdl "tmp/unified_csr_top/$Stamp"
$Archive = Join-Path $Hdl `
    "sim_results/vivado_xsim/sessions/${Stamp}_unified_csr_top"

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
    "$Csr32/my_axil_csr32_pkg.vhd",
    "$Csr32/axil_fsm_32.vhd",
    "$Csr32/axil_ctrl_regs_32.vhd",
    "$Csr32/axil_stat_regs_32.vhd",
    "$Csr32/axil_intr_32.vhd",
    "$Csr32/my_axil_csr32_top.vhd",
    "$Hdl/system_integration/rtl/lidar_unified_csr_pkg.vhd",
    "$Hdl/system_integration/rtl/lidar_unified_csr_top.vhd",
    "$Hdl/system_integration/tb/tb_lidar_unified_csr_top.vhd"
)
$Project = Join-Path $Work "unified_csr_top_vhdl.prj"
$ProjectLines = foreach ($File in $Files) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $Project

$CompileLog = Join-Path $Work "xvhdl.log"
$ElabLog = Join-Path $Work "xelab.log"
$SimLog = Join-Path $Work "xsim.log"
$Snapshot = "tb_lidar_unified_csr_top_${Stamp}_snap"
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
        "xil_defaultlib.tb_lidar_unified_csr_top",
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
if ($SimText -notmatch "LIDAR_UNIFIED_CSR_TOP_PASS") {
    throw "Unified CSR top did not report PASS. See $SimLog"
}

Copy-Item -Force -LiteralPath `
    $CompileLog, $ElabLog, $SimLog, $Project, $RunTcl -Destination $Archive

Write-Output "LIDAR_UNIFIED_CSR_TOP_PASS"
Write-Output "Result: $Archive"
