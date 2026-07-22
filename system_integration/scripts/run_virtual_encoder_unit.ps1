param(
    [string]$SourceRoot = "C:/Projects/my_sp/lib/IP/motor_decoder/HDL",
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [int]$RunTimeMs = 50
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$SourceRoot = (Resolve-Path -LiteralPath $SourceRoot).Path.Replace('\', '/')
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Work = Join-Path $Hdl "tmp/virtual_encoder/$Stamp"
$Archive = Join-Path $Hdl "sim_results/vivado_xsim/sessions/${Stamp}_virtual_encoder_unit"

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
    "$SourceRoot/enc_pkg.vhd",
    "$SourceRoot/enc_param_apply_ctrl.vhd",
    "$SourceRoot/enc_fractional_scheduler.vhd",
    "$SourceRoot/enc_tick_counter.vhd",
    "$SourceRoot/enc_phase_counter.vhd",
    "$SourceRoot/enc_position_counter.vhd",
    "$SourceRoot/enc_top.vhd",
    "$SourceRoot/enc_top_tb.vhd"
)

foreach ($File in $Files) {
    if (-not (Test-Path -LiteralPath $File -PathType Leaf)) {
        throw "Missing virtual encoder source: $File"
    }
}

$Project = Join-Path $Work "virtual_encoder_vhdl.prj"
$ProjectLines = foreach ($File in $Files) {
    "vhdl2008 xil_defaultlib `"$File`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $Project

$CompileLog = Join-Path $Work "xvhdl.log"
$ElabLog = Join-Path $Work "xelab.log"
$SimLog = Join-Path $Work "xsim.log"
$Snapshot = "virtual_encoder_${Stamp}_snap"
$RunTcl = Join-Path $Work "run.tcl"
$RunTclXsim = $RunTcl.Replace('\', '/')
$SimLogXsim = $SimLog.Replace('\', '/')

@(
    "run ${RunTimeMs} ms",
    "quit"
) | Set-Content -Encoding ASCII -LiteralPath $RunTcl

Push-Location $Work
try {
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--relax", "-prj", $Project, "-log", $CompileLog
    )
    Invoke-Checked "$Vivado/xelab.bat" @(
        "--debug", "typical",
        "--relax",
        "--mt", "2",
        "-L", "xil_defaultlib",
        "-L", "unisim",
        "-L", "xpm",
        "--snapshot", $Snapshot,
        "xil_defaultlib.enc_top_tb",
        "-log", $ElabLog
    )
    Invoke-Checked "$Vivado/xsim.bat" @(
        $Snapshot,
        "-tclbatch", $RunTclXsim,
        "-log", $SimLogXsim
    )
}
finally {
    Pop-Location
}

$SimText = Get-Content -Raw -LiteralPath $SimLog
if ($SimText -notmatch "Result\s*:\s*ALL PASS") {
    throw "Virtual encoder regression did not report ALL PASS. See $SimLog"
}

Copy-Item -Force -LiteralPath $CompileLog, $ElabLog, $SimLog -Destination $Archive
Copy-Item -Force -LiteralPath $Project, $RunTcl -Destination $Archive

Write-Output "VIRTUAL_ENCODER_UNIT_PASS"
Write-Output "Archive: $Archive"
