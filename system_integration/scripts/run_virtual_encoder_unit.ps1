param(
    [string]$SourceRoot = "C:/Projects/my_sp/lib/IP/virtual_encoder/HDL",
    [string]$MotorSourceRoot = "C:/Projects/my_sp/lib/IP/motor_decoder/HDL",
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [int]$RunTimeMs = 50,
    [ValidateSet(
        "enc_top_tb",
        "tb_motor_cfg_commit_atomic",
        "tb_enc_timing_generator",
        "tb_enc_startup_ab",
        "tb_enc_z_long_interval",
        "tb_enc_param_boundary",
        "tb_enc_apply_hold"
    )]
    [string]$Top = "enc_top_tb"
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$SourceRoot = (Resolve-Path -LiteralPath $SourceRoot).Path.Replace('\', '/')
$MotorSourceRoot = (Resolve-Path -LiteralPath $MotorSourceRoot).Path.Replace('\', '/')
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Work = Join-Path $Hdl "tmp/virtual_encoder/$Stamp"
$Archive = Join-Path $Hdl "sim_results/vivado_xsim/sessions/${Stamp}_${Top}"

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

$Files = switch ($Top) {
    "tb_enc_timing_generator" {
        "$SourceRoot/enc_pkg.vhd"
        "$SourceRoot/enc_timing_generator.vhd"
        "$SourceRoot/tb_enc_timing_generator.vhd"
    }
    { $_ -in @(
        "enc_top_tb",
        "tb_enc_startup_ab",
        "tb_enc_z_long_interval",
        "tb_enc_param_boundary",
        "tb_enc_apply_hold"
    ) } {
        "$SourceRoot/enc_pkg.vhd"
        "$SourceRoot/enc_param_apply_ctrl.vhd"
        "$SourceRoot/enc_phase_counter.vhd"
        "$SourceRoot/enc_position_tracker.vhd"
        "$SourceRoot/enc_index_pulse.vhd"
        "$SourceRoot/enc_timing_generator.vhd"
        "$SourceRoot/enc_top.vhd"
        if ($Top -eq "enc_top_tb") {
            "$SourceRoot/enc_top_tb.vhd"
        } else {
            "$SourceRoot/${Top}.vhd"
        }
    }
    "tb_motor_cfg_commit_atomic" {
        "$MotorSourceRoot/enc_pkg.vhd"
        "$MotorSourceRoot/motor_decoder_cfg_pkg.vhd"
        "$MotorSourceRoot/motor_cfg_commit_ctrl.vhd"
        "$Hdl/system_integration/tb/tb_motor_cfg_commit_atomic.vhd"
    }
}

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
$Snapshot = "${Top}_${Stamp}_snap"
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
        "xil_defaultlib.$Top",
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

$PassPattern = switch ($Top) {
    "enc_top_tb" { "Result\s*:\s*ALL PASS" }
    "tb_motor_cfg_commit_atomic" { "MOTOR_CFG_ATOMIC_PASS" }
    "tb_enc_timing_generator" { "ENC_TIMING_GENERATOR_PASS" }
    "tb_enc_startup_ab" { "ENC_STARTUP_AB_PASS" }
    "tb_enc_z_long_interval" { "ENC_Z_LONG_INTERVAL_PASS" }
    "tb_enc_param_boundary" { "ENC_PARAM_BOUNDARY_PASS" }
    "tb_enc_apply_hold" { "ENC_APPLY_HOLD_PASS" }
}

$SimText = Get-Content -Raw -LiteralPath $SimLog
if ($SimText -notmatch $PassPattern) {
    throw "$Top did not report its PASS marker. See $SimLog"
}

Copy-Item -Force -LiteralPath $CompileLog, $ElabLog, $SimLog -Destination $Archive
Copy-Item -Force -LiteralPath $Project, $RunTcl -Destination $Archive

Write-Output "${Top}_PASS"
Write-Output "Archive: $Archive"
