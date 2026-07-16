# =============================================================================
# run_p0_shot_stall_top_int.ps1
# CHAIN-P0-01 integration-level closure: shot-boundary output stall.
#
# Runs the c06 v002 regression as compile prerequisite (also re-validates
# tb_tdc_gpx_top_int default behavior), then runs tb_tdc_gpx_top_int with
# G_BP_SHOT_STALL_CLKS=40 at 64- and 32-bit output widths: both VDMA lanes'
# tready are held low across the mid-face shot boundary, and the face must
# still complete with the expected beats/tlast.
#
# Usage:
#   powershell -NoProfile -ExecutionPolicy Bypass -File scripts\run_p0_shot_stall_top_int.ps1
# =============================================================================
param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Root = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl"
$Hdl  = "$Root/HDL"
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Tmp  = "$Hdl/tmp/p0_shot_stall"

New-Item -ItemType Directory -Force -Path $Tmp | Out-Null

function Invoke-Checked {
    param([string]$Exe, [string[]]$ArgList)
    & $Exe @ArgList
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Exe $($ArgList -join ' ')"
    }
}

function Assert-SimLog {
    param([string]$Log, [string[]]$PassPatterns)
    if (Select-String -Path $Log -Pattern "Failure:|Fatal:|assertion error|Error:|\bfailed\b" -CaseSensitive:$false -Quiet) {
        throw "Simulation log contains failure/error marker: $Log"
    }
    foreach ($p in $PassPatterns) {
        if (-not (Select-String -Path $Log -Pattern $p -SimpleMatch -Quiet)) {
            throw "Simulation log missing PASS marker '$p': $Log"
        }
    }
}

Push-Location $Hdl
try {
    & "$Hdl/scripts/run_c06_v002_regression.ps1" -Stamp $Stamp -NoArchiveOnExit
    if ($LASTEXITCODE -ne 0) {
        throw "c06 prerequisite compile/regression failed"
    }

    foreach ($w in @(64, 32)) {
        $snap = "tb_p0_shot_stall_w${w}_snap"
        $elog = "xelab_p0_shot_stall_w${w}_$Stamp.log"
        $slog = "xsim_p0_shot_stall_w${w}_$Stamp.log"

        $argFile = Join-Path $Tmp "$snap.f"
        @(
            "--debug typical", "--relax", "--mt 2",
            "-L xil_defaultlib", "-L unisims_ver", "-L unimacro_ver",
            "-L secureip", "-L xpm",
            "--snapshot $snap",
            "--generic_top `"G_TDATA_WIDTH=$w`"",
            "--generic_top `"G_BP_SHOT_STALL_CLKS=40`"",
            "xil_defaultlib.tb_tdc_gpx_top_int",
            "xil_defaultlib.glbl",
            "-log $elog"
        ) | Set-Content -Encoding ASCII $argFile
        Invoke-Checked "$Vivado/xelab.bat" @("-f", $argFile)

        Invoke-Checked "$Vivado/xsim.bat" @($snap, "-runall", "-log", $slog)
        Assert-SimLog -Log $slog -PassPatterns @(
            "shot-boundary stall preserved beats/tlast - PASS",
            "output streams emitted beats/tlast as expected - PASS"
        )
        Write-Host "PASS: top_int shot-boundary stall w$w"
    }

    Write-Host "=== p0 shot-boundary stall (top_int): ALL PASS (stamp $Stamp) ==="
}
finally {
    & "$Hdl/scripts/archive_vivado_xsim_outputs.ps1" -Stamp $Stamp -Label "p0_shot_stall_top_int"
    Pop-Location
}
