# =============================================================================
# run_shot_bp_regression.ps1
# CHAIN-P0-01 shot-boundary FIFO flush guard regression.
#
# Runs:
#   1. tb_tdc_gpx_output_stage_shot_bp  (scenario A sanity + scenario B
#      shot-boundary backpressure; scenario B fails on pre-fix RTL)
#   2. tb_tdc_gpx_output_stage           (64-bit scenarios 1-2)
#   3. width wrappers w32 / w128
#   4. max_hits sweep wrappers w32 / w64 / w128
#
# Usage:
#   powershell -NoProfile -ExecutionPolicy Bypass -File scripts\run_shot_bp_regression.ps1
# =============================================================================
param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$NoArchiveOnExit
)

$ErrorActionPreference = "Stop"

$Root = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl"
$Hdl  = "$Root/HDL"
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Tmp  = "$Hdl/tmp/shot_bp_$Stamp"

New-Item -ItemType Directory -Force -Path $Tmp | Out-Null

function Invoke-Checked {
    param([string]$Exe, [string[]]$ArgList)
    & $Exe @ArgList
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Exe $($ArgList -join ' ')"
    }
}

function Assert-SimLog {
    param([string]$Log, [string]$PassPattern)
    if (Select-String -Path $Log -Pattern "Failure:|Fatal:|assertion error|ERROR:|\bFAIL\b" -CaseSensitive:$false -Quiet) {
        throw "Simulation log contains failure marker: $Log"
    }
    if (-not (Select-String -Path $Log -Pattern $PassPattern -SimpleMatch -Quiet)) {
        throw "Simulation log missing PASS marker '$PassPattern': $Log"
    }
}

Push-Location $Hdl
try {
    # -------------------------------------------------------------------------
    # Compile
    # -------------------------------------------------------------------------
    $vlogPrj = Join-Path $Tmp "shot_bp_vlog.prj"
    @(
        "verilog xil_defaultlib `"$Root/tdc_gpx_ctrl.sim/sim_1/behav/xsim/glbl.v`"",
        "nosort"
    ) | Set-Content -Encoding ASCII $vlogPrj

    $vhdlPrj = Join-Path $Tmp "shot_bp_vhdl.prj"
    @(
        "vhdl2008 xil_defaultlib `"$Hdl/tdc_gpx_pkg.vhd`"",
        "vhdl2008 xil_defaultlib `"$Hdl/tdc_gpx_cfg_pkg.vhd`"",
        "vhdl2008 xil_defaultlib `"$Hdl/tdc_gpx_skid_buffer.vhd`"",
        "vhdl2008 xil_defaultlib `"$Hdl/tdc_gpx_sync_fifo.vhd`"",
        "vhdl2008 xil_defaultlib `"$Hdl/tdc_gpx_line_packer.vhd`"",
        "vhdl2008 xil_defaultlib `"$Hdl/tdc_gpx_header_inserter.vhd`"",
        "vhdl2008 xil_defaultlib `"$Hdl/tdc_gpx_face_assembler.vhd`"",
        "vhdl2008 xil_defaultlib `"$Hdl/tdc_gpx_output_stage.vhd`"",
        "vhdl2008 xil_defaultlib `"$Hdl/tb_tdc_gpx_output_stage_shot_bp.vhd`"",
        "vhdl2008 xil_defaultlib `"$Hdl/tb_tdc_gpx_output_stage.vhd`"",
        "vhdl2008 xil_defaultlib `"$Hdl/tb_tdc_gpx_output_stage_w32.vhd`"",
        "vhdl2008 xil_defaultlib `"$Hdl/tb_tdc_gpx_output_stage_w128.vhd`"",
        "vhdl2008 xil_defaultlib `"$Hdl/tb_tdc_gpx_output_stage_maxhits_w32.vhd`"",
        "vhdl2008 xil_defaultlib `"$Hdl/tb_tdc_gpx_output_stage_maxhits_w64.vhd`"",
        "vhdl2008 xil_defaultlib `"$Hdl/tb_tdc_gpx_output_stage_maxhits_w128.vhd`"",
        "nosort"
    ) | Set-Content -Encoding ASCII $vhdlPrj

    Invoke-Checked "$Vivado/xvlog.bat" @("--relax", "-prj", $vlogPrj, "-log", "xvlog_shot_bp_$Stamp.log")
    Invoke-Checked "$Vivado/xvhdl.bat" @("--relax", "-prj", $vhdlPrj, "-log", "xvhdl_shot_bp_$Stamp.log")

    # -------------------------------------------------------------------------
    # Elaborate + simulate (serialized)
    # -------------------------------------------------------------------------
    $runs = @(
        @{ top = "tb_tdc_gpx_output_stage_shot_bp";      snap = "shot_bp_snap_$Stamp";  pass = "*** TB_SHOT_BP ALL PASS ***" },
        @{ top = "tb_tdc_gpx_output_stage";              snap = "regr64_snap_$Stamp";   pass = "*** SCENARIO 2 (slope-independent abort) PASS ***" },
        @{ top = "tb_tdc_gpx_output_stage_w32";          snap = "regr32_snap_$Stamp";   pass = "*** SCENARIO 2 (slope-independent abort) PASS ***" },
        @{ top = "tb_tdc_gpx_output_stage_w128";         snap = "regr128_snap_$Stamp";  pass = "*** SCENARIO 2 (slope-independent abort) PASS ***" },
        @{ top = "tb_tdc_gpx_output_stage_maxhits_w32";  snap = "mh32_snap_$Stamp";     pass = "*** SCENARIO 3 (max_hits_cfg sweep 0..7) PASS ***" },
        @{ top = "tb_tdc_gpx_output_stage_maxhits_w64";  snap = "mh64_snap_$Stamp";     pass = "*** SCENARIO 3 (max_hits_cfg sweep 0..7) PASS ***" },
        @{ top = "tb_tdc_gpx_output_stage_maxhits_w128"; snap = "mh128_snap_$Stamp";    pass = "*** SCENARIO 3 (max_hits_cfg sweep 0..7) PASS ***" }
    )

    foreach ($r in $runs) {
        $argFile = Join-Path $Tmp "$($r.snap).f"
        @(
            "--debug typical", "--relax", "--mt 2",
            "-L xil_defaultlib", "-L unisims_ver", "-L unimacro_ver",
            "-L secureip", "-L xpm",
            "--snapshot $($r.snap)",
            "xil_defaultlib.$($r.top)",
            "xil_defaultlib.glbl",
            "-log xelab_$($r.snap).log"
        ) | Set-Content -Encoding ASCII $argFile
        Invoke-Checked "$Vivado/xelab.bat" @("-f", $argFile)

        $simLog = "xsim_$($r.snap).log"
        Invoke-Checked "$Vivado/xsim.bat" @($r.snap, "-runall", "-log", $simLog)
        Assert-SimLog -Log $simLog -PassPattern $r.pass
        Write-Host "PASS: $($r.top)"
    }

    Write-Host "=== shot_bp regression: ALL PASS (stamp $Stamp) ==="
}
finally {
    if (-not $NoArchiveOnExit) {
        & "$Hdl/scripts/archive_vivado_xsim_outputs.ps1" -Stamp $Stamp -Label "shot_bp_regression"
    }
    Pop-Location
}
