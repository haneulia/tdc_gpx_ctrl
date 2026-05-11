param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$RunProjectSyntax
)

$ErrorActionPreference = "Stop"

$Root = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl"
$Hdl = "$Root/HDL"
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Tmp = "$Hdl/tmp/c06_v002"

New-Item -ItemType Directory -Force -Path $Tmp | Out-Null

$vhdlPrj = Join-Path $Tmp "c06_v002_vhdl.prj"
$vlogPrj = Join-Path $Tmp "c06_v002_vlog.prj"

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

function Assert-SimLog {
    param(
        [string]$Log,
        [string]$PassPattern
    )
    if (Select-String -Path $Log -Pattern "Failure:|ERROR:|\bfailed\b" -CaseSensitive:$false -Quiet) {
        throw "Simulation log contains failure/error marker: $Log"
    }
    if (-not (Select-String -Path $Log -Pattern $PassPattern -Quiet)) {
        throw "Simulation log does not contain expected PASS marker '$PassPattern': $Log"
    }
}

function Invoke-Xelab {
    param(
        [string]$Snapshot,
        [string]$Log,
        [string]$Top,
        [string[]]$Generics = @()
    )
    $argFile = Join-Path $Tmp "$Snapshot.f"
    $lines = @(
        "--debug typical",
        "--relax",
        "--mt 2",
        "-L xil_defaultlib",
        "-L unisims_ver",
        "-L unimacro_ver",
        "-L secureip",
        "-L xpm",
        "--snapshot $Snapshot"
    )
    foreach ($g in $Generics) {
        $lines += "--generic_top `"$g`""
    }
    $lines += $Top
    $lines += "xil_defaultlib.glbl"
    $lines += "-log $Log"
    $lines | Set-Content -Encoding ASCII $argFile
    Invoke-Checked "$Vivado/xelab.bat" @("-f", $argFile)
}

$ipFiles = @(
    "$Root/tdc_gpx_ctrl.gen/sources_1/ip/tdc_gpx_axil_csr32_chip/src/axil_ctrl_regs_32.vhd",
    "$Root/tdc_gpx_ctrl.gen/sources_1/ip/tdc_gpx_axil_csr32_chip/src/axil_fsm_32.vhd",
    "$Root/tdc_gpx_ctrl.gen/sources_1/ip/tdc_gpx_axil_csr32_chip/src/axil_intr_32.vhd",
    "$Root/tdc_gpx_ctrl.gen/sources_1/ip/tdc_gpx_axil_csr32_chip/src/axil_stat_regs_32.vhd",
    "$Root/tdc_gpx_ctrl.gen/sources_1/ip/tdc_gpx_axil_csr32_chip/src/my_axil_csr32_top.vhd",
    "$Root/tdc_gpx_ctrl.gen/sources_1/ip/tdc_gpx_axil_csr32_chip/sim/tdc_gpx_axil_csr32_chip.vhd",
    "$Root/tdc_gpx_ctrl.gen/sources_1/ip/tdc_gpx_axil_csr_pipeline/src/axil_ctrl_regs.vhd",
    "$Root/tdc_gpx_ctrl.gen/sources_1/ip/tdc_gpx_axil_csr_pipeline/src/axil_fsm.vhd",
    "$Root/tdc_gpx_ctrl.gen/sources_1/ip/tdc_gpx_axil_csr_pipeline/src/axil_intr.vhd",
    "$Root/tdc_gpx_ctrl.gen/sources_1/ip/tdc_gpx_axil_csr_pipeline/src/axil_stat_regs.vhd",
    "$Root/tdc_gpx_ctrl.gen/sources_1/ip/tdc_gpx_axil_csr_pipeline/src/my_axil_csr_top.vhd",
    "$Root/tdc_gpx_ctrl.gen/sources_1/ip/tdc_gpx_axil_csr_pipeline/sim/tdc_gpx_axil_csr_pipeline.vhd"
)

$vhdl2008Files = @(
    "$Hdl/px_utility_pkg.vhd",
    "$Hdl/tdc_gpx_pkg.vhd",
    "$Hdl/tdc_gpx_cfg_pkg.vhd",
    "$Hdl/tdc_gpx_bus_phy.vhd",
    "$Hdl/tdc_gpx_cell_builder.vhd",
    "$Hdl/tdc_gpx_cell_pipe.vhd",
    "$Hdl/tdc_gpx_chip_init.vhd",
    "$Hdl/tdc_gpx_chip_run.vhd",
    "$Hdl/tdc_gpx_chip_reg.vhd",
    "$Hdl/tdc_gpx_skid_buffer.vhd",
    "$Hdl/tdc_gpx_sync_fifo.vhd",
    "$Hdl/tdc_gpx_chip_ctrl.vhd",
    "$Hdl/tdc_gpx_csr_chip.vhd",
    "$Hdl/tdc_gpx_cmd_arb.vhd",
    "$Hdl/tdc_gpx_err_handler.vhd",
    "$Hdl/tdc_gpx_stop_cfg_decode.vhd",
    "$Hdl/tdc_gpx_config_ctrl.vhd",
    "$Hdl/tdc_gpx_decoder_i_mode.vhd",
    "$Hdl/tdc_gpx_raw_event_builder.vhd",
    "$Hdl/tdc_gpx_decode_pipe.vhd",
    "$Hdl/tdc_gpx_face_assembler.vhd",
    "$Hdl/tdc_gpx_header_inserter.vhd",
    "$Hdl/tdc_gpx_face_seq.vhd",
    "$Hdl/tdc_gpx_output_stage.vhd",
    "$Hdl/tdc_gpx_csr_pipeline.vhd",
    "$Hdl/tdc_gpx_status_agg.vhd",
    "$Hdl/tdc_gpx_top.vhd",
    "$Hdl/tb_tdc_gpx_top_int.vhd",
    "$Hdl/tb_tdc_gpx_face_seq.vhd",
    "$Hdl/tb_tdc_gpx_status_agg_c06_fix.vhd"
)

$vhdlLines = @()
foreach ($f in $ipFiles) { $vhdlLines += "vhdl xil_defaultlib `"$f`"" }
foreach ($f in $vhdl2008Files) { $vhdlLines += "vhdl2008 xil_defaultlib `"$f`"" }
$vhdlLines += "nosort"
$vhdlLines | Set-Content -Encoding ASCII $vhdlPrj

@(
    "verilog xil_defaultlib `"$Root/tdc_gpx_ctrl.sim/sim_1/behav/xsim/glbl.v`"",
    "nosort"
) | Set-Content -Encoding ASCII $vlogPrj

Push-Location $Hdl
try {
    if ($RunProjectSyntax) {
        Invoke-Checked "$Vivado/vivado.bat" @(
            "-mode", "batch",
            "-source", "$Hdl/scripts/check_face_seq_syntax.tcl",
            "-log", "$Hdl/vivado_c06_v002_check_syntax_$Stamp.log",
            "-journal", "$Hdl/vivado_c06_v002_check_syntax_$Stamp.jou"
        )
    }
    Invoke-Checked "$Vivado/xvlog.bat" @("--relax", "-prj", $vlogPrj, "-log", "xvlog_c06_v002_$Stamp.log")
    Invoke-Checked "$Vivado/xvhdl.bat" @("--relax", "-prj", $vhdlPrj, "-log", "xvhdl_c06_v002_$Stamp.log")

    Invoke-Xelab "tb_c06_v002_face_seq_snap" "xelab_c06_v002_face_seq_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_face_seq"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_face_seq_snap", "-runall", "-log", "xsim_c06_v002_face_seq_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_face_seq_$Stamp.log" "ALL SCENARIOS PASSED"

    Invoke-Xelab "tb_c06_v002_status_agg_snap" "xelab_c06_v002_status_agg_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_status_agg_c06_fix"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_status_agg_snap", "-runall", "-log", "xsim_c06_v002_status_agg_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_status_agg_$Stamp.log" "ALL STATUS_AGG C06 SCENARIOS PASSED"

    foreach ($w in @(32, 64, 128)) {
        $snap = "tb_c06_v002_top_int_w${w}_snap"
        Invoke-Xelab $snap "xelab_c06_v002_top_int_w${w}_$Stamp.log" `
            "xil_defaultlib.tb_tdc_gpx_top_int" @("G_TDATA_WIDTH=$w")
        Invoke-Checked "$Vivado/xsim.bat" @($snap, "-runall", "-log", "xsim_c06_v002_top_int_w${w}_$Stamp.log")
        Assert-SimLog "xsim_c06_v002_top_int_w${w}_$Stamp.log" "output streams emitted beats/tlast as expected - PASS"
    }

    $bpSnap = "tb_c06_v002_top_int_w64_bp_snap"
    Invoke-Xelab $bpSnap "xelab_c06_v002_top_int_w64_bp_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_top_int" @("G_TDATA_WIDTH=64", "G_BP_TREADY_GAP=17")
    Invoke-Checked "$Vivado/xsim.bat" @($bpSnap, "-runall", "-log", "xsim_c06_v002_top_int_w64_bp_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_top_int_w64_bp_$Stamp.log" "bounded output backpressure preserved beats/tlast - PASS"
}
finally {
    Pop-Location
}
