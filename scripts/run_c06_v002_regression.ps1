param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$RunProjectSyntax,
    [switch]$NoArchiveOnExit
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
    if (Select-String -Path $Log -Pattern "Failure:|Fatal:|assertion error|ERROR:|\bfailed\b" -CaseSensitive:$false -Quiet) {
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
    "$Hdl/tdc_gpx_atomic_snapshot_cdc.vhd",
    "$Hdl/tb_tdc_gpx_pkg.vhd",
    "$Hdl/tdc_gpx_bus_phy.vhd",
    "$Hdl/tdc_gpx_skid_buffer.vhd",
    "$Hdl/tdc_gpx_sync_fifo.vhd",
    "$Hdl/tdc_gpx_cell_builder.vhd",
    "$Hdl/tdc_gpx_cell_pipe.vhd",
    "$Hdl/tdc_gpx_chip_init.vhd",
    "$Hdl/tdc_gpx_chip_run.vhd",
    "$Hdl/tdc_gpx_chip_reg.vhd",
    "$Hdl/tdc_gpx_chip_ctrl.vhd",
    "$Hdl/tdc_gpx_csr_chip.vhd",
    "$Hdl/tdc_gpx_cmd_arb.vhd",
    "$Hdl/tdc_gpx_err_handler.vhd",
    "$Hdl/tdc_gpx_cfg_image_override.vhd",
    "$Hdl/tdc_gpx_reg_rsp_cdc.vhd",
    "$Hdl/tdc_gpx_config_ctrl.vhd",
    "$Hdl/tdc_gpx_decoder_i_mode.vhd",
    "$Hdl/tdc_gpx_raw_event_builder.vhd",
    "$Hdl/tdc_gpx_decode_pipe.vhd",
    "$Hdl/tdc_gpx_face_assembler.vhd",
    "$Hdl/tdc_gpx_line_packer.vhd",
    "$Hdl/tb_tdc_gpx_line_packer.vhd",
    "$Hdl/tb_tdc_gpx_line_packer_widths.vhd",
    "$Hdl/tdc_gpx_header_inserter.vhd",
    "$Hdl/tdc_gpx_face_seq.vhd",
    "$Hdl/tdc_gpx_output_stage.vhd",
    "$Hdl/tdc_gpx_csr_pipeline.vhd",
    "$Hdl/tdc_gpx_status_agg.vhd",
    "$Hdl/tdc_gpx_top.vhd",
    "$Hdl/tb_tdc_gpx_atomic_snapshot_cdc.vhd",
    "$Hdl/tb_tdc_gpx_range_ticks.vhd",
    "$Hdl/tb_tdc_gpx_cfg_image_override.vhd",
    "$Hdl/tb_tdc_gpx_config_ctrl.vhd",
    "$Hdl/tb_tdc_gpx_request_loss.vhd",
    "$Hdl/tb_tdc_gpx_chip_init_cfg_owner.vhd",
    "$Hdl/tb_tdc_gpx_chip_ctrl.vhd",
    "$Hdl/tb_tdc_gpx_top_int.vhd",
    "$Hdl/tb_tdc_gpx_top_int_slope_profiles.vhd",
    "$Hdl/tb_tdc_gpx_top_int_build_profile.vhd",
    "$Hdl/tb_tdc_gpx_face_seq.vhd",
    "$Hdl/tb_tdc_gpx_status_agg_c06_fix.vhd",
    "$Hdl/tb_tdc_gpx_cell_pipe_lane_mask.vhd",
    "$Hdl/tb_tdc_gpx_cell_pipe_lane_mask_dedicated.vhd",
    "$Hdl/tb_tdc_gpx_reg_rsp_cdc.vhd",
    "$Hdl/tb_tdc_gpx_csr_chip_image_cdc.vhd",
    "$Hdl/tb_tdc_gpx_top_int_masked_slope_stat.vhd"
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

    Invoke-Xelab "tb_c06_v002_atomic_snapshot_cdc_snap" "xelab_c06_v002_atomic_snapshot_cdc_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_atomic_snapshot_cdc"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_atomic_snapshot_cdc_snap", "-runall", "-log", "xsim_c06_v002_atomic_snapshot_cdc_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_atomic_snapshot_cdc_$Stamp.log" "ATOMIC_SNAPSHOT_CDC ASYNC/RETRIGGER/RESET PASS"

    Invoke-Xelab "tb_c06_v002_range_ticks_snap" "xelab_c06_v002_range_ticks_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_range_ticks"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_range_ticks_snap", "-runall", "-log", "xsim_c06_v002_range_ticks_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_range_ticks_$Stamp.log" "5 ns range tick conversion matrix - PASS"

    Invoke-Xelab "tb_c06_v002_cfg_image_override_snap" "xelab_c06_v002_cfg_image_override_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_cfg_image_override"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_cfg_image_override_snap", "-runall", "-log", "xsim_c06_v002_cfg_image_override_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_cfg_image_override_$Stamp.log" "tb_tdc_gpx_cfg_image_override: ALL TESTS PASSED"

    Invoke-Xelab "tb_c06_v002_config_ctrl_snap" "xelab_c06_v002_config_ctrl_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_config_ctrl"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_config_ctrl_snap", "-runall", "-log", "xsim_c06_v002_config_ctrl_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_config_ctrl_$Stamp.log" "EF-authoritative config_ctrl integration completed"

    Invoke-Xelab "tb_c06_v002_request_loss_snap" "xelab_c06_v002_request_loss_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_request_loss"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_request_loss_snap", "-runall", "-log", "xsim_c06_v002_request_loss_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_request_loss_$Stamp.log" "REQUEST_LOSS_DIAGNOSTIC_COLLAPSE PASS"

    Invoke-Xelab "tb_c06_v002_chip_init_cfg_owner_snap" "xelab_c06_v002_chip_init_cfg_owner_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_chip_init_cfg_owner"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_chip_init_cfg_owner_snap", "-runall", "-log", "xsim_c06_v002_chip_init_cfg_owner_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_chip_init_cfg_owner_$Stamp.log" "CHIP_INIT_COORDINATOR_CFG_OWNER PASS"

    $linePackerCases = @(
        @{ Name = "w32_mh7";   Top = "xil_defaultlib.tb_tdc_gpx_line_packer_w32" },
        @{ Name = "w64_mh7";   Top = "xil_defaultlib.tb_tdc_gpx_line_packer_w64" },
        @{ Name = "w128_mh7";  Top = "xil_defaultlib.tb_tdc_gpx_line_packer_w128" },
        @{ Name = "w32_mh3";   Top = "xil_defaultlib.tb_tdc_gpx_line_packer_w32_mh3" },
        @{ Name = "w64_mh3";   Top = "xil_defaultlib.tb_tdc_gpx_line_packer_w64_mh3" },
        @{ Name = "w128_mh3";  Top = "xil_defaultlib.tb_tdc_gpx_line_packer_w128_mh3" }
    )
    foreach ($lp in $linePackerCases) {
        $snap = "tb_c06_v002_line_packer_$($lp.Name)_snap"
        $xelabLog = "xelab_c06_v002_line_packer_$($lp.Name)_$Stamp.log"
        $xsimLog = "xsim_c06_v002_line_packer_$($lp.Name)_$Stamp.log"
        Invoke-Xelab $snap $xelabLog $lp.Top
        Invoke-Checked "$Vivado/xsim.bat" @($snap, "-runall", "-log", $xsimLog)
        Assert-SimLog $xsimLog "PASS: line_packer width="
    }

    Invoke-Xelab "tb_c06_v002_face_seq_snap" "xelab_c06_v002_face_seq_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_face_seq"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_face_seq_snap", "-runall", "-log", "xsim_c06_v002_face_seq_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_face_seq_$Stamp.log" "ALL SCENARIOS PASSED"

    Invoke-Xelab "tb_c06_v002_status_agg_snap" "xelab_c06_v002_status_agg_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_status_agg_c06_fix"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_status_agg_snap", "-runall", "-log", "xsim_c06_v002_status_agg_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_status_agg_$Stamp.log" "ALL STATUS_AGG C06 SCENARIOS PASSED"

    Invoke-Xelab "tb_c06_v002_lane_mask_snap" "xelab_c06_v002_lane_mask_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_cell_pipe_lane_mask"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_lane_mask_snap", "-runall", "-log", "xsim_c06_v002_lane_mask_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_lane_mask_$Stamp.log" "TB_LANE_MASK ALL PASS"

    Invoke-Xelab "tb_c06_v002_lane_mask_dedicated_snap" "xelab_c06_v002_lane_mask_dedicated_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_cell_pipe_lane_mask_dedicated"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_lane_mask_dedicated_snap", "-runall", "-log", "xsim_c06_v002_lane_mask_dedicated_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_lane_mask_dedicated_$Stamp.log" "TB_LANE_MASK ALL PASS"

    Invoke-Xelab "tb_c06_v002_reg_rsp_cdc_snap" "xelab_c06_v002_reg_rsp_cdc_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_reg_rsp_cdc"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_reg_rsp_cdc_snap", "-runall", "-log", "xsim_c06_v002_reg_rsp_cdc_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_reg_rsp_cdc_$Stamp.log" "REG_RSP_CDC 200-to-50 MHz atomic transfer - PASS"

    Invoke-Xelab "tb_c06_v002_csr_image_cdc_snap" "xelab_c06_v002_csr_image_cdc_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_csr_chip_image_cdc"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_csr_image_cdc_snap", "-runall", "-log", "xsim_c06_v002_csr_image_cdc_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_csr_image_cdc_$Stamp.log" "CFG_IMAGE_PACKED_CDC ALL PASS"

    Invoke-Xelab "tb_c06_v002_chip_ctrl_snap" "xelab_c06_v002_chip_ctrl_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_chip_ctrl"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_chip_ctrl_snap", "-runall", "-log", "xsim_c06_v002_chip_ctrl_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_chip_ctrl_$Stamp.log" "ALL TESTS PASSED"

    foreach ($w in @(32, 64, 128)) {
        $snap = "tb_c06_v002_top_int_w${w}_snap"
        Invoke-Xelab $snap "xelab_c06_v002_top_int_w${w}_$Stamp.log" `
            "xil_defaultlib.tb_tdc_gpx_top_int" @("G_TDATA_WIDTH=$w")
        Invoke-Checked "$Vivado/xsim.bat" @($snap, "-runall", "-log", "xsim_c06_v002_top_int_w${w}_$Stamp.log")
        Assert-SimLog "xsim_c06_v002_top_int_w${w}_$Stamp.log" "output streams emitted beats/tlast as expected - PASS"
    }

    $slopeProfileCases = @(
        @{ Name = "runtime_rise_only"; Top = "xil_defaultlib.tb_tdc_gpx_top_int_runtime_rise_only" },
        @{ Name = "rise_only_build_32ch"; Top = "xil_defaultlib.tb_tdc_gpx_top_int_rise_only_build_32ch" },
        @{ Name = "three_chip_2r1f"; Top = "xil_defaultlib.tb_tdc_gpx_top_int_three_chip_2r1f" },
        @{ Name = "one_chip_dual_edge"; Top = "xil_defaultlib.tb_tdc_gpx_top_int_one_chip_dual_edge" }
    )
    foreach ($case in $slopeProfileCases) {
        $snap = "tb_c06_v002_$($case.Name)_snap"
        $xelabLog = "xelab_c06_v002_$($case.Name)_$Stamp.log"
        $xsimLog = "xsim_c06_v002_$($case.Name)_$Stamp.log"
        Invoke-Xelab $snap $xelabLog $case.Top
        Invoke-Checked "$Vivado/xsim.bat" @($snap, "-runall", "-log", $xsimLog)
        Assert-SimLog $xsimLog "output streams emitted beats/tlast as expected - PASS"
    }

    Invoke-Xelab "tb_c06_v002_build_profile_snap" "xelab_c06_v002_build_profile_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_top_int_build_profile"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_build_profile_snap", "-runall", "-log", "xsim_c06_v002_build_profile_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_build_profile_$Stamp.log" "output streams emitted beats/tlast as expected - PASS"

    Invoke-Xelab "tb_c06_v002_build_profile_zero_alias_snap" "xelab_c06_v002_build_profile_zero_alias_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_top_int_build_profile_zero_alias"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_build_profile_zero_alias_snap", "-runall", "-log", "xsim_c06_v002_build_profile_zero_alias_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_build_profile_zero_alias_$Stamp.log" "output streams emitted beats/tlast as expected - PASS"

    $dualSnap = "tb_c06_v002_top_int_axis150_tdc200_snap"
    Invoke-Xelab $dualSnap "xelab_c06_v002_top_int_axis150_tdc200_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_top_int" @(
            "G_TDATA_WIDTH=64",
            "G_AXIS_CLK_MHZ=150.0",
            "G_TDC_CLK_MHZ=200.0"
        )
    Invoke-Checked "$Vivado/xsim.bat" @($dualSnap, "-runall", "-log", "xsim_c06_v002_top_int_axis150_tdc200_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_top_int_axis150_tdc200_$Stamp.log" "output streams emitted beats/tlast as expected - PASS"

    $bpSnap = "tb_c06_v002_top_int_w64_bp_snap"
    Invoke-Xelab $bpSnap "xelab_c06_v002_top_int_w64_bp_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_top_int" @("G_TDATA_WIDTH=64", "G_BP_TREADY_GAP=17")
    Invoke-Checked "$Vivado/xsim.bat" @($bpSnap, "-runall", "-log", "xsim_c06_v002_top_int_w64_bp_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_top_int_w64_bp_$Stamp.log" "bounded output backpressure preserved beats/tlast - PASS"

    Invoke-Xelab "tb_c06_v002_masked_slope_stat_snap" "xelab_c06_v002_masked_slope_stat_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_top_int_masked_slope_stat"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c06_v002_masked_slope_stat_snap", "-runall", "-log", "xsim_c06_v002_masked_slope_stat_$Stamp.log")
    Assert-SimLog "xsim_c06_v002_masked_slope_stat_$Stamp.log" "masked-slope sticky soft-clear lifecycle - PASS"
}
finally {
    Pop-Location
    if (-not $NoArchiveOnExit) {
        & "$Hdl/scripts/archive_vivado_xsim_outputs.ps1" `
            -Root $Hdl `
            -Stamp $Stamp `
            -Label "c06_v002_regression"
    }
}
