# =============================================================================
# recreate_project.tcl
#   삭제된 tdc_gpx_ctrl.xpr 을 HDL + IP repo 로부터 재생성한다.
#   목적: 통합 시뮬레이션 (tb_tdc_gpx_top_int) 실행 가능한 최소 xpr.
#
# 재생성 범위
#   · Part   : xc7z020clg484-2 (Zynq-7020) — 기존 sibling 프로젝트와 동일
#   · 소스   : HDL/tdc_gpx_*.vhd (RTL + TB) + HDL/px_utility_pkg.vhd
#   · IP     : tdc_gpx_axil_csr_pipeline (7-bit, 8 CTL / 8 STAT / 1 IRQ)
#              tdc_gpx_axil_csr32_chip  (9-bit, 32 CTL / 32 STAT / 1 IRQ)
#   · IP repo: C:/Projects/my_sp/lib/IP/my_axil_csr/ip_repo
#              C:/Projects/my_sp/lib/IP/my_axil_csr32/ip_repo
#   · Top    : tdc_gpx_top (synth), tb_tdc_gpx_top_int (sim)
#
# 주의: close_project 시 -delete 사용 금지.
# =============================================================================

set prj_dir  "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl"
set prj_name "tdc_gpx_ctrl"
set part     "xc7z020clg484-2"
set hdl_dir  "$prj_dir/HDL"
set ip_repo_axil   "C:/Projects/my_sp/lib/IP/my_axil_csr/ip_repo"
set ip_repo_axil32 "C:/Projects/my_sp/lib/IP/my_axil_csr32/ip_repo"

# -----------------------------------------------------------------------------
# Create project
# -----------------------------------------------------------------------------
create_project $prj_name $prj_dir -part $part -force
set_property target_language VHDL [current_project]
set_property simulator_language Mixed [current_project]
set_property default_lib xil_defaultlib [current_project]

# -----------------------------------------------------------------------------
# IP repository
# -----------------------------------------------------------------------------
set_property ip_repo_paths [list $ip_repo_axil $ip_repo_axil32] [current_project]
update_ip_catalog -rebuild

# -----------------------------------------------------------------------------
# RTL + TB sources (tdc_gpx_ctrl 단독 구성)
# -----------------------------------------------------------------------------
set rtl_files {
    tdc_gpx_pkg.vhd
    tdc_gpx_cfg_pkg.vhd
    px_utility_pkg.vhd
    tb_tdc_gpx_pkg.vhd
    tdc_gpx_skid_buffer.vhd
    tdc_gpx_bus_phy.vhd
    tdc_gpx_chip_init.vhd
    tdc_gpx_chip_run.vhd
    tdc_gpx_chip_reg.vhd
    tdc_gpx_chip_ctrl.vhd
    tdc_gpx_err_handler.vhd
    tdc_gpx_cmd_arb.vhd
    tdc_gpx_csr_chip.vhd
    tdc_gpx_csr_pipeline.vhd
    tdc_gpx_stop_cfg_decode.vhd
    tdc_gpx_config_ctrl.vhd
    tdc_gpx_decoder_i_mode.vhd
    tdc_gpx_raw_event_builder.vhd
    tdc_gpx_decode_pipe.vhd
    tdc_gpx_cell_builder.vhd
    tdc_gpx_cell_pipe.vhd
    tdc_gpx_face_assembler.vhd
    tdc_gpx_face_seq.vhd
    tdc_gpx_header_inserter.vhd
    tdc_gpx_output_stage.vhd
    tdc_gpx_status_agg.vhd
    tdc_gpx_top.vhd
}

foreach f $rtl_files {
    set full "$hdl_dir/$f"
    if {[file exists $full]} {
        add_files -fileset sources_1 -norecurse $full
        set_property file_type "VHDL 2008" [get_files $full]
    } else {
        puts "WARN: missing $full"
    }
}

# -----------------------------------------------------------------------------
# Create IPs (2 개)
#   1) tdc_gpx_axil_csr_pipeline — my_axil_csr (7-bit addr)
#   2) tdc_gpx_axil_csr32_chip  — my_axil_csr32_top (9-bit addr)
# -----------------------------------------------------------------------------

# IP #1: pipeline CSR (7-bit)
create_ip \
    -vendor victek.co.kr -library my_ip -name my_axil_csr -version 1.0 \
    -module_name tdc_gpx_axil_csr_pipeline -dir "$prj_dir/$prj_name.srcs/sources_1/ip"
set_property -dict [list \
    CONFIG.num_data_bits        32 \
    CONFIG.num_ctl_regs         8 \
    CONFIG.num_stat_regs        8 \
    CONFIG.has_interrupt_regs   true \
    CONFIG.num_intr_regs        4 \
    CONFIG.num_interrupt_src    1 \
] [get_ips tdc_gpx_axil_csr_pipeline]
generate_target {instantiation_template synthesis simulation} \
    [get_files tdc_gpx_axil_csr_pipeline.xci]

# IP #2: chip CSR (9-bit)
create_ip \
    -vendor xilinx.com -library user -name my_axil_csr32_top -version 1.0 \
    -module_name tdc_gpx_axil_csr32_chip -dir "$prj_dir/$prj_name.srcs/sources_1/ip"
set_property -dict [list \
    CONFIG.num_data_bits        32 \
    CONFIG.num_ctl_regs         32 \
    CONFIG.num_stat_regs        32 \
    CONFIG.has_interrupt_regs   true \
    CONFIG.num_intr_regs        4 \
    CONFIG.num_interrupt_src    2 \
] [get_ips tdc_gpx_axil_csr32_chip]
generate_target {instantiation_template synthesis simulation} \
    [get_files tdc_gpx_axil_csr32_chip.xci]

# -----------------------------------------------------------------------------
# Simulation sources (TB)
# -----------------------------------------------------------------------------
set tb_files {
    tb_tdc_gpx_bus_phy.vhd
    tb_tdc_gpx_chip_ctrl.vhd
    tb_tdc_gpx_config_ctrl.vhd
    tb_tdc_gpx_decode_pipe.vhd
    tb_tdc_gpx_cell_pipe.vhd
    tb_tdc_gpx_downstream.vhd
    tb_tdc_gpx_output_stage.vhd
    tb_tdc_gpx_face_seq.vhd
    tb_tdc_gpx_scenarios.vhd
    tb_tdc_gpx_mask_sweep.vhd
    tb_tdc_gpx_top_int.vhd
}
foreach f $tb_files {
    set full "$hdl_dir/$f"
    if {[file exists $full]} {
        add_files -fileset sim_1 -norecurse $full
        set_property file_type "VHDL 2008" \
            [get_files -of_objects [get_filesets sim_1] [file tail $full]]
    } else {
        puts "WARN: missing tb $full"
    }
}

# -----------------------------------------------------------------------------
# Tops
# -----------------------------------------------------------------------------
set_property top tdc_gpx_top [current_fileset]
set_property top_lib xil_defaultlib [current_fileset]

set_property top tb_tdc_gpx_top_int [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {200us} \
    -objects [get_filesets sim_1]

# IP 의 compile / synthesis 순서 갱신
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

# Locked 방지 — IP 를 OOC (out-of-context) 로 생성 (기본)
# synth_1 run 은 아직 기동하지 않음 (시뮬레이션 우선)

# -----------------------------------------------------------------------------
# IP TB 파일을 simset 에서 제외 (컴파일 순서 꼬임 방지)
# -----------------------------------------------------------------------------
foreach pat {
    "*tdc_gpx_axil_csr32_chip/src/tb_my_axil_csr_32.vhd"
    "*tdc_gpx_axil_csr_pipeline/src/tb_my_axil_csr.vhd"
} {
    foreach f [get_files -quiet $pat] {
        set_property used_in_simulation false $f
    }
}

puts "========================================================"
puts "  Project recreated at: $prj_dir/$prj_name.xpr"
puts "  Synth top  : tdc_gpx_top"
puts "  Sim   top  : tb_tdc_gpx_top_int"
puts "========================================================"

close_project
