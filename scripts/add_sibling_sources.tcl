# =============================================================================
# add_sibling_sources.tcl
#   이웃 모듈 (motor_decoder, laser_ctrl, echo_receiver) 의 HDL 을
#   **복사 없이 참조만** (add_files -norecurse) 로 등록.
#   그리고 이웃이 사용하는 IP 3 개 (laser_ctl_axil_csr, my_axil_csr_0,
#   echo_receiver_axil_csr32) 를 추가 생성.
#
#   → import_files 는 사용하지 않음 (파일 복사 발생).
# =============================================================================

set prj_dir "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl"
open_project "$prj_dir/tdc_gpx_ctrl.xpr"

# -----------------------------------------------------------------------------
# 이웃 HDL 경로
# -----------------------------------------------------------------------------
set md_dir  "C:/Projects/my_sp/lib/IP/motor_decoder/HDL"
set lc_dir  "C:/Projects/my_sp/lib/IP/laser_ctrl/HDL"
set er_dir  "C:/Projects/my_sp/lib/IP/echo_receiver/HDL"

set sibling_rtl [list \
    "$md_dir/enc_pkg.vhd" \
    "$md_dir/enc_phase_counter.vhd" \
    "$md_dir/enc_position_counter.vhd" \
    "$md_dir/enc_tick_counter.vhd" \
    "$md_dir/enc_fractional_scheduler.vhd" \
    "$md_dir/enc_param_apply_ctrl.vhd" \
    "$md_dir/enc_top.vhd" \
    "$md_dir/quad_decoder.vhd" \
    "$md_dir/mirror_active_detect.vhd" \
    "$md_dir/motor_irq_bridge.vhd" \
    "$md_dir/motor_decoder_cfg_pkg.vhd" \
    "$md_dir/motor_cfg_commit_ctrl.vhd" \
    "$md_dir/motor_axis_stream_out.vhd" \
    "$md_dir/motor_decoder_csr.vhd" \
    "$md_dir/motor_decoder_top.vhd" \
    \
    "$lc_dir/laser_ctrl_types_pkg.vhd" \
    "$lc_dir/laser_ctrl_cfg_pkg.vhd" \
    "$lc_dir/laser_ctrl_math_pkg.vhd" \
    "$lc_dir/laser_ctrl_pkg.vhd" \
    "$lc_dir/laser_ctrl_status.vhd" \
    "$lc_dir/laser_ctrl_tdc.vhd" \
    "$lc_dir/laser_ctrl_result.vhd" \
    "$lc_dir/laser_ctrl_echo_capture.vhd" \
    "$lc_dir/laser_ctrl_scheduler.vhd" \
    "$lc_dir/laser_ctrl_csr.vhd" \
    "$lc_dir/laser_ctrl_metrics.vhd" \
    "$lc_dir/laser_ctrl_axis_in.vhd" \
    "$lc_dir/laser_ctrl_executor.vhd" \
    "$lc_dir/laser_ctrl_top.vhd" \
    \
    "$er_dir/echo_receiver_pkg.vhd" \
    "$er_dir/echo_receiver_core.vhd" \
    "$er_dir/echo_receiver_csr.vhd" \
    "$er_dir/echo_receiver_top.vhd" \
]

set sibling_tb [list \
    "$md_dir/tb_motor_decoder_pkg.vhd" \
    "$md_dir/motor_decoder_top_tb.vhd" \
    "$md_dir/enc_top_tb.vhd" \
    \
    "$lc_dir/tb_laser_ctrl_tests_pkg.vhd" \
    "$lc_dir/tb_laser_ctrl_pkg.vhd" \
    "$lc_dir/tb_laser_ctrl_monitors.vhd" \
    "$lc_dir/tb_laser_ctrl_axis_in_unit.vhd" \
    "$lc_dir/tb_laser_ctrl_scheduler_unit.vhd" \
    "$lc_dir/tb_laser_ctrl_executor_unit.vhd" \
    "$lc_dir/tb_laser_ctrl_metrics_unit.vhd" \
    "$lc_dir/tb_laser_ctrl.vhd" \
    \
    "$er_dir/tb_echo_receiver_pkg.vhd" \
    "$er_dir/tb_echo_receiver_core_only.vhd" \
    "$er_dir/tb_echo_receiver.vhd" \
]

puts "INFO: adding sibling RTL (reference, no copy)"
foreach f $sibling_rtl {
    if {[file exists $f]} {
        add_files -fileset sources_1 -norecurse $f
        set_property file_type "VHDL 2008" [get_files $f]
    } else {
        puts "WARN: missing $f"
    }
}

puts "INFO: adding sibling TB (reference, no copy)"
foreach f $sibling_tb {
    if {[file exists $f]} {
        add_files -fileset sim_1 -norecurse $f
        set_property file_type "VHDL 2008" \
            [get_files -of_objects [get_filesets sim_1] [file tail $f]]
    } else {
        puts "WARN: missing tb $f"
    }
}

# -----------------------------------------------------------------------------
# 추가 IP 3 개 (이웃이 사용)
# -----------------------------------------------------------------------------
proc mk_ip {ip_name vendor library name ver ctl stat intr_src} {
    global prj_dir
    puts "INFO: creating IP $ip_name"
    create_ip -vendor $vendor -library $library -name $name -version $ver \
        -module_name $ip_name -dir "$prj_dir/tdc_gpx_ctrl.srcs/sources_1/ip"
    set_property -dict [list \
        CONFIG.num_data_bits      32 \
        CONFIG.num_ctl_regs       $ctl \
        CONFIG.num_stat_regs      $stat \
        CONFIG.has_interrupt_regs true \
        CONFIG.num_intr_regs      4 \
        CONFIG.num_interrupt_src  $intr_src \
    ] [get_ips $ip_name]
    generate_target {instantiation_template synthesis simulation} \
        [get_files $ip_name.xci]
}

# laser_ctl_axil_csr — my_axil_csr 7-bit, 8/8 CTL/STAT, 1 intr_src
mk_ip laser_ctl_axil_csr  victek.co.kr my_ip my_axil_csr        1.0  8  8 1
# my_axil_csr_0 — motor_decoder 용 (ALU: 3 intr src 기존값 유지)
mk_ip my_axil_csr_0       victek.co.kr my_ip my_axil_csr        1.0  8  8 3
# echo_receiver_axil_csr32 — 9-bit, 17 CTL / 4 STAT, 1 intr_src
mk_ip echo_receiver_axil_csr32 xilinx.com user  my_axil_csr32_top 1.0 17  4 1

# -----------------------------------------------------------------------------
# IP 내부 tb 파일 excludes (컴파일 순서 꼬임 방지)
# -----------------------------------------------------------------------------
foreach pat {
    "*tdc_gpx_axil_csr32_chip/src/tb_my_axil_csr_32.vhd"
    "*tdc_gpx_axil_csr_pipeline/src/tb_my_axil_csr.vhd"
    "*laser_ctl_axil_csr/src/tb_my_axil_csr.vhd"
    "*my_axil_csr_0/src/tb_my_axil_csr.vhd"
    "*echo_receiver_axil_csr32/src/tb_my_axil_csr_32.vhd"
} {
    foreach f [get_files -quiet $pat] {
        puts "INFO: set used_in_simulation false  -> $f"
        set_property used_in_simulation false $f
    }
}

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

# tops 재확인
set_property top tdc_gpx_top [get_filesets sources_1]
set_property top tb_tdc_gpx_top_int [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sources_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]

puts "========================================================"
puts "  Sibling HDL + IPs 추가 완료"
puts "========================================================"

close_project
