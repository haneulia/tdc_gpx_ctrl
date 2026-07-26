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
set ip_root "C:/Projects/my_sp/lib/IP"
source "$prj_dir/HDL/scripts/tdc_gpx_integration_source_manifest.tcl"
set sibling_rtl [tdc_gpx_integration_rtl_manifest $ip_root]
set sibling_tb  [tdc_gpx_integration_tb_manifest $ip_root]

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
# laser_ctl_axil_csr — my_axil_csr 7-bit, 8/8 CTL/STAT, 1 intr_src
# CSR entities are compiled directly from their canonical source directories.
# my_axil_csr_0 — motor_decoder 용 (ALU: 3 intr src 기존값 유지)
# echo_receiver_axil_csr32 — 9-bit, 17 CTL / 4 STAT, 1 intr_src
source "$prj_dir/HDL/scripts/tdc_gpx_csr_source_manifest.tcl"
foreach f [tdc_gpx_csr_source_manifest $ip_root] {
    if {![file exists $f]} {
        error "missing canonical CSR source: $f"
    }
    if {[llength [get_files -all -quiet $f]] == 0} {
        add_files -fileset sources_1 -norecurse $f
    }
    set_property file_type {VHDL 2008} [get_files -all $f]
}

# -----------------------------------------------------------------------------
# IP 내부 tb 파일 excludes (컴파일 순서 꼬임 방지)
# -----------------------------------------------------------------------------
foreach pat {
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
