# =============================================================================
# fix_ip_tb_files.tcl
#   IP 내부 tb_my_axil_csr*.vhd 파일을 simulation 대상에서 제외.
#   (Vivado 가 해당 IP 테스트벤치를 자동으로 simset 에 포함시키면서
#    dependency 순서가 깨져 xvhdl 컴파일 실패 발생)
# =============================================================================

set project_file "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/tdc_gpx_ctrl.xpr"
open_project $project_file

set ip_tb_patterns {
    "*tdc_gpx_axil_csr32_chip/src/tb_my_axil_csr_32.vhd"
    "*tdc_gpx_axil_csr_pipeline/src/tb_my_axil_csr.vhd"
    "*laser_ctl_axil_csr/src/tb_my_axil_csr.vhd"
    "*my_axil_csr_0/src/tb_my_axil_csr.vhd"
}

foreach pat $ip_tb_patterns {
    set files [get_files -quiet $pat]
    foreach f $files {
        puts "INFO: excluding from simulation: $f"
        set_property used_in_simulation false $f
    }
}

# Re-run update_compile_order so sim_1 .prj 를 재생성
update_compile_order -fileset sim_1

# 재생성: sim_1 의 compile/elaborate/simulate 스크립트
set_property top tb_tdc_gpx_top_int [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]

# Scripts-only 모드로 .prj / .bat 갱신
launch_simulation -simset sim_1 -mode behavioral -scripts_only
close_project -delete
