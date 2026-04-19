# =============================================================================
# add_and_run_top_int_tb.tcl
# tb_tdc_gpx_top_int 를 xpr simset 에 등록 + 시뮬레이션 실행
# =============================================================================
# 실행 방법:
#   vivado -mode batch -source HDL/scripts/add_and_run_top_int_tb.tcl \
#          -nojournal -nolog
# =============================================================================

set project_file "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/tdc_gpx_ctrl.xpr"
set tb_file      "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/tb_tdc_gpx_top_int.vhd"
set tb_top       "tb_tdc_gpx_top_int"

# -----------------------------------------------------------------------------
# Open project
# -----------------------------------------------------------------------------
open_project $project_file

# -----------------------------------------------------------------------------
# Add TB file if not already present
# -----------------------------------------------------------------------------
set existing [get_files -quiet -of_objects [get_filesets sim_1] \
              [file tail $tb_file]]
if {[llength $existing] == 0} {
    puts "INFO: adding $tb_file to sim_1"
    add_files -fileset sim_1 -norecurse $tb_file
    set_property file_type "VHDL 2008" \
        [get_files -of_objects [get_filesets sim_1] [file tail $tb_file]]
} else {
    puts "INFO: $tb_file already in sim_1"
    set_property file_type "VHDL 2008" $existing
}

# -----------------------------------------------------------------------------
# Set TB as simset top
# -----------------------------------------------------------------------------
set_property top $tb_top [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
update_compile_order -fileset sim_1

# -----------------------------------------------------------------------------
# Configure simulation runtime (200 us — 통합 TB 의 watchdog 과 매칭)
# -----------------------------------------------------------------------------
set_property -name {xsim.simulate.runtime} -value {200us} \
    -objects [get_filesets sim_1]
set_property -name {xsim.elaborate.debug_level} -value {typical} \
    -objects [get_filesets sim_1]

# -----------------------------------------------------------------------------
# Launch simulation (compile + elaborate + run)
# -----------------------------------------------------------------------------
puts "INFO: launching xsim for $tb_top"
launch_simulation -simset sim_1 -mode behavioral

puts "INFO: simulation launched — waiting for completion"
# xsim 는 batch 에서 자동 실행됨. 완료 후 close 하지 않으면 blocking 될 수
# 있으므로 즉시 시뮬레이션 종료/정리.
close_sim -quiet
close_project -delete
