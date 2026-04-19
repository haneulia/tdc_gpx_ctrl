# =============================================================================
# launch_top_int_sim.tcl
#   xpr 를 열고 sim_1 (top = tb_tdc_gpx_top_int) 로 xsim 실행.
#   compile/elaborate/simulate 전체 수행.
# =============================================================================

set prj_dir "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl"
open_project "$prj_dir/tdc_gpx_ctrl.xpr"

# sim 주제 확인
set_property top tb_tdc_gpx_top_int [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {200us} \
    -objects [get_filesets sim_1]

update_compile_order -fileset sim_1

launch_simulation -simset sim_1 -mode behavioral
puts "INFO: simulation launched"

# -- close_project 만 사용 (절대 -delete 금지)
close_sim -quiet
close_project
