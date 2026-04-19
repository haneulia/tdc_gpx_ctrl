# =============================================================================
# run_all_tbs.tcl
#   sim_1 simset 의 top 을 순차적으로 바꿔가며 launch_simulation.
#   각 TB 별 compile/elaborate/run 을 수행하고 sim log 를 별도 파일로 보존.
#   pass/fail 은 이후 bash 에서 log 파싱.
# =============================================================================

set prj_dir "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl"
open_project "$prj_dir/tdc_gpx_ctrl.xpr"

set tbs {
    tb_tdc_gpx_bus_phy
    tb_tdc_gpx_chip_ctrl
    tb_tdc_gpx_config_ctrl
    tb_tdc_gpx_decode_pipe
    tb_tdc_gpx_cell_pipe
    tb_tdc_gpx_downstream
    tb_tdc_gpx_output_stage
    tb_tdc_gpx_face_seq
    tb_tdc_gpx_mask_sweep
    tb_tdc_gpx_scenarios
    tb_tdc_gpx_top_int
}

set runtimes {
    200us   200us   200us   200us   200us
    200us   200us   100us   200us   500us
    200us
}

foreach tb $tbs runtime $runtimes {
    puts "=============================================================="
    puts "  Running $tb  (runtime=$runtime)"
    puts "=============================================================="

    set_property top $tb [get_filesets sim_1]
    set_property top_lib xil_defaultlib [get_filesets sim_1]
    set_property -name {xsim.simulate.runtime} -value $runtime -objects [get_filesets sim_1]

    update_compile_order -fileset sim_1

    # scripts-only 로 .bat / .prj 생성만. 실제 컴파일/실행은 bash 에서.
    # (Vivado 의 spawn 버그 회피)
    if {[catch {launch_simulation -simset sim_1 -mode behavioral -scripts_only} err]} {
        puts "WARN: scripts_only for $tb: $err"
    }
    # .prj 를 TB 별 이름으로 보존
    set src "$prj_dir/tdc_gpx_ctrl.sim/sim_1/behav/xsim"
    foreach suffix {_vhdl.prj _vlog.prj} {
        set gen "$src/$tb$suffix"
        set bak "$src/__${tb}${suffix}"
        if {[file exists $gen]} {
            file copy -force $gen $bak
        }
    }
}

close_project
