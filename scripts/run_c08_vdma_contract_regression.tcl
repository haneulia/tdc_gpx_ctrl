# C08 packed VDMA contract integration regression.
# Adds the line packer and C08 wrappers to the real project, then runs the
# dedicated 32/64/128-bit and shared-dual-edge integration profiles.

set required_vivado "2025.2"
set running_vivado [version -short]
if {[package vcompare $running_vivado $required_vivado] < 0} {
    puts "ERROR: C08 project requires Vivado $required_vivado or newer; running $running_vivado"
    exit 2
}

set prj_dir "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl"
set hdl_dir "$prj_dir/HDL"
set c08_rtl_sources [list \
    "$hdl_dir/tdc_gpx_atomic_snapshot_cdc.vhd" \
    "$hdl_dir/tdc_gpx_sync_fifo.vhd" \
    "$hdl_dir/tdc_gpx_reg_rsp_cdc.vhd" \
    "$hdl_dir/tdc_gpx_line_packer.vhd"]
set c08_tb_sources [list \
    "$hdl_dir/tb_tdc_gpx_top_int_c07_4chip_target.vhd" \
    "$hdl_dir/tb_tdc_gpx_top_int_c08_vdma_widths.vhd" \
    "$hdl_dir/tb_tdc_gpx_top_int_c08_dual_edge_shared.vhd"]

open_project "$prj_dir/tdc_gpx_ctrl.xpr"

foreach rtl $c08_rtl_sources {
    if {[llength [get_files -quiet $rtl]] == 0} {
        puts "C08: adding [file tail $rtl] to sources_1"
        add_files -norecurse -fileset sources_1 $rtl
    }
    set_property file_type {VHDL 2008} [get_files $rtl]
}

foreach tb $c08_tb_sources {
    if {[llength [get_files -quiet $tb]] == 0} {
        puts "C08: adding [file tail $tb] to sim_1"
        add_files -norecurse -fileset sim_1 $tb
    }
    set_property file_type {VHDL 2008} [get_files $tb]
}

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

# Package/interface changes can leave dependent wrapper VDB files stale even
# after update_compile_order. Start the matrix from a clean simulation state so
# every wrapper is rebuilt against the current tdc_gpx_pkg interface.
reset_simulation

set tops [list \
    tb_tdc_gpx_top_int_c08_vdma_w32 \
    tb_tdc_gpx_top_int_c08_vdma_w64 \
    tb_tdc_gpx_top_int_c08_vdma_w128 \
    tb_tdc_gpx_top_int_c08_dual_edge_shared]

foreach top $tops {
    puts "C08: running $top"
    set_property top $top [get_filesets sim_1]
    set_property top_lib xil_defaultlib [get_filesets sim_1]
    set_property -name {xsim.simulate.runtime} -value {200us} \
        -objects [get_filesets sim_1]
    launch_simulation -simset sim_1 -mode behavioral
    run all
    close_sim -quiet
}
close_project
exit 0
