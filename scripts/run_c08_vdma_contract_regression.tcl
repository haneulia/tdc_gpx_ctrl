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
set packer "$hdl_dir/tdc_gpx_line_packer.vhd"
set c08_sources [list \
    $packer \
    "$hdl_dir/tb_tdc_gpx_top_int_c08_vdma_widths.vhd" \
    "$hdl_dir/tb_tdc_gpx_top_int_c08_dual_edge_shared.vhd"]

open_project "$prj_dir/tdc_gpx_ctrl.xpr"

if {[llength [get_files -quiet $packer]] == 0} {
    puts "C08: adding tdc_gpx_line_packer.vhd to sources_1"
    add_files -norecurse -fileset sources_1 $packer
}
set_property file_type {VHDL 2008} [get_files $packer]

foreach tb [lrange $c08_sources 1 end] {
    if {[llength [get_files -quiet $tb]] == 0} {
        puts "C08: adding [file tail $tb] to sim_1"
        add_files -norecurse -fileset sim_1 $tb
    }
    set_property file_type {VHDL 2008} [get_files $tb]
}

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

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
