# Recreate tdc_gpx_ctrl.xpr from canonical source-level RTL.
#
# Generated CSR XCI products are intentionally not used. Project, regression,
# parent-reference, and packaged-IP builds all consume the same CSR sources.

set prj_dir  "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl"
set prj_name "tdc_gpx_ctrl"
set part     "xc7z020clg484-2"
set hdl_dir  "$prj_dir/HDL"
set ip_root  "C:/Projects/my_sp/lib/IP"

create_project $prj_name $prj_dir -part $part -force
set_property target_language VHDL [current_project]
set_property simulator_language Mixed [current_project]
set_property default_lib xil_defaultlib [current_project]

# Canonical CSR8 and CSR32 sources.
source [file join $hdl_dir scripts tdc_gpx_csr_source_manifest.tcl]
foreach source [tdc_gpx_csr_source_manifest $ip_root] {
    if {![file exists $source]} {
        error "missing canonical CSR source: $source"
    }
    add_files -fileset sources_1 -norecurse $source
    set_property file_type {VHDL 2008} [get_files $source]
}

# Canonical TDC-GPX synthesis sources.
source [file join $hdl_dir scripts tdc_gpx_rtl_manifest.tcl]
foreach file_name [tdc_gpx_rtl_manifest] {
    set source [file join $hdl_dir $file_name]
    if {![file exists $source]} {
        error "missing RTL source: $source"
    }
    add_files -fileset sources_1 -norecurse $source
    set_property file_type {VHDL 2008} [get_files $source]
}

# Standalone simulation support.
set tb_files {
    tb_tdc_gpx_pkg.vhd
    tb_tdc_gpx_atomic_snapshot_cdc.vhd
    tb_tdc_gpx_bus_phy.vhd
    tb_tdc_gpx_chip_ctrl.vhd
    tb_tdc_gpx_config_ctrl.vhd
    tb_tdc_gpx_reg_rsp_cdc.vhd
    tb_tdc_gpx_decode_pipe.vhd
    tb_tdc_gpx_cell_pipe.vhd
    tb_tdc_gpx_downstream.vhd
    tb_tdc_gpx_output_stage.vhd
    tb_tdc_gpx_face_seq.vhd
    tb_tdc_gpx_scenarios.vhd
    tb_tdc_gpx_mask_sweep.vhd
    tb_tdc_gpx_top_int.vhd
}
foreach file_name $tb_files {
    set source [file join $hdl_dir $file_name]
    if {[file exists $source]} {
        add_files -fileset sim_1 -norecurse $source
        set_property file_type {VHDL 2008} [get_files -all $source]
    } else {
        puts "WARN: missing testbench source: $source"
    }
}

set_property top tdc_gpx_top [get_filesets sources_1]
set_property top_lib xil_defaultlib [get_filesets sources_1]
set_property top tb_tdc_gpx_top_int [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {200us} \
    -objects [get_filesets sim_1]

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

puts "========================================================"
puts "  Project recreated at: $prj_dir/$prj_name.xpr"
puts "  Synth top : tdc_gpx_top"
puts "  Sim top   : tb_tdc_gpx_top_int"
puts "  CSR mode  : canonical source-level RTL"
puts "========================================================"

close_project
