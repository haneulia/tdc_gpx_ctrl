# Synthesize the exact source set shipped in the LiDAR unified CSR package.

set script_dir [file normalize [file dirname [info script]]]
set hdl_dir [file normalize [file join $script_dir ../..]]
set ip_root [file normalize [file join $hdl_dir ../..]]
if {[llength $argv] > 0} {
    set package_dir [file normalize [lindex $argv 0]]
} else {
    set package_dir [file normalize \
        [file join $ip_root lidar_unified_csr ip_repo]]
}
if {[llength $argv] > 1} {
    set out_dir [file normalize [lindex $argv 1]]
} else {
    set out_dir [file normalize [file join $hdl_dir signoff_results \
        sessions lidar_unified_csr_packaged_ip_ooc]]
}
file mkdir $out_dir

set packaged_sources [list \
    [file join $package_dir src csr my_axil_csr32_pkg.vhd] \
    [file join $package_dir src csr axil_fsm_32.vhd] \
    [file join $package_dir src csr axil_ctrl_regs_32.vhd] \
    [file join $package_dir src csr axil_stat_regs_32.vhd] \
    [file join $package_dir src csr axil_intr_32.vhd] \
    [file join $package_dir src csr my_axil_csr32_top.vhd] \
    [file join $package_dir src lidar_unified_csr_pkg.vhd] \
    [file join $package_dir src lidar_unified_csr_top.vhd] \
    [file join $package_dir src lidar_unified_csr_ip_top.vhd]]
foreach source $packaged_sources {
    if {![file exists $source]} {
        error "Packaged unified CSR source is missing: $source"
    }
}

# Avoid a damaged per-user Tcl Store blocking ordinary project creation.
set install_tcl_store [file normalize \
    [file join $::env(XILINX_VIVADO) data XilinxTclStore]]
lappend auto_path [file join $install_tcl_store support appinit]
foreach vendor_dir [glob -nocomplain -type d \
        [file join $install_tcl_store tclapp *]] {
    foreach app_dir [glob -nocomplain -type d [file join $vendor_dir *]] {
        lappend auto_path $app_dir
    }
}
package require ::tclapp::support::appinit 1.2

create_project -in_memory -part xc7z020clg484-2
set_property target_language VHDL [current_project]
set_property simulator_language Mixed [current_project]
set_property default_lib xil_defaultlib [current_project]
foreach source $packaged_sources {
    read_vhdl -vhdl2008 -library xil_defaultlib $source
}
update_compile_order -fileset sources_1

synth_design -top lidar_unified_csr_ip_top -part xc7z020clg484-2 \
    -mode out_of_context -flatten_hierarchy none

set black_boxes [get_cells -hier -quiet -filter {IS_BLACKBOX == 1}]
if {[llength $black_boxes] != 0} {
    error "Packaged unified CSR contains black boxes: $black_boxes"
}
set csr_owners [get_cells -hier -quiet -filter \
    {REF_NAME == my_axil_csr32_top || ORIG_REF_NAME == my_axil_csr32_top}]
if {[llength $csr_owners] != 1} {
    error "Expected exactly one CSR32 owner: $csr_owners"
}
set legacy_owners [get_cells -hier -quiet -filter \
    {REF_NAME == my_axil_csr_top || ORIG_REF_NAME == my_axil_csr_top}]
if {[llength $legacy_owners] != 0} {
    error "Packaged unified CSR contains a legacy CSR owner: $legacy_owners"
}

foreach {port_name width} {
    s_axi_csr_awaddr 9
    s_axi_csr_araddr 9
    o_motor_sys_ctrl 32
    o_laser_sys_ctrl 32
    o_echo_sys_ctrl 32
    o_tdc_sys_ctrl 32
    i_echo_irq_cause 5
    i_tdc_irq_cause 7
} {
    set actual [llength [get_ports -quiet $port_name]]
    if {$actual != $width} {
        error "$port_name synthesized width is $actual, expected $width"
    }
}

report_utilization -hierarchical -hierarchical_depth 6 \
    -file [file join $out_dir utilization_hier.rpt]
write_checkpoint -force [file join $out_dir post_synth.dcp]

puts "BLACK_BOX_COUNT=[llength $black_boxes]"
puts "CSR32_OWNER_COUNT=[llength $csr_owners]"
puts "LEGACY_CSR_OWNER_COUNT=[llength $legacy_owners]"
puts {LIDAR_UNIFIED_CSR_PACKAGED_IP_OOC_PASS}
close_project
exit
