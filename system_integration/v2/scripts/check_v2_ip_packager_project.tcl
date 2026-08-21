# Verify the disposable Edit Packaged IP project and canonical V2 component.

if {$argc != 3} {
    error "usage: check_v2_ip_packager_project.tcl <project.xpr> <component.xml> <expected_vlnv>"
}

set project_path [file normalize [lindex $argv 0]]
set component [file normalize [lindex $argv 1]]
set expected_vlnv [lindex $argv 2]
if {![file exists $project_path] || ![file exists $component]} {
    error {V2 IP Packager project or component.xml is missing}
}

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

open_project $project_path
set ip_repo [file dirname [file dirname $component]]
set_property ip_repo_paths [list $ip_repo] [current_project]
update_ip_catalog
set core [ipx::open_core $component]
if {[get_property vlnv $core] ne $expected_vlnv} {
    error "Unexpected V2 VLNV: [get_property vlnv $core]"
}

foreach {group_name label} {
    xilinx_anylanguagesynthesis synthesis
    xilinx_anylanguagebehavioralsimulation simulation
    xilinx_xpgui xgui
} {
    set group [ipx::get_file_groups -quiet $group_name -of_objects $core]
    if {[llength $group] != 1} {
        error "Expected one V2 $label file group"
    }
}
set synthesis_group [ipx::get_file_groups \
    xilinx_anylanguagesynthesis -of_objects $core]
set synthesis_files [ipx::get_files -of_objects $synthesis_group]
if {[llength $synthesis_files] != 88} {
    error "Expected 88 V2 synthesis files, found [llength $synthesis_files]"
}
set subcores {}
foreach group [ipx::get_file_groups -of_objects $core] {
    foreach dependency [get_property component_subcores $group] {
        if {$dependency ni $subcores} { lappend subcores $dependency }
    }
    foreach packaged_file [ipx::get_files -of_objects $group] {
        set name [string map {\\ /} [get_property name $packaged_file]]
        if {[string match "../*" $name] || [string match "*/../*" $name] ||
            [string match "*.xci" $name]} {
            error "Forbidden V2 package dependency: $name"
        }
    }
}
if {[llength $subcores] != 0} {
    error "V2 package unexpectedly contains subcores: $subcores"
}

set xgui_group [ipx::get_file_groups xilinx_xpgui -of_objects $core]
set xgui_files [ipx::get_files -of_objects $xgui_group]
if {[llength $xgui_files] != 1 ||
    [get_property name [lindex $xgui_files 0]] ne
        {xgui/tdc_gpx_lidar_ctrl_v2_v2_0.tcl}} {
    error "Unexpected V2 XGUI file group: $xgui_files"
}
foreach required_port {
    s_axi_csr_aclk proc_aclk i_tdc_clk io_tdc_d o_tdc_csn
    m_axis_rise_tdata m_axis_fall_tdata o_irq
} {
    if {[llength [ipx::get_ports -quiet $required_port \
            -of_objects $core]] != 1} {
        error "Required V2 port is missing: $required_port"
    }
}

set drc [ipx::check_integrity -verbose $core]
if {!$drc} {
    error {V2 IP Packager integrity DRC failed}
}
puts "TDC_GPX_LIDAR_CTRL_V2_IP_PACKAGER_DRC=$drc"
puts "TDC_GPX_LIDAR_CTRL_V2_IP_PACKAGER_SYNTH_FILES=[llength $synthesis_files]"
puts {TDC_GPX_LIDAR_CTRL_V2_IP_PACKAGER_CHECK_PASS}
close_project
exit 0
