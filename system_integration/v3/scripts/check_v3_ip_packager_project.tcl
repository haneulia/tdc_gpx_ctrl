# Verify that a generated Edit Packaged IP project still targets the canonical
# self-contained V3 package and has not acquired external source references.

if {$argc != 2} {
    error "usage: check_v3_ip_packager_project.tcl <project.xpr> <component.xml>"
}

set project_path [file normalize [lindex $argv 0]]
set component [file normalize [lindex $argv 1]]
if {![file exists $project_path]} {
    error "V3 IP Packager project does not exist: $project_path"
}
if {![file exists $component]} {
    error "V3 component.xml does not exist: $component"
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
set core [ipx::open_core $component]
if {[get_property vlnv $core] ne \
        {victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3:3.0}} {
    error "Unexpected V3 VLNV: [get_property vlnv $core]"
}

set synthesis_group [ipx::get_file_groups -quiet \
    xilinx_anylanguagesynthesis -of_objects $core]
set simulation_group [ipx::get_file_groups -quiet \
    xilinx_anylanguagebehavioralsimulation -of_objects $core]
set xgui_group [ipx::get_file_groups -quiet xilinx_xpgui -of_objects $core]
foreach {group label} [list \
        $synthesis_group synthesis \
        $simulation_group simulation \
        $xgui_group xgui] {
    if {[llength $group] != 1} {
        error "Expected one V3 $label file group"
    }
}

set synthesis_files [ipx::get_files -of_objects $synthesis_group]
if {[llength $synthesis_files] < 100} {
    error "V3 synthesis closure is unexpectedly small: [llength $synthesis_files]"
}
foreach group [ipx::get_file_groups -of_objects $core] {
    if {[get_property component_subcores $group] ne {}} {
        error "V3 package must remain self-contained: [get_property name $group]"
    }
    foreach packaged_file [ipx::get_files -of_objects $group] {
        set name [string map {\\ /} [get_property name $packaged_file]]
        if {[string match "../*" $name] || [string match "*/../*" $name]} {
            error "External source reference entered the V3 package: $name"
        }
        if {[string match "*.xci" $name]} {
            error "Child XCI entered the self-contained V3 package: $name"
        }
    }
}

set xgui_files [ipx::get_files -of_objects $xgui_group]
if {[llength $xgui_files] != 1 ||
    [get_property name [lindex $xgui_files 0]] ne \
        {xgui/tdc_gpx_lidar_ctrl_v3_v3_0.tcl}} {
    error "Unexpected V3 XGUI file group: $xgui_files"
}
foreach required_port {
    s_axi_csr_aclk proc_aclk i_tdc_clk io_tdc_d o_tdc_csn
    m_axis_rise_tdata m_axis_fall_tdata o_irq
} {
    if {[llength [ipx::get_ports -quiet $required_port \
            -of_objects $core]] != 1} {
        error "Required V3 port is missing: $required_port"
    }
}

set drc [ipx::check_integrity -verbose $core]
if {!$drc} {
    error {V3 IP Packager integrity DRC failed}
}
puts "TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGER_DRC=$drc"
puts "TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGER_SYNTH_FILES=[llength $synthesis_files]"
puts {TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGER_CHECK_PASS}
close_project
exit 0
