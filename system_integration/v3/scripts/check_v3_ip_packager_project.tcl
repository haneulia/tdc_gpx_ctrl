# Verify an Edit Packaged IP project for either V3 packaging profile.

if {$argc != 4} {
    error "usage: check_v3_ip_packager_project.tcl <project.xpr> <component.xml> <expected_vlnv> <profile>"
}

set project_path [file normalize [lindex $argv 0]]
set component [file normalize [lindex $argv 1]]
set expected_vlnv [lindex $argv 2]
set profile [string toupper [lindex $argv 3]]
switch -- $profile {
    EMBEDDED_RTL {
        set minimum_synthesis_files 100
        set expected_xgui {xgui/tdc_gpx_lidar_ctrl_v3_v3_0.tcl}
    }
    HLS_IP {
        set minimum_synthesis_files 80
        set expected_xgui {xgui/tdc_gpx_lidar_ctrl_v3_hls_ip_v3_0.tcl}
    }
    default { error "Unknown V3 IP Packager profile: $profile" }
}
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
set ip_repo [file dirname [file dirname $component]]
set_property ip_repo_paths [list $ip_repo] [current_project]
update_ip_catalog
set core [ipx::open_core $component]
if {[get_property vlnv $core] ne $expected_vlnv} {
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
if {[llength $synthesis_files] < $minimum_synthesis_files} {
    error "V3 synthesis closure is unexpectedly small: [llength $synthesis_files]"
}
set found_subcores {}
set xci_count 0
foreach group [ipx::get_file_groups -of_objects $core] {
    foreach dependency [get_property component_subcores $group] {
        if {$dependency ni $found_subcores} {
            lappend found_subcores $dependency
        }
    }
    foreach packaged_file [ipx::get_files -of_objects $group] {
        set name [string map {\\ /} [get_property name $packaged_file]]
        if {[string match "../*" $name] || [string match "*/../*" $name]} {
            error "External source reference entered the V3 package: $name"
        }
        if {[string match "*hls_generated*" $name] && $profile eq {HLS_IP}} {
            error "Embedded HLS RTL entered the child-IP profile: $name"
        }
        if {[string match "*.xci" $name]} {
            incr xci_count
        }
        if {$profile eq {EMBEDDED_RTL} && [string match "*.xci" $name]} {
            error "Child XCI entered the self-contained V3 package: $name"
        }
    }
}
if {$profile eq {EMBEDDED_RTL} && [llength $found_subcores] != 0} {
    error "Embedded V3 package acquired child dependencies: $found_subcores"
}
if {$profile eq {HLS_IP}} {
    set expected_subcores {
        victek.co.kr:hls_ip:gpx_hit_decoder_hls:3.0
        victek.co.kr:hls_ip:gpx_cell_collector_hls:3.0
        victek.co.kr:hls_ip:gpx_frame_assembler_hls:3.0
        victek.co.kr:hls_ip:gpx_lane_word_formatter_hls:3.0
    }
    foreach dependency $expected_subcores {
        if {$dependency ni $found_subcores} {
            error "HLS child dependency is missing: $dependency"
        }
    }
    # Each imported XCI appears once in synthesis and once in simulation.
    if {$xci_count != 8} {
        error "Expected eight HLS child XCI file-group entries, found $xci_count"
    }
}

set xgui_files [ipx::get_files -of_objects $xgui_group]
if {[llength $xgui_files] != 1 ||
    [get_property name [lindex $xgui_files 0]] ne $expected_xgui} {
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
puts "TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGER_PROFILE=$profile"
puts {TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGER_CHECK_PASS}
close_project
exit 0
