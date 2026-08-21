# Reopen the V2 edit project and canonical core in the Package IP workflow.

if {$argc != 4} {
    error "usage: open_v2_ip_packager_gui.tcl <project.xpr> <component.xml> <expected_vlnv> <xgui.tcl>"
}

set project_path [file normalize [lindex $argv 0]]
set component [file normalize [lindex $argv 1]]
set expected_vlnv [lindex $argv 2]
set packaged_xgui [file normalize [lindex $argv 3]]
foreach required [list $project_path $component $packaged_xgui] {
    if {![file exists $required]} { error "V2 GUI input is missing: $required" }
}

open_project $project_path
set ip_repo [file dirname [file dirname $component]]
set_property ip_repo_paths [list $ip_repo] [current_project]
update_ip_catalog
set core [ipx::open_core $component]
if {[get_property vlnv $core] ne $expected_vlnv} {
    error "Unexpected V2 core in GUI: [get_property vlnv $core]"
}
file mtime $packaged_xgui [expr {[clock seconds] + 2}]
puts "TDC_GPX_LIDAR_CTRL_V2_IP_PACKAGER_GUI_CORE=[get_property vlnv $core]"
puts {TDC_GPX_LIDAR_CTRL_V2_IP_PACKAGER_GUI_READY}
