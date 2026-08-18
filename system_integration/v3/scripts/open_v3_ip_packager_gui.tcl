# Reopen both the disposable edit project and the canonical packaged core so
# Vivado enters the Package IP workflow instead of showing a plain project.

if {$argc != 4} {
    error "usage: open_v3_ip_packager_gui.tcl <project.xpr> <component.xml> <expected_vlnv> <xgui.tcl>"
}

set project_path [file normalize [lindex $argv 0]]
set component [file normalize [lindex $argv 1]]
set expected_vlnv [lindex $argv 2]
set packaged_xgui [file normalize [lindex $argv 3]]
if {![file exists $project_path]} {
    error "V3 IP Packager project does not exist: $project_path"
}
if {![file exists $component]} {
    error "V3 component.xml does not exist: $component"
}
if {![file exists $packaged_xgui]} {
    error "V3 packaged XGUI does not exist: $packaged_xgui"
}

open_project $project_path
set ip_repo [file dirname [file dirname $component]]
set_property ip_repo_paths [list $ip_repo] [current_project]
update_ip_catalog

set core [ipx::open_core $component]
if {[get_property vlnv $core] ne $expected_vlnv} {
    error "Unexpected V3 core in GUI: [get_property vlnv $core]"
}

# ipx::open_core materializes the current Customization Parameter state.
# The XGUI callbacks are already Vivado-native; refresh only timestamp order
# before the Package IP page renders Layout and Preview.
file mtime $packaged_xgui [expr {[clock seconds] + 2}]

puts "TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGER_GUI_CORE=[get_property vlnv $core]"
puts {TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGER_GUI_READY}
