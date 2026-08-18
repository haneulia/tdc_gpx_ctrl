# Reopen both the disposable edit project and the canonical packaged core so
# Vivado enters the Package IP workflow instead of showing a plain project.

if {$argc != 2} {
    error "usage: open_v3_ip_packager_gui.tcl <project.xpr> <component.xml>"
}

set project_path [file normalize [lindex $argv 0]]
set component [file normalize [lindex $argv 1]]
if {![file exists $project_path]} {
    error "V3 IP Packager project does not exist: $project_path"
}
if {![file exists $component]} {
    error "V3 component.xml does not exist: $component"
}

open_project $project_path
set core [ipx::open_core $component]
if {[get_property vlnv $core] ne \
        {victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3:3.0}} {
    error "Unexpected V3 core in GUI: [get_property vlnv $core]"
}

puts "TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGER_GUI_CORE=[get_property vlnv $core]"
puts {TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGER_GUI_READY}
