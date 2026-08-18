# Open the actual end-user Customize IP dialog for the packaged V2 core.

if {$argc != 5} {
    error "usage: open_v2_customize_ip_gui.tcl <project_dir> <project_name> <ip_repo> <vlnv> <module_name>"
}
set project_dir [file normalize [lindex $argv 0]]
set project_name [lindex $argv 1]
set ip_repo [file normalize [lindex $argv 2]]
set expected_vlnv [lindex $argv 3]
set module_name [lindex $argv 4]
if {![file isdirectory $ip_repo]} {
    error "V2 IP repository does not exist: $ip_repo"
}

file mkdir $project_dir
set project_path [file join $project_dir "$project_name.xpr"]
if {[file exists $project_path]} {
    open_project $project_path
} else {
    create_project $project_name $project_dir -part xc7z020clg484-2
    set_property target_language VHDL [current_project]
    set_property simulator_language Mixed [current_project]
}
set_property ip_repo_paths [list $ip_repo] [current_project]
update_ip_catalog
if {[llength [get_ipdefs -all -quiet $expected_vlnv]] != 1} {
    error "Expected one V2 IP definition for $expected_vlnv"
}
set customization [get_ips -quiet $module_name]
if {[llength $customization] == 0} {
    puts {TDC_GPX_LIDAR_CTRL_V2_CUSTOMIZE_IP_GUI_CREATE}
    start_ip_gui -vlnv $expected_vlnv -module_name $module_name
} elseif {[llength $customization] == 1} {
    puts {TDC_GPX_LIDAR_CTRL_V2_CUSTOMIZE_IP_GUI_REOPEN}
    start_ip_gui -ip $customization
} else {
    error "Expected at most one V2 preview customization: $customization"
}
puts {TDC_GPX_LIDAR_CTRL_V2_CUSTOMIZE_IP_GUI_REQUESTED}
