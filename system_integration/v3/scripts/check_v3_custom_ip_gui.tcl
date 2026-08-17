# Verify that the writable Parent uses the current packaged V3 IP and that the
# instance can be customized without an Upgrade IP operation.

if {$argc != 2} {
    error "usage: check_v3_custom_ip_gui.tcl <project.xpr> <v3_ip_repo>"
}

set project_path [file normalize [lindex $argv 0]]
set ip_repo_path [file normalize [lindex $argv 1]]

if {![file exists $project_path]} {
    error "Writable Parent project does not exist: $project_path"
}
if {![file exists [file join $ip_repo_path tdc_gpx_lidar_ctrl_v3_3_0 component.xml]]} {
    error "V3 packaged IP does not exist below: $ip_repo_path"
}

open_project $project_path
set_property ip_repo_paths [list $ip_repo_path] [current_project]
update_ip_catalog -rebuild

set v3_ips [get_ips -quiet *tdc_gpx_lidar_ctrl_v3*]
if {[llength $v3_ips] != 1} {
    error "Expected one V3 IP instance, found [llength $v3_ips]: $v3_ips"
}

set v3_ip [lindex $v3_ips 0]
set ipdef [get_property IPDEF $v3_ip]
set is_locked [get_property IS_LOCKED $v3_ip]
set upgrade_versions [get_property UPGRADE_VERSIONS $v3_ip]

if {$ipdef ne "victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3:3.0"} {
    error "Unexpected V3 IP definition: $ipdef"
}
if {$is_locked} {
    error "V3 IP is locked. Recreate the writable Parent from the current IP package."
}
if {[string length [string trim $upgrade_versions]] != 0} {
    error "V3 IP still advertises an upgrade target: $upgrade_versions"
}

puts "LIDAR_V3_CUSTOM_IP_GUI_IPDEF=$ipdef"
puts "LIDAR_V3_CUSTOM_IP_GUI_LOCKED=$is_locked"
puts "LIDAR_V3_CUSTOM_IP_GUI_UPGRADE_VERSIONS=NONE"
puts "LIDAR_V3_CUSTOM_IP_GUI_VALIDATE_PASS"
close_project
