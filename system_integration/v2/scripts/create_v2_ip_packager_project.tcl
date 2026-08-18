# Create a disposable Vivado Edit Packaged IP project for the canonical V2 IP.

if {$argc != 4} {
    error "usage: create_v2_ip_packager_project.tcl <component.xml> <edit_dir> <project_name> <expected_vlnv>"
}

set component [file normalize [lindex $argv 0]]
set edit_dir [file normalize [lindex $argv 1]]
set project_name [lindex $argv 2]
set expected_vlnv [lindex $argv 3]
if {![file exists $component]} {
    error "V2 component.xml does not exist: $component"
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

file mkdir $edit_dir
create_project -in_memory ${project_name}_bootstrap -part xc7z020clg484-2
set ip_repo [file dirname [file dirname $component]]
set_property ip_repo_paths [list $ip_repo] [current_project]
update_ip_catalog
ipx::edit_ip_in_project -upgrade true -name $project_name \
    -directory $edit_dir $component

set project [current_project]
set core [ipx::current_core]
if {$project eq {} || $core eq {}} {
    error {Vivado did not create and open the V2 IP Packager project}
}
if {[get_property vlnv $core] ne $expected_vlnv} {
    error "Unexpected V2 core in edit project: [get_property vlnv $core]"
}
set xpr [file join [get_property directory $project] \
    "[get_property name $project].xpr"]
if {![file exists $xpr]} {
    error "Vivado did not create the expected edit project: $xpr"
}

puts "TDC_GPX_LIDAR_CTRL_V2_IP_PACKAGER_PROJECT=$xpr"
puts {TDC_GPX_LIDAR_CTRL_V2_IP_PACKAGER_CREATE_PASS}
close_project
exit 0
