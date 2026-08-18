# Create a persistent Vivado "Edit Packaged IP" project for the canonical V3
# component.xml. The project is disposable; the packaged IP remains the source
# consumed by downstream IP catalogs.

if {$argc != 4} {
    error "usage: create_v3_ip_packager_project.tcl <component.xml> <edit_dir> <project_name> <expected_vlnv>"
}

set component [file normalize [lindex $argv 0]]
set edit_dir [file normalize [lindex $argv 1]]
set project_name [lindex $argv 2]
set expected_vlnv [lindex $argv 3]

if {![file exists $component]} {
    error "V3 component.xml does not exist: $component"
}
if {[file extension $component] ne {.xml}} {
    error "Expected a component.xml input: $component"
}
if {[string trim $project_name] eq {}} {
    error {IP Packager project name must not be empty}
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
# edit_ip_in_project is an IP catalog command and requires an active Vivado
# project context even though it creates the actual edit project itself.
create_project -in_memory ${project_name}_bootstrap -part xc7z020clg484-2
set ip_repo [file dirname [file dirname $component]]
set_property ip_repo_paths [list $ip_repo] [current_project]
update_ip_catalog
ipx::edit_ip_in_project -upgrade true -name $project_name \
    -directory $edit_dir $component

set project [current_project]
if {$project eq {}} {
    error {Vivado did not create an IP Packager edit project}
}
set_property ip_repo_paths [list $ip_repo] $project
update_ip_catalog
set core [ipx::current_core]
if {$core eq {}} {
    error {Vivado did not open the packaged V3 core}
}
if {[get_property vlnv $core] ne $expected_vlnv} {
    error "Unexpected V3 core in edit project: [get_property vlnv $core]"
}

set xpr [file join [get_property directory $project] \
    "[get_property name $project].xpr"]
if {![file exists $xpr]} {
    error "Vivado did not create the expected edit project: $xpr"
}

puts "TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGER_PROJECT=$xpr"
puts {TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGER_CREATE_PASS}
close_project
exit 0
