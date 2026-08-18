# Synthesize the two V3 packaging profiles at the supported 32/64-bit widths.
#
# argv 0: V3 IP repository
# argv 1: short-path Vivado work directory
# argv 2: optional selector: ALL, EMBEDDED32, EMBEDDED64, HLS32, HLS64

if {[llength $argv] < 2} {
    error {Usage: check_v3_dual_package_ooc.tcl <ip_repo> <work_dir> ?selector?}
}
set ip_repo [file normalize [lindex $argv 0]]
set work_dir [file normalize [lindex $argv 1]]
set selector ALL
if {[llength $argv] > 2} {
    set selector [string toupper [lindex $argv 2]]
}

set profiles {
    {EMBEDDED32 embedded_w32 victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3:3.0 32}
    {EMBEDDED64 embedded_w64 victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3:3.0 64}
    {HLS32 hls_ip_w32 victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3_hls_ip:3.0 32}
    {HLS64 hls_ip_w64 victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3_hls_ip:3.0 64}
}
set selected_profiles {}
foreach profile $profiles {
    lassign $profile profile_name module_name vlnv width
    if {$selector eq {ALL} || $selector eq $profile_name} {
        lappend selected_profiles $profile
    }
}
if {[llength $selected_profiles] == 0} {
    error "Unknown V3 dual-package OOC selector: $selector"
}

# The workstation's per-user Tcl Store may be stale or partially installed.
# Pin batch verification to the Tcl Store shipped with this Vivado release.
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

if {[file exists $work_dir]} {
    file delete -force $work_dir
}
create_project -force v3_dual_package_ooc $work_dir -part xc7z020clg484-2
set_property target_language VHDL [current_project]
set_property simulator_language Mixed [current_project]
set_property ip_repo_paths [list $ip_repo] [current_project]
update_ip_catalog
set ip_dir [file join $work_dir ip]
file mkdir $ip_dir

foreach profile $selected_profiles {
    lassign $profile profile_name module_name vlnv width
    if {[llength [get_ipdefs -all -quiet $vlnv]] != 1} {
        error "V3 OOC IP definition is missing: $vlnv"
    }
    create_ip -vlnv $vlnv -module_name $module_name \
        -dir $ip_dir
    set ip [get_ips $module_name]
    set_property -dict [list CONFIG.G_OUTPUT_WIDTH $width] $ip
    set parent_xci [get_property IP_FILE $ip]
    set_property generate_synth_checkpoint false [get_files $parent_xci]
    generate_target all $ip
    update_compile_order -fileset sources_1

    # Run synthesis in this Vivado process. launch_runs/synth_ip delegates to
    # xcd.exe on Windows; a workstation-local xcd crash must not block the
    # deterministic IP sign-off path.
    synth_design -top $module_name -part xc7z020clg484-2 \
        -mode out_of_context
    set blackboxes [get_cells -hier -quiet -filter {IS_BLACKBOX == 1}]
    if {[llength $blackboxes] != 0} {
        error "V3 OOC synthesis left black boxes: profile=$profile_name cells=$blackboxes"
    }
    set dcp [file join $work_dir ${module_name}.dcp]
    write_checkpoint -force $dcp
    report_utilization -file [file join $work_dir ${module_name}_utilization.rpt]
    puts "LIDAR_V3_DUAL_PACKAGE_OOC_PASS profile=$profile_name width=$width dcp=$dcp"
    close_design
}

puts "TDC_GPX_LIDAR_CTRL_V3_DUAL_PACKAGE_OOC_CHECK_PASS selector=$selector"
exit 0
