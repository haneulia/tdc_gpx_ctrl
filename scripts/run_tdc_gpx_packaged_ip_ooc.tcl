# Instantiate and synthesize the packaged tdc_gpx_top IP at 150/200 MHz.

set script_dir [file normalize [file dirname [info script]]]
set hdl_dir [file normalize [file join $script_dir ..]]
set package_dir [file normalize [file join $hdl_dir .. ip_repo]]
if {$argc > 0} {
    set out_dir [file normalize [lindex $argv 0]]
} else {
    set out_dir [file join $hdl_dir signoff_results packaged_ip_ooc_150_200]
}
file mkdir $out_dir

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

create_project -force tdc_gpx_packaged_ooc \
    [file join $out_dir project] -part xc7z020clg484-2
set_property target_language VHDL [current_project]
set_property simulator_language Mixed [current_project]
set_property ip_repo_paths [list $package_dir] [current_project]
update_ip_catalog -rebuild

set vlnv victek.co.kr:my_ip:tdc_gpx_top:1.0
if {[llength [get_ipdefs -all $vlnv]] != 1} {
    error "Packaged TDC-GPX VLNV is unavailable: $vlnv"
}
create_ip -vlnv $vlnv -module_name tdc_gpx_packaged_0
set_property -dict [list \
    CONFIG.g_OUTPUT_WIDTH 32 \
    CONFIG.g_NUM_CHIPS 4 \
    CONFIG.g_PRESENT_CHIP_MASK {"1111"} \
    CONFIG.g_RISE_CHIP_MASK {"0011"} \
    CONFIG.g_FALL_CHIP_MASK {"1100"} \
    CONFIG.g_MAX_STOPS_PER_CHIP 8 \
    CONFIG.g_MAX_HITS_PER_STOP 7 \
    CONFIG.g_AXIS_CLK_MHZ 150 \
    CONFIG.g_TDC_CLK_MHZ 200 \
    CONFIG.g_STREAM_CLK_MODE ASYNC] \
    [get_ips tdc_gpx_packaged_0]

set packaged_ip [get_ips tdc_gpx_packaged_0]
generate_target {instantiation_template synthesis} $packaged_ip
create_ip_run $packaged_ip

set synth_run [get_runs tdc_gpx_packaged_0_synth_1]
if {[llength $synth_run] != 1} {
    error {Packaged-IP synthesis run was not created}
}
launch_runs $synth_run -jobs 2
wait_on_run $synth_run

set progress [get_property PROGRESS $synth_run]
set status [get_property STATUS $synth_run]
puts "TDC_GPX_PACKAGED_IP_RUN status=$status progress=$progress"
if {$progress ne {100%} || [string first {Complete} $status] < 0} {
    error "Packaged-IP synthesis did not complete: $status ($progress)"
}

open_run $synth_run
report_utilization -hierarchical \
    -file [file join $out_dir utilization_hier.rpt]
set black_boxes [get_cells -quiet -hier -filter {IS_BLACKBOX == 1}]
if {[llength $black_boxes] != 0} {
    error "Packaged-IP synthesis contains black boxes: $black_boxes"
}

set io_d [get_ports io_tdc_d]
set csn [get_ports o_tdc_csn]
if {[llength $io_d] != 112} {
    error "Default io_tdc_d width is not 112 bits"
}
if {[llength $csn] != 4} {
    error "Default o_tdc_csn width is not four bits"
}

puts {TDC_GPX_PACKAGED_IP_OOC_PASS axis_mhz=150 tdc_mhz=200 width=32 chips=4}
close_project
exit
