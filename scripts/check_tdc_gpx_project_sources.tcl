# Verify that the legacy tdc_gpx_ctrl.xpr references only current sibling and
# source-level CSR files. This check does not modify the project.

set script_dir [file normalize [file dirname [info script]]]
set hdl_dir [file normalize [file join $script_dir ..]]
set project_path [file normalize [file join $hdl_dir .. tdc_gpx_ctrl.xpr]]
set ip_root [file normalize [file join $hdl_dir .. ..]]

source [file join $script_dir tdc_gpx_csr_source_manifest.tcl]
source [file join $script_dir tdc_gpx_integration_source_manifest.tcl]

open_project $project_path

set legacy_names {
    tdc_gpx_axil_csr_pipeline.xci
    tdc_gpx_axil_csr32_chip.xci
    laser_ctl_axil_csr.xci
    my_axil_csr_0.xci
    echo_receiver_axil_csr32.xci
    enc_position_counter.vhd
    enc_tick_counter.vhd
    enc_fractional_scheduler.vhd
    laser_ctrl_math_pkg.vhd
    laser_ctrl_pkg.vhd
    laser_ctrl_echo_capture.vhd
    tdc_gpx_stop_cfg_decode.vhd
}

set legacy_found [list]
set missing_found [list]
foreach project_file [get_files -all -quiet] {
    set file_name [get_property NAME $project_file]
    if {[lsearch -exact $legacy_names [file tail $file_name]] >= 0} {
        lappend legacy_found $file_name
    }
    if {![file exists $file_name]} {
        lappend missing_found $file_name
    }
}

if {[llength $legacy_found] != 0} {
    error "Legacy project references remain: $legacy_found"
}
if {[llength $missing_found] != 0} {
    error "Project contains missing file references: $missing_found"
}

set expected_sources [concat \
    [tdc_gpx_csr_source_manifest $ip_root] \
    [tdc_gpx_integration_rtl_manifest $ip_root] \
    [tdc_gpx_integration_tb_manifest $ip_root]]
foreach source $expected_sources {
    if {[llength [get_files -all -quiet $source]] == 0} {
        error "Canonical project source is not registered: $source"
    }
}

puts "TDC_GPX_PROJECT_REGISTERED_FILES=[llength [get_files -all -quiet]]"
puts "TDC_GPX_PROJECT_CANONICAL_FILES=[llength $expected_sources]"
puts {TDC_GPX_PROJECT_SOURCE_CHECK_PASS}
close_project
exit
