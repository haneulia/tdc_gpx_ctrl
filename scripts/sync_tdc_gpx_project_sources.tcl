# Replace generated CSR XCI references in tdc_gpx_ctrl.xpr with canonical
# source-level CSR8/CSR32 RTL. Generated directories are left on disk.

set script_dir [file normalize [file dirname [info script]]]
set hdl_dir [file normalize [file join $script_dir ..]]
set project_path [file normalize [file join $hdl_dir .. tdc_gpx_ctrl.xpr]]
set ip_root [file normalize [file join $hdl_dir .. ..]]

if {![file exists $project_path]} {
    error "TDC-GPX project not found: $project_path"
}

source [file join $script_dir tdc_gpx_csr_source_manifest.tcl]
source [file join $script_dir tdc_gpx_integration_source_manifest.tcl]
set csr_sources [tdc_gpx_csr_source_manifest $ip_root]
set sibling_rtl [tdc_gpx_integration_rtl_manifest $ip_root]
set sibling_tb [tdc_gpx_integration_tb_manifest $ip_root]
foreach source [concat $csr_sources $sibling_rtl $sibling_tb] {
    if {![file exists $source]} {
        error "Canonical integration source is missing: $source"
    }
}

open_project $project_path

set removed_count 0
foreach legacy_xci {
    tdc_gpx_axil_csr_pipeline.xci
    tdc_gpx_axil_csr32_chip.xci
    laser_ctl_axil_csr.xci
    my_axil_csr_0.xci
    echo_receiver_axil_csr32.xci
} {
    set matches [get_files -all -quiet *$legacy_xci]
    if {[llength $matches] > 0} {
        remove_files $matches
        incr removed_count [llength $matches]
    }
}

# These filenames belonged to pre-refactor sibling implementations. Remove
# project references only; no source file is deleted from disk.
foreach legacy_source {
    enc_position_counter.vhd
    enc_tick_counter.vhd
    enc_fractional_scheduler.vhd
    laser_ctrl_math_pkg.vhd
    laser_ctrl_pkg.vhd
    laser_ctrl_echo_capture.vhd
    tdc_gpx_stop_cfg_decode.vhd
} {
    set matches [list]
    foreach project_file [get_files -all -quiet] {
        if {[file tail [get_property NAME $project_file]] eq $legacy_source} {
            lappend matches $project_file
        }
    }
    if {[llength $matches] > 0} {
        remove_files $matches
        incr removed_count [llength $matches]
    }
}

set added_count 0
foreach source $csr_sources {
    if {[llength [get_files -all -quiet $source]] == 0} {
        add_files -fileset sources_1 -norecurse $source
        incr added_count
    }
    set_property file_type {VHDL 2008} [get_files -all $source]
}

foreach source $sibling_rtl {
    if {[llength [get_files -all -quiet $source]] == 0} {
        add_files -fileset sources_1 -norecurse $source
        incr added_count
    }
    set_property file_type {VHDL 2008} [get_files -all $source]
}

foreach source $sibling_tb {
    if {[llength [get_files -all -quiet $source]] == 0} {
        add_files -fileset sim_1 -norecurse $source
        incr added_count
    }
    set_property file_type {VHDL 2008} [get_files -all $source]
}

set_property top tdc_gpx_top [get_filesets sources_1]
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

set legacy_remaining [get_files -all -quiet -filter {
    NAME =~ "*tdc_gpx_axil_csr_pipeline.xci" ||
    NAME =~ "*tdc_gpx_axil_csr32_chip.xci" ||
    NAME =~ "*laser_ctl_axil_csr.xci" ||
    NAME =~ "*my_axil_csr_0.xci" ||
    NAME =~ "*echo_receiver_axil_csr32.xci"
}]
if {[llength $legacy_remaining] != 0} {
    error "Legacy TDC-GPX CSR XCI references remain: $legacy_remaining"
}

close_project

puts "TDC_GPX_PROJECT_CSR_XCI_REMOVED=$removed_count"
puts "TDC_GPX_PROJECT_CSR_SOURCES_ADDED=$added_count"
puts {TDC_GPX_PROJECT_SOURCE_SYNC_PASS}
exit
