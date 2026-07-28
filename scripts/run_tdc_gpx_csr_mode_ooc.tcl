# Synthesize the packaged TDC-GPX sources in both control-plane ownership
# modes. Keeping both runs in this Vivado process avoids nested run launchers.
# Optional argv0: package directory. Optional argv1: output directory.

set script_dir [file normalize [file dirname [info script]]]
set hdl_dir [file normalize [file join $script_dir ..]]
if {$argc > 0} {
    set package_dir [file normalize [lindex $argv 0]]
} else {
    set package_dir [file normalize [file join $hdl_dir .. ip_repo]]
}
if {$argc > 1} {
    set out_dir [file normalize [lindex $argv 1]]
} else {
    set out_dir [file normalize [file join $hdl_dir tmp \
        tdc_gpx_csr_mode_ooc]]
}
file mkdir $out_dir

if {![file exists [file join $package_dir component.xml]]} {
    error "Packaged TDC-GPX component is missing: $package_dir"
}

set csr_files [list \
    axil_fsm.vhd \
    axil_ctrl_regs.vhd \
    axil_stat_regs.vhd \
    axil_intr.vhd \
    my_axil_csr_top.vhd \
    my_axil_csr32_pkg.vhd \
    axil_fsm_32.vhd \
    axil_ctrl_regs_32.vhd \
    axil_stat_regs_32.vhd \
    axil_intr_32.vhd \
    my_axil_csr32_top.vhd]

source [file join $script_dir tdc_gpx_rtl_manifest.tcl]
set packaged_sources {}
foreach file_name $csr_files {
    lappend packaged_sources [file join $package_dir src csr $file_name]
}
set packaged_rtl [lsearch -all -inline -not -exact \
    [tdc_gpx_rtl_manifest] px_utility_pkg.vhd]
foreach file_name $packaged_rtl {
    lappend packaged_sources [file join $package_dir src $file_name]
}
foreach source_file $packaged_sources {
    if {![file exists $source_file]} {
        error "Packaged source is missing: $source_file"
    }
}

# A damaged per-user Tcl Store must not prevent ordinary project creation.
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

proc require_mode_hierarchy {mode_name local_csr out_dir} {
    set black_boxes [get_cells -quiet -hier -filter {IS_BLACKBOX == 1}]
    if {[llength $black_boxes] != 0} {
        error "CSR mode $mode_name contains black boxes: $black_boxes"
    }

    set local_pipeline [get_cells -quiet -hier *u_csr_pipeline*]
    set local_chip [get_cells -quiet -hier *u_csr_chip*]
    set unified_adapter [get_cells -quiet -hier *u_unified_adapter*]
    if {$local_csr} {
        if {[llength $local_pipeline] == 0 || [llength $local_chip] == 0} {
            error {Local mode does not contain both legacy CSR owners}
        }
        if {[llength $unified_adapter] != 0} {
            error {Local mode still contains the unified CSR adapter}
        }
    } else {
        if {[llength $local_pipeline] != 0 || [llength $local_chip] != 0} {
            error {Unified mode still contains a legacy local CSR owner}
        }
        if {[llength $unified_adapter] == 0} {
            error {Unified mode does not contain the unified CSR adapter}
        }
    }

    report_utilization -hierarchical -hierarchical_depth 6 \
        -file [file join $out_dir utilization_${mode_name}.rpt]
}

foreach {mode_name local_csr} {local true unified false} {
    create_project -in_memory -part xc7z020clg484-2
    set_property target_language VHDL [current_project]
    set_property simulator_language Mixed [current_project]
    set_property default_lib xil_defaultlib [current_project]

    foreach source_file $packaged_sources {
        read_vhdl -vhdl2008 -library xil_defaultlib $source_file
    }
    update_compile_order -fileset sources_1

    set generics [list \
        "g_ENABLE_LOCAL_CSR=$local_csr" \
        {g_OUTPUT_WIDTH=32} \
        {g_NUM_CHIPS=4} \
        {g_PRESENT_CHIP_MASK=4'b1111} \
        {g_RISE_CHIP_MASK=4'b0011} \
        {g_FALL_CHIP_MASK=4'b1100} \
        {g_MAX_STOPS_PER_CHIP=8} \
        {g_MAX_HITS_PER_STOP=7} \
        {g_AXIS_CLK_MHZ=150} \
        {g_TDC_CLK_MHZ=200} \
        {g_STREAM_CLK_MODE=ASYNC}]

    synth_design \
        -top tdc_gpx_top \
        -part xc7z020clg484-2 \
        -mode out_of_context \
        -flatten_hierarchy none \
        -generic $generics

    require_mode_hierarchy $mode_name $local_csr $out_dir
    puts "TDC_GPX_CSR_MODE_OOC_PASS mode=$mode_name axis_mhz=150 tdc_mhz=200"
    close_project
}

puts {TDC_GPX_CSR_MODE_OOC_MATRIX_PASS modes=local,unified}
exit
