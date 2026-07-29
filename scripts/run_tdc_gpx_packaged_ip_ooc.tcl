# Configure the packaged tdc_gpx_top IP and synthesize the exact packaged
# source set at 150/200 MHz for every supported output width.  Synthesis runs
# in the current Vivado process so the gate does not depend on the Windows
# rundef.js child-run launcher.

set script_dir [file normalize [file dirname [info script]]]
set hdl_dir [file normalize [file join $script_dir ..]]
set package_dir [file normalize [file join $hdl_dir .. ip_repo]]
if {$argc > 0} {
    set out_dir [file normalize [lindex $argv 0]]
} else {
    set out_dir [file join $hdl_dir signoff_results sessions \
        packaged_ip_ooc_width_matrix_150_200]
}
set output_widths {32 64 128}
if {$argc > 1} {
    set requested_width [lindex $argv 1]
    if {$requested_width ni $output_widths} {
        error "Output width must be 32, 64, or 128: $requested_width"
    }
    set output_widths [list $requested_width]
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

set catalog_project [file join $out_dir catalog_project]
create_project -force tdc_gpx_packaged_catalog \
    $catalog_project -part xc7z020clg484-2
set_property target_language VHDL [current_project]
set_property simulator_language Mixed [current_project]
set_property ip_repo_paths [list $package_dir] [current_project]
update_ip_catalog -rebuild

set vlnv victek.co.kr:my_ip:tdc_gpx_top:1.0
if {[llength [get_ipdefs -all $vlnv]] != 1} {
    error "Packaged TDC-GPX VLNV is unavailable: $vlnv"
}
foreach width $output_widths {
    set module_name tdc_gpx_packaged_w${width}
    create_ip -vlnv $vlnv -module_name $module_name
    set packaged_ip [get_ips $module_name]
    set_property -dict [list \
        CONFIG.g_OUTPUT_WIDTH $width \
        CONFIG.g_NUM_CHIPS 4 \
        CONFIG.g_PRESENT_CHIP_MASK {"1111"} \
        CONFIG.g_RISE_CHIP_MASK {"0011"} \
        CONFIG.g_FALL_CHIP_MASK {"1100"} \
        CONFIG.g_MAX_STOPS_PER_CHIP 8 \
        CONFIG.g_MAX_HITS_PER_STOP 7 \
        CONFIG.g_AXIS_CLK_MHZ 150 \
        CONFIG.g_TDC_CLK_MHZ 200 \
        CONFIG.g_STREAM_CLK_MODE ASYNC] $packaged_ip
    generate_target instantiation_template $packaged_ip
    if {[get_property CONFIG.g_OUTPUT_WIDTH $packaged_ip] != $width} {
        error "Packaged-IP did not retain output width $width"
    }
}
close_project

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

proc require_port_width {port_name expected_width} {
    set port [get_ports -quiet $port_name]
    if {[llength $port] != $expected_width} {
        error "$port_name width is [llength $port], expected $expected_width"
    }
}

foreach width $output_widths {
    create_project -in_memory -part xc7z020clg484-2
    set_property target_language VHDL [current_project]
    set_property simulator_language Mixed [current_project]
    set_property default_lib xil_defaultlib [current_project]
    foreach source_file $packaged_sources {
        read_vhdl -vhdl2008 -library xil_defaultlib $source_file
    }
    update_compile_order -fileset sources_1

    set generics [list \
        {g_ENABLE_LOCAL_CSR=true} \
        "g_OUTPUT_WIDTH=$width" \
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

    report_utilization -hierarchical \
        -file [file join $out_dir utilization_hier_w${width}.rpt]
    set black_boxes [get_cells -quiet -hier -filter {IS_BLACKBOX == 1}]
    if {[llength $black_boxes] != 0} {
        error "Packaged-IP width-$width synthesis contains black boxes: $black_boxes"
    }

    require_port_width io_tdc_d 112
    require_port_width o_tdc_csn 4
    foreach data_port {o_m_axis_tdata o_m_axis_fall_tdata} {
        require_port_width $data_port $width
    }
    foreach byte_port {
        o_m_axis_tkeep o_m_axis_tstrb
        o_m_axis_fall_tkeep o_m_axis_fall_tstrb
    } {
        require_port_width $byte_port [expr {$width / 8}]
    }

    puts "TDC_GPX_PACKAGED_IP_OOC_PASS axis_mhz=150 tdc_mhz=200 width=$width chips=4"
    close_project
}

puts "TDC_GPX_PACKAGED_IP_WIDTH_MATRIX_PASS widths=$output_widths"
exit
