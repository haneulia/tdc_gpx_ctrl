# Static packaged-IP and XGUI contract checks for tdc_gpx_top.

set script_dir [file normalize [file dirname [info script]]]
set hdl_dir [file normalize [file join $script_dir ..]]
set package_dir [file normalize [file join $hdl_dir .. ip_repo]]
set component [file join $package_dir component.xml]
set xgui [file join $package_dir xgui tdc_gpx_top_v1_0.tcl]

foreach required [list $component $xgui] {
    if {![file exists $required]} {
        error "Packaged-IP artifact is missing: $required"
    }
}

# Avoid the damaged per-user Tcl Store on this workstation.
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

create_project -in_memory tdc_gpx_ip_check -part xc7z020clg484-2
set core [ipx::open_core $component]

proc require_one {objects label} {
    if {[llength $objects] != 1} {
        error "Expected one $label, found [llength $objects]"
    }
    return [lindex $objects 0]
}

foreach interface_name {
    s_axi s_axi_pipe o_m_axis o_m_axis_fall
    i_axis_aclk i_tdc_clk s_axi_aclk o_irq o_irq_pipe
} {
    require_one [ipx::get_bus_interfaces -quiet $interface_name \
        -of_objects $core] "bus interface $interface_name"
}

foreach {clock_name expected_busif} {
    i_axis_aclk {o_m_axis:o_m_axis_fall}
    s_axi_aclk {s_axi:s_axi_pipe}
} {
    set clock [require_one [ipx::get_bus_interfaces $clock_name \
        -of_objects $core] "clock $clock_name"]
    set associated [require_one [ipx::get_bus_parameters ASSOCIATED_BUSIF \
        -of_objects $clock] "$clock_name ASSOCIATED_BUSIF"]
    if {[get_property value $associated] ne $expected_busif} {
        error "$clock_name ASSOCIATED_BUSIF mismatch"
    }
}

foreach {interface_name generic_name} {
    o_m_axis g_AXIS_CLK_MHZ
    o_m_axis_fall g_AXIS_CLK_MHZ
    i_axis_aclk g_AXIS_CLK_MHZ
    i_tdc_clk g_TDC_CLK_MHZ
} {
    set interface [require_one [ipx::get_bus_interfaces $interface_name \
        -of_objects $core] "interface $interface_name"]
    set frequency [require_one [ipx::get_bus_parameters FREQ_HZ \
        -of_objects $interface] "$interface_name FREQ_HZ"]
    if {[get_property value_resolve_type $frequency] ne {dependent}} {
        error "$interface_name FREQ_HZ is not generic-dependent"
    }
    if {[string first $generic_name \
            [get_property value_dependency $frequency]] < 0} {
        error "$interface_name FREQ_HZ does not depend on $generic_name"
    }
}

set axi_clock [ipx::get_bus_interfaces s_axi_aclk -of_objects $core]
if {[llength [ipx::get_bus_parameters -quiet FREQ_HZ \
        -of_objects $axi_clock]] != 0} {
    error {s_axi_aclk must not publish a fixed CSR frequency}
}

set output_width [require_one [ipx::get_user_parameters g_OUTPUT_WIDTH \
    -of_objects $core] {user parameter g_OUTPUT_WIDTH}]
if {[get_property value $output_width] != 32} {
    error {g_OUTPUT_WIDTH default must remain 32 bits}
}
if {[get_property value_validation_type $output_width] ne {list}} {
    error {g_OUTPUT_WIDTH must use list validation}
}
if {[get_property value_validation_list $output_width] ne {32 64 128}} {
    error "g_OUTPUT_WIDTH choices must be exactly 32, 64, and 128: [get_property value_validation_list $output_width]"
}

foreach {port_name dependency_fragment} {
    io_tdc_d {* 28}
    o_tdc_adr {* 4}
    o_tdc_csn {g_NUM_CHIPS}
    o_m_axis_tdata {g_OUTPUT_WIDTH}
    o_m_axis_tkeep {g_OUTPUT_WIDTH}
    o_m_axis_tstrb {g_OUTPUT_WIDTH}
    o_m_axis_fall_tdata {g_OUTPUT_WIDTH}
    o_m_axis_fall_tkeep {g_OUTPUT_WIDTH}
    o_m_axis_fall_tstrb {g_OUTPUT_WIDTH}
} {
    set port [require_one [ipx::get_ports $port_name -of_objects $core] \
        "port $port_name"]
    set dependency [get_property size_left_dependency $port]
    if {[string first $dependency_fragment $dependency] < 0} {
        error "$port_name width dependency is incorrect: $dependency"
    }
}

set packaged_files {}
foreach file_group [ipx::get_file_groups -of_objects $core] {
    foreach packaged_file [ipx::get_files -of_objects $file_group] {
        lappend packaged_files [get_property name $packaged_file]
    }
}
foreach forbidden {.xci ../ tdc_gpx_axil_csr_pipeline tdc_gpx_axil_csr32_chip} {
    foreach packaged_file $packaged_files {
        if {[string first $forbidden $packaged_file] >= 0} {
            error "Forbidden packaged dependency: $packaged_file"
        }
    }
}
foreach required_file {
    src/csr/my_axil_csr_top.vhd
    src/csr/my_axil_csr32_top.vhd
    src/tdc_gpx_top.vhd
    xgui/tdc_gpx_top_v1_0.tcl
} {
    if {$required_file ni $packaged_files} {
        error "Required package file is missing: $required_file"
    }
}

source $xgui
foreach {label command expected} [list \
    output_width_32 \
        {::tdc_gpx_xgui::validate_output_width 32} true \
    output_width_64 \
        {::tdc_gpx_xgui::validate_output_width 64} true \
    output_width_128 \
        {::tdc_gpx_xgui::validate_output_width 128} true \
    output_width_invalid \
        {::tdc_gpx_xgui::validate_output_width 96} false \
    default_topology \
        {::tdc_gpx_xgui::validate_topology 4 1111 0011 1100} true \
    one_chip_dual_edge \
        {::tdc_gpx_xgui::validate_topology 1 0001 0001 0001} true \
    three_chip_2r1f \
        {::tdc_gpx_xgui::validate_topology 3 0111 0011 0100} true \
    invalid_chip_count \
        {::tdc_gpx_xgui::validate_topology 4 0111 0011 0100} false \
    invalid_more_fall \
        {::tdc_gpx_xgui::validate_topology 3 0111 0001 0110} false \
    valid_async_clock \
        {::tdc_gpx_xgui::validate_clocks 150 200 ASYNC} true \
    invalid_axis_faster \
        {::tdc_gpx_xgui::validate_clocks 200 150 ASYNC} false \
    invalid_sync_split \
        {::tdc_gpx_xgui::validate_clocks 150 200 SYNC} false] {
    lassign [uplevel #0 $command] valid message
    if {$valid ne $expected} {
        error "$label expected valid=$expected, got valid=$valid: $message"
    }
    puts "TDC_GPX_XGUI_CASE_PASS name=$label valid=$valid"
}

ipx::unload_core $core
close_project
puts {TDC_GPX_IP_PACKAGE_STATIC_PASS}
exit
