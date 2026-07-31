# Static packaged-IP and XGUI contract checks for tdc_gpx_top.

set script_dir [file normalize [file dirname [info script]]]
set hdl_dir [file normalize [file join $script_dir ..]]
if {[llength $argv] > 0} {
    set package_dir [file normalize [lindex $argv 0]]
} else {
    set package_dir [file normalize [file join $hdl_dir .. ip_repo]]
}
set component [file join $package_dir component.xml]
set xgui [file join $package_dir xgui tdc_gpx_top_v1_0.tcl]
set unified_bus_xml [file join $package_dir interfaces \
    tdc_gpx_unified_csr.xml]
set unified_rtl_xml [file join $package_dir interfaces \
    tdc_gpx_unified_csr_rtl.xml]

foreach required [list $component $xgui $unified_bus_xml $unified_rtl_xml] {
    if {![file exists $required]} {
        error "Packaged-IP artifact is missing: $required"
    }
}

proc tdc_read_binary {path} {
    set handle [open $path r]
    fconfigure $handle -translation binary
    set data [read $handle]
    close $handle
    return $data
}

proc tdc_require_file_equal {canonical packaged label} {
    if {![file exists $canonical] || ![file exists $packaged]} {
        error "$label source is missing: $canonical or $packaged"
    }
    if {[tdc_read_binary $canonical] ne [tdc_read_binary $packaged]} {
        error "$label package copy is stale: $packaged"
    }
}

set ip_root [file normalize [file join $hdl_dir ../..]]
source [file join $script_dir tdc_gpx_rtl_manifest.tcl]
source [file join $script_dir tdc_gpx_csr_source_manifest.tcl]
set canonical_rtl [lsearch -all -inline -not -exact \
    [tdc_gpx_rtl_manifest] px_utility_pkg.vhd]
foreach filename $canonical_rtl {
    tdc_require_file_equal [file join $hdl_dir $filename] \
        [file join $package_dir src $filename] "TDC-GPX RTL $filename"
}
foreach canonical [tdc_gpx_csr_source_manifest $ip_root] {
    set filename [file tail $canonical]
    tdc_require_file_equal $canonical \
        [file join $package_dir src csr $filename] "CSR source $filename"
}
tdc_require_file_equal [file join $hdl_dir tdc_gpx_xgui.tcl] $xgui \
    {TDC-GPX XGUI}
tdc_require_file_equal [file join $hdl_dir interfaces \
    tdc_gpx_unified_csr.xml] $unified_bus_xml \
    {TDC-GPX unified bus definition}
tdc_require_file_equal [file join $hdl_dir interfaces \
    tdc_gpx_unified_csr_rtl.xml] $unified_rtl_xml \
    {TDC-GPX unified abstraction definition}
puts {TDC_GPX_IP_PACKAGE_SOURCE_SYNC_PASS}

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
set unified_bus_definition [ipx::open_bus_definition $unified_bus_xml]
set unified_abstraction_definition \
    [ipx::open_abstraction_definition $unified_rtl_xml]
set core [ipx::open_core $component]

if {[get_property core_revision $core] != 7} {
    error "Unexpected TDC-GPX core revision: [get_property core_revision $core]"
}

proc require_one {objects label} {
    if {[llength $objects] != 1} {
        error "Expected one $label, found [llength $objects]"
    }
    return [lindex $objects 0]
}

foreach interface_name {
    s_axi s_axi_pipe o_m_axis o_m_axis_fall
    i_axis_aclk i_tdc_clk s_axi_aclk s_axi_aresetn o_irq o_irq_pipe
    tdc_unified_csr i_unified_cfg_clk i_unified_cfg_rst_n
} {
    require_one [ipx::get_bus_interfaces -quiet $interface_name \
        -of_objects $core] "bus interface $interface_name"
}

foreach {interface_name ready_port} {
    o_m_axis      i_m_axis_tready
    o_m_axis_fall i_m_axis_fall_tready
} {
    set interface [require_one [ipx::get_bus_interfaces $interface_name \
        -of_objects $core] "interface $interface_name"]
    if {[get_property interface_mode $interface] ne {master}} {
        error "$interface_name must remain an AXI4-Stream master"
    }
    set ready_map [require_one [ipx::get_port_maps TREADY \
        -of_objects $interface] "$interface_name TREADY map"]
    if {[get_property physical_name $ready_map] ne $ready_port} {
        error "$interface_name TREADY must map to $ready_port"
    }
    set physical [require_one [ipx::get_ports $ready_port -of_objects $core] \
        "physical READY port $ready_port"]
    if {[get_property direction $physical] ne {in}} {
        error "$ready_port must remain an input"
    }
}

foreach {clock_name expected_busif} {
    i_axis_aclk {o_m_axis:o_m_axis_fall}
    s_axi_aclk {s_axi:s_axi_pipe}
    i_unified_cfg_clk {tdc_unified_csr}
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
set unified_clock [ipx::get_bus_interfaces i_unified_cfg_clk \
    -of_objects $core]
if {[llength [ipx::get_bus_parameters -quiet FREQ_HZ \
        -of_objects $unified_clock]] != 0} {
    error {i_unified_cfg_clk must inherit the parent unified CSR clock}
}

set local_csr [require_one [ipx::get_user_parameters g_ENABLE_LOCAL_CSR \
    -of_objects $core] {user parameter g_ENABLE_LOCAL_CSR}]
set local_csr_model [require_one \
    [ipx::get_hdl_parameters g_ENABLE_LOCAL_CSR -of_objects $core] \
    {HDL parameter g_ENABLE_LOCAL_CSR}]
if {[get_property value $local_csr] ne {true} ||
    [get_property value_format $local_csr] ne {bool}} {
    error {g_ENABLE_LOCAL_CSR user default/type must be true/bool}
}
if {[get_property value $local_csr_model] ne {true} ||
    [get_property data_type $local_csr_model] ne {boolean}} {
    error {g_ENABLE_LOCAL_CSR HDL default/type must be true/boolean}
}

proc require_mode_dependency {object mode} {
    if {[get_property enablement_resolve_type $object] ne {dependent}} {
        error "[get_property name $object] must use dependent enablement"
    }
    set dependency [get_property enablement_dependency $object]
    if {[string first {g_ENABLE_LOCAL_CSR} $dependency] < 0 ||
        [string first "= $mode" $dependency] < 0} {
        error "[get_property name $object] has wrong mode dependency: $dependency"
    }
}

foreach interface_name {
    s_axi s_axi_pipe s_axi_aclk s_axi_aresetn o_irq o_irq_pipe
} {
    require_mode_dependency [ipx::get_bus_interfaces $interface_name \
        -of_objects $core] true
}
foreach interface_name {
    tdc_unified_csr i_unified_cfg_clk i_unified_cfg_rst_n
} {
    require_mode_dependency [ipx::get_bus_interfaces $interface_name \
        -of_objects $core] false
}

foreach {interface_names mode} [list \
        {s_axi s_axi_pipe s_axi_aclk s_axi_aresetn o_irq o_irq_pipe} true \
        {tdc_unified_csr i_unified_cfg_clk i_unified_cfg_rst_n} false] {
    foreach interface_name $interface_names {
        set interface [ipx::get_bus_interfaces $interface_name \
            -of_objects $core]
        foreach port_map [ipx::get_port_maps -of_objects $interface] {
            set physical_name [get_property physical_name $port_map]
            require_mode_dependency [ipx::get_ports $physical_name \
                -of_objects $core] $mode
        }
    }
}

set unified_interface [ipx::get_bus_interfaces tdc_unified_csr \
    -of_objects $core]
if {[get_property interface_mode $unified_interface] ne {slave} ||
    [get_property bus_type_vlnv $unified_interface] ne
        {victek.co.kr:interface:tdc_gpx_unified_csr:1.0} ||
    [get_property abstraction_type_vlnv $unified_interface] ne
        {victek.co.kr:interface:tdc_gpx_unified_csr_rtl:1.0}} {
    error {tdc_unified_csr type or mode mismatch}
}
foreach logical_name {
    SYS_CTRL SYS_CFG_APPLY TDC_BUS_TIMING TDC_START_OFFSET TDC_CFG_REG7
    TDC_IMAGE_CMD TDC_IMAGE_DATA TDC_SCAN_CFG TDC_PIPELINE_MAIN
    TDC_RANGE_COLS TDC_AUX_CMD TDC_CHIP0_RESULT TDC_CHIP1_RESULT
    TDC_CHIP2_RESULT TDC_CHIP3_RESULT TDC_PIPELINE_STATUS TDC_STATUS_EXT
    TDC_STATUS_EXT2 CFG_EPOCH_ACCEPTED RESET_EPOCH_ACCEPTED CFG_BUSY
    CFG_REJECT CFG_VALID CMD_EPOCH_ACCEPTED CMD_BUSY COMMAND_REJECT
    IMAGE_EPOCH_ACCEPTED IMAGE_REJECT IMAGE_SELECTED_DATA IRQ_CAUSE
} {
    require_one [ipx::get_port_maps $logical_name \
        -of_objects $unified_interface] \
        "tdc_unified_csr logical port $logical_name"
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

foreach {label parameter} [list \
        user [require_one \
            [ipx::get_user_parameters g_DRAIN_MARGIN_TIME_NS \
                -of_objects $core] \
            {user parameter g_DRAIN_MARGIN_TIME_NS}] \
        model [require_one \
            [ipx::get_hdl_parameters g_DRAIN_MARGIN_TIME_NS \
                -of_objects $core] \
            {HDL parameter g_DRAIN_MARGIN_TIME_NS}]] {
    if {[get_property value $parameter] != 6000} {
        error "g_DRAIN_MARGIN_TIME_NS $label default must remain 6000 ns"
    }
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
    src/tdc_gpx_unified_cdc_snapshot.vhd
    src/tdc_gpx_unified_csr_adapter.vhd
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
        {::tdc_gpx_xgui::validate_clocks 150 200 SYNC} false \
    read_capture_default \
        {::tdc_gpx_xgui::validate_time_limit 25 200 253 {Bus read capture window}} true \
    read_capture_user_max \
        {::tdc_gpx_xgui::validate_time_limit 140 200 253 {Bus read capture window}} true \
    read_capture_capacity_overflow \
        {::tdc_gpx_xgui::validate_time_limit 1270 200 253 {Bus read capture window}} false] {
    lassign [uplevel #0 $command] valid message
    if {$valid ne $expected} {
        error "$label expected valid=$expected, got valid=$valid: $message"
    }
    puts "TDC_GPX_XGUI_CASE_PASS name=$label valid=$valid"
}

ipx::unload_core $core
ipx::unload_abstraction_definition $unified_abstraction_definition
ipx::unload_bus_definition $unified_bus_definition

proc require_bd_present {objects label} {
    if {[llength $objects] != 1} {
        error "Expected visible $label, found [llength $objects]"
    }
}
proc require_bd_absent {objects label} {
    if {[llength $objects] != 0} {
        error "Expected hidden $label, found [llength $objects]"
    }
}

set_property ip_repo_paths [list $package_dir] [current_project]
update_ip_catalog -rebuild
set vlnv victek.co.kr:my_ip:tdc_gpx_top:1.0
require_one [get_ipdefs -all $vlnv] {packaged TDC-GPX IP definition}
create_bd_design csr_mode_visibility
set tdc [create_bd_cell -type ip -vlnv $vlnv tdc]

foreach interface_name {s_axi s_axi_pipe} {
    require_bd_present [get_bd_intf_pins -quiet tdc/$interface_name] \
        "local-mode interface $interface_name"
}
foreach pin_name {s_axi_aclk s_axi_aresetn o_irq o_irq_pipe} {
    require_bd_present [get_bd_pins -quiet tdc/$pin_name] \
        "local-mode pin $pin_name"
}
require_bd_absent [get_bd_intf_pins -quiet tdc/tdc_unified_csr] \
    {unified interface in local mode}
foreach pin_name {
    i_unified_cfg_clk i_unified_cfg_rst_n i_unified_sys_ctrl
    o_unified_tdc_pipeline_status
} {
    require_bd_absent [get_bd_pins -quiet tdc/$pin_name] \
        "unified pin $pin_name in local mode"
}

set_property CONFIG.g_ENABLE_LOCAL_CSR false $tdc
foreach interface_name {s_axi s_axi_pipe} {
    require_bd_absent [get_bd_intf_pins -quiet tdc/$interface_name] \
        "local interface $interface_name in unified mode"
}
foreach pin_name {s_axi_aclk s_axi_aresetn o_irq o_irq_pipe} {
    require_bd_absent [get_bd_pins -quiet tdc/$pin_name] \
        "local pin $pin_name in unified mode"
}
require_bd_present [get_bd_intf_pins -quiet tdc/tdc_unified_csr] \
    {unified interface in unified mode}
foreach pin_name {
    i_unified_cfg_clk i_unified_cfg_rst_n i_unified_sys_ctrl
    o_unified_tdc_pipeline_status
} {
    require_bd_present [get_bd_pins -quiet tdc/$pin_name] \
        "unified pin $pin_name in unified mode"
}
puts {TDC_GPX_IP_PACKAGE_BD_MODE_PASS}

close_project
puts {TDC_GPX_IP_PACKAGE_STATIC_PASS}
exit
