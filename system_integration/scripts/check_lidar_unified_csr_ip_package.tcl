# Static package, XGUI, source-sync, and child-interface checks for the LiDAR
# unified CSR IP.

set script_dir [file normalize [file dirname [info script]]]
set hdl_dir [file normalize [file join $script_dir ../..]]
set ip_root [file normalize [file join $hdl_dir ../..]]
if {[llength $argv] > 0} {
    set package_dir [file normalize [lindex $argv 0]]
} else {
    set package_dir [file normalize \
        [file join $ip_root lidar_unified_csr ip_repo]]
}

set component [file join $package_dir component.xml]
set xgui [file join $package_dir xgui lidar_unified_csr_v1_0.tcl]
foreach required [list $component $xgui] {
    if {![file exists $required]} {
        error "Unified CSR package artifact is missing: $required"
    }
}

proc read_binary {path} {
    set handle [open $path r]
    fconfigure $handle -translation binary
    set data [read $handle]
    close $handle
    return $data
}

proc require_file_equal {canonical packaged label} {
    if {![file exists $canonical] || ![file exists $packaged]} {
        error "$label source is missing: $canonical or $packaged"
    }
    if {[read_binary $canonical] ne [read_binary $packaged]} {
        error "$label package copy is stale: $packaged"
    }
}

set csr32_dir [file join $ip_root my_axil_csr32 HDL]
set compared_file_count 0
foreach filename {
    my_axil_csr32_pkg.vhd axil_fsm_32.vhd axil_ctrl_regs_32.vhd
    axil_stat_regs_32.vhd axil_intr_32.vhd my_axil_csr32_top.vhd
} {
    require_file_equal [file join $csr32_dir $filename] \
        [file join $package_dir src csr $filename] "CSR32 $filename"
    incr compared_file_count
}
set rtl_dir [file join $hdl_dir system_integration rtl]
foreach filename {
    lidar_unified_csr_pkg.vhd lidar_unified_csr_top.vhd
    lidar_unified_csr_ip_top.vhd
} {
    require_file_equal [file join $rtl_dir $filename] \
        [file join $package_dir src $filename] "Unified RTL $filename"
    incr compared_file_count
}
require_file_equal [file join $hdl_dir system_integration \
    lidar_unified_csr_xgui.tcl] $xgui {Unified CSR XGUI}
incr compared_file_count

foreach {canonical packaged_name} [list \
    [file join $ip_root motor_decoder HDL \
        motor_decoder_unified_csr.xml] motor_decoder_unified_csr.xml \
    [file join $ip_root motor_decoder HDL \
        motor_decoder_unified_csr_rtl.xml] motor_decoder_unified_csr_rtl.xml \
    [file join $ip_root laser_ctrl HDL \
        laser_ctrl_unified_csr.xml] laser_ctrl_unified_csr.xml \
    [file join $ip_root laser_ctrl HDL \
        laser_ctrl_unified_csr_rtl.xml] laser_ctrl_unified_csr_rtl.xml \
    [file join $ip_root echo_receiver HDL \
        echo_receiver_unified_csr.xml] echo_receiver_unified_csr.xml \
    [file join $ip_root echo_receiver HDL \
        echo_receiver_unified_csr_rtl.xml] echo_receiver_unified_csr_rtl.xml \
    [file join $hdl_dir interfaces \
        tdc_gpx_unified_csr.xml] tdc_gpx_unified_csr.xml \
    [file join $hdl_dir interfaces \
        tdc_gpx_unified_csr_rtl.xml] tdc_gpx_unified_csr_rtl.xml] {
    require_file_equal $canonical \
        [file join $package_dir interfaces $packaged_name] \
        "Interface $packaged_name"
    incr compared_file_count
}
if {$compared_file_count != 18} {
    error "Unexpected unified CSR package source count: $compared_file_count"
}
puts "LIDAR_UNIFIED_CSR_IP_SOURCE_FILE_COUNT=$compared_file_count"
puts {LIDAR_UNIFIED_CSR_IP_SOURCE_SYNC_PASS}

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

create_project -in_memory lidar_unified_csr_ip_check \
    -part xc7z020clg484-2
set opened_bus_definitions {}
set opened_abstraction_definitions {}
foreach basename {
    motor_decoder_unified_csr laser_ctrl_unified_csr
    echo_receiver_unified_csr tdc_gpx_unified_csr
} {
    lappend opened_bus_definitions [ipx::open_bus_definition \
        [file join $package_dir interfaces ${basename}.xml]]
}
foreach basename {
    motor_decoder_unified_csr_rtl laser_ctrl_unified_csr_rtl
    echo_receiver_unified_csr_rtl tdc_gpx_unified_csr_rtl
} {
    lappend opened_abstraction_definitions \
        [ipx::open_abstraction_definition \
            [file join $package_dir interfaces ${basename}.xml]]
}
set core [ipx::open_core $component]

proc require_one {objects label} {
    if {[llength $objects] != 1} {
        error "Expected one $label, found [llength $objects]"
    }
    return [lindex $objects 0]
}

if {[get_property vlnv $core] ne \
        {victek.co.kr:my_ip:lidar_unified_csr:1.0}} {
    error "Unexpected unified CSR VLNV: [get_property vlnv $core]"
}
if {[get_property core_revision $core] != 3} {
    error "Unexpected unified CSR core revision: [get_property core_revision $core]"
}
foreach {name expected_value} {
    g_VERSION_WORD {"01001100000000010000000000000000"}
    g_CAPABILITY_WORD {"00000001000001000001101011111111"}
} {
    set user_param [require_one [ipx::get_user_parameters -quiet $name \
        -of_objects $core] "user parameter $name"]
    set hdl_param [require_one [ipx::get_hdl_parameters -quiet $name \
        -of_objects $core] "HDL parameter $name"]
    foreach parameter [list $user_param $hdl_param] {
        if {[get_property value_format $parameter] ne {bitString} ||
            [get_property value_bit_string_length $parameter] != 32 ||
            [get_property value $parameter] ne $expected_value} {
            error "$name must remain a quoted 32-bit VHDL bit string"
        }
    }
}
foreach interface_name {
    s_axi_csr s_axi_csr_aclk s_axi_csr_aresetn o_irq
    motor_unified_csr laser_unified_csr echo_unified_csr tdc_unified_csr
} {
    require_one [ipx::get_bus_interfaces -quiet $interface_name \
        -of_objects $core] "bus interface $interface_name"
}
foreach interface_name {
    motor_unified_csr laser_unified_csr echo_unified_csr tdc_unified_csr
} {
    set interface [ipx::get_bus_interfaces $interface_name -of_objects $core]
    if {[get_property interface_mode $interface] ne {master}} {
        error "$interface_name must be a Master interface"
    }
    if {![get_property connection_required $interface]} {
        error "$interface_name must require a connection"
    }
}

set clock_if [ipx::get_bus_interfaces s_axi_csr_aclk -of_objects $core]
set associated [require_one [ipx::get_bus_parameters ASSOCIATED_BUSIF \
    -of_objects $clock_if] {unified CSR clock ASSOCIATED_BUSIF}]
set expected_associated \
    {s_axi_csr:motor_unified_csr:laser_unified_csr:echo_unified_csr:tdc_unified_csr}
if {[get_property value $associated] ne $expected_associated} {
    error "Unified CSR clock association mismatch: [get_property value $associated]"
}

foreach {port_name width} {
    s_axi_csr_awaddr 9
    s_axi_csr_araddr 9
    i_echo_irq_cause 5
    i_tdc_cmd_epoch_accepted 8
    i_tdc_image_epoch_accepted 8
} {
    set port [require_one [ipx::get_ports -quiet $port_name \
        -of_objects $core] "port $port_name"]
    set actual [expr {[get_property size_left $port] -
        [get_property size_right $port] + 1}]
    if {$actual != $width} {
        error "$port_name width is $actual, expected $width"
    }
}

set packaged_files {}
foreach file_group [ipx::get_file_groups -of_objects $core] {
    foreach packaged [ipx::get_files -of_objects $file_group] {
        set name [get_property name $packaged]
        lappend packaged_files $name
        if {[string match "../*" $name] || [string match "*.xci" $name]} {
            error "Forbidden packaged dependency: $name"
        }
    }
}
foreach required_file {
    src/csr/my_axil_csr32_top.vhd
    src/lidar_unified_csr_pkg.vhd
    src/lidar_unified_csr_top.vhd
    src/lidar_unified_csr_ip_top.vhd
    xgui/lidar_unified_csr_v1_0.tcl
} {
    if {$required_file ni $packaged_files} {
        error "Required packaged file is absent: $required_file"
    }
}

set component_text [read_binary $component]
if {[string first {XGUI_VERSION_2} $component_text] < 0} {
    error {Unified CSR XGUI is not marked version 2}
}
set xgui_text [read_binary $xgui]
foreach required_text {
    {set_property enabled false $widget}
    {set_property enabled false ${PARAM_VALUE.g_VERSION_WORD}}
    {set_property enabled false ${PARAM_VALUE.g_CAPABILITY_WORD}}
    {32 Control, 32 Status}
    {IRQ4..7 Motor}
} {
    if {[string first $required_text $xgui_text] < 0} {
        error "Required XGUI contract is missing: $required_text"
    }
}
puts {LIDAR_UNIFIED_CSR_IP_STATIC_PASS}

ipx::unload_core $core
foreach object $opened_abstraction_definitions {
    ipx::unload_abstraction_definition $object
}
foreach object $opened_bus_definitions {
    ipx::unload_bus_definition $object
}

# Prove that all four custom Master interfaces connect directly to the
# currently packaged child Slave interfaces in IP Integrator.
set child_repos [list \
    [file join $ip_root motor_decoder ip_repo] \
    [file join $ip_root laser_ctrl ip_repo] \
    [file join $ip_root motor_laser_ctrl ip_repo] \
    [file join $ip_root echo_receiver ip_repo] \
    [file join $ip_root tdc_gpx_ctrl ip_repo]]
set_property ip_repo_paths [linsert $child_repos 0 $package_dir] \
    [current_project]
update_ip_catalog -rebuild

foreach vlnv {
    victek.co.kr:my_ip:lidar_unified_csr:1.0
    victek.co.kr:my_ip:motor_decoder_top:1.0
    victek.co.kr:my_ip:laser_ctrl_top:1.0
    victek.co.kr:my_ip:motor_laser_ctrl_top:1.0
    victek.co.kr:my_ip:echo_receiver_top:1.0
    victek.co.kr:my_ip:tdc_gpx_top:1.0
} {
    require_one [get_ipdefs -all $vlnv] "IP definition $vlnv"
}

create_bd_design ucsr_child_if
set csr [create_bd_cell -type ip \
    -vlnv victek.co.kr:my_ip:lidar_unified_csr:1.0 csr]
set motor [create_bd_cell -type ip \
    -vlnv victek.co.kr:my_ip:motor_decoder_top:1.0 motor]
set laser [create_bd_cell -type ip \
    -vlnv victek.co.kr:my_ip:laser_ctrl_top:1.0 laser]
set echo [create_bd_cell -type ip \
    -vlnv victek.co.kr:my_ip:echo_receiver_top:1.0 echo]
set tdc [create_bd_cell -type ip \
    -vlnv victek.co.kr:my_ip:tdc_gpx_top:1.0 tdc]
foreach child [list $motor $laser $echo $tdc] {
    set_property CONFIG.g_ENABLE_LOCAL_CSR false $child
}

foreach {master_pin slave_pin} {
    csr/motor_unified_csr motor/motor_unified_csr
    csr/laser_unified_csr laser/laser_unified_csr
    csr/echo_unified_csr echo/echo_unified_csr
    csr/tdc_unified_csr tdc/tdc_unified_csr
} {
    require_one [get_bd_intf_pins -quiet $master_pin] \
        "Master interface pin $master_pin"
    require_one [get_bd_intf_pins -quiet $slave_pin] \
        "Slave interface pin $slave_pin"
    connect_bd_intf_net [get_bd_intf_pins $master_pin] \
        [get_bd_intf_pins $slave_pin]
}
save_bd_design
puts {LIDAR_UNIFIED_CSR_IP_CHILD_INTERFACE_PASS}

# Repeat the bus contract with the combined Motor/Laser IP used by the parent
# project. The combined wrapper owns unique physical SYS_CTRL/APPLY inputs for
# each named interface, so connecting both buses cannot create multiple
# drivers on one HDL port.
create_bd_design ucsr_parent_if
set parent_csr [create_bd_cell -type ip \
    -vlnv victek.co.kr:my_ip:lidar_unified_csr:1.0 parent_csr]
set motor_laser [create_bd_cell -type ip \
    -vlnv victek.co.kr:my_ip:motor_laser_ctrl_top:1.0 motor_laser]
set parent_echo [create_bd_cell -type ip \
    -vlnv victek.co.kr:my_ip:echo_receiver_top:1.0 parent_echo]
set parent_tdc [create_bd_cell -type ip \
    -vlnv victek.co.kr:my_ip:tdc_gpx_top:1.0 parent_tdc]
foreach child [list $motor_laser $parent_echo $parent_tdc] {
    set_property CONFIG.g_ENABLE_LOCAL_CSR false $child
}

foreach {master_pin slave_pin} {
    parent_csr/motor_unified_csr motor_laser/motor_unified_csr
    parent_csr/laser_unified_csr motor_laser/laser_unified_csr
    parent_csr/echo_unified_csr parent_echo/echo_unified_csr
    parent_csr/tdc_unified_csr parent_tdc/tdc_unified_csr
} {
    require_one [get_bd_intf_pins -quiet $master_pin] \
        "Parent Master interface pin $master_pin"
    require_one [get_bd_intf_pins -quiet $slave_pin] \
        "Parent Slave interface pin $slave_pin"
    connect_bd_intf_net [get_bd_intf_pins $master_pin] \
        [get_bd_intf_pins $slave_pin]
}
foreach hidden_local {s_axi_motor s_axi_laser} {
    if {[llength [get_bd_intf_pins -quiet \
            motor_laser/$hidden_local]] != 0} {
        error "Combined IP local interface remains visible: $hidden_local"
    }
}
save_bd_design
puts {LIDAR_UNIFIED_CSR_IP_PARENT_INTERFACE_PASS}

close_project
puts {LIDAR_UNIFIED_CSR_IP_PACKAGE_CHECK_PASS}
exit
