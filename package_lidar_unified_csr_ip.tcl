# Build the source-only LiDAR unified CSR packaged IP.
#
# Vivado batch:
#   vivado -mode batch -source HDL/package_lidar_unified_csr_ip.tcl
# Optional arguments:
#   0: alternate package directory
#   1: alternate temporary package-work directory

set script_dir [file normalize [file dirname [info script]]]
set ip_root [file normalize [file join $script_dir ../..]]
if {[llength $argv] > 0} {
    set package_dir [file normalize [lindex $argv 0]]
} else {
    set package_dir [file normalize \
        [file join $ip_root lidar_unified_csr ip_repo]]
}

set csr32_dir [file join $ip_root my_axil_csr32 HDL]
set csr_files [list \
    my_axil_csr32_pkg.vhd \
    axil_fsm_32.vhd \
    axil_ctrl_regs_32.vhd \
    axil_stat_regs_32.vhd \
    axil_intr_32.vhd \
    my_axil_csr32_top.vhd]
set rtl_files [list \
    lidar_unified_csr_pkg.vhd \
    lidar_unified_csr_top.vhd \
    lidar_unified_csr_ip_top.vhd]
set rtl_dir [file join $script_dir system_integration rtl]
set xgui_source [file join $script_dir system_integration \
    lidar_unified_csr_xgui.tcl]

set interface_sources [list \
    [file join $ip_root motor_decoder HDL motor_decoder_unified_csr.xml] \
    [file join $ip_root motor_decoder HDL motor_decoder_unified_csr_rtl.xml] \
    [file join $ip_root laser_ctrl HDL laser_ctrl_unified_csr.xml] \
    [file join $ip_root laser_ctrl HDL laser_ctrl_unified_csr_rtl.xml] \
    [file join $ip_root echo_receiver HDL echo_receiver_unified_csr.xml] \
    [file join $ip_root echo_receiver HDL echo_receiver_unified_csr_rtl.xml] \
    [file join $script_dir interfaces tdc_gpx_unified_csr.xml] \
    [file join $script_dir interfaces tdc_gpx_unified_csr_rtl.xml]]

foreach filename $csr_files {
    set source [file join $csr32_dir $filename]
    if {![file exists $source]} {
        error "Canonical CSR32 source is missing: $source"
    }
}
foreach filename $rtl_files {
    set source [file join $rtl_dir $filename]
    if {![file exists $source]} {
        error "Unified CSR RTL source is missing: $source"
    }
}
foreach source [concat [list $xgui_source] $interface_sources] {
    if {![file exists $source]} {
        error "Required package source is missing: $source"
    }
}

set package_src [file join $package_dir src]
set package_csr [file join $package_src csr]
set package_xgui [file join $package_dir xgui]
set package_interfaces [file join $package_dir interfaces]
file mkdir $package_dir
foreach generated_dir [list $package_src $package_xgui $package_interfaces] {
    if {[file exists $generated_dir]} {
        file delete -force $generated_dir
    }
    file mkdir $generated_dir
}
file mkdir $package_csr

foreach filename $csr_files {
    file copy -force [file join $csr32_dir $filename] \
        [file join $package_csr $filename]
}
foreach filename $rtl_files {
    file copy -force [file join $rtl_dir $filename] \
        [file join $package_src $filename]
}
file copy -force $xgui_source \
    [file join $package_xgui lidar_unified_csr_v1_0.tcl]
foreach source $interface_sources {
    file copy -force $source \
        [file join $package_interfaces [file tail $source]]
}

# Avoid a damaged per-user Tcl Store masking the installation appinit package.
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

if {[llength $argv] > 1} {
    set package_work [file normalize [lindex $argv 1]]
} elseif {[info exists ::env(TEMP)]} {
    set package_work [file normalize \
        [file join $::env(TEMP) lidar_unified_csr_package_work]]
} else {
    set package_work [file normalize \
        [file join C:/tmp lidar_unified_csr_package_work]]
}
if {[file exists $package_work]} {
    file delete -force $package_work
}
create_project -force lidar_unified_csr_package $package_work \
    -part xc7z020clg484-2
set_property target_language VHDL [current_project]
set_property simulator_language Mixed [current_project]

set opened_bus_definitions {}
set opened_abstraction_definitions {}
foreach basename {
    motor_decoder_unified_csr
    laser_ctrl_unified_csr
    echo_receiver_unified_csr
    tdc_gpx_unified_csr
} {
    lappend opened_bus_definitions [ipx::open_bus_definition \
        [file join $package_interfaces ${basename}.xml]]
}
foreach basename {
    motor_decoder_unified_csr_rtl
    laser_ctrl_unified_csr_rtl
    echo_receiver_unified_csr_rtl
    tdc_gpx_unified_csr_rtl
} {
    lappend opened_abstraction_definitions \
        [ipx::open_abstraction_definition \
            [file join $package_interfaces ${basename}.xml]]
}
foreach object $opened_bus_definitions {
    if {$object eq {}} {
        error {Failed to open a unified CSR bus definition}
    }
    puts "LIDAR_UNIFIED_CSR_OPEN_BUS=[get_property vlnv $object]"
}
foreach object $opened_abstraction_definitions {
    if {$object eq {}} {
        error {Failed to open a unified CSR abstraction definition}
    }
    puts "LIDAR_UNIFIED_CSR_OPEN_ABSTRACTION=[get_property vlnv $object]"
}

set ordered_sources {}
foreach filename $csr_files {
    lappend ordered_sources [file join $package_csr $filename]
}
foreach filename $rtl_files {
    lappend ordered_sources [file join $package_src $filename]
}
foreach source $ordered_sources {
    add_files -fileset sources_1 -norecurse $source
    set_property file_type VHDL [get_files $source]
}
set_property top lidar_unified_csr_ip_top [get_filesets sources_1]
update_compile_order -fileset sources_1

ipx::package_project \
    -root_dir $package_dir \
    -vendor victek.co.kr \
    -library my_ip \
    -name lidar_unified_csr \
    -version 1.0 \
    -taxonomy {/VictekIP} \
    -set_current true \
    -force
set core [ipx::current_core]
if {$core eq {}} {
    error {Unable to obtain the packaged LiDAR unified CSR core object}
}

# package_project creates a default GUI. Restore the canonical read-only GUI.
file copy -force $xgui_source \
    [file join $package_xgui lidar_unified_csr_v1_0.tcl]

set_property vendor victek.co.kr $core
set_property library my_ip $core
set_property name lidar_unified_csr $core
set_property version 1.0 $core
set_property taxonomy {/VictekIP} $core
set_property display_name {LiDAR Unified CSR (32 Control / 32 Status)} $core
set_property description \
    {Single AXI4-Lite control-plane owner for Motor, Laser, Echo Receiver, and TDC-GPX with coherent status, epoch handshakes, and 32 interrupt causes.} \
    $core
set_property core_revision 3 $core
set_property supported_families {zynq Production} $core

foreach {name display description value} {
    g_VERSION_WORD {ABI Version Word} \
        {Read-only STAT0 ABI identity word.} {"01001100000000010000000000000000"}
    g_CAPABILITY_WORD {Capability Word} \
        {Read-only STAT1 capability word.} {"00000001000001000001101011111111"}
} {
    set user_param [ipx::get_user_parameters -quiet $name -of_objects $core]
    set hdl_param [ipx::get_hdl_parameters -quiet $name -of_objects $core]
    if {[llength $user_param] != 1 || [llength $hdl_param] != 1} {
        error "Expected one user and HDL parameter named $name"
    }
    set_property display_name $display $user_param
    set_property description $description $user_param
    set_property value_format bitString $user_param
    set_property value_bit_string_length 32 $user_param
    set_property value $value $user_param
    set_property value_resolve_type user $user_param
    set_property data_type {std_logic_vector(31 downto 0)} $hdl_param
    set_property value_format bitString $hdl_param
    set_property value_bit_string_length 32 $hdl_param
    set_property value $value $hdl_param
    set_property value_resolve_type generated $hdl_param
}

proc ucsr_set_bus_parameter {interface name value} {
    set parameter [ipx::get_bus_parameters -quiet $name \
        -of_objects $interface]
    if {[llength $parameter] == 0} {
        set parameter [ipx::add_bus_parameter $name $interface]
    } elseif {[llength $parameter] != 1} {
        error "Expected at most one $name parameter"
    }
    set_property value $value $parameter
}

proc ucsr_ensure_scalar_interface {
    core name logical_name physical_name bus_vlnv abstraction_vlnv mode
} {
    set interface [ipx::get_bus_interfaces -quiet $name -of_objects $core]
    if {[llength $interface] == 0} {
        set interface [ipx::add_bus_interface $name $core]
    } elseif {[llength $interface] != 1} {
        error "Expected at most one interface named $name"
    }
    set_property interface_mode $mode $interface
    set_property bus_type_vlnv $bus_vlnv $interface
    set_property abstraction_type_vlnv $abstraction_vlnv $interface
    set port_map [ipx::get_port_maps -quiet $logical_name \
        -of_objects $interface]
    if {[llength $port_map] == 0} {
        set port_map [ipx::add_port_map $logical_name $interface]
    } elseif {[llength $port_map] != 1} {
        error "Expected at most one $logical_name map on $name"
    }
    set_property logical_name $logical_name $port_map
    set_property physical_name $physical_name $port_map
    return $interface
}

proc ucsr_add_master_interface {
    core name display bus_vlnv abstraction_vlnv mappings
} {
    set old [ipx::get_bus_interfaces -quiet $name -of_objects $core]
    if {[llength $old] == 1} {
        ipx::remove_bus_interface $name $core
    } elseif {[llength $old] != 0} {
        error "Expected at most one existing interface named $name"
    }
    set interface [ipx::add_bus_interface $name $core]
    set_property display_name $display $interface
    set_property interface_mode master $interface
    set_property bus_type_vlnv $bus_vlnv $interface
    set_property abstraction_type_vlnv $abstraction_vlnv $interface
    set_property connection_required true $interface
    foreach {logical physical} $mappings {
        set port_map [ipx::add_port_map $logical $interface]
        set_property logical_name $logical $port_map
        set_property physical_name $physical $port_map
    }
    return $interface
}

set motor_if [ucsr_add_master_interface $core motor_unified_csr \
    {Motor Decoder Unified CSR} \
    {victek.co.kr:interface:motor_decoder_unified_csr:1.0} \
    {victek.co.kr:interface:motor_decoder_unified_csr_rtl:1.0} {
        SYS_CTRL o_motor_sys_ctrl
        SYS_CFG_APPLY o_motor_sys_cfg_apply
        MOTOR_CFG o_motor_cfg
        MOTOR_TICKS_LO o_motor_ticks_lo
        MOTOR_SCHED_LATENCY o_motor_sched_latency
        MOTOR_Z_PARAM o_motor_z_param
        MOTOR_FACE_INDEX o_motor_face_index
        MOTOR_FACE_GEOMETRY o_motor_face_geometry
        MOTOR_STATUS i_motor_status
        MOTOR_FACE_STATUS i_motor_face_geometry
        MOTOR_CFG_STATUS i_motor_cfg_status
        MOTOR_QUAD_INVALID i_motor_quad_invalid
        MOTOR_AXIS_DROP i_motor_axis_drop
        MOTOR_REV_PERIOD i_motor_rev_period
        IRQ_CAUSE i_motor_irq_cause
    }]

set laser_if [ucsr_add_master_interface $core laser_unified_csr \
    {Laser Controller Unified CSR} \
    {victek.co.kr:interface:laser_ctrl_unified_csr:1.0} \
    {victek.co.kr:interface:laser_ctrl_unified_csr_rtl:1.0} {
        SYS_CTRL o_laser_sys_ctrl
        SYS_CFG_APPLY o_laser_sys_cfg_apply
        LASER_FIRE_CFG o_laser_fire_cfg
        LASER_ROUNDTRIP o_laser_roundtrip
        LASER_TDC_WIDTH o_laser_tdc_width
        LASER_SIM_DELAY o_laser_sim_delay
        LASER_SCHED0 o_laser_sched0
        LASER_SCHED1 o_laser_sched1
        LASER_SCHED2 o_laser_sched2
        LASER_STATUS i_laser_status
        LASER_ENCODER_TO_FIRE i_laser_encoder_to_fire
        LASER_FIRE_DONE i_laser_fire_done
        LASER_FIRE_TO_TDC i_laser_fire_to_tdc
        LASER_METRIC_FLAGS i_laser_metric_flags
        LASER_FRAME_COUNT i_laser_frame_count
        LASER_TIMEOUT_COUNT i_laser_timeout_count
        CFG_EPOCH_ACCEPTED i_laser_cfg_epoch_accepted
        RESET_EPOCH_ACCEPTED i_laser_reset_epoch_accepted
        CFG_BUSY i_laser_cfg_busy
        CFG_REJECT i_laser_cfg_reject
        CFG_VALID i_laser_cfg_valid
        IRQ_CAUSE i_laser_irq_cause
    }]

set echo_if [ucsr_add_master_interface $core echo_unified_csr \
    {Echo Receiver Unified CSR} \
    {victek.co.kr:interface:echo_receiver_unified_csr:1.0} \
    {victek.co.kr:interface:echo_receiver_unified_csr_rtl:1.0} {
        SYS_CTRL o_echo_sys_ctrl
        ECHO_DELAY_CMD o_echo_delay_cmd
        ECHO_DELAY_DATA o_echo_delay_data
        ECHO_RISE_MASK i_echo_rise_mask
        ECHO_FALL_MASK i_echo_fall_mask
        ECHO_STATUS i_echo_status
        ECHO_DELAY_READBACK i_echo_delay_readback
        RESET_EPOCH_ACCEPTED i_echo_reset_epoch_accepted
        IRQ_CAUSE i_echo_irq_cause
    }]

set tdc_if [ucsr_add_master_interface $core tdc_unified_csr \
    {TDC-GPX Unified CSR} \
    {victek.co.kr:interface:tdc_gpx_unified_csr:1.0} \
    {victek.co.kr:interface:tdc_gpx_unified_csr_rtl:1.0} {
        SYS_CTRL o_tdc_sys_ctrl
        SYS_CFG_APPLY o_tdc_sys_cfg_apply
        TDC_BUS_TIMING o_tdc_bus_timing
        TDC_START_OFFSET o_tdc_start_offset
        TDC_CFG_REG7 o_tdc_cfg_reg7
        TDC_IMAGE_CMD o_tdc_image_cmd
        TDC_IMAGE_DATA o_tdc_image_data
        TDC_SCAN_CFG o_tdc_scan_cfg
        TDC_PIPELINE_MAIN o_tdc_pipeline_main
        TDC_RANGE_COLS o_tdc_range_cols
        TDC_AUX_CMD o_tdc_aux_cmd
        TDC_CHIP0_RESULT i_tdc_chip0_result
        TDC_CHIP1_RESULT i_tdc_chip1_result
        TDC_CHIP2_RESULT i_tdc_chip2_result
        TDC_CHIP3_RESULT i_tdc_chip3_result
        TDC_PIPELINE_STATUS i_tdc_pipeline_status
        TDC_STATUS_EXT i_tdc_status_ext
        TDC_STATUS_EXT2 i_tdc_status_ext2
        CFG_EPOCH_ACCEPTED i_tdc_cfg_epoch_accepted
        RESET_EPOCH_ACCEPTED i_tdc_reset_epoch_accepted
        CFG_BUSY i_tdc_cfg_busy
        CFG_REJECT i_tdc_cfg_reject
        CFG_VALID i_tdc_cfg_valid
        CMD_EPOCH_ACCEPTED i_tdc_cmd_epoch_accepted
        CMD_BUSY i_tdc_cmd_busy
        COMMAND_REJECT i_tdc_command_reject
        IMAGE_EPOCH_ACCEPTED i_tdc_image_epoch_accepted
        IMAGE_REJECT i_tdc_image_reject
        IMAGE_SELECTED_DATA i_tdc_image_selected_data
        IRQ_CAUSE i_tdc_irq_cause
    }]

set axi_if [ipx::get_bus_interfaces -quiet s_axi_csr -of_objects $core]
if {[llength $axi_if] != 1} {
    error {Expected one inferred s_axi_csr AXI4-Lite interface}
}
set clock_if [ucsr_ensure_scalar_interface $core s_axi_csr_aclk CLK \
    s_axi_csr_aclk {xilinx.com:signal:clock:1.0} \
    {xilinx.com:signal:clock_rtl:1.0} slave]
set reset_if [ucsr_ensure_scalar_interface $core s_axi_csr_aresetn RST \
    s_axi_csr_aresetn {xilinx.com:signal:reset:1.0} \
    {xilinx.com:signal:reset_rtl:1.0} slave]
set irq_if [ucsr_ensure_scalar_interface $core o_irq INTERRUPT o_irq \
    {xilinx.com:signal:interrupt:1.0} \
    {xilinx.com:signal:interrupt_rtl:1.0} master]

set freq [ipx::get_bus_parameters -quiet FREQ_HZ -of_objects $clock_if]
if {[llength $freq] == 1} {
    ipx::remove_bus_parameter FREQ_HZ $clock_if
}
ucsr_set_bus_parameter $clock_if ASSOCIATED_BUSIF \
    {s_axi_csr:motor_unified_csr:laser_unified_csr:echo_unified_csr:tdc_unified_csr}
ucsr_set_bus_parameter $clock_if ASSOCIATED_RESET s_axi_csr_aresetn
ucsr_set_bus_parameter $reset_if POLARITY ACTIVE_LOW
ucsr_set_bus_parameter $irq_if SENSITIVITY EDGE_RISING

# Rebuild source views deterministically. No child XCI or external path is
# allowed in the distributable package.
foreach group_name {
    xilinx_anylanguagesynthesis
    xilinx_anylanguagebehavioralsimulation
} {
    set file_group [ipx::get_file_groups $group_name -of_objects $core]
    if {[llength $file_group] != 1} {
        error "Expected one file group named $group_name"
    }
    ipx::remove_all_file $file_group
    set_property language VHDL $file_group
    set_property component_subcores {} $file_group
    foreach filename $csr_files {
        set packaged [ipx::add_file src/csr/$filename $file_group]
        set_property type vhdlSource $packaged
    }
    foreach filename $rtl_files {
        set packaged [ipx::add_file src/$filename $file_group]
        set_property type vhdlSource $packaged
    }
}

set xgui_group [ipx::get_file_groups xilinx_xpgui -of_objects $core]
if {[llength $xgui_group] != 1} {
    error {Expected one XGUI file group}
}
ipx::remove_all_file $xgui_group
set xgui_file [ipx::add_file \
    xgui/lidar_unified_csr_v1_0.tcl $xgui_group]
set_property type tclSource $xgui_file
set_property xgui_version 2 $xgui_file

set testbench_group [ipx::get_file_groups -quiet xilinx_testbench \
    -of_objects $core]
if {[llength $testbench_group] == 1} {
    ipx::remove_all_file $testbench_group
}

foreach file_group [ipx::get_file_groups -of_objects $core] {
    foreach packaged [ipx::get_files -of_objects $file_group] {
        set name [get_property name $packaged]
        if {[string match "../*" $name] || [string match "*.xci" $name]} {
            error "Forbidden packaged dependency: $name"
        }
    }
}

foreach required_port {
    s_axi_csr_awaddr s_axi_csr_araddr o_irq
    o_motor_sys_ctrl o_laser_sys_ctrl o_echo_sys_ctrl o_tdc_sys_ctrl
    i_echo_irq_cause i_tdc_cmd_epoch_accepted i_tdc_image_epoch_accepted
} {
    if {[llength [ipx::get_ports -quiet $required_port \
            -of_objects $core]] != 1} {
        error "Required packaged port is missing: $required_port"
    }
}

# Register the just-built repository before integrity checking so Vivado can
# resolve the four custom bus-definition XML files by VLNV.
set_property ip_repo_paths [list $package_dir] [current_project]
update_ip_catalog -rebuild

ipx::update_checksums $core
set drc_result [ipx::check_integrity -verbose $core]
puts "LIDAR_UNIFIED_CSR_IP_PACKAGER_DRC=$drc_result"
ipx::save_core $core
ipx::unload_core $core
foreach object $opened_abstraction_definitions {
    ipx::unload_abstraction_definition $object
}
foreach object $opened_bus_definitions {
    ipx::unload_bus_definition $object
}
close_project

puts "LIDAR_UNIFIED_CSR_IP_COMPONENT=[file join $package_dir component.xml]"
puts {LIDAR_UNIFIED_CSR_IP_PACKAGE_PASS}
exit
