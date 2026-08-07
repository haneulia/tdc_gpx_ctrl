# Verify the self-contained tdc_gpx_lidar_ctrl_v2:2.0 package, XGUI and
# coexistence with the board-proven tdc_gpx_top:1.0 package.

set script_dir [file normalize [file dirname [info script]]]
set v2_dir [file normalize [file join $script_dir ..]]
set hdl_root [file normalize [file join $v2_dir ../..]]
if {[llength $argv] > 0} {
    set package_dir [file normalize [lindex $argv 0]]
} else {
    set package_dir [file join $v2_dir ip_repo \
        tdc_gpx_lidar_ctrl_v2_2_0]
}

set component [file join $package_dir component.xml]
set packaged_xgui [file join $package_dir xgui \
    tdc_gpx_lidar_ctrl_v2_v2_0.tcl]
set packaged_guide [file join $package_dir doc PRODUCT_GUIDE_KO.md]
set canonical_xgui [file join $v2_dir ip_package \
    tdc_gpx_lidar_ctrl_v2_xgui.tcl]
set canonical_guide [file join $v2_dir ip_package PRODUCT_GUIDE_KO.md]
foreach required [list $component $packaged_xgui $packaged_guide \
        $canonical_xgui $canonical_guide] {
    if {![file exists $required]} {
        error "Required v2 package artifact is missing: $required"
    }
}

proc v2_read_binary {path} {
    set channel [open $path r]
    fconfigure $channel -translation binary
    set data [read $channel]
    close $channel
    return $data
}

proc v2_require_file_equal {canonical packaged label} {
    if {![file exists $canonical] || ![file exists $packaged]} {
        error "$label is missing: $canonical or $packaged"
    }
    if {[v2_read_binary $canonical] ne [v2_read_binary $packaged]} {
        error "$label package copy is stale: $packaged"
    }
}

source [file join $v2_dir ip_package v2_ip_package_manifest.tcl]
set entries [lidar_v2_ip_package_manifest $hdl_root]
if {[llength $entries] != 87} {
    error "Expected 87 transitive production sources, got [llength $entries]"
}
foreach entry $entries {
    lassign $entry canonical relative
    v2_require_file_equal $canonical [file join $package_dir src $relative] \
        "RTL $relative"
}
v2_require_file_equal $canonical_xgui $packaged_xgui {v2 XGUI}
v2_require_file_equal $canonical_guide $packaged_guide {v2 Product Guide}
if {[string first {C_MAX_CHIPS} [v2_read_binary $component]] >= 0} {
    error {component.xml exposes unresolved C_MAX_CHIPS in a public HDL type}
}
puts {LIDAR_V2_K010_SOURCE_SYNC_PASS files=89 rtl=87 xgui=1 guide=1}

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

create_project -in_memory lidar_v2_ip_check -part xc7z020clg484-2
set core [ipx::open_core $component]

proc v2_require_one {objects label} {
    if {[llength $objects] != 1} {
        error "Expected one $label, found [llength $objects]"
    }
    return [lindex $objects 0]
}

if {[get_property vlnv $core] ne \
        {victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v2:2.0}} {
    error "Unexpected v2 VLNV: [get_property vlnv $core]"
}
if {[get_property core_revision $core] != 1} {
    error "Unexpected v2 core revision: [get_property core_revision $core]"
}

set generic_names {
    G_CSR_CLK_MHZ G_PROC_CLK_MHZ G_TDC_CLK_MHZ G_STREAM_CLK_MODE
    G_NUM_CHIPS G_STOPS_PER_CHIP G_MAX_RETURNS_PER_STOP
    G_RISE_CAPABILITY_MASK G_FALL_CAPABILITY_MASK G_OUTPUT_WIDTH G_NUM_FACES
    G_ENABLE_ECHO_RECEIVER G_ENABLE_ECHO_SIMULATION G_OEN_MODE
    G_PHASE_TIMEOUT_US G_POWERUP_TIME_NS G_RECOVERY_TIME_NS
    G_ALU_PULSE_TIME_NS G_BUS_IDLE_STABLE_TIME_NS G_DRAIN_MARGIN_TIME_NS
}
if {[llength $generic_names] != 20} {
    error {Internal K0-10 generic audit list must contain 20 names}
}
foreach name $generic_names {
    v2_require_one [ipx::get_user_parameters -quiet $name -of_objects $core] \
        "user parameter $name"
    v2_require_one [ipx::get_hdl_parameters -quiet $name -of_objects $core] \
        "HDL parameter $name"
}

foreach {name default choices} {
    G_CSR_CLK_MHZ 100 {50 100 125 150 200}
    G_PROC_CLK_MHZ 150 {50 100 125 150 200}
    G_TDC_CLK_MHZ 200 {50 100 125 150 200}
    G_STREAM_CLK_MODE ASYNC {ASYNC SYNC}
    G_NUM_CHIPS 4 {1 2 3 4}
    G_STOPS_PER_CHIP 8 {1 2 3 4 5 6 7 8}
    G_MAX_RETURNS_PER_STOP 7 {1 2 3 4 5 6 7}
    G_OUTPUT_WIDTH 32 {32 64 128}
    G_NUM_FACES 5 {1 2 3 4 5}
    G_OEN_MODE DYNAMIC_CONNECTED \
        {DYNAMIC_CONNECTED PULLUP_OR_NOT_CONNECTED}
} {
    set parameter [ipx::get_user_parameters $name -of_objects $core]
    if {[get_property value $parameter] ne $default} {
        error "$name default mismatch: [get_property value $parameter]"
    }
    set validation_type [get_property value_validation_type $parameter]
    if {$validation_type ni {list pairs}} {
        error "$name must use list or labeled-pair validation"
    }
    set available [concat \
        [get_property value_validation_list $parameter] \
        [get_property value_validation_pairs $parameter]]
    foreach choice $choices {
        if {$choice ni $available} {
            error "$name is missing legal choice $choice: $available"
        }
    }
}

foreach interface_name {
    s_axi_csr s_axi_csr_aclk s_axi_csr_aresetn
    proc_aclk proc_aresetn i_tdc_clk i_tdc_aresetn
    m_axis_monitor m_axis_rise m_axis_fall o_irq
} {
    v2_require_one [ipx::get_bus_interfaces -quiet $interface_name \
        -of_objects $core] "bus interface $interface_name"
}

foreach {interface_name generic_name} {
    s_axi_csr G_CSR_CLK_MHZ
    s_axi_csr_aclk G_CSR_CLK_MHZ
    proc_aclk G_PROC_CLK_MHZ
    m_axis_monitor G_PROC_CLK_MHZ
    m_axis_rise G_PROC_CLK_MHZ
    m_axis_fall G_PROC_CLK_MHZ
    i_tdc_clk G_TDC_CLK_MHZ
} {
    set interface [ipx::get_bus_interfaces $interface_name -of_objects $core]
    set frequency [v2_require_one [ipx::get_bus_parameters FREQ_HZ \
        -of_objects $interface] "$interface_name FREQ_HZ"]
    if {[get_property value_resolve_type $frequency] ne {dependent} ||
        [string first $generic_name \
            [get_property value_dependency $frequency]] < 0} {
        error "$interface_name FREQ_HZ is not dependent on $generic_name"
    }
}

foreach {clock_name expected_busif expected_reset} {
    s_axi_csr_aclk s_axi_csr s_axi_csr_aresetn
    proc_aclk {m_axis_monitor:m_axis_rise:m_axis_fall} proc_aresetn
} {
    set clock [ipx::get_bus_interfaces $clock_name -of_objects $core]
    set associated [v2_require_one [ipx::get_bus_parameters ASSOCIATED_BUSIF \
        -of_objects $clock] "$clock_name ASSOCIATED_BUSIF"]
    set reset [v2_require_one [ipx::get_bus_parameters ASSOCIATED_RESET \
        -of_objects $clock] "$clock_name ASSOCIATED_RESET"]
    if {[get_property value $associated] ne $expected_busif ||
        [get_property value $reset] ne $expected_reset} {
        error "$clock_name association mismatch"
    }
}
set tdc_clock [ipx::get_bus_interfaces i_tdc_clk -of_objects $core]
set tdc_reset_name [v2_require_one [ipx::get_bus_parameters ASSOCIATED_RESET \
    -of_objects $tdc_clock] {i_tdc_clk ASSOCIATED_RESET}]
if {[get_property value $tdc_reset_name] ne {i_tdc_aresetn}} {
    error {i_tdc_clk reset association mismatch}
}
foreach reset_name {s_axi_csr_aresetn proc_aresetn i_tdc_aresetn} {
    set reset [ipx::get_bus_interfaces $reset_name -of_objects $core]
    set polarity [v2_require_one [ipx::get_bus_parameters POLARITY \
        -of_objects $reset] "$reset_name POLARITY"]
    if {[get_property value $polarity] ne {ACTIVE_LOW}} {
        error "$reset_name must remain ACTIVE_LOW"
    }
}
set irq [ipx::get_bus_interfaces o_irq -of_objects $core]
set sensitivity [v2_require_one [ipx::get_bus_parameters SENSITIVITY \
    -of_objects $irq] {o_irq SENSITIVITY}]
if {[get_property value $sensitivity] ne {LEVEL_HIGH}} {
    error {o_irq must remain LEVEL_HIGH}
}

foreach name {i_pd_lvds_p i_pd_lvds_n o_tdc_stop} {
    set port [v2_require_one [ipx::get_ports -quiet $name -of_objects $core] \
        "Echo port $name"]
    if {[get_property enablement_resolve_type $port] ne {dependent} ||
        [string first G_ENABLE_ECHO_RECEIVER \
            [get_property enablement_dependency $port]] < 0} {
        error "$name is not hidden by the Echo Receiver build option"
    }
}
set echo_sim [ipx::get_user_parameters G_ENABLE_ECHO_SIMULATION \
    -of_objects $core]
if {[get_property enablement_resolve_type $echo_sim] ne {dependent} ||
    [string first G_ENABLE_ECHO_RECEIVER \
        [get_property enablement_dependency $echo_sim]] < 0} {
    error {Echo simulation GUI enablement does not follow Echo Receiver}
}

set synth_group [v2_require_one [ipx::get_file_groups \
    xilinx_anylanguagesynthesis -of_objects $core] {synthesis file group}]
set synth_files [ipx::get_files -of_objects $synth_group]
if {[llength $synth_files] != 87} {
    error "Packaged synthesis source count is [llength $synth_files], expected 87"
}
if {[get_property component_subcores $synth_group] ne {}} {
    error {Packaged v2 synthesis group must not contain child IP subcores}
}
foreach packaged_file $synth_files {
    set name [get_property name $packaged_file]
    if {[string match "../*" $name] || [string match "*.xci" $name]} {
        error "Forbidden packaged dependency: $name"
    }
    set expected_type vhdlSource-2008
    if {[file tail $name] eq {tdc_gpx_lidar_ctrl_v2_top.vhd}} {
        set expected_type vhdlSource
    }
    if {[get_property type $packaged_file] ne $expected_type} {
        error "$name source type mismatch: [get_property type $packaged_file]"
    }
}
set guide_group [v2_require_one [ipx::get_file_groups xilinx_productguide \
    -of_objects $core] {Product Guide group}]
v2_require_one [ipx::get_files -of_objects $guide_group] {Product Guide file}
puts {LIDAR_V2_K010_COMPONENT_CONTRACT_PASS}

source $packaged_xgui
if {[::tglcv2_xgui::vhdl_bit_string 0011 4 \
        {Rising capability mask}] ne {"0011"}} {
    error {XGUI did not convert the rising mask to a VHDL bit-string literal}
}
foreach {label command expected} [list \
    async_tdc_faster \
        {::tglcv2_xgui::validate_clock_contract 150 200 ASYNC} true \
    async_proc_faster \
        {::tglcv2_xgui::validate_clock_contract 200 150 ASYNC} true \
    sync_equal \
        {::tglcv2_xgui::validate_clock_contract 150 150 SYNC} true \
    sync_split_invalid \
        {::tglcv2_xgui::validate_clock_contract 150 200 SYNC} false \
    dedicated_four_chip \
        {::tglcv2_xgui::validate_topology 4 0011 1100} true \
    all_dual_four_chip \
        {::tglcv2_xgui::validate_topology 4 1111 1111} true \
    rising_only_four_chip \
        {::tglcv2_xgui::validate_topology 4 1111 0000} true \
    one_chip_dual \
        {::tglcv2_xgui::validate_topology 1 0001 0001} true \
    more_fall_invalid \
        {::tglcv2_xgui::validate_topology 3 0001 0110} false \
    outside_chip_invalid \
        {::tglcv2_xgui::validate_topology 2 0111 0000} false] {
    lassign [uplevel #0 $command] actual message
    if {$actual ne $expected} {
        error "XGUI validation $label expected $expected, got $actual: $message"
    }
}
puts {LIDAR_V2_K010_XGUI_CONTRACT_PASS}

ipx::unload_core $core

# The new v2 IP must coexist with the board-proven v1 core in one catalog.
set v1_repo [file normalize [file join $hdl_root .. ip_repo]]
set v2_repo [file dirname $package_dir]
set_property ip_repo_paths [list $v2_repo $v1_repo] [current_project]
update_ip_catalog -rebuild
foreach vlnv {
    victek.co.kr:my_ip:tdc_gpx_top:1.0
    victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v2:2.0
} {
    v2_require_one [get_ipdefs -all $vlnv] "catalog definition $vlnv"
}
puts {LIDAR_V2_K010_V1_V2_CATALOG_COEXIST_PASS}

set v2_vlnv victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v2:2.0
foreach profile {
    {async32_tdc_faster 150 200 ASYNC 32 true}
    {async128_proc_faster 200 150 ASYNC 128 false}
    {sync64_equal 150 150 SYNC 64 true}
} {
    lassign $profile label proc_mhz tdc_mhz mode width echo_enabled
    set module_name v2_${label}
    create_ip -vlnv $v2_vlnv -module_name $module_name
    set packaged_ip [get_ips $module_name]
    set_property -dict [list \
        CONFIG.G_CSR_CLK_MHZ 100 \
        CONFIG.G_PROC_CLK_MHZ $proc_mhz \
        CONFIG.G_TDC_CLK_MHZ $tdc_mhz \
        CONFIG.G_STREAM_CLK_MODE $mode \
        CONFIG.G_NUM_CHIPS 4 \
        CONFIG.G_STOPS_PER_CHIP 8 \
        CONFIG.G_MAX_RETURNS_PER_STOP 7 \
        CONFIG.G_RISE_CAPABILITY_MASK 0011 \
        CONFIG.G_FALL_CAPABILITY_MASK 1100 \
        CONFIG.G_OUTPUT_WIDTH $width \
        CONFIG.G_NUM_FACES 5 \
        CONFIG.G_ENABLE_ECHO_RECEIVER $echo_enabled \
        CONFIG.G_ENABLE_ECHO_SIMULATION false] $packaged_ip
    generate_target instantiation_template $packaged_ip
    foreach {property expected} [list \
        CONFIG.G_PROC_CLK_MHZ $proc_mhz \
        CONFIG.G_TDC_CLK_MHZ $tdc_mhz \
        CONFIG.G_STREAM_CLK_MODE $mode \
        CONFIG.G_OUTPUT_WIDTH $width \
        CONFIG.G_ENABLE_ECHO_RECEIVER $echo_enabled] {
        if {[get_property $property $packaged_ip] ne $expected} {
            error "$label did not retain $property=$expected"
        }
    }
    puts "LIDAR_V2_K010_CUSTOMIZATION_PASS profile=$label"
}

close_project
puts {LIDAR_V2_K010_IP_PACKAGE_CHECK_PASS}
exit 0
