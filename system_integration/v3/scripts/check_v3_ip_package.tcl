# Verify the self-contained tdc_gpx_lidar_ctrl_v3:3.0 package, XGUI and
# coexistence with the board-proven tdc_gpx_top:1.0 package.

set script_dir [file normalize [file dirname [info script]]]
set v3_dir [file normalize [file join $script_dir ..]]
set hdl_root [file normalize [file join $v3_dir ../..]]
if {[llength $argv] > 0} {
    set package_dir [file normalize [lindex $argv 0]]
} else {
    set package_dir [file join $v3_dir ip_repo \
        tdc_gpx_lidar_ctrl_v3_3_0]
}

set component [file join $package_dir component.xml]
set packaged_xgui [file join $package_dir xgui \
    tdc_gpx_lidar_ctrl_v3_v3_0.tcl]
set packaged_guide [file join $package_dir doc PRODUCT_GUIDE_KO.md]
set packaged_maintenance_guide [file join $package_dir doc \
    V3_HLS_MIGRATION_PLAN_KO.md]
set packaged_code_reading_guide [file join $package_dir doc \
    V3_HLS_CODE_READING_GUIDE_KO.md]
set packaged_testbench_guide [file join $package_dir doc \
    V3_H6_TESTBENCH_GUIDE_KO.md]
set packaged_parent_checkpoint [file join $package_dir doc \
    V3_H6B3B_PARENT_IMPLEMENTATION_CHECKPOINT_KO.md]
set packaged_ip_packager_guide [file join $package_dir doc \
    V3_IP_PACKAGER_MAINTENANCE_GUIDE_KO.md]
set packaged_xgui_regeneration_rules [file join $package_dir doc \
    V3_IP_XACT_XGUI_REGENERATION_RULES_KO.md]
set packaged_dual_packaging_guide [file join $package_dir doc \
    V3_DUAL_HLS_PACKAGING_GUIDE_KO.md]
set canonical_xgui [file join $v3_dir ip_package \
    tdc_gpx_lidar_ctrl_v3_xgui.tcl]
set canonical_guide [file join $v3_dir ip_package PRODUCT_GUIDE_KO.md]
set canonical_maintenance_guide [file join $v3_dir docs \
    V3_HLS_MIGRATION_PLAN_KO.md]
set canonical_code_reading_guide [file join $v3_dir docs \
    V3_HLS_CODE_READING_GUIDE_KO.md]
set canonical_testbench_guide [file join $v3_dir docs \
    V3_H6_TESTBENCH_GUIDE_KO.md]
set canonical_parent_checkpoint [file join $v3_dir docs \
    V3_H6B3B_PARENT_IMPLEMENTATION_CHECKPOINT_KO.md]
set canonical_ip_packager_guide [file join $v3_dir docs \
    V3_IP_PACKAGER_MAINTENANCE_GUIDE_KO.md]
set canonical_xgui_regeneration_rules [file join $v3_dir docs \
    V3_IP_XACT_XGUI_REGENERATION_RULES_KO.md]
set canonical_dual_packaging_guide [file join $v3_dir docs \
    V3_DUAL_HLS_PACKAGING_GUIDE_KO.md]
foreach required [list $component $packaged_xgui $packaged_guide \
        $packaged_maintenance_guide $canonical_xgui $canonical_guide \
        $canonical_maintenance_guide $packaged_code_reading_guide \
        $canonical_code_reading_guide $packaged_testbench_guide \
        $canonical_testbench_guide $packaged_parent_checkpoint \
        $canonical_parent_checkpoint $packaged_ip_packager_guide \
        $canonical_ip_packager_guide $packaged_xgui_regeneration_rules \
        $canonical_xgui_regeneration_rules $packaged_dual_packaging_guide \
        $canonical_dual_packaging_guide] {
    if {![file exists $required]} {
        error "Required v3 package artifact is missing: $required"
    }
}

proc v3_read_binary {path} {
    set channel [open $path r]
    fconfigure $channel -translation binary
    set data [read $channel]
    close $channel
    return $data
}

proc v3_require_file_equal {canonical packaged label} {
    if {![file exists $canonical] || ![file exists $packaged]} {
        error "$label is missing: $canonical or $packaged"
    }
    if {[v3_read_binary $canonical] ne [v3_read_binary $packaged]} {
        error "$label package copy is stale: $packaged"
    }
}

proc v3_require_text_equal {canonical packaged label} {
    set canonical_text [string trimright [string map \
        [list "\r\n" "\n" "\r" "\n"] [v3_read_binary $canonical]]]
    set packaged_text [string trimright [string map \
        [list "\r\n" "\n" "\r" "\n"] [v3_read_binary $packaged]]]
    if {$canonical_text ne $packaged_text} {
        error "$label package copy changed its maintained content: $packaged"
    }
}

source [file join $v3_dir ip_package v3_ip_package_manifest.tcl]
set entries [lidar_v3_ip_package_manifest $hdl_root]
if {[llength $entries] < 100} {
    error "V3 package dependency closure is unexpectedly small: [llength $entries]"
}
foreach entry $entries {
    lassign $entry canonical relative type
    v3_require_file_equal $canonical [file join $package_dir src $relative] \
        "RTL $relative"
}
v3_require_text_equal $canonical_xgui $packaged_xgui {v3 XGUI}
source [file join $script_dir check_v3_xgui_source_contract.tcl]
v3_check_xgui_source_contract $packaged_xgui
v3_require_file_equal $canonical_guide $packaged_guide {v3 Product Guide}
v3_require_file_equal $canonical_maintenance_guide \
    $packaged_maintenance_guide {v3 RTL Maintenance Guide}
v3_require_file_equal $canonical_code_reading_guide \
    $packaged_code_reading_guide {v3 HLS Code Reading Guide}
v3_require_file_equal $canonical_testbench_guide \
    $packaged_testbench_guide {v3 Testbench Coverage Guide}
v3_require_file_equal $canonical_parent_checkpoint \
    $packaged_parent_checkpoint {v3 Parent Implementation Checkpoint}
v3_require_file_equal $canonical_ip_packager_guide \
    $packaged_ip_packager_guide {v3 IP Packager Maintenance Guide}
v3_require_file_equal $canonical_xgui_regeneration_rules \
    $packaged_xgui_regeneration_rules {v3 IP-XACT XGUI Regeneration Rules}
v3_require_file_equal $canonical_dual_packaging_guide \
    $packaged_dual_packaging_guide {v3 Dual HLS Packaging Guide}
if {[string first {C_MAX_CHIPS} [v3_read_binary $component]] >= 0} {
    error {component.xml exposes unresolved C_MAX_CHIPS in a public HDL type}
}
puts "LIDAR_V3_SOURCE_SYNC_PASS rtl=[llength $entries] xgui=1 guides=8"

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

create_project -in_memory lidar_v3_ip_check -part xc7z020clg484-2
set core [ipx::open_core $component]

proc v3_require_one {objects label} {
    if {[llength $objects] != 1} {
        error "Expected one $label, found [llength $objects]"
    }
    return [lindex $objects 0]
}

if {[get_property vlnv $core] ne \
        {victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3:3.0}} {
    error "Unexpected v3 VLNV: [get_property vlnv $core]"
}
if {[get_property core_revision $core] != 1} {
    error "Unexpected v3 core revision: [get_property core_revision $core]"
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
    v3_require_one [ipx::get_user_parameters -quiet $name -of_objects $core] \
        "user parameter $name"
    v3_require_one [ipx::get_hdl_parameters -quiet $name -of_objects $core] \
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
    G_OUTPUT_WIDTH 32 {32 64}
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
    v3_require_one [ipx::get_bus_interfaces -quiet $interface_name \
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
    set frequency [v3_require_one [ipx::get_bus_parameters FREQ_HZ \
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
    set associated [v3_require_one [ipx::get_bus_parameters ASSOCIATED_BUSIF \
        -of_objects $clock] "$clock_name ASSOCIATED_BUSIF"]
    set reset [v3_require_one [ipx::get_bus_parameters ASSOCIATED_RESET \
        -of_objects $clock] "$clock_name ASSOCIATED_RESET"]
    if {[get_property value $associated] ne $expected_busif ||
        [get_property value $reset] ne $expected_reset} {
        error "$clock_name association mismatch"
    }
}
set tdc_clock [ipx::get_bus_interfaces i_tdc_clk -of_objects $core]
set tdc_reset_name [v3_require_one [ipx::get_bus_parameters ASSOCIATED_RESET \
    -of_objects $tdc_clock] {i_tdc_clk ASSOCIATED_RESET}]
if {[get_property value $tdc_reset_name] ne {i_tdc_aresetn}} {
    error {i_tdc_clk reset association mismatch}
}
foreach reset_name {s_axi_csr_aresetn proc_aresetn i_tdc_aresetn} {
    set reset [ipx::get_bus_interfaces $reset_name -of_objects $core]
    set polarity [v3_require_one [ipx::get_bus_parameters POLARITY \
        -of_objects $reset] "$reset_name POLARITY"]
    if {[get_property value $polarity] ne {ACTIVE_LOW}} {
        error "$reset_name must remain ACTIVE_LOW"
    }
}
set irq [ipx::get_bus_interfaces o_irq -of_objects $core]
set sensitivity [v3_require_one [ipx::get_bus_parameters SENSITIVITY \
    -of_objects $irq] {o_irq SENSITIVITY}]
if {[get_property value $sensitivity] ne {LEVEL_HIGH}} {
    error {o_irq must remain LEVEL_HIGH}
}

foreach name {i_pd_lvds_p i_pd_lvds_n o_tdc_stop} {
    set port [v3_require_one [ipx::get_ports -quiet $name -of_objects $core] \
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

set synth_group [v3_require_one [ipx::get_file_groups \
    xilinx_anylanguagesynthesis -of_objects $core] {synthesis file group}]
set synth_files [ipx::get_files -of_objects $synth_group]
if {[llength $synth_files] != [llength $entries]} {
    error "Packaged synthesis source count is [llength $synth_files], expected [llength $entries]"
}
if {[get_property component_subcores $synth_group] ne {}} {
    error {Packaged v3 synthesis group must not contain child IP subcores}
}
foreach packaged_file $synth_files {
    set name [get_property name $packaged_file]
    if {[string match "../*" $name] || [string match "*.xci" $name]} {
        error "Forbidden packaged dependency: $name"
    }
    set expected_type {}
    foreach entry $entries {
        lassign $entry canonical relative type
        if {$name eq [string map {\\ /} [file join src $relative]]} {
            switch -- $type {
                vhdl { set expected_type vhdlSource }
                vhdl2008 { set expected_type vhdlSource-2008 }
                verilog { set expected_type verilogSource }
                data { set expected_type data }
            }
            break
        }
    }
    if {$expected_type eq {}} {
        error "Packaged source is not present in the canonical manifest: $name"
    }
    if {[get_property type $packaged_file] ne $expected_type} {
        error "$name source type mismatch: [get_property type $packaged_file]"
    }
}
set guide_group [v3_require_one [ipx::get_file_groups xilinx_productguide \
    -of_objects $core] {Product Guide group}]
set guide_files [ipx::get_files -of_objects $guide_group]
if {[llength $guide_files] != 8} {
    error "Expected eight Korean guide files, found [llength $guide_files]"
}
foreach required_guide [list $packaged_guide $packaged_maintenance_guide \
        $packaged_code_reading_guide $packaged_testbench_guide \
        $packaged_parent_checkpoint $packaged_ip_packager_guide \
        $packaged_xgui_regeneration_rules $packaged_dual_packaging_guide] {
    set relative_guide [file join doc [file tail $required_guide]]
    if {[llength [ipx::get_files -quiet $relative_guide \
            -of_objects $guide_group]] != 1} {
        error "Required Korean guide is not registered: $relative_guide"
    }
}
puts {LIDAR_V3_COMPONENT_CONTRACT_PASS}

# Single-parameter lists/ranges and Echo port enablement are verified above
# from component.xml. Cross-parameter combinations are intentionally checked
# by fn_validate_build_config so the XGUI file remains Vivado-native and the
# IP Packager can expose its editable Layout and Preview panes.
puts {LIDAR_V3_XGUI_IPXACT_AND_NATIVE_CONTRACT_PASS}

ipx::unload_core $core

# The new v3 IP must coexist with the board-proven v1 core in one catalog.
set v1_repo [file normalize [file join $hdl_root .. ip_repo]]
set v3_repo [file dirname $package_dir]
set_property ip_repo_paths [list $v3_repo $v1_repo] [current_project]
update_ip_catalog -rebuild
foreach vlnv {
    victek.co.kr:my_ip:tdc_gpx_top:1.0
    victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3:3.0
} {
    v3_require_one [get_ipdefs -all $vlnv] "catalog definition $vlnv"
}
puts {LIDAR_V3_V1_V3_CATALOG_COEXIST_PASS}

set v3_vlnv victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3:3.0
foreach profile {
    {async32_tdc_faster 150 200 ASYNC 32 true 0011 1100}
    {async64_proc_faster 200 150 ASYNC 64 false 1111 1111}
} {
    lassign $profile label proc_mhz tdc_mhz mode width echo_enabled \
        rise_mask fall_mask
    set module_name v3_${label}
    create_ip -vlnv $v3_vlnv -module_name $module_name
    set packaged_ip [get_ips $module_name]
    set_property -dict [list \
        CONFIG.G_CSR_CLK_MHZ 100 \
        CONFIG.G_PROC_CLK_MHZ $proc_mhz \
        CONFIG.G_TDC_CLK_MHZ $tdc_mhz \
        CONFIG.G_STREAM_CLK_MODE $mode \
        CONFIG.G_NUM_CHIPS 4 \
        CONFIG.G_STOPS_PER_CHIP 8 \
        CONFIG.G_MAX_RETURNS_PER_STOP 7 \
        CONFIG.G_RISE_CAPABILITY_MASK $rise_mask \
        CONFIG.G_FALL_CAPABILITY_MASK $fall_mask \
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
        CONFIG.G_RISE_CAPABILITY_MASK $rise_mask \
        CONFIG.G_FALL_CAPABILITY_MASK $fall_mask \
        CONFIG.G_ENABLE_ECHO_RECEIVER $echo_enabled] {
        if {[get_property $property $packaged_ip] ne $expected} {
            error "$label did not retain $property=$expected"
        }
    }
    puts "LIDAR_V3_CUSTOMIZATION_PASS profile=$label"
}

close_project
puts {TDC_GPX_LIDAR_CTRL_V3_IP_CHECK_PASS}
exit 0
