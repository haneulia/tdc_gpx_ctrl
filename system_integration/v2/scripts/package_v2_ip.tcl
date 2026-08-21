# Build the self-contained tdc_gpx_lidar_ctrl_v2:2.0 packaged IP.
# Optional first argument: alternate package directory whose final directory
# name must remain tdc_gpx_lidar_ctrl_v2_2_0.

set script_dir [file normalize [file dirname [info script]]]
set v2_dir [file normalize [file join $script_dir ..]]
set hdl_root [file normalize [file join $v2_dir ../..]]
set package_parent [file join $v2_dir ip_repo]
set package_name tdc_gpx_lidar_ctrl_v2_2_0
if {[llength $argv] > 0} {
    set package_dir [file normalize [lindex $argv 0]]
} else {
    set package_dir [file join $package_parent $package_name]
}
if {[file tail $package_dir] ne $package_name} {
    error "Refusing to replace unexpected package directory: $package_dir"
}

set manifest_source [file join $v2_dir ip_package v2_ip_package_manifest.tcl]
set xgui_source [file join $v2_dir ip_package \
    tdc_gpx_lidar_ctrl_v2_xgui.tcl]
set product_guide_source [file join $v2_dir ip_package PRODUCT_GUIDE_KO.md]
set maintenance_guide_source [file join $v2_dir .. v2_architecture \
    V2_RTL_MAINTENANCE_GUIDE_KO.md]
set testbench_guide_source [file join $v2_dir .. v2_architecture \
    V2_TESTBENCH_COVERAGE_GUIDE_KO.md]
set packager_guide_source [file join $v2_dir .. v2_architecture \
    V2_IP_PACKAGER_MAINTENANCE_GUIDE_KO.md]
set xgui_rules_source [file join $v2_dir .. v2_architecture \
    V2_IP_XACT_XGUI_REGENERATION_RULES_KO.md]
set xgui_checker [file join $script_dir check_v2_xgui_source_contract.tcl]
foreach required [list $manifest_source $xgui_source $product_guide_source \
        $maintenance_guide_source $testbench_guide_source \
        $packager_guide_source $xgui_rules_source $xgui_checker] {
    if {![file exists $required]} {
        error "Required package source is missing: $required"
    }
}
source $xgui_checker
v2_check_xgui_source_contract $xgui_source
source $manifest_source
set entries [lidar_v2_ip_package_manifest $hdl_root]
if {[llength $entries] != 88} {
    error "Expected 88 transitive production sources, got [llength $entries]"
}

if {[file exists $package_dir]} {
    file delete -force $package_dir
}
set package_src [file join $package_dir src]
set package_xgui [file join $package_dir xgui]
set package_doc [file join $package_dir doc]
file mkdir $package_src
file mkdir $package_xgui
file mkdir $package_doc

set packaged_sources {}
set packaged_relpaths {}
foreach entry $entries {
    lassign $entry source relative
    set destination [file join $package_src $relative]
    file mkdir [file dirname $destination]
    file copy -force $source $destination
    lappend packaged_sources $destination
    lappend packaged_relpaths [file join src $relative]
}
set xgui_name tdc_gpx_lidar_ctrl_v2_v2_0.tcl
set xgui_destination [file join $package_xgui $xgui_name]
file copy -force $xgui_source $xgui_destination
set product_guide_name PRODUCT_GUIDE_KO.md
set product_guide_destination [file join $package_doc $product_guide_name]
file copy -force $product_guide_source $product_guide_destination
set maintenance_guide_name V2_RTL_MAINTENANCE_GUIDE_KO.md
set maintenance_guide_destination [file join $package_doc \
    $maintenance_guide_name]
file copy -force $maintenance_guide_source $maintenance_guide_destination
set testbench_guide_name V2_TESTBENCH_COVERAGE_GUIDE_KO.md
set testbench_guide_destination [file join $package_doc \
    $testbench_guide_name]
file copy -force $testbench_guide_source $testbench_guide_destination
set packager_guide_name V2_IP_PACKAGER_MAINTENANCE_GUIDE_KO.md
set packager_guide_destination [file join $package_doc \
    $packager_guide_name]
file copy -force $packager_guide_source $packager_guide_destination
set xgui_rules_name V2_IP_XACT_XGUI_REGENERATION_RULES_KO.md
set xgui_rules_destination [file join $package_doc $xgui_rules_name]
file copy -force $xgui_rules_source $xgui_rules_destination

# Avoid a stale per-user Tcl Store cache. Use the installed Vivado store as the
# reproducible packaging source.
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

set work_dir [file join $hdl_root tmp v2_k010_ip_package_work]
if {[file exists $work_dir]} {
    file delete -force $work_dir
}
create_project -force tdc_gpx_lidar_ctrl_v2_package $work_dir \
    -part xc7z020clg484-2
set_property target_language VHDL [current_project]
set_property simulator_language Mixed [current_project]

add_files -fileset sources_1 -norecurse $packaged_sources
foreach source $packaged_sources {
    if {[file tail $source] eq {tdc_gpx_lidar_ctrl_v2_top.vhd}} {
        set_property file_type VHDL [get_files $source]
    } else {
        set_property file_type {VHDL 2008} [get_files $source]
    }
}
set_property top tdc_gpx_lidar_ctrl_v2_top [get_filesets sources_1]
update_compile_order -fileset sources_1

ipx::package_project -root_dir $package_dir \
    -vendor victek.co.kr -library my_ip \
    -name tdc_gpx_lidar_ctrl_v2 -version 2.0 \
    -taxonomy {/VictekIP} -set_current true -force
set core [ipx::current_core]
if {$core eq {}} {
    error {Unable to obtain packaged v2 core object}
}

# package_project creates a default GUI. Restore the canonical tabbed GUI.
file copy -force $xgui_source $xgui_destination
set_property vendor victek.co.kr $core
set_property library my_ip $core
set_property name tdc_gpx_lidar_ctrl_v2 $core
set_property version 2.0 $core
set_property taxonomy {/VictekIP} $core
set_property display_name {TDC-GPX LiDAR Integrated Controller V2} $core
set_property vendor_display_name {Victek Co., Ltd.} $core
set_property company_url {https://www.victek.co.kr} $core
set_property description \
    {V2 integrated Motor/Laser, optional Echo LVDS frontend, four-chip TDC-GPX acquisition, PACKED17 Rise/Fall AXI streams, unified 32-control/32-status CSR and runtime IRQ.} \
    $core
set_property core_revision 1 $core
set_property supported_families {zynq Production} $core

proc v2_user_parameter {core name} {
    set parameter [ipx::get_user_parameters -quiet $name -of_objects $core]
    if {[llength $parameter] != 1} {
        error "Expected one user parameter named $name"
    }
    return $parameter
}

proc v2_hdl_parameter {core name} {
    set parameter [ipx::get_hdl_parameters -quiet $name -of_objects $core]
    if {[llength $parameter] != 1} {
        error "Expected one HDL parameter named $name"
    }
    return $parameter
}

proc v2_set_list {core name value choices pairs display description} {
    set user [v2_user_parameter $core $name]
    set hdl [v2_hdl_parameter $core $name]
    set_property value_resolve_type user $user
    set_property value_validation_type list $user
    set_property value_validation_list $choices $user
    if {[llength $pairs] > 0} {
        set_property value_validation_pairs $pairs $user
    }
    set_property value $value $user
    set_property display_name $display $user
    set_property description $description $user
    set_property value_resolve_type generated $hdl
    set_property value $value $hdl
}

proc v2_set_long {core name value minimum maximum display description} {
    set user [v2_user_parameter $core $name]
    set hdl [v2_hdl_parameter $core $name]
    foreach parameter [list $user $hdl] {
        set_property value_format long $parameter
        set_property value_validation_type range_long $parameter
        set_property value_validation_range_minimum $minimum $parameter
        set_property value_validation_range_maximum $maximum $parameter
        set_property value $value $parameter
    }
    set_property value_resolve_type user $user
    set_property display_name $display $user
    set_property description $description $user
    set_property value_resolve_type generated $hdl
}

proc v2_set_bitstring {core name value display description} {
    set user [v2_user_parameter $core $name]
    set hdl [v2_hdl_parameter $core $name]
    foreach parameter [list $user $hdl] {
        set_property value_format bitString $parameter
        set_property value $value $parameter
    }
    set_property value_resolve_type user $user
    set_property display_name $display $user
    set_property description $description $user
    set_property value_resolve_type generated $hdl
}

proc v2_set_description {core name display description} {
    set user [v2_user_parameter $core $name]
    set_property value_resolve_type user $user
    set_property display_name $display $user
    set_property description $description $user
}

set clock_choices {50 100 125 150 200}
set clock_pairs [list {50 MHz} 50 {100 MHz} 100 {125 MHz} 125 \
    {150 MHz} 150 {200 MHz} 200]
v2_set_list $core G_CSR_CLK_MHZ 100 $clock_choices $clock_pairs \
    {CSR clock frequency (MHz)} \
    {Must match s_axi_csr_aclk. Phase timeout conversion uses this value.}
v2_set_list $core G_PROC_CLK_MHZ 150 $clock_choices $clock_pairs \
    {Processing clock frequency (MHz)} \
    {Must match proc_aclk. Routine sign-off profiles use 150 or 200 MHz.}
v2_set_list $core G_TDC_CLK_MHZ 200 $clock_choices $clock_pairs \
    {TDC bus clock frequency (MHz)} \
    {Must match i_tdc_clk. Physical timing values are converted in this domain.}
v2_set_list $core G_STREAM_CLK_MODE ASYNC {ASYNC SYNC} \
    [list {Independent clocks (ASYNC)} ASYNC \
          {Shared clock (SYNC)} SYNC] \
    {TDC result stream clock mode} \
    {SYNC requires equal frequencies and one shared physical clock net.}
v2_set_list $core G_NUM_CHIPS 4 {1 2 3 4} {} \
    {Physical TDC-GPX chip count} \
    {Controls all per-chip GPX bus and flag vector widths.}
v2_set_list $core G_STOPS_PER_CHIP 8 {1 2 3 4 5 6 7 8} {} \
    {STOP channels per chip} \
    {Controls Echo LVDS/STOP width and logical APD capacity.}
v2_set_list $core G_MAX_RETURNS_PER_STOP 7 {1 2 3 4 5 6 7} {} \
    {Maximum Returns per STOP} \
    {Compile-time capacity. Runtime selection may use 1 through this value.}
v2_set_bitstring $core G_RISE_CAPABILITY_MASK {"0011"} \
    {Rising-capable chip mask} \
    {Four-bit mask; may overlap the falling mask for per-chip dual-edge capture.}
v2_set_bitstring $core G_FALL_CAPABILITY_MASK {"1100"} \
    {Falling-capable chip mask} \
    {Use 0000 for rising-only hardware.}
v2_set_list $core G_OUTPUT_WIDTH 32 {32 64 128} \
    [list {32 bit} 32 {64 bit} 64 {128 bit} 128] \
    {PACKED17 AXI output width} \
    {Build-time width for both Rise and Fall AXI streams.}
v2_set_list $core G_NUM_FACES 5 {1 2 3 4 5} {} \
    {Mirror face count} \
    {Build-time polygon face count; runtime geometry uses only these faces.}
v2_set_list $core G_OEN_MODE DYNAMIC_CONNECTED \
    {DYNAMIC_CONNECTED PULLUP_OR_NOT_CONNECTED} \
    [list {Dynamic OEN pin} DYNAMIC_CONNECTED \
          {Pull-up or not connected} PULLUP_OR_NOT_CONNECTED] \
    {TDC-GPX OEN wiring mode} \
    {Select the contract that matches the parent PCB wiring.}

foreach {name display description} {
    G_ENABLE_ECHO_RECEIVER {Enable Echo Receiver} {Compile the low-latency LVDS-to-GPX STOP frontend and expose its ports.}
    G_ENABLE_ECHO_SIMULATION {Enable Echo simulation} {Compile the synthetic Echo source. Requires Echo Receiver enabled.}
} {
    v2_set_description $core $name $display $description
}

foreach {name value display description} {
    G_PHASE_TIMEOUT_US 1000 {Operation phase timeout (us)} {CSR-domain bound for configuration and operation phases.}
    G_POWERUP_TIME_NS 240 {TDC power-up time (ns)} {Converted to TDC clocks with ceiling arithmetic.}
    G_RECOVERY_TIME_NS 40 {TDC recovery time (ns)} {Converted to TDC clocks with ceiling arithmetic.}
    G_ALU_PULSE_TIME_NS 20 {TDC ALU pulse time (ns)} {Converted to TDC clocks with ceiling arithmetic.}
    G_BUS_IDLE_STABLE_TIME_NS 20480 {TDC bus idle-stable time (ns)} {Physical quiet interval before bus ownership changes.}
    G_DRAIN_MARGIN_TIME_NS 6000 {TDC drain margin (ns)} {Bounded post-read margin for physical GPX drain.}
} {
    v2_set_long $core $name $value 1 2147483647 $display $description
}

foreach required_interface {
    s_axi_csr s_axi_csr_aclk s_axi_csr_aresetn
    proc_aclk proc_aresetn i_tdc_clk i_tdc_aresetn
    m_axis_monitor m_axis_rise m_axis_fall o_irq
} {
    if {[llength [ipx::get_bus_interfaces -quiet $required_interface \
            -of_objects $core]] != 1} {
        error "Expected inferred interface $required_interface"
    }
}

ipx::associate_bus_interfaces -busif s_axi_csr \
    -clock s_axi_csr_aclk -reset s_axi_csr_aresetn $core
foreach stream {m_axis_monitor m_axis_rise m_axis_fall} {
    ipx::associate_bus_interfaces -busif $stream \
        -clock proc_aclk -reset proc_aresetn $core
}

proc v2_set_bus_parameter {interface name value} {
    set parameter [ipx::get_bus_parameters -quiet $name -of_objects $interface]
    if {[llength $parameter] == 0} {
        set parameter [ipx::add_bus_parameter $name $interface]
    }
    set_property value $value $parameter
    return $parameter
}

proc v2_set_frequency {interface generic_name default_hz} {
    set parameter [v2_set_bus_parameter $interface FREQ_HZ $default_hz]
    set_property value_format long $parameter
    set_property value_resolve_type dependent $parameter
    set_property value_dependency \
        "(spirit:decode(id('PARAM_VALUE.$generic_name')) * 1000000)" \
        $parameter
    set_property value_source default $parameter
}

set csr_clock [ipx::get_bus_interfaces s_axi_csr_aclk -of_objects $core]
set csr_reset [ipx::get_bus_interfaces s_axi_csr_aresetn -of_objects $core]
set proc_clock [ipx::get_bus_interfaces proc_aclk -of_objects $core]
set proc_reset [ipx::get_bus_interfaces proc_aresetn -of_objects $core]
set tdc_clock [ipx::get_bus_interfaces i_tdc_clk -of_objects $core]
set tdc_reset [ipx::get_bus_interfaces i_tdc_aresetn -of_objects $core]
foreach interface [list $csr_clock \
        [ipx::get_bus_interfaces s_axi_csr -of_objects $core]] {
    v2_set_frequency $interface G_CSR_CLK_MHZ 100000000
}
foreach interface [list $proc_clock \
        [ipx::get_bus_interfaces m_axis_monitor -of_objects $core] \
        [ipx::get_bus_interfaces m_axis_rise -of_objects $core] \
        [ipx::get_bus_interfaces m_axis_fall -of_objects $core]] {
    v2_set_frequency $interface G_PROC_CLK_MHZ 150000000
}
v2_set_frequency $tdc_clock G_TDC_CLK_MHZ 200000000
v2_set_bus_parameter $csr_clock ASSOCIATED_BUSIF s_axi_csr
v2_set_bus_parameter $csr_clock ASSOCIATED_RESET s_axi_csr_aresetn
v2_set_bus_parameter $proc_clock ASSOCIATED_BUSIF \
    {m_axis_monitor:m_axis_rise:m_axis_fall}
v2_set_bus_parameter $proc_clock ASSOCIATED_RESET proc_aresetn
v2_set_bus_parameter $tdc_clock ASSOCIATED_RESET i_tdc_aresetn
v2_set_bus_parameter $csr_reset POLARITY ACTIVE_LOW
v2_set_bus_parameter $proc_reset POLARITY ACTIVE_LOW
v2_set_bus_parameter $tdc_reset POLARITY ACTIVE_LOW

set irq_interface [ipx::get_bus_interfaces o_irq -of_objects $core]
set irq_sensitivity [v2_set_bus_parameter $irq_interface SENSITIVITY LEVEL_HIGH]
set_property value_resolve_type user $irq_sensitivity
set_property connection_required false \
    [ipx::get_bus_interfaces m_axis_fall -of_objects $core]

proc v2_set_enablement {object dependency default_value} {
    set_property enablement_resolve_type dependent $object
    set_property enablement_dependency $dependency $object
    set_property enablement_value $default_value $object
}
set echo_dependency \
    {spirit:decode(id('MODELPARAM_VALUE.G_ENABLE_ECHO_RECEIVER')) = true}
foreach port_name {i_pd_lvds_p i_pd_lvds_n o_tdc_stop} {
    set port [ipx::get_ports -quiet $port_name -of_objects $core]
    if {[llength $port] != 1} {
        error "Expected one Echo physical port named $port_name"
    }
    v2_set_enablement $port $echo_dependency true
    if {[get_property direction $port] eq {in}} {
        set_property driver_value 0 $port
    }
}
set echo_sim [v2_user_parameter $core G_ENABLE_ECHO_SIMULATION]
v2_set_enablement $echo_sim $echo_dependency true

# Rebuild deterministic, self-contained source and XGUI views.
foreach group_name {
    xilinx_anylanguagesynthesis xilinx_anylanguagebehavioralsimulation
} {
    set group [ipx::get_file_groups $group_name -of_objects $core]
    if {[llength $group] != 1} {
        error "Expected one file group named $group_name"
    }
    ipx::remove_all_file $group
    set_property language VHDL $group
    set_property component_subcores {} $group
    foreach relative $packaged_relpaths {
        set packaged_file [ipx::add_file $relative $group]
        if {[file tail $relative] eq {tdc_gpx_lidar_ctrl_v2_top.vhd}} {
            set_property type vhdlSource $packaged_file
        } else {
            set_property type vhdlSource-2008 $packaged_file
        }
    }
}
set xgui_group [ipx::get_file_groups xilinx_xpgui -of_objects $core]
if {[llength $xgui_group] != 1} {
    error {Expected one XGUI file group}
}
ipx::remove_all_file $xgui_group
set xgui_file [ipx::add_file xgui/$xgui_name $xgui_group]
set_property type tclSource $xgui_file
set_property xgui_version 2 $xgui_file

set product_guide_group [ipx::get_file_groups -quiet xilinx_productguide \
    -of_objects $core]
if {[llength $product_guide_group] == 0} {
    ipx::add_file_group -type xilinx_productguide \
        xilinx_productguide $core
    set product_guide_group [ipx::get_file_groups -quiet \
        xilinx_productguide -of_objects $core]
}
if {[llength $product_guide_group] != 1} {
    error {Expected one Product Guide file group}
}
ipx::remove_all_file $product_guide_group
ipx::add_file doc/$product_guide_name $product_guide_group
ipx::add_file doc/$maintenance_guide_name $product_guide_group
ipx::add_file doc/$testbench_guide_name $product_guide_group
ipx::add_file doc/$packager_guide_name $product_guide_group
ipx::add_file doc/$xgui_rules_name $product_guide_group

foreach group [ipx::get_file_groups -of_objects $core] {
    set_property component_subcores {} $group
    foreach packaged_file [ipx::get_files -of_objects $group] {
        set packaged_path [get_property name $packaged_file]
        if {[string match "../*" $packaged_path]} {
            error "External source reference remains: $packaged_path"
        }
        if {[string match "*.xci" $packaged_path]} {
            error "Generated child IP remains: $packaged_path"
        }
    }
}

foreach required_port {
    s_axi_csr_aclk proc_aclk i_tdc_clk i_external_laser_permit
    i_enc_a i_enc_b i_enc_z i_fire_done o_fire_pulse o_start_tdc
    i_pd_lvds_p i_pd_lvds_n o_tdc_stop io_tdc_d o_tdc_adr o_tdc_csn
    o_tdc_oen i_tdc_ef1 i_tdc_ef2 i_tdc_lf1 i_tdc_lf2
    i_tdc_irflag i_tdc_errflag m_axis_rise_tdata m_axis_fall_tdata
} {
    if {[llength [ipx::get_ports -quiet $required_port -of_objects $core]] != 1} {
        error "Required packaged port is missing: $required_port"
    }
}

# VDMA profile transport is an internal processing-to-CSR transaction. The
# Parent reads CTL25..29, programs its VDMA, then acknowledges through CTL25.
foreach removed_profile_port {
    o_vdma_rise_cfg_valid i_vdma_rise_cfg_ready
    o_vdma_rise_cfg_enable o_vdma_rise_hsize_bytes
    o_vdma_rise_vsize_lines o_vdma_rise_stride_bytes
    o_vdma_fall_cfg_valid i_vdma_fall_cfg_ready
    o_vdma_fall_cfg_enable o_vdma_fall_hsize_bytes
    o_vdma_fall_vsize_lines o_vdma_fall_stride_bytes
} {
    if {[llength [ipx::get_ports -quiet $removed_profile_port \
            -of_objects $core]] != 0} {
        error "Removed external VDMA profile port remains: $removed_profile_port"
    }
}

ipx::update_checksums $core
set drc [ipx::check_integrity -verbose $core]
puts "TDC_GPX_LIDAR_CTRL_V2_IP_DRC=$drc"
ipx::save_core $core
ipx::unload_core $core
close_project

# Generate the editable Package IP Layout/Preview only after the final
# IP-XACT parameter and interface state has been saved, unloaded, and reopened.
# Generating from the still-open packaging object can retain stale parameter
# relationships and collapse the maintained tab hierarchy into one flat page.
set component [file join $package_dir component.xml]
create_project -in_memory tdc_gpx_lidar_ctrl_v2_xgui_sync \
    -part xc7z020clg484-2
set core [ipx::open_core $component]
common::load_features ipservices
ipx::create_xgui_files $core
# Do not rewrite the generated file, including trailing whitespace. Vivado's
# Package IP preview treats any post-generation rewrite as a hand edit even
# when the Tcl procedures remain textually equivalent.
v2_check_xgui_source_contract $xgui_destination
ipx::update_checksums $core
ipx::save_core $core
ipx::unload_core $core
close_project

if {[file exists $work_dir]} {
    file delete -force $work_dir
}
set channel [open $component r]
set component_text [read $channel]
close $channel
foreach forbidden {.xci ../} {
    if {[string first $forbidden $component_text] >= 0} {
        error "Forbidden packaged dependency remains: $forbidden"
    }
}

# Vivado IP Packager compares the generated XGUI modification time with the
# final component.xml modification time. ipx::save_core updates component.xml
# after ipx::create_xgui_files, so restore the required ordering explicitly.
# Git does not preserve file timestamps; every deterministic package build
# must therefore establish this preview freshness contract again.
set component_mtime [file mtime $component]
file mtime $xgui_destination [expr {$component_mtime + 2}]
if {[file mtime $xgui_destination] <= $component_mtime} {
    error "Unable to establish V2 XGUI preview freshness: $xgui_destination"
}

puts "TDC_GPX_LIDAR_CTRL_V2_IP_COMPONENT=$component"
puts {TDC_GPX_LIDAR_CTRL_V2_XGUI_PREVIEW_FRESH=1}
puts {TDC_GPX_LIDAR_CTRL_V2_IP_PACKAGE_PASS}
exit 0
