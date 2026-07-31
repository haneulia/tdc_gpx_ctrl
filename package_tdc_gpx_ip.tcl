# Build the source-only tdc_gpx_top packaged IP from canonical RTL.
#
# Vivado batch:
#   vivado -mode batch -source HDL/package_tdc_gpx_ip.tcl
# Optional first argument: alternate package directory.

set script_dir [file normalize [file dirname [info script]]]
set project_dir [file normalize [file join $script_dir ..]]
set ip_root [file normalize [file join $project_dir ..]]
if {[llength $argv] > 0} {
    set package_dir [file normalize [lindex $argv 0]]
} else {
    set package_dir [file normalize [file join $project_dir ip_repo]]
}

source [file join $script_dir scripts tdc_gpx_rtl_manifest.tcl]
source [file join $script_dir scripts tdc_gpx_csr_source_manifest.tcl]
# px_utility_pkg is a testbench-only AXI-Lite driver helper. Keep it in the
# repository regression manifest, but never ship it as synthesizable IP RTL.
set rtl_files [lsearch -all -inline -not -exact \
    [tdc_gpx_rtl_manifest] px_utility_pkg.vhd]
set csr_sources [tdc_gpx_csr_source_manifest $ip_root]
set xgui_source [file join $script_dir tdc_gpx_xgui.tcl]
set interface_sources [list \
    [file join $script_dir interfaces tdc_gpx_unified_csr.xml] \
    [file join $script_dir interfaces tdc_gpx_unified_csr_rtl.xml]]

foreach source [concat $csr_sources [list $xgui_source] $interface_sources] {
    if {![file exists $source]} {
        error "Required canonical source is missing: $source"
    }
}
foreach filename $rtl_files {
    if {![file exists [file join $script_dir $filename]]} {
        error "Required TDC-GPX RTL source is missing: $filename"
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

set csr_files {}
foreach source $csr_sources {
    set filename [file tail $source]
    lappend csr_files $filename
    file copy -force $source [file join $package_csr $filename]
}
foreach filename $rtl_files {
    file copy -force [file join $script_dir $filename] \
        [file join $package_src $filename]
}
file copy -force $xgui_source \
    [file join $package_xgui tdc_gpx_top_v1_0.tcl]
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

set package_work [file join $script_dir .package_work]
if {[file exists $package_work]} {
    file delete -force $package_work
}
create_project -force tdc_gpx_package $package_work -part xc7z020clg484-2
set_property target_language VHDL [current_project]
set_property simulator_language Mixed [current_project]
set unified_bus_definition [ipx::open_bus_definition \
    [file join $package_interfaces tdc_gpx_unified_csr.xml]]
set unified_abstraction_definition [ipx::open_abstraction_definition \
    [file join $package_interfaces tdc_gpx_unified_csr_rtl.xml]]

set ordered_sources {}
foreach filename $csr_files {
    lappend ordered_sources [file join $package_csr $filename]
}
foreach filename $rtl_files {
    lappend ordered_sources [file join $package_src $filename]
}
foreach source $ordered_sources {
    add_files -fileset sources_1 -norecurse $source
    if {[file tail $source] eq {tdc_gpx_top.vhd}} {
        # The top uses VHDL-93-compatible syntax. Marking only the package top
        # as VHDL avoids Vivado's VHDL-2008 top packaging warning.
        set_property file_type VHDL [get_files $source]
    } else {
        set_property file_type {VHDL 2008} [get_files $source]
    }
}
set_property top tdc_gpx_top [get_filesets sources_1]
update_compile_order -fileset sources_1

ipx::package_project \
    -root_dir $package_dir \
    -vendor victek.co.kr \
    -library my_ip \
    -name tdc_gpx_top \
    -version 1.0 \
    -taxonomy {/VictekIP} \
    -set_current true \
    -force
set core [ipx::current_core]
if {$core eq {}} {
    error {Unable to obtain the packaged TDC-GPX core object}
}

# package_project writes a default XGUI. Restore the canonical multi-tab GUI.
file copy -force $xgui_source \
    [file join $package_xgui tdc_gpx_top_v1_0.tcl]

set_property vendor victek.co.kr $core
set_property library my_ip $core
set_property name tdc_gpx_top $core
set_property version 1.0 $core
set_property taxonomy {/VictekIP} $core
set_property display_name {TDC-GPX Multi-chip Acquisition Controller} $core
set_property description \
    {Configurable one-to-four-chip TDC-GPX acquisition with selectable local or unified CSR ownership, rising/falling lane processing, dual VDMA-ready AXI4-Stream outputs, diagnostics, and interrupts.} \
    $core
set_property core_revision 5 $core
set_property supported_families {zynq Production} $core

proc tdc_ensure_long {
    core name value minimum maximum display_name description
} {
    set user_param [ipx::get_user_parameters -quiet $name -of_objects $core]
    set hdl_param [ipx::get_hdl_parameters -quiet $name -of_objects $core]
    if {[llength $user_param] != 1 || [llength $hdl_param] != 1} {
        error "Expected one user and HDL parameter named $name"
    }
    set_property value_format long $user_param
    set_property value_resolve_type user $user_param
    set_property value $value $user_param
    set_property display_name $display_name $user_param
    set_property description $description $user_param
    set_property data_type integer $hdl_param
    set_property value_format long $hdl_param
    set_property value_resolve_type generated $hdl_param
    set_property value $value $hdl_param
    foreach parameter [list $user_param $hdl_param] {
        set_property value_validation_type range_long $parameter
        set_property value_validation_range_minimum $minimum $parameter
        set_property value_validation_range_maximum $maximum $parameter
    }
    return $user_param
}

proc tdc_set_vector {core name value display_name description} {
    set user_param [ipx::get_user_parameters -quiet $name -of_objects $core]
    set hdl_param [ipx::get_hdl_parameters -quiet $name -of_objects $core]
    if {[llength $user_param] != 1 || [llength $hdl_param] != 1} {
        error "Expected one user and HDL vector parameter named $name"
    }
    set_property value_resolve_type user $user_param
    set_property value $value $user_param
    set_property display_name $display_name $user_param
    set_property description $description $user_param
    set_property value_resolve_type generated $hdl_param
    set_property value $value $hdl_param
    return $user_param
}

proc tdc_set_string {core name value values display_name description} {
    set user_param [ipx::get_user_parameters -quiet $name -of_objects $core]
    set hdl_param [ipx::get_hdl_parameters -quiet $name -of_objects $core]
    if {[llength $user_param] != 1 || [llength $hdl_param] != 1} {
        error "Expected one user and HDL string parameter named $name"
    }
    set_property value_resolve_type user $user_param
    set_property value $value $user_param
    set_property display_name $display_name $user_param
    set_property description $description $user_param
    set_property value_validation_type list $user_param
    set_property value_validation_list $values $user_param
    set_property value_resolve_type generated $hdl_param
    set_property value $value $hdl_param
    return $user_param
}

proc tdc_set_boolean {core name value display_name description} {
    set user_param [ipx::get_user_parameters -quiet $name -of_objects $core]
    set hdl_param [ipx::get_hdl_parameters -quiet $name -of_objects $core]
    if {[llength $user_param] != 1 || [llength $hdl_param] != 1} {
        error "Expected one user and HDL boolean parameter named $name"
    }
    set_property value_format bool $user_param
    set_property value_resolve_type user $user_param
    set_property value $value $user_param
    set_property display_name $display_name $user_param
    set_property description $description $user_param
    set_property data_type boolean $hdl_param
    set_property value_format bool $hdl_param
    set_property value_resolve_type generated $hdl_param
    set_property value $value $hdl_param
    return $user_param
}

tdc_set_vector $core g_HW_VERSION {"00000000000000010000000000000000"} \
    {Hardware version} \
    {Build identifier reported through runtime status and output metadata.}
tdc_set_boolean $core g_ENABLE_LOCAL_CSR true \
    {Enable local CSR banks} \
    {Enabled exposes the two legacy AXI4-Lite banks. Disabled removes both local banks and exposes the named unified CSR interface.}
tdc_set_vector $core g_PRESENT_CHIP_MASK {"1111"} \
    {Present chip mask} \
    {Four logical slots. The number of set bits must equal the physical chip count.}
tdc_set_vector $core g_RISE_CHIP_MASK {"0011"} \
    {Rising-capable chip mask} \
    {Present chips assigned to the rising lane. It may overlap the falling mask.}
tdc_set_vector $core g_FALL_CHIP_MASK {"1100"} \
    {Falling-capable chip mask} \
    {Present chips assigned to the falling lane. Set 0000 for a rising-only build.}

set output_width [tdc_ensure_long $core g_OUTPUT_WIDTH 32 32 128 \
    {AXI4-Stream output width (bits)} \
    {Width of both rising and falling output streams. Supported values are 32, 64, and 128 bits.}]
set_property value_validation_type list $output_width
set_property value_validation_list {32 64 128} $output_width

set num_chips [tdc_ensure_long $core g_NUM_CHIPS 4 1 4 \
    {Physical TDC-GPX chip count} \
    {Controls the number of physical TDC pin lanes exposed by the packaged IP.}]
set_property value_validation_type list $num_chips
set_property value_validation_list {1 2 3 4} $num_chips

set max_stops [tdc_ensure_long $core g_MAX_STOPS_PER_CHIP 8 2 8 \
    {Maximum STOP channels per chip} \
    {Compile-time STOP-channel capacity. Runtime stops_per_chip cannot exceed it.}]
set_property value_validation_type list $max_stops
set_property value_validation_list {2 3 4 5 6 7 8} $max_stops

set max_hits [tdc_ensure_long $core g_MAX_HITS_PER_STOP 7 1 7 \
    {Maximum hits per STOP} \
    {Compile-time per-STOP hit capacity. Runtime max_hits cannot exceed it.}]
set_property value_validation_type list $max_hits
set_property value_validation_list {1 2 3 4 5 6 7} $max_hits

foreach {name value display_name description} {
    g_AXIS_CLK_MHZ 150 {AXIS processing clock frequency (MHz)} {Must equal i_axis_aclk and must not exceed the TDC clock.}
    g_TDC_CLK_MHZ 200 {TDC bus clock frequency (MHz)} {Must equal i_tdc_clk. Physical timing generics are converted using this value.}
} {
    set parameter [tdc_ensure_long $core $name $value 50 200 $display_name $description]
    set_property value_validation_type list $parameter
    set_property value_validation_list {50 100 125 150 200} $parameter
    set_property value_validation_pairs [list \
        {50 MHz} 50 {100 MHz} 100 {125 MHz} 125 \
        {150 MHz} 150 {200 MHz} 200] $parameter
}

foreach {name value maximum display_name description} {
    g_POWERUP_TIME_NS 240 327675000 {Power-up wait (ns)} {TDC-domain wait after reset; local clocks are rounded up.}
    g_RECOVERY_TIME_NS 40 1000000 {Recovery wait (ns)} {TDC-domain recovery wait after a retry or reinitialization request.}
    g_ALU_PULSE_TIME_NS 20 1000000 {ALU trigger pulse (ns)} {TDC-domain ALUTRIGGER pulse width.}
    g_BUS_READ_PERIOD_MIN_TIME_NS 25 140 {Minimum bus read capture window (ns)} {Minimum RDN-low-to-input-capture interval. Runtime divider and ticks are clamped together; the converted count must fit the div=63/ticks=7 scheduler capacity.}
    g_BUS_IDLE_STABLE_TIME_NS 20480 2147483647 {Bus idle stable time (ns)} {Idle qualification before a new command sequence.}
    g_DRAIN_MARGIN_TIME_NS 6000 1000000 {Drain margin (ns)} {Extra TDC-domain allowance while draining IFIFO data. Return 7 at the default 200 MHz TDC timing requires the 6 us verified budget.}
    g_ERR_DEBOUNCE_TIME_NS 25 1000000 {Error debounce (ns)} {AXIS-domain error qualification time.}
    g_CELL_QUARANTINE_MARGIN_TIME_NS 3410 327675000 {Cell quarantine margin (ns)} {AXIS-domain late-data quarantine allowance.}
    g_CELL_IFIFO2_MARGIN_TIME_NS 1705 327675000 {IFIFO2 cell margin (ns)} {AXIS-domain IFIFO2 drain allowance.}
} {
    tdc_ensure_long $core $name $value 1 $maximum $display_name $description
}
tdc_ensure_long $core g_ERR_MAX_RETRIES 3 1 255 \
    {Maximum automatic retries} \
    {Number of recovery attempts before the error handler latches fatal state.}

set oen_mode [tdc_set_string $core g_OEN_MODE DYNAMIC_CONNECTED \
    {DYNAMIC_CONNECTED} {TDC data-bus output-enable mode} \
    {Fixed production mode. OEN follows the bus direction state machine.}]
set_property enablement_resolve_type immediate $oen_mode
set_property enablement_value false $oen_mode
tdc_set_string $core g_STREAM_CLK_MODE ASYNC {ASYNC SYNC} \
    {Internal stream clock mode} \
    {ASYNC supports the verified 150/200 MHz split. SYNC requires equal AXIS and TDC clocks.}

proc tdc_find_interface {core logical_name physical_name} {
    foreach interface [ipx::get_bus_interfaces -of_objects $core] {
        set map [ipx::get_port_maps -quiet $logical_name -of_objects $interface]
        if {[llength $map] == 1 &&
            [get_property physical_name $map] eq $physical_name} {
            return $interface
        }
    }
    error "No bus interface maps $logical_name to $physical_name"
}

proc tdc_set_bus_parameter {interface name value} {
    set parameter [ipx::get_bus_parameters -quiet $name -of_objects $interface]
    if {[llength $parameter] == 0} {
        ipx::add_bus_parameter $name $interface
        set parameter [ipx::get_bus_parameters -quiet $name -of_objects $interface]
    }
    if {[llength $parameter] != 1} {
        error "Expected one $name parameter on [get_property name $interface], found [llength $parameter]"
    }
    set_property value $value $parameter
    return $parameter
}

set chip_axi [tdc_find_interface $core AWADDR s_axi_awaddr]
set pipe_axi [tdc_find_interface $core AWADDR s_axi_pipe_awaddr]
set rise_axis [tdc_find_interface $core TDATA o_m_axis_tdata]
set fall_axis [tdc_find_interface $core TDATA o_m_axis_fall_tdata]
set axis_clock [tdc_find_interface $core CLK i_axis_aclk]
set tdc_clock [tdc_find_interface $core CLK i_tdc_clk]
set axi_clock [tdc_find_interface $core CLK s_axi_aclk]

# Vivado infers each AXI4-Stream from the o_m_axis* output prefix, but the
# corresponding READY ports intentionally use the i_m_axis* input prefix.
# Bind them explicitly so an IPI interface connection carries backpressure all
# the way into the RTL instead of leaving READY as an unrelated loose pin.
foreach {interface physical_name} [list \
        $rise_axis i_m_axis_tready \
        $fall_axis i_m_axis_fall_tready] {
    set ready_map [ipx::get_port_maps -quiet TREADY -of_objects $interface]
    if {[llength $ready_map] == 0} {
        ipx::add_port_map TREADY $interface
        set ready_map [ipx::get_port_maps -quiet TREADY \
            -of_objects $interface]
    }
    if {[llength $ready_map] != 1} {
        error "Expected one TREADY map on [get_property name $interface], found [llength $ready_map]"
    }
    set_property physical_name $physical_name $ready_map
}

set axis_frequency_dependency \
    {(spirit:decode(id('PARAM_VALUE.g_AXIS_CLK_MHZ')) * 1000000)}
foreach interface [list $rise_axis $fall_axis $axis_clock] {
    set parameter [tdc_set_bus_parameter $interface FREQ_HZ 150000000]
    set_property value_format long $parameter
    set_property value_resolve_type dependent $parameter
    set_property value_dependency $axis_frequency_dependency $parameter
    set_property value_source default $parameter
}
set tdc_frequency_dependency \
    {(spirit:decode(id('PARAM_VALUE.g_TDC_CLK_MHZ')) * 1000000)}
set parameter [tdc_set_bus_parameter $tdc_clock FREQ_HZ 200000000]
set_property value_format long $parameter
set_property value_resolve_type dependent $parameter
set_property value_dependency $tdc_frequency_dependency $parameter
set_property value_source default $parameter

tdc_set_bus_parameter $axis_clock ASSOCIATED_BUSIF \
    "[get_property name $rise_axis]:[get_property name $fall_axis]"
tdc_set_bus_parameter $axis_clock ASSOCIATED_RESET i_axis_aresetn
tdc_set_bus_parameter $axi_clock ASSOCIATED_BUSIF \
    "[get_property name $chip_axi]:[get_property name $pipe_axi]"
tdc_set_bus_parameter $axi_clock ASSOCIATED_RESET s_axi_aresetn

# AXI-Lite CSR logic is frequency-independent. Let IPI propagate the connected
# PS FCLK instead of advertising a fixed frequency.
set axi_freq [ipx::get_bus_parameters -quiet FREQ_HZ -of_objects $axi_clock]
if {[llength $axi_freq] == 1} {
    ipx::remove_bus_parameter FREQ_HZ $axi_clock
}

set_property connection_required false $fall_axis
foreach irq_port {o_irq o_irq_pipe} {
    set irq_if {}
    foreach candidate [ipx::get_bus_interfaces -of_objects $core] {
        set map [ipx::get_port_maps -quiet INTERRUPT -of_objects $candidate]
        if {[llength $map] == 1 &&
            [get_property physical_name $map] eq $irq_port} {
            set irq_if $candidate
            break
        }
    }
    if {$irq_if eq {}} {
        set irq_if [ipx::add_bus_interface $irq_port $core]
        set_property abstraction_type_vlnv \
            xilinx.com:signal:interrupt_rtl:1.0 $irq_if
        set_property bus_type_vlnv xilinx.com:signal:interrupt:1.0 $irq_if
        set_property interface_mode master $irq_if
        set irq_map [ipx::add_port_map INTERRUPT $irq_if]
        set_property physical_name $irq_port $irq_map
    }
    set_property connection_required false $irq_if
    tdc_set_bus_parameter $irq_if SENSITIVITY LEVEL_HIGH
}

# -----------------------------------------------------------------------------
# Mutually exclusive local/unified control-plane interfaces.
# The same generic controls RTL generate branches and IP-XACT visibility so a
# hidden AXI port can never leave an active local CSR block in the netlist.
# -----------------------------------------------------------------------------
proc tdc_set_enablement {object dependency default_value} {
    set_property enablement_resolve_type dependent $object
    set_property enablement_dependency $dependency $object
    set_property enablement_value $default_value $object
}

proc tdc_set_interface_enablement {core interface dependency default_value} {
    tdc_set_enablement $interface $dependency $default_value
    foreach port_map [ipx::get_port_maps -of_objects $interface] {
        set physical_name [get_property physical_name $port_map]
        set physical_port [ipx::get_ports -quiet $physical_name \
            -of_objects $core]
        if {[llength $physical_port] != 1} {
            error "Expected one physical port named $physical_name"
        }
        tdc_set_enablement $physical_port $dependency $default_value
    }
}

proc tdc_ensure_scalar_interface {
    core name logical_name physical_name bus_vlnv abstraction_vlnv
} {
    set interface [ipx::get_bus_interfaces -quiet $name -of_objects $core]
    if {[llength $interface] == 0} {
        set interface [ipx::add_bus_interface $name $core]
    } elseif {[llength $interface] != 1} {
        error "Expected at most one interface named $name"
    }
    set_property interface_mode slave $interface
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

set local_csr_dependency \
    {spirit:decode(id('MODELPARAM_VALUE.g_ENABLE_LOCAL_CSR')) = true}
set unified_csr_dependency \
    {spirit:decode(id('MODELPARAM_VALUE.g_ENABLE_LOCAL_CSR')) = false}

set local_reset [tdc_ensure_scalar_interface $core s_axi_aresetn RST \
    s_axi_aresetn {xilinx.com:signal:reset:1.0} \
    {xilinx.com:signal:reset_rtl:1.0}]
tdc_set_bus_parameter $local_reset POLARITY ACTIVE_LOW

foreach local_interface [list $chip_axi $pipe_axi $axi_clock $local_reset \
        [ipx::get_bus_interfaces o_irq -of_objects $core] \
        [ipx::get_bus_interfaces o_irq_pipe -of_objects $core]] {
    tdc_set_interface_enablement $core $local_interface \
        $local_csr_dependency true
}

foreach inferred_name {i_unified o_unified} {
    set inferred_interface [ipx::get_bus_interfaces -quiet $inferred_name \
        -of_objects $core]
    if {[llength $inferred_interface] == 1} {
        ipx::remove_bus_interface $inferred_name $core
    } elseif {[llength $inferred_interface] != 0} {
        error "Expected at most one inferred interface named $inferred_name"
    }
}
set old_unified [ipx::get_bus_interfaces -quiet tdc_unified_csr \
    -of_objects $core]
if {[llength $old_unified] == 1} {
    ipx::remove_bus_interface tdc_unified_csr $core
} elseif {[llength $old_unified] != 0} {
    error {Expected at most one existing tdc_unified_csr interface}
}
set unified_interface [ipx::add_bus_interface tdc_unified_csr $core]
set_property display_name {TDC-GPX Unified CSR} $unified_interface
set_property description \
    {Named TDC control/status words connected to the parent unified CSR owner.} \
    $unified_interface
set_property interface_mode slave $unified_interface
set_property bus_type_vlnv \
    {victek.co.kr:interface:tdc_gpx_unified_csr:1.0} $unified_interface
set_property abstraction_type_vlnv \
    {victek.co.kr:interface:tdc_gpx_unified_csr_rtl:1.0} $unified_interface
set_property connection_required true $unified_interface

foreach {logical_name physical_name} {
    SYS_CTRL                    i_unified_sys_ctrl
    SYS_CFG_APPLY               i_unified_sys_cfg_apply
    TDC_BUS_TIMING              i_unified_tdc_bus_timing
    TDC_START_OFFSET            i_unified_tdc_start_offset
    TDC_CFG_REG7                i_unified_tdc_cfg_reg7
    TDC_IMAGE_CMD               i_unified_tdc_image_cmd
    TDC_IMAGE_DATA              i_unified_tdc_image_data
    TDC_SCAN_CFG                i_unified_tdc_scan_cfg
    TDC_PIPELINE_MAIN           i_unified_tdc_pipeline_main
    TDC_RANGE_COLS              i_unified_tdc_range_cols
    TDC_AUX_CMD                 i_unified_tdc_aux_cmd
    TDC_CHIP0_RESULT            o_unified_tdc_chip0_result
    TDC_CHIP1_RESULT            o_unified_tdc_chip1_result
    TDC_CHIP2_RESULT            o_unified_tdc_chip2_result
    TDC_CHIP3_RESULT            o_unified_tdc_chip3_result
    TDC_PIPELINE_STATUS         o_unified_tdc_pipeline_status
    TDC_STATUS_EXT              o_unified_tdc_status_ext
    TDC_STATUS_EXT2             o_unified_tdc_status_ext2
    CFG_EPOCH_ACCEPTED          o_unified_cfg_epoch_accepted
    RESET_EPOCH_ACCEPTED        o_unified_reset_epoch_accepted
    CFG_BUSY                    o_unified_cfg_busy
    CFG_REJECT                  o_unified_cfg_reject
    CFG_VALID                   o_unified_cfg_valid
    CMD_EPOCH_ACCEPTED          o_unified_cmd_epoch_accepted
    CMD_BUSY                    o_unified_cmd_busy
    COMMAND_REJECT              o_unified_command_reject
    IMAGE_EPOCH_ACCEPTED        o_unified_image_epoch_accepted
    IMAGE_REJECT                o_unified_image_reject
    IMAGE_SELECTED_DATA         o_unified_image_selected_data
    IRQ_CAUSE                   o_unified_irq_cause
} {
    set port_map [ipx::add_port_map $logical_name $unified_interface]
    set_property logical_name $logical_name $port_map
    set_property physical_name $physical_name $port_map
}

set unified_clock [tdc_ensure_scalar_interface $core i_unified_cfg_clk CLK \
    i_unified_cfg_clk {xilinx.com:signal:clock:1.0} \
    {xilinx.com:signal:clock_rtl:1.0}]
set unified_reset [tdc_ensure_scalar_interface $core i_unified_cfg_rst_n RST \
    i_unified_cfg_rst_n {xilinx.com:signal:reset:1.0} \
    {xilinx.com:signal:reset_rtl:1.0}]
set unified_freq [ipx::get_bus_parameters -quiet FREQ_HZ \
    -of_objects $unified_clock]
if {[llength $unified_freq] == 1} {
    ipx::remove_bus_parameter FREQ_HZ $unified_clock
}
tdc_set_bus_parameter $unified_clock ASSOCIATED_BUSIF tdc_unified_csr
tdc_set_bus_parameter $unified_clock ASSOCIATED_RESET i_unified_cfg_rst_n
tdc_set_bus_parameter $unified_reset POLARITY ACTIVE_LOW
foreach interface [list $unified_interface $unified_clock $unified_reset] {
    tdc_set_interface_enablement $core $interface \
        $unified_csr_dependency false
}

# Rebuild source views so the package is deterministic and has no child XCI.
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
        set packaged_file [ipx::add_file src/csr/$filename $file_group]
        set_property type vhdlSource-2008 $packaged_file
    }
    foreach filename $rtl_files {
        set packaged_file [ipx::add_file src/$filename $file_group]
        if {$filename eq {tdc_gpx_top.vhd}} {
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
set xgui_file [ipx::add_file xgui/tdc_gpx_top_v1_0.tcl $xgui_group]
set_property type tclSource $xgui_file
set_property xgui_version 2 $xgui_file

foreach file_group [ipx::get_file_groups -of_objects $core] {
    foreach packaged_file [ipx::get_files -of_objects $file_group] {
        set packaged_name [get_property name $packaged_file]
        if {[string match "../*" $packaged_name]} {
            error "Package contains an external source reference: $packaged_name"
        }
        if {[string match "*.xci" $packaged_name]} {
            error "Package contains a generated child IP: $packaged_name"
        }
    }
}

foreach required_port {
    i_axis_aclk i_tdc_clk s_axi_aclk i_unified_cfg_clk io_tdc_d o_tdc_csn
    i_shot_start i_stop_tdc o_m_axis_tdata i_m_axis_tready
    o_m_axis_fall_tdata i_m_axis_fall_tready
    o_vdma_hsize_bytes_rise o_irq o_irq_pipe
    i_unified_sys_ctrl o_unified_tdc_pipeline_status
} {
    if {[llength [ipx::get_ports -quiet $required_port -of_objects $core]] != 1} {
        error "Required packaged port is missing: $required_port"
    }
}

# Delay catalog registration until after package_project has inferred the
# standard interfaces. Otherwise Vivado splits the i_unified/o_unified naming
# prefixes into two incomplete custom interfaces before our bidirectional map
# is installed.
set_property ip_repo_paths [list $package_dir] [current_project]
update_ip_catalog -rebuild

ipx::update_checksums $core
set drc_result [ipx::check_integrity -verbose $core]
puts "TDC_GPX_IP_PACKAGER_DRC=$drc_result"
ipx::save_core $core
ipx::unload_abstraction_definition $unified_abstraction_definition
ipx::unload_bus_definition $unified_bus_definition
close_project

set component [file join $package_dir component.xml]
set fh [open $component r]
set component_text [read $fh]
close $fh
foreach forbidden {.xci ../ tdc_gpx_axil_csr_pipeline tdc_gpx_axil_csr32_chip} {
    if {[string first $forbidden $component_text] >= 0} {
        error "Forbidden packaged dependency remains: $forbidden"
    }
}

puts "TDC_GPX_IP_COMPONENT=$component"
puts {TDC_GPX_IP_PACKAGE_PASS}
exit
