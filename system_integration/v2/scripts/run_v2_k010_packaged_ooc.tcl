# Synthesize the exact self-contained package source set for the three K0-10
# representative clock, stream-mode, width and Echo-frontend configurations.

set script_dir [file normalize [file dirname [info script]]]
set v2_dir [file normalize [file join $script_dir ..]]
set hdl_root [file normalize [file join $v2_dir ../..]]
set package_dir [file join $v2_dir ip_repo tdc_gpx_lidar_ctrl_v2_2_0]
if {[llength $argv] > 0} {
    set out_dir [file normalize [lindex $argv 0]]
} else {
    set out_dir [file join $hdl_root signoff_results sessions \
        v2_k010_packaged_ooc]
}
file mkdir $out_dir

# Avoid the damaged per-user Tcl Store on this workstation before Vivado
# initializes project commands used by the synthesis loop.  Source the
# installation copy directly: package require would invoke the damaged
# per-user package resolver again before it can use the appended auto_path.
set install_tcl_store [file normalize \
    [file join $::env(XILINX_VIVADO) data XilinxTclStore]]
set install_appinit [file join $install_tcl_store support appinit appinit.tcl]
if {![file exists $install_appinit]} {
    error "Vivado installation Tcl Store appinit is missing: $install_appinit"
}
foreach vendor_dir [glob -nocomplain -type d \
        [file join $install_tcl_store tclapp *]] {
    foreach app_dir [glob -nocomplain -type d [file join $vendor_dir *]] {
        lappend auto_path $app_dir
    }
}
source $install_appinit
set_msg_config -id {Synth 8-5799} -limit 5
set_msg_config -id {Constraints 18-5572} -limit 5

set manifest_source [file join $v2_dir ip_package \
    v2_ip_package_manifest.tcl]
source $manifest_source
set entries [lidar_v2_ip_package_manifest $hdl_root]
if {[llength $entries] != 87} {
    error "Expected 87 packaged RTL sources, got [llength $entries]"
}
set packaged_sources {}
foreach entry $entries {
    lassign $entry canonical relative
    set packaged [file join $package_dir src $relative]
    if {![file exists $packaged]} {
        error "Packaged RTL source is missing: $packaged"
    }
    lappend packaged_sources $packaged
}

proc v2_require_port_width {name expected} {
    set ports [get_ports -quiet $name]
    if {[llength $ports] != $expected} {
        error "$name width is [llength $ports], expected $expected"
    }
}

foreach profile {
    {async32_tdc_faster 150 200 ASYNC 32 true}
    {async128_proc_faster 200 150 ASYNC 128 false}
    {sync64_equal 150 150 SYNC 64 true}
} {
    lassign $profile label proc_mhz tdc_mhz mode width echo_enabled
    create_project -in_memory -part xc7z020clg484-2
    set_property target_language VHDL [current_project]
    set_property simulator_language Mixed [current_project]
    set_property default_lib xil_defaultlib [current_project]
    foreach source $packaged_sources {
        read_vhdl -vhdl2008 -library xil_defaultlib $source
    }
    update_compile_order -fileset sources_1

    set generics [list \
        {G_CSR_CLK_MHZ=100} \
        "G_PROC_CLK_MHZ=$proc_mhz" \
        "G_TDC_CLK_MHZ=$tdc_mhz" \
        "G_STREAM_CLK_MODE=$mode" \
        {G_NUM_CHIPS=4} \
        {G_STOPS_PER_CHIP=8} \
        {G_MAX_RETURNS_PER_STOP=7} \
        {G_RISE_CAPABILITY_MASK=4'b0011} \
        {G_FALL_CAPABILITY_MASK=4'b1100} \
        "G_OUTPUT_WIDTH=$width" \
        {G_NUM_FACES=5} \
        "G_ENABLE_ECHO_RECEIVER=$echo_enabled" \
        {G_ENABLE_ECHO_SIMULATION=false} \
        {G_OEN_MODE=DYNAMIC_CONNECTED} \
        {G_PHASE_TIMEOUT_US=1000} \
        {G_POWERUP_TIME_NS=240} \
        {G_RECOVERY_TIME_NS=40} \
        {G_ALU_PULSE_TIME_NS=20} \
        {G_BUS_IDLE_STABLE_TIME_NS=20480} \
        {G_DRAIN_MARGIN_TIME_NS=6000}]

    synth_design -top tdc_gpx_lidar_ctrl_v2_top \
        -part xc7z020clg484-2 -mode out_of_context \
        -flatten_hierarchy rebuilt -generic $generics

    set black_boxes [get_cells -quiet -hier -filter {IS_BLACKBOX == 1}]
    set latches [get_cells -quiet -hier \
        -filter {REF_NAME == LDCE || REF_NAME == LDPE}]
    if {[llength $black_boxes] != 0} {
        error "$label contains black boxes: $black_boxes"
    }
    if {[llength $latches] != 0} {
        error "$label contains inferred latches: $latches"
    }

    v2_require_port_width io_tdc_d 112
    v2_require_port_width o_tdc_adr 16
    v2_require_port_width o_tdc_csn 4
    v2_require_port_width i_pd_lvds_p 32
    v2_require_port_width i_pd_lvds_n 32
    v2_require_port_width o_tdc_stop 32
    v2_require_port_width m_axis_monitor_tdata 64
    foreach port {m_axis_rise_tdata m_axis_fall_tdata} {
        v2_require_port_width $port $width
    }
    foreach port {
        m_axis_rise_tkeep m_axis_rise_tstrb
        m_axis_fall_tkeep m_axis_fall_tstrb
    } {
        v2_require_port_width $port [expr {$width / 8}]
    }

    report_utilization -hierarchical \
        -file [file join $out_dir utilization_${label}.rpt]
    puts "LIDAR_V2_K010_PACKAGED_OOC_PASS profile=$label proc_mhz=$proc_mhz tdc_mhz=$tdc_mhz mode=$mode width=$width echo=$echo_enabled"
    close_project
}

puts {LIDAR_V2_K010_PACKAGED_OOC_MATRIX_PASS profiles=3}
exit 0
