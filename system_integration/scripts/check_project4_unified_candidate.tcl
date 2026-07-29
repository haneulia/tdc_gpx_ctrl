# Read-only structural checker for project_4/design_1_unified.

proc assert_count {objects expected label} {
    set actual [llength $objects]
    if {$actual != $expected} {
        error "$label count mismatch: expected $expected, got $actual ($objects)"
    }
}

proc require_one {objects label} {
    assert_count $objects 1 $label
    return [lindex $objects 0]
}

proc assert_equal {actual expected label} {
    if {$actual ne $expected} {
        error "$label mismatch: expected '$expected', got '$actual'"
    }
}

proc assert_config {cell_name property expected} {
    set cell [require_one [get_bd_cells -quiet $cell_name] \
        "cell $cell_name"]
    assert_equal [get_property CONFIG.$property $cell] $expected \
        "$cell_name CONFIG.$property"
}

proc assert_same_net {first_name second_name label} {
    set first [require_one [get_bd_pins -quiet $first_name] \
        "pin $first_name"]
    set second [require_one [get_bd_pins -quiet $second_name] \
        "pin $second_name"]
    set first_net [require_one [get_bd_nets -quiet -of_objects $first] \
        "net of $first_name"]
    set second_net [require_one [get_bd_nets -quiet -of_objects $second] \
        "net of $second_name"]
    assert_equal $first_net $second_net $label
}

proc assert_same_intf_net {first_name second_name label} {
    set first [require_one [get_bd_intf_pins -quiet $first_name] \
        "interface $first_name"]
    set second [require_one [get_bd_intf_pins -quiet $second_name] \
        "interface $second_name"]
    set first_net [require_one \
        [get_bd_intf_nets -quiet -of_objects $first] \
        "interface net of $first_name"]
    set second_net [require_one \
        [get_bd_intf_nets -quiet -of_objects $second] \
        "interface net of $second_name"]
    assert_equal $first_net $second_net $label
}

proc object_width {object} {
    set left [get_property LEFT $object]
    set right [get_property RIGHT $object]
    if {$left eq {} || $right eq {}} {
        return 1
    }
    return [expr {abs($left - $right) + 1}]
}

if {[llength $argv] > 0} {
    set project_dir [file normalize [lindex $argv 0]]
} else {
    set project_dir {C:/Projects/my_sp/ALINX/Logic/project_4}
}
set project_xpr [file join $project_dir project_4.xpr]
set candidate_bd [file join $project_dir project_4.srcs sources_1 bd \
    design_1_unified design_1_unified.bd]
set clock_domain_xdc [file join $project_dir project_4.srcs constrs_1 \
    imports XDC project4_unified_clock_domains.xdc]
set candidate_constrset_name design_1_unified_constrs
foreach required [list $project_xpr $candidate_bd $clock_domain_xdc] {
    if {![file exists $required]} {
        error "Required candidate artifact is missing: $required"
    }
}

open_project $project_xpr
set candidate_constrset [require_one \
    [get_filesets -quiet $candidate_constrset_name] \
    {unified candidate constraint set}]
assert_count [get_files -quiet -of_objects [get_filesets constrs_1] \
    $clock_domain_xdc] 0 {baseline clock-domain constraint}
set clock_domain_file [require_one [get_files -quiet \
    -of_objects $candidate_constrset $clock_domain_xdc] \
    {unified clock-domain constraint}]
assert_equal [get_property PROCESSING_ORDER $clock_domain_file] LATE \
    {unified clock-domain processing order}
set channel [open $clock_domain_xdc r]
set clock_domain_text [read $channel]
close $channel
foreach contract_text {
    clk_out1_design_1_unified_proc_clk_wiz_0
    clk_fpga_1
    {set_clock_groups -name lidar_unified_async_domains -asynchronous}
    {-group [get_clocks clk_fpga_0]}
    {-group [get_clocks clk_fpga_2]}
} {
    if {[string first $contract_text $clock_domain_text] < 0} {
        error "Unified clock-domain XDC lacks contract text: $contract_text"
    }
}
set baseline_synth [require_one [get_runs -quiet synth_1] \
    {baseline synthesis run}]
assert_equal [get_property CONSTRSET $baseline_synth] constrs_1 \
    {baseline synthesis constraint set}
open_bd_design $candidate_bd

foreach cell_name {
    processing_system7_0
    axi_interconnect_0
    motor_laser_ctrl_top_0
    echo_receiver_top_0
    tdc_gpx_top_0
    lidar_unified_csr_0
    proc_clk_wiz
} {
    assert_count [get_bd_cells -quiet $cell_name] 1 "required $cell_name"
}
foreach cell_name {
    virtual_encoder_top_0
    motor_decoder_top_0
    laser_ctrl_top_0
    system_ila_0
} {
    assert_count [get_bd_cells -quiet $cell_name] 0 "retired $cell_name"
}

assert_config axi_interconnect_0 NUM_MI 3
assert_config processing_system7_0 PCW_FPGA1_PERIPHERAL_FREQMHZ 100
assert_config proc_clk_wiz PRIM_IN_FREQ 100.000
assert_config proc_clk_wiz CLKOUT1_REQUESTED_OUT_FREQ 150.000
assert_config motor_laser_ctrl_top_0 g_ENABLE_LOCAL_CSR false
assert_config motor_laser_ctrl_top_0 g_PROC_CLK_MHZ 150
assert_config echo_receiver_top_0 g_ENABLE_LOCAL_CSR false
assert_config echo_receiver_top_0 g_AXIS_CLK_MHZ 150
assert_config echo_receiver_top_0 g_ENABLE_SIM_PATH false
assert_config tdc_gpx_top_0 g_ENABLE_LOCAL_CSR false
assert_config tdc_gpx_top_0 g_AXIS_CLK_MHZ 150
assert_config tdc_gpx_top_0 g_TDC_CLK_MHZ 200
assert_config tdc_gpx_top_0 g_STREAM_CLK_MODE ASYNC
assert_config tdc_gpx_top_0 g_OUTPUT_WIDTH 32
assert_config tdc_gpx_top_0 g_NUM_CHIPS 4
assert_config tdc_gpx_top_0 g_PRESENT_CHIP_MASK {"1111"}
assert_config tdc_gpx_top_0 g_RISE_CHIP_MASK {"0011"}
assert_config tdc_gpx_top_0 g_FALL_CHIP_MASK {"1100"}
assert_config tdc_gpx_top_0 g_MAX_STOPS_PER_CHIP 8
assert_config tdc_gpx_top_0 g_MAX_HITS_PER_STOP 7

foreach local_interface {
    motor_laser_ctrl_top_0/s_axi_motor
    motor_laser_ctrl_top_0/s_axi_laser
    echo_receiver_top_0/s_axi
    tdc_gpx_top_0/s_axi
} {
    assert_count [get_bd_intf_pins -quiet $local_interface] 0 \
        "disabled local CSR $local_interface"
}

foreach {master slave label} {
    lidar_unified_csr_0/motor_unified_csr \
        motor_laser_ctrl_top_0/motor_unified_csr {Motor CSR plane}
    lidar_unified_csr_0/laser_unified_csr \
        motor_laser_ctrl_top_0/laser_unified_csr {Laser CSR plane}
    lidar_unified_csr_0/echo_unified_csr \
        echo_receiver_top_0/echo_unified_csr {Echo CSR plane}
    lidar_unified_csr_0/tdc_unified_csr \
        tdc_gpx_top_0/tdc_unified_csr {TDC CSR plane}
} {
    assert_same_intf_net $master $slave $label
}
assert_same_intf_net axi_interconnect_0/M00_AXI \
    lidar_unified_csr_0/s_axi_csr {central AXI-Lite path}

foreach {source sink label} {
    motor_laser_ctrl_top_0/o_shot_start \
        echo_receiver_top_0/i_shot_start {shot marker to Echo}
    motor_laser_ctrl_top_0/o_stop_tdc \
        echo_receiver_top_0/i_stop_tdc {stop marker to Echo}
    motor_laser_ctrl_top_0/o_shot_start \
        tdc_gpx_top_0/i_shot_start {shot marker to TDC}
    motor_laser_ctrl_top_0/o_stop_tdc \
        tdc_gpx_top_0/i_stop_tdc {stop marker to TDC}
    motor_laser_ctrl_top_0/o_shot_face_index \
        tdc_gpx_top_0/i_shot_face_index {face index to TDC}
    motor_laser_ctrl_top_0/o_n_faces \
        tdc_gpx_top_0/i_n_faces {face count to TDC}
} {
    assert_same_net $source $sink $label
}
assert_same_intf_net motor_laser_ctrl_top_0/m_axis \
    echo_receiver_top_0/s_laser_evt {Motor/Laser event stream}

foreach sink {
    motor_laser_ctrl_top_0/proc_aclk
    echo_receiver_top_0/axis_aclk
    tdc_gpx_top_0/i_axis_aclk
    system_ila_2/clk
    rst_ps7_0_100M/slowest_sync_clk
} {
    assert_same_net proc_clk_wiz/clk_out1 $sink "150 MHz clock $sink"
}
assert_same_net processing_system7_0/FCLK_CLK1 proc_clk_wiz/clk_in1 \
    {100 MHz PS FCLK1 source}
assert_same_net proc_clk_wiz/locked rst_ps7_0_100M/dcm_locked \
    {150 MHz reset lock}
assert_same_net processing_system7_0/FCLK_CLK2 tdc_gpx_top_0/i_tdc_clk \
    {200 MHz TDC clock}

foreach {external internal label} {
    m_axis_tdc_rise tdc_gpx_top_0/o_m_axis {rising result AXIS}
    m_axis_tdc_fall tdc_gpx_top_0/o_m_axis_fall {falling result AXIS}
} {
    set external_port [require_one [get_bd_intf_ports -quiet $external] \
        "external interface $external"]
    assert_equal [get_property MODE $external_port] Master "$label mode"
    set internal_pin [require_one [get_bd_intf_pins -quiet $internal] \
        "internal interface $internal"]
    set external_net [require_one \
        [get_bd_intf_nets -quiet -of_objects $external_port] \
        "$label external net"]
    set internal_net [require_one \
        [get_bd_intf_nets -quiet -of_objects $internal_pin] \
        "$label internal net"]
    assert_equal $external_net $internal_net $label
}
foreach {external_port internal_pin label} {
    m_axis_tdc_rise_tready tdc_gpx_top_0/i_m_axis_tready \
        {rising TREADY}
    m_axis_tdc_fall_tready tdc_gpx_top_0/i_m_axis_fall_tready \
        {falling TREADY}
} {
    set port [require_one [get_bd_ports -quiet $external_port] \
        "external port $external_port"]
    set pin [require_one [get_bd_pins -quiet $internal_pin] \
        "internal pin $internal_pin"]
    assert_equal [get_property DIR $port] I "$label external direction"
    assert_equal [get_property DIR $pin] I "$label internal direction"
}

set generated_vhdl [file join $project_dir project_4.gen sources_1 bd \
    design_1_unified synth design_1_unified.vhd]
if {![file exists $generated_vhdl]} {
    error "Generated candidate VHDL is missing: $generated_vhdl"
}
set channel [open $generated_vhdl r]
set generated_text [read $channel]
close $channel
foreach mapping {
    {i_m_axis_tready => m_axis_tdc_rise_tready}
    {i_m_axis_fall_tready => m_axis_tdc_fall_tready}
} {
    if {[string first $mapping $generated_text] < 0} {
        error "Generated candidate VHDL lacks AXIS mapping: $mapping"
    }
}

foreach {relative_path mappings} [list \
    [file join ip design_1_unified_tdc_gpx_top_0_0 synth \
        design_1_unified_tdc_gpx_top_0_0.vhd] [list \
        {g_RISE_CHIP_MASK => B"0011"} \
        {g_FALL_CHIP_MASK => B"1100"} \
        {g_PRESENT_CHIP_MASK => B"1111"}] \
    [file join ip design_1_unified_lidar_unified_csr_0_0 synth \
        design_1_unified_lidar_unified_csr_0_0.vhd] [list \
        {g_VERSION_WORD => B"01001100000000010000000000000000"} \
        {g_CAPABILITY_WORD => B"00000001000001000001101011111111"}]] {
    set wrapper [file join $project_dir project_4.gen sources_1 bd \
        design_1_unified $relative_path]
    if {![file exists $wrapper]} {
        error "Generated IP wrapper is missing: $wrapper"
    }
    set channel [open $wrapper r]
    set wrapper_text [read $channel]
    close $channel
    foreach mapping $mappings {
        if {[string first $mapping $wrapper_text] < 0} {
            error "Generated IP wrapper lacks VHDL vector mapping: $mapping"
        }
    }
}

foreach {port_name expected_width} {
    io_tdc_d 112
    o_tdc_adr 16
    o_tdc_csn 4
    o_tdc_rdn 4
    o_tdc_wrn 4
    o_tdc_oen 4
    o_tdc_stopdis 4
    o_tdc_alutrigger 4
    o_tdc_puresn 4
    i_tdc_ef1 4
    i_tdc_ef2 4
    i_tdc_lf1 4
    i_tdc_lf2 4
    i_tdc_irflag 4
    i_tdc_errflag 4
} {
    set port [require_one [get_bd_ports -quiet $port_name] \
        "external port $port_name"]
    assert_equal [object_width $port] $expected_width "$port_name width"
}

set address_segment [require_one [get_bd_addr_segs -quiet \
    processing_system7_0/Data/SEG_lidar_unified_csr_0_reg0] \
    {unified CSR address segment}]
assert_equal [get_property OFFSET $address_segment] 0x40000000 \
    {unified CSR base address}
assert_count [get_bd_addr_segs -quiet \
    -of_objects [get_bd_addr_spaces processing_system7_0/Data]] 3 \
    {PS mapped segments}

foreach stale_net {
    echo_receiver_top_0_o_irq
    laser_ctrl_top_0_o_irq
    motor_decoder_top_0_irq
    motor_laser_ctrl_top_0_o_irq
} {
    assert_count [get_bd_nets -quiet $stale_net] 0 "stale net $stale_net"
}

validate_bd_design -force
puts {PROJECT4_UNIFIED_CANDIDATE_CHECK_PASS}
puts {PROJECT4_UNIFIED_CSR_BASE_CHECK=0x40000000}
puts {PROJECT4_UNIFIED_PROCESSING_CLOCK_CHECK=150MHz}
puts {PROJECT4_UNIFIED_TDC_CLOCK_CHECK=200MHz}
close_project
exit
