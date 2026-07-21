# =============================================================================
# create_parent_ref.tcl
#
# Rebuild a board-independent Zynq-7020 parent reference for tdc_gpx_top.
# The reference owns PS FCLKs, domain resets, AXI-Lite control, two independent
# S2MM VDMA paths, DDR HP ports, geometry readback, and interrupt aggregation.
# TDC package pins and I/O standards intentionally remain board-owned.
#
# Usage:
#   vivado -mode batch -source create_parent_ref.tcl \
#       -tclargs BUILD_DIR RESULT_DIR RUN_MODE
#
# RUN_MODE: VALIDATE, SYNTH, or IMPL
# =============================================================================

if {$argc != 3} {
    error "usage: create_parent_ref.tcl BUILD_DIR RESULT_DIR VALIDATE|SYNTH|IMPL"
}

set build_dir  [file normalize [lindex $argv 0]]
set result_dir [file normalize [lindex $argv 1]]
set run_mode   [string toupper [lindex $argv 2]]

if {$run_mode ni {VALIDATE SYNTH IMPL}} {
    error "unsupported RUN_MODE '$run_mode'"
}

set script_dir [file dirname [file normalize [info script]]]
set parent_dir [file normalize [file join $script_dir ..]]
set hdl_dir    [file normalize [file join $parent_dir ..]]
set project_name tdc_gpx_parent_ref
set project_dir  $build_dir
set part_name    xc7z020clg484-2
set bd_name      tdc_gpx_parent

set parent_manual_cdc_groups [list \
    [list tdc_pin_status \
        {^.*/u_config_ctrl/.*u_bus_phy/s_(ef1|ef2|lf1|lf2|irflag|errflag)_meta_r_reg/D$} 24 24] \
    [list diagnostic_status \
        {^.*/u_config_ctrl/s_(cmd_collision|err_bus_fatal|init_cfg_coalesced)_meta_r_reg\[[0-9]+\]/D$} 12 12] \
    [list csr_idle \
        {^.*/s_cdc_all_idle_ff_reg\[0\]/D$} 2 2] \
    [list command_toggle \
        {^.*/s_cmd_(soft_reset|force_reinit)_sync_tdc_r_reg\[0\]/D$} 2 2]]

proc verify_parent_manual_cdc {stage report_path groups} {
    set report [open $report_path w]
    fconfigure $report -encoding ascii -translation lf
    set detail_path [file join [file dirname $report_path] ${stage}_manual_cdc_detail.rpt]
    set detail [open $detail_path w]
    fconfigure $detail -encoding ascii -translation lf
    set expected_total 40
    set total 0

    foreach spec $groups {
        lassign $spec group pattern synth_count route_count
        set expected_count [expr {$stage eq "post_route" ? $route_count : $synth_count}]
        set pins [get_pins -quiet -hier -regexp $pattern]
        set actual_count [llength $pins]
        if {$actual_count != $expected_count} {
            puts $report "FAIL stage=$stage group=$group expected=$expected_count actual=$actual_count"
            close $detail
            close $report
            error "parent CDC group '$group' expected $expected_count first-stage pins, got $actual_count"
        }

        set cells [get_cells -quiet -of_objects $pins]
        set async_count 0
        foreach async_reg [get_property ASYNC_REG $cells] {
            if {[string toupper $async_reg] in {1 TRUE}} {
                incr async_count
            }
        }
        if {[llength $cells] != $actual_count || $async_count != $actual_count} {
            puts $report "FAIL stage=$stage group=$group cells=[llength $cells] async_reg_cells=$async_count"
            close $detail
            close $report
            error "parent CDC group '$group' matched a non-ASYNC_REG first-stage pin"
        }

        incr total $actual_count
        foreach pin $pins {
            puts $detail "PIN stage=$stage group=$group name=$pin"
        }
        puts $report "PASS stage=$stage group=$group first_stage_pins=$actual_count async_reg_pins=$actual_count"
    }

    if {$total != $expected_total} {
        puts $report "FAIL stage=$stage total_first_stage_pins=$total expected=$expected_total"
        close $detail
        close $report
        error "parent CDC first-stage total expected $expected_total pins, got $total"
    }
    puts $report "PASS stage=$stage total_first_stage_pins=$total"
    close $detail
    close $report
    puts "PARENT_CDC_CONSTRAINT stage=$stage first_stage_pins=$total"
}

proc report_parent_control_sets {stage result_dir} {
    set cfg_cells [get_cells -quiet -hier -filter {
        NAME =~ */u_tdc_gpx/u_config_ctrl/*
    }]
    if {[llength $cfg_cells] == 0} {
        error "parent control-set report found no u_config_ctrl leaf cells"
    }

    report_control_sets -hierarchical -hierarchical_depth 8 \
        -file [file join $result_dir ${stage}_control_sets_hier.rpt]
    report_control_sets -verbose -sort_by {clk clkEn set} -cells $cfg_cells \
        -file [file join $result_dir ${stage}_config_ctrl_control_sets.rpt]
    puts "PARENT_CONTROL_SETS stage=$stage cfg_cells=[llength $cfg_cells]"
}

set ip_repo_axil   C:/Projects/my_sp/lib/IP/my_axil_csr/ip_repo
set ip_repo_axil32 C:/Projects/my_sp/lib/IP/my_axil_csr32/ip_repo

file mkdir $build_dir
file mkdir $result_dir

puts "PARENT_REF_CONFIG part=$part_name axi=100MHz axis=150MHz tdc=200MHz width=32 present=1111 rise=0011 fall=1100 mode=$run_mode"

create_project $project_name $project_dir -part $part_name -force
set_property target_language VHDL [current_project]
set_property simulator_language Mixed [current_project]
set_property default_lib xil_defaultlib [current_project]
set_property ip_repo_paths [list $ip_repo_axil $ip_repo_axil32] [current_project]
update_ip_catalog -rebuild

set rtl_files [list \
    px_utility_pkg.vhd \
    tdc_gpx_pkg.vhd \
    tdc_gpx_cfg_pkg.vhd \
    tdc_gpx_atomic_snapshot_cdc.vhd \
    tdc_gpx_bus_phy.vhd \
    tdc_gpx_skid_buffer.vhd \
    tdc_gpx_sync_fifo.vhd \
    tdc_gpx_cell_builder.vhd \
    tdc_gpx_cell_pipe.vhd \
    tdc_gpx_chip_init.vhd \
    tdc_gpx_chip_run.vhd \
    tdc_gpx_chip_reg.vhd \
    tdc_gpx_chip_ctrl.vhd \
    tdc_gpx_csr_chip.vhd \
    tdc_gpx_cmd_arb.vhd \
    tdc_gpx_err_handler.vhd \
    tdc_gpx_stop_cfg_decode.vhd \
    tdc_gpx_reg_rsp_cdc.vhd \
    tdc_gpx_config_ctrl.vhd \
    tdc_gpx_decoder_i_mode.vhd \
    tdc_gpx_raw_event_builder.vhd \
    tdc_gpx_decode_pipe.vhd \
    tdc_gpx_face_assembler.vhd \
    tdc_gpx_line_packer.vhd \
    tdc_gpx_header_inserter.vhd \
    tdc_gpx_face_seq.vhd \
    tdc_gpx_output_stage.vhd \
    tdc_gpx_csr_pipeline.vhd \
    tdc_gpx_status_agg.vhd \
    tdc_gpx_top.vhd]

foreach file_name $rtl_files {
    set source [file join $hdl_dir $file_name]
    if {![file exists $source]} {
        error "missing RTL source: $source"
    }
    add_files -fileset sources_1 -norecurse $source
    set_property file_type {VHDL 2008} [get_files $source]
}

set parent_core [file join $parent_dir rtl tdc_gpx_parent_core.vhd]
add_files -fileset sources_1 -norecurse $parent_core
# Vivado IP Integrator module references reject a VHDL-2008 top file. This
# wrapper intentionally uses only VHDL-93 syntax; the production RTL beneath
# it remains VHDL-2008.
set_property file_type VHDL [get_files $parent_core]

set parent_xdc [file join $parent_dir constraints tdc_gpx_parent_ref.xdc]
if {![file exists $parent_xdc]} {
    error "missing parent constraint: $parent_xdc"
}
add_files -fileset constrs_1 -norecurse $parent_xdc

# Generate the two CSR IPs from their source repositories. The parent project
# does not rely on generated products from the standalone tdc_gpx_ctrl.xpr.
create_ip \
    -vendor victek.co.kr -library my_ip -name my_axil_csr -version 1.0 \
    -module_name tdc_gpx_axil_csr_pipeline \
    -dir [file join $project_dir ${project_name}.srcs sources_1 ip]
set_property -dict [list \
    CONFIG.num_data_bits 32 \
    CONFIG.num_ctl_regs 8 \
    CONFIG.num_stat_regs 8 \
    CONFIG.has_interrupt_regs true \
    CONFIG.num_intr_regs 4 \
    CONFIG.num_interrupt_src 1] \
    [get_ips tdc_gpx_axil_csr_pipeline]

create_ip \
    -vendor xilinx.com -library user -name my_axil_csr32_top -version 1.0 \
    -module_name tdc_gpx_axil_csr32_chip \
    -dir [file join $project_dir ${project_name}.srcs sources_1 ip]
set_property -dict [list \
    CONFIG.num_data_bits 32 \
    CONFIG.num_ctl_regs 32 \
    CONFIG.num_stat_regs 32 \
    CONFIG.has_interrupt_regs true \
    CONFIG.num_intr_regs 4 \
    CONFIG.num_interrupt_src 2] \
    [get_ips tdc_gpx_axil_csr32_chip]

generate_target {instantiation_template synthesis simulation} \
    [get_ips {tdc_gpx_axil_csr_pipeline tdc_gpx_axil_csr32_chip}]
update_compile_order -fileset sources_1

create_bd_design $bd_name
current_bd_design $bd_name

set ps7 [create_bd_cell -type ip \
    -vlnv xilinx.com:ip:processing_system7:* processing_system7_0]
set_property -dict [list \
    CONFIG.PCW_USE_M_AXI_GP0 1 \
    CONFIG.PCW_USE_S_AXI_HP0 1 \
    CONFIG.PCW_USE_S_AXI_HP1 1 \
    CONFIG.PCW_S_AXI_HP0_DATA_WIDTH 64 \
    CONFIG.PCW_S_AXI_HP1_DATA_WIDTH 64 \
    CONFIG.PCW_FPGA_FCLK0_ENABLE 1 \
    CONFIG.PCW_FPGA_FCLK1_ENABLE 1 \
    CONFIG.PCW_FPGA_FCLK2_ENABLE 1 \
    CONFIG.PCW_EN_CLK0_PORT 1 \
    CONFIG.PCW_EN_CLK1_PORT 1 \
    CONFIG.PCW_EN_CLK2_PORT 1 \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ 100 \
    CONFIG.PCW_FPGA1_PERIPHERAL_FREQMHZ 150 \
    CONFIG.PCW_FPGA2_PERIPHERAL_FREQMHZ 200 \
    CONFIG.PCW_USE_FABRIC_INTERRUPT 1 \
    CONFIG.PCW_IRQ_F2P_INTR 1] $ps7

make_bd_intf_pins_external [get_bd_intf_pins $ps7/DDR]
make_bd_intf_pins_external [get_bd_intf_pins $ps7/FIXED_IO]
set_property name DDR [get_bd_intf_ports DDR_0]
set_property name FIXED_IO [get_bd_intf_ports FIXED_IO_0]

set const_zero_1 [create_bd_cell -type ip \
    -vlnv xilinx.com:ip:xlconstant:* const_zero_1]
set_property -dict [list CONFIG.CONST_WIDTH 1 CONFIG.CONST_VAL 0] $const_zero_1
set const_one_1 [create_bd_cell -type ip \
    -vlnv xilinx.com:ip:xlconstant:* const_one_1]
set_property -dict [list CONFIG.CONST_WIDTH 1 CONFIG.CONST_VAL 1] $const_one_1
set const_zero_4 [create_bd_cell -type ip \
    -vlnv xilinx.com:ip:xlconstant:* const_zero_4]
set_property -dict [list CONFIG.CONST_WIDTH 4 CONFIG.CONST_VAL 0] $const_zero_4
set const_ones_4 [create_bd_cell -type ip \
    -vlnv xilinx.com:ip:xlconstant:* const_ones_4]
set_property -dict [list CONFIG.CONST_WIDTH 4 CONFIG.CONST_VAL 15] $const_ones_4
set const_zero_12 [create_bd_cell -type ip \
    -vlnv xilinx.com:ip:xlconstant:* const_zero_12]
set_property -dict [list CONFIG.CONST_WIDTH 12 CONFIG.CONST_VAL 0] $const_zero_12
set const_zero_16 [create_bd_cell -type ip \
    -vlnv xilinx.com:ip:xlconstant:* const_zero_16]
set_property -dict [list CONFIG.CONST_WIDTH 16 CONFIG.CONST_VAL 0] $const_zero_16
set const_zero_32 [create_bd_cell -type ip \
    -vlnv xilinx.com:ip:xlconstant:* const_zero_32]
set_property -dict [list CONFIG.CONST_WIDTH 32 CONFIG.CONST_VAL 0] $const_zero_32

# One synchronized reset output per FCLK domain. All three reset synchronizers
# use the PS fabric reset request and the same locked indication.
foreach {rst_name clk_pin} [list \
    rst_axi_100 FCLK_CLK0 \
    rst_axis_150 FCLK_CLK1 \
    rst_tdc_200 FCLK_CLK2] {
    set rst_cell [create_bd_cell -type ip \
        -vlnv xilinx.com:ip:proc_sys_reset:* $rst_name]
    connect_bd_net [get_bd_pins $ps7/$clk_pin] \
        [get_bd_pins $rst_cell/slowest_sync_clk]
    connect_bd_net [get_bd_pins $ps7/FCLK_RESET0_N] \
        [get_bd_pins $rst_cell/ext_reset_in]
    connect_bd_net [get_bd_pins $const_zero_1/dout] \
        [get_bd_pins $rst_cell/aux_reset_in] \
        [get_bd_pins $rst_cell/mb_debug_sys_rst]
    connect_bd_net [get_bd_pins $const_one_1/dout] \
        [get_bd_pins $rst_cell/dcm_locked]
}

set tdc [create_bd_cell -type module \
    -reference tdc_gpx_parent_core tdc_gpx_0]
set_property -dict [list \
    CONFIG.g_OUTPUT_WIDTH 32 \
    CONFIG.g_PRESENT_CHIP_MASK 1111 \
    CONFIG.g_RISE_CHIP_MASK 0011 \
    CONFIG.g_FALL_CHIP_MASK 1100 \
    CONFIG.g_MAX_STOPS_PER_CHIP 8 \
    CONFIG.g_MAX_HITS_PER_STOP 7 \
    CONFIG.g_AXIS_CLK_MHZ 150 \
    CONFIG.g_TDC_CLK_MHZ 200 \
    CONFIG.g_STREAM_CLK_MODE ASYNC] $tdc

# Module-reference bus grouping is inferred from AXI/AXIS signal names. The
# neutral clock names leave the multi-interface association writable here.
set_property -dict [list \
    CONFIG.ASSOCIATED_BUSIF {s_axi:s_axi_pipe} \
    CONFIG.FREQ_HZ 100000000] [get_bd_pins $tdc/i_ctrl_aclk]
set_property -dict [list \
    CONFIG.ASSOCIATED_BUSIF {m_axis:m_axis_fall} \
    CONFIG.FREQ_HZ 150000000] [get_bd_pins $tdc/i_axis_aclk]
set_property CONFIG.FREQ_HZ 200000000 \
    [get_bd_pins $tdc/i_tdc_clk]
foreach axis_if {m_axis m_axis_fall} {
    set_property CONFIG.FREQ_HZ 150000000 \
        [get_bd_intf_pins $tdc/$axis_if]
}
foreach axil_if {s_axi s_axi_pipe} {
    set_property CONFIG.FREQ_HZ 100000000 \
        [get_bd_intf_pins $tdc/$axil_if]
}

connect_bd_net [get_bd_pins $ps7/FCLK_CLK0] \
    [get_bd_pins $tdc/i_ctrl_aclk] \
    [get_bd_pins $ps7/M_AXI_GP0_ACLK]
connect_bd_net [get_bd_pins rst_axi_100/peripheral_aresetn] \
    [get_bd_pins $tdc/i_ctrl_aresetn]
connect_bd_net [get_bd_pins $ps7/FCLK_CLK1] \
    [get_bd_pins $tdc/i_axis_aclk] \
    [get_bd_pins $ps7/S_AXI_HP0_ACLK] \
    [get_bd_pins $ps7/S_AXI_HP1_ACLK]
connect_bd_net [get_bd_pins rst_axis_150/peripheral_aresetn] \
    [get_bd_pins $tdc/i_axis_aresetn]
connect_bd_net [get_bd_pins $ps7/FCLK_CLK2] \
    [get_bd_pins $tdc/i_tdc_clk]

# Upstream laser/echo modules are not part of this first parent timing shell.
# Keep their interface pins explicit at the module boundary and drive idle
# values in the BD. Hierarchical module synthesis preserves the TDC core.
connect_bd_net [get_bd_pins $const_zero_1/dout] \
    [get_bd_pins $tdc/i_lsr_valid] \
    [get_bd_pins $tdc/i_shot_start] \
    [get_bd_pins $tdc/i_stop_tdc] \
    [get_bd_pins $tdc/i_stop_evt_valid] \
    [get_bd_pins $tdc/i_fire_count_valid] \
    [get_bd_pins $tdc/i_fire_count_last]
connect_bd_net [get_bd_pins $const_zero_32/dout] \
    [get_bd_pins $tdc/i_lsr_data] \
    [get_bd_pins $tdc/i_stop_evt_data] \
    [get_bd_pins $tdc/i_stop_evt_user] \
    [get_bd_pins $tdc/i_fire_count_data] \
    [get_bd_pins $tdc/i_k_dist_fixed]
connect_bd_net [get_bd_pins $const_ones_4/dout] \
    [get_bd_pins $tdc/i_stop_evt_keep] \
    [get_bd_pins $tdc/i_fire_count_keep]
connect_bd_net [get_bd_pins $const_zero_16/dout] \
    [get_bd_pins $tdc/i_bin_resolution_ps]

# Two independent S2MM VDMA channels preserve the rise/fall lane ownership.
foreach lane {rise fall} {
    set vdma [create_bd_cell -type ip \
        -vlnv xilinx.com:ip:axi_vdma:* vdma_$lane]
    set_property -dict [list \
        CONFIG.c_include_mm2s 0 \
        CONFIG.c_include_s2mm 1 \
        CONFIG.c_m_axi_s2mm_data_width 64 \
        CONFIG.c_num_fstores 3 \
        CONFIG.c_s2mm_linebuffer_depth 512] $vdma

    connect_bd_net [get_bd_pins $ps7/FCLK_CLK0] \
        [get_bd_pins $vdma/s_axi_lite_aclk]
    connect_bd_net [get_bd_pins $ps7/FCLK_CLK1] \
        [get_bd_pins $vdma/m_axi_s2mm_aclk] \
        [get_bd_pins $vdma/s_axis_s2mm_aclk]
    connect_bd_net [get_bd_pins rst_axi_100/peripheral_aresetn] \
        [get_bd_pins $vdma/axi_resetn]
}

connect_bd_intf_net [get_bd_intf_pins $tdc/M_AXIS] \
    [get_bd_intf_pins vdma_rise/S_AXIS_S2MM]
connect_bd_intf_net [get_bd_intf_pins $tdc/M_AXIS_FALL] \
    [get_bd_intf_pins vdma_fall/S_AXIS_S2MM]

# AXI-Lite control fanout: two TDC CSR banks, two VDMAs, geometry GPIO.
set ctrl_ic [create_bd_cell -type ip \
    -vlnv xilinx.com:ip:axi_interconnect:* axi_ctrl]
set_property -dict [list CONFIG.NUM_SI 1 CONFIG.NUM_MI 5] $ctrl_ic
connect_bd_intf_net [get_bd_intf_pins $ps7/M_AXI_GP0] \
    [get_bd_intf_pins $ctrl_ic/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins $ctrl_ic/M00_AXI] \
    [get_bd_intf_pins $tdc/S_AXI]
connect_bd_intf_net [get_bd_intf_pins $ctrl_ic/M01_AXI] \
    [get_bd_intf_pins $tdc/S_AXI_PIPE]
connect_bd_intf_net [get_bd_intf_pins $ctrl_ic/M02_AXI] \
    [get_bd_intf_pins vdma_rise/S_AXI_LITE]
connect_bd_intf_net [get_bd_intf_pins $ctrl_ic/M03_AXI] \
    [get_bd_intf_pins vdma_fall/S_AXI_LITE]

connect_bd_net [get_bd_pins $ps7/FCLK_CLK0] \
    [get_bd_pins $ctrl_ic/ACLK] \
    [get_bd_pins $ctrl_ic/S00_ACLK] \
    [get_bd_pins $ctrl_ic/M00_ACLK] \
    [get_bd_pins $ctrl_ic/M01_ACLK] \
    [get_bd_pins $ctrl_ic/M02_ACLK] \
    [get_bd_pins $ctrl_ic/M03_ACLK] \
    [get_bd_pins $ctrl_ic/M04_ACLK]
connect_bd_net [get_bd_pins rst_axi_100/interconnect_aresetn] \
    [get_bd_pins $ctrl_ic/ARESETN]
connect_bd_net [get_bd_pins rst_axi_100/peripheral_aresetn] \
    [get_bd_pins $ctrl_ic/S00_ARESETN] \
    [get_bd_pins $ctrl_ic/M00_ARESETN] \
    [get_bd_pins $ctrl_ic/M01_ARESETN] \
    [get_bd_pins $ctrl_ic/M02_ARESETN] \
    [get_bd_pins $ctrl_ic/M03_ARESETN] \
    [get_bd_pins $ctrl_ic/M04_ARESETN]

# Active VDMA geometry is software-readable without introducing another custom
# register bank: channel 1 packs rise/fall HSIZE, channel 2 exposes VSIZE.
set geometry [create_bd_cell -type ip \
    -vlnv xilinx.com:ip:axi_gpio:* geometry_gpio]
set_property -dict [list \
    CONFIG.C_ALL_INPUTS 1 \
    CONFIG.C_GPIO_WIDTH 32 \
    CONFIG.C_IS_DUAL 1 \
    CONFIG.C_ALL_INPUTS_2 1 \
    CONFIG.C_GPIO2_WIDTH 16 \
    CONFIG.C_INTERRUPT_PRESENT 0] $geometry
set geometry_concat [create_bd_cell -type ip \
    -vlnv xilinx.com:ip:xlconcat:* geometry_concat]
set_property -dict [list CONFIG.NUM_PORTS 2 \
    CONFIG.IN0_WIDTH 16 CONFIG.IN1_WIDTH 16] $geometry_concat
connect_bd_net [get_bd_pins $tdc/o_vdma_hsize_bytes_rise] \
    [get_bd_pins $geometry_concat/In0]
connect_bd_net [get_bd_pins $tdc/o_vdma_hsize_bytes_fall] \
    [get_bd_pins $geometry_concat/In1]
connect_bd_net [get_bd_pins $geometry_concat/dout] \
    [get_bd_pins $geometry/gpio_io_i]
connect_bd_net [get_bd_pins $tdc/o_vdma_vsize_lines] \
    [get_bd_pins $geometry/gpio2_io_i]
connect_bd_intf_net [get_bd_intf_pins $ctrl_ic/M04_AXI] \
    [get_bd_intf_pins $geometry/S_AXI]
connect_bd_net [get_bd_pins $ps7/FCLK_CLK0] \
    [get_bd_pins $geometry/s_axi_aclk]
connect_bd_net [get_bd_pins rst_axi_100/peripheral_aresetn] \
    [get_bd_pins $geometry/s_axi_aresetn]

# One VDMA per HP port avoids arbitration coupling between rise and fall.
foreach {lane hp_port} {rise HP0 fall HP1} {
    set data_sc [create_bd_cell -type ip \
        -vlnv xilinx.com:ip:smartconnect:* data_$lane]
    set_property -dict [list CONFIG.NUM_SI 1 CONFIG.NUM_MI 1] $data_sc
    connect_bd_intf_net [get_bd_intf_pins vdma_$lane/M_AXI_S2MM] \
        [get_bd_intf_pins $data_sc/S00_AXI]
    connect_bd_intf_net [get_bd_intf_pins $data_sc/M00_AXI] \
        [get_bd_intf_pins $ps7/S_AXI_$hp_port]
    connect_bd_net [get_bd_pins $ps7/FCLK_CLK1] \
        [get_bd_pins $data_sc/aclk]
    connect_bd_net [get_bd_pins rst_axis_150/peripheral_aresetn] \
        [get_bd_pins $data_sc/aresetn]
}

# Four active interrupt sources occupy IRQ_F2P[3:0]; upper bits are zero.
set irq_concat [create_bd_cell -type ip \
    -vlnv xilinx.com:ip:xlconcat:* irq_concat]
set_property -dict [list CONFIG.NUM_PORTS 5 \
    CONFIG.IN0_WIDTH 1 CONFIG.IN1_WIDTH 1 CONFIG.IN2_WIDTH 1 \
    CONFIG.IN3_WIDTH 1 CONFIG.IN4_WIDTH 12] $irq_concat
connect_bd_net [get_bd_pins $tdc/o_irq] [get_bd_pins $irq_concat/In0]
connect_bd_net [get_bd_pins $tdc/o_irq_pipe] [get_bd_pins $irq_concat/In1]
connect_bd_net [get_bd_pins vdma_rise/s2mm_introut] \
    [get_bd_pins $irq_concat/In2]
connect_bd_net [get_bd_pins vdma_fall/s2mm_introut] \
    [get_bd_pins $irq_concat/In3]
connect_bd_net [get_bd_pins $const_zero_12/dout] \
    [get_bd_pins $irq_concat/In4]
connect_bd_net [get_bd_pins $irq_concat/dout] \
    [get_bd_pins $ps7/IRQ_F2P]

# Board-owned TDC pins are the only PL interfaces made external.
foreach pin_name [list \
    io_tdc0_d io_tdc1_d io_tdc2_d io_tdc3_d \
    o_tdc0_adr o_tdc1_adr o_tdc2_adr o_tdc3_adr \
    o_tdc_csn o_tdc_rdn o_tdc_wrn o_tdc_oen o_tdc_stopdis \
    o_tdc_alutrigger o_tdc_puresn \
    i_tdc_ef1 i_tdc_ef2 i_tdc_lf1 i_tdc_lf2 \
    i_tdc_irflag i_tdc_errflag] {
    make_bd_pins_external [get_bd_pins $tdc/$pin_name]
}

assign_bd_address
save_bd_design
validate_bd_design
save_bd_design

set summary [open [file join $result_dir parent_ref_contract.txt] w]
puts $summary "PART=$part_name"
puts $summary "FCLK0_AXI_MHZ=100"
puts $summary "FCLK1_AXIS_MHZ=150"
puts $summary "FCLK2_TDC_MHZ=200"
puts $summary "OUTPUT_WIDTH=32"
puts $summary "PRESENT_CHIP_MASK=1111"
puts $summary "RISE_CHIP_MASK=0011"
puts $summary "FALL_CHIP_MASK=1100"
puts $summary "VDMA_CHANNELS=2"
puts $summary "BOARD_PIN_XDC=OPEN"
close $summary

if {$run_mode eq "VALIDATE"} {
    puts "PARENT_REF_VALIDATE_PASS"
    close_project
    exit 0
}

set bd_file [get_files [file join $project_dir \
    ${project_name}.srcs sources_1 bd $bd_name ${bd_name}.bd]]
set_property synth_checkpoint_mode Hierarchical $bd_file
generate_target all $bd_file

set wrappers [make_wrapper -files $bd_file -top]
add_files -norecurse $wrappers
set_property top ${bd_name}_wrapper [get_filesets sources_1]
update_compile_order -fileset sources_1

if {$run_mode in {SYNTH IMPL}} {
    reset_run synth_1
    launch_runs synth_1 -jobs 8
    wait_on_run synth_1
    set synth_status [get_property STATUS [get_runs synth_1]]
    if {![string match "*Complete*" $synth_status]} {
        error "parent reference synthesis failed: status=$synth_status"
    }
    open_run synth_1
    verify_parent_manual_cdc post_synth \
        [file join $result_dir post_synth_manual_cdc.rpt] \
        $parent_manual_cdc_groups
    report_parent_control_sets post_synth $result_dir
    report_utilization -hierarchical -hierarchical_depth 6 \
        -file [file join $result_dir post_synth_utilization_hier.rpt]
    report_timing_summary -delay_type min_max -report_unconstrained \
        -max_paths 50 -file [file join $result_dir post_synth_timing_summary.rpt]
    report_bus_skew -file [file join $result_dir post_synth_bus_skew.rpt]
    report_cdc -details -file [file join $result_dir post_synth_cdc.rpt]
    report_clock_interaction \
        -file [file join $result_dir post_synth_clock_interaction.rpt]
    report_methodology -file [file join $result_dir post_synth_methodology.rpt]
    report_drc -file [file join $result_dir post_synth_drc.rpt]
    close_design
}

if {$run_mode eq "IMPL"} {
    reset_run impl_1
    set_property strategy Performance_Explore [get_runs impl_1]
    launch_runs impl_1 -to_step route_design -jobs 8
    wait_on_run impl_1
    set impl_status [get_property STATUS [get_runs impl_1]]
    if {![string match "*Complete*" $impl_status]} {
        error "parent reference implementation failed: status=$impl_status"
    }
    open_run impl_1
    verify_parent_manual_cdc post_route \
        [file join $result_dir post_route_manual_cdc.rpt] \
        $parent_manual_cdc_groups
    report_parent_control_sets post_route $result_dir
    report_utilization -hierarchical -hierarchical_depth 6 \
        -file [file join $result_dir post_route_utilization_hier.rpt]
    report_timing_summary -delay_type min_max -report_unconstrained \
        -max_paths 50 -file [file join $result_dir post_route_timing_summary.rpt]
    report_bus_skew -file [file join $result_dir post_route_bus_skew.rpt]
    report_cdc -details -file [file join $result_dir post_route_cdc.rpt]
    report_clock_interaction \
        -file [file join $result_dir post_route_clock_interaction.rpt]
    report_methodology -file [file join $result_dir post_route_methodology.rpt]
    report_drc -file [file join $result_dir post_route_drc.rpt]
    report_route_status -file [file join $result_dir post_route_status.rpt]
    write_checkpoint -force [file join $result_dir post_route.dcp]
    close_design
}

puts "PARENT_REF_${run_mode}_PASS project=$project_dir results=$result_dir"
close_project
