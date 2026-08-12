# Reopen and validate a generated Stage L0 parent project.
# This checks persisted BD structure rather than trusting the create session.

if {[llength $argv] != 2} {
    error {Usage: validate_v3_l0_parent.tcl OUTPUT_ROOT OUTPUT_WIDTH}
}
lassign $argv output_root output_width
set output_root [file normalize $output_root]
if {$output_width ni {32 64}} {
    error "Unsupported L0 AXIS width: $output_width"
}

set project_path [file join $output_root project_4_lidar_v3_l0.xpr]
set bd_path [file join $output_root project_4_lidar_v3_l0.srcs sources_1 bd \
    design_1_lidar_ctrl_v3 design_1_lidar_ctrl_v3.bd]
set wrapper_path [file join $output_root project_4_lidar_v3_l0.gen sources_1 bd \
    design_1_lidar_ctrl_v3 hdl design_1_lidar_ctrl_v3_wrapper.vhd]
set base_pin_xdc [file join $output_root constraints \
    project4_vt_hrl2_tdc01_service.xdc]
set ext_pin_xdc [file join $output_root constraints \
    project4_vt_hrl2_tdc23_extension.xdc]
foreach required [list $project_path $bd_path $wrapper_path $base_pin_xdc \
        $ext_pin_xdc] {
    if {![file exists $required]} {
        error "Missing generated L0 artifact: $required"
    }
}

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

proc l0_expect_equal {label actual expected} {
    if {$actual ne $expected} {
        error "$label mismatch: expected=$expected actual=$actual"
    }
}

proc l0_expect_count {label objects expected} {
    set actual [llength $objects]
    if {$actual != $expected} {
        error "$label count mismatch: expected=$expected actual=$actual"
    }
}

proc l0_read_text {path} {
    set channel [open $path r]
    set text [read $channel]
    close $channel
    return $text
}

proc l0_expect_xci_value {label xci_text parameter expected} {
    set name_text "\"${parameter}\":"
    set name_index [string first $name_text $xci_text]
    if {$name_index < 0} {
        error "$label XCI parameter is missing: $parameter"
    }
    set parameter_block [string range $xci_text $name_index \
        [expr {$name_index + 320}]]
    set value_text "\"value\": \"${expected}\""
    if {[string first $value_text $parameter_block] < 0} {
        error "$label XCI $parameter mismatch: expected=$expected"
    }
}

proc l0_expect_same_intf_net {label endpoint_a endpoint_b} {
    set net_a [get_bd_intf_nets -quiet -of_objects [get_bd_intf_pins $endpoint_a]]
    set net_b [get_bd_intf_nets -quiet -of_objects [get_bd_intf_pins $endpoint_b]]
    l0_expect_count "$label endpoint A net" $net_a 1
    l0_expect_count "$label endpoint B net" $net_b 1
    l0_expect_equal "$label interface net" $net_a $net_b
}

proc l0_expect_same_net {label endpoint_a endpoint_b} {
    set net_a [get_bd_nets -quiet -of_objects [get_bd_pins $endpoint_a]]
    set net_b [get_bd_nets -quiet -of_objects [get_bd_pins $endpoint_b]]
    l0_expect_count "$label endpoint A net" $net_a 1
    l0_expect_count "$label endpoint B net" $net_b 1
    l0_expect_equal "$label scalar net" $net_a $net_b
}

open_project $project_path
open_bd_design $bd_path

l0_expect_equal project_part [get_property PART [current_project]] xc7z020clg484-2
l0_expect_equal project_top [get_property TOP [current_fileset]] \
    design_1_lidar_ctrl_v3_wrapper
l0_expect_count only_v3_bd [get_files -quiet -filter {FILE_TYPE == "Block Designs"}] 1

set ps7 [get_bd_cells processing_system7_0]
set lidar [get_bd_cells tdc_gpx_lidar_ctrl_v3_0]
set rise_vdma [get_bd_cells axi_vdma_rise]
set fall_vdma [get_bd_cells axi_vdma_fall]
set proc_clk_wiz [get_bd_cells proc_clk_wiz]
set rise_converter [get_bd_cells rise_hp_axi4_to_axi3]
set fall_converter [get_bd_cells fall_hp_axi4_to_axi3]
l0_expect_count profile_bridge_sources \
    [get_files -quiet -filter {NAME =~ "*l0_vdma_profile_bridge.vhd"}] 0
foreach obsolete_cell {
        rise_profile_bridge fall_profile_bridge vdma_profile_ctrl
        vdma_rise_geometry vdma_fall_geometry prof_status_cat
        vdma_rise_geometry_concat vdma_fall_geometry_concat
        rise_cfg_ack_slice fall_cfg_ack_slice} {
    l0_expect_count "removed Parent cell $obsolete_cell" \
        [get_bd_cells -quiet $obsolete_cell] 0
}

l0_expect_equal v3_vlnv [get_property VLNV $lidar] \
    victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3:3.0
foreach {property expected} [list \
        CONFIG.G_CSR_CLK_MHZ 100 \
        CONFIG.G_PROC_CLK_MHZ 150 \
        CONFIG.G_TDC_CLK_MHZ 200 \
        CONFIG.G_STREAM_CLK_MODE ASYNC \
        CONFIG.G_NUM_CHIPS 4 \
        CONFIG.G_STOPS_PER_CHIP 8 \
        CONFIG.G_MAX_RETURNS_PER_STOP 7 \
        CONFIG.G_RISE_CAPABILITY_MASK 0011 \
        CONFIG.G_FALL_CAPABILITY_MASK 1100 \
        CONFIG.G_OUTPUT_WIDTH $output_width \
        CONFIG.G_ENABLE_ECHO_RECEIVER false \
        CONFIG.G_ENABLE_ECHO_SIMULATION false \
        CONFIG.G_OEN_MODE PULLUP_OR_NOT_CONNECTED] {
    l0_expect_equal "v3 $property" [get_property $property $lidar] $expected
}

foreach {property expected} [list \
        CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ 100 \
        CONFIG.PCW_FPGA1_PERIPHERAL_FREQMHZ 100 \
        CONFIG.PCW_FPGA2_PERIPHERAL_FREQMHZ 200 \
        CONFIG.PCW_USE_S_AXI_HP0 1 \
        CONFIG.PCW_USE_S_AXI_HP1 1 \
        CONFIG.PCW_S_AXI_HP0_DATA_WIDTH 64 \
        CONFIG.PCW_S_AXI_HP1_DATA_WIDTH 64] {
    l0_expect_equal "PS7 $property" [get_property $property $ps7] $expected
}
l0_expect_equal proc_clock_input_mhz \
    [get_property CONFIG.PRIM_IN_FREQ $proc_clk_wiz] 100.000
l0_expect_equal proc_clock_output_mhz \
    [get_property CONFIG.CLKOUT1_REQUESTED_OUT_FREQ $proc_clk_wiz] 150.000

foreach cell [list $rise_vdma $fall_vdma] {
    l0_expect_equal "$cell stream width" \
        [get_property CONFIG.c_s_axis_s2mm_tdata_width $cell] $output_width
    l0_expect_equal "$cell memory width" \
        [get_property CONFIG.c_m_axi_s2mm_data_width $cell] 64
    l0_expect_equal "$cell S2MM enabled" \
        [get_property CONFIG.c_include_s2mm $cell] 1
    l0_expect_equal "$cell MM2S disabled" \
        [get_property CONFIG.c_include_mm2s $cell] 0
    l0_expect_equal "$cell Frame Store count" \
        [get_property CONFIG.c_num_fstores $cell] 3
}

# CONFIG properties alone do not prove that the generated VDMA includes the
# frame/delay counters consumed by the PS driver.  Inspect each persisted XCI
# so Parent regeneration cannot silently drop the one-Face completion IRQ
# contract while the Block Design still looks valid.
set parent_ip_root [file join $output_root project_4_lidar_v3_l0.srcs \
    sources_1 bd design_1_lidar_ctrl_v3 ip]
foreach cell_name {axi_vdma_rise axi_vdma_fall} {
    set xci_files [glob -nocomplain -type f \
        [file join $parent_ip_root *${cell_name}_0 *.xci]]
    l0_expect_count "$cell_name generated XCI" $xci_files 1
    set xci_text [l0_read_text [lindex $xci_files 0]]
    l0_expect_xci_value $cell_name $xci_text c_include_mm2s 0
    l0_expect_xci_value $cell_name $xci_text c_include_s2mm 1
    l0_expect_xci_value $cell_name $xci_text c_num_fstores 3
    l0_expect_xci_value $cell_name $xci_text \
        c_s_axis_s2mm_tdata_width $output_width
    l0_expect_xci_value $cell_name $xci_text C_ENABLE_DEBUG_INFO_14 1
    l0_expect_xci_value $cell_name $xci_text C_ENABLE_DEBUG_INFO_15 1
}
puts "LIDAR_V3_L0_VDMA_XCI_CONTRACT_PASS width=$output_width"
foreach cell [list $rise_converter $fall_converter] {
    l0_expect_equal "$cell slave protocol" \
        [get_property CONFIG.SI_PROTOCOL $cell] AXI4
    l0_expect_equal "$cell master protocol" \
        [get_property CONFIG.MI_PROTOCOL $cell] AXI3
    l0_expect_equal "$cell data width" [get_property CONFIG.DATA_WIDTH $cell] 64
}
l0_expect_equal control_interconnect_masters \
    [get_property CONFIG.NUM_MI [get_bd_cells axi_control_interconnect]] 3

l0_expect_same_intf_net rise_stream \
    tdc_gpx_lidar_ctrl_v3_0/m_axis_rise axi_vdma_rise/S_AXIS_S2MM
l0_expect_same_intf_net fall_stream \
    tdc_gpx_lidar_ctrl_v3_0/m_axis_fall axi_vdma_fall/S_AXIS_S2MM
l0_expect_same_intf_net rise_vdma_to_converter \
    axi_vdma_rise/M_AXI_S2MM rise_hp_axi4_to_axi3/S_AXI
l0_expect_same_intf_net rise_converter_to_hp0 \
    rise_hp_axi4_to_axi3/M_AXI processing_system7_0/S_AXI_HP0
l0_expect_same_intf_net fall_vdma_to_converter \
    axi_vdma_fall/M_AXI_S2MM fall_hp_axi4_to_axi3/S_AXI
l0_expect_same_intf_net fall_converter_to_hp1 \
    fall_hp_axi4_to_axi3/M_AXI processing_system7_0/S_AXI_HP1

l0_expect_same_net processing_clock \
    proc_clk_wiz/clk_out1 tdc_gpx_lidar_ctrl_v3_0/proc_aclk
l0_expect_same_net tdc_clock \
    processing_system7_0/FCLK_CLK2 tdc_gpx_lidar_ctrl_v3_0/i_tdc_clk
l0_expect_same_net laser_safety_tieoff \
    const_zero_1/dout tdc_gpx_lidar_ctrl_v3_0/i_external_laser_permit

foreach removed_pin {
        o_vdma_rise_cfg_valid i_vdma_rise_cfg_ready
        o_vdma_rise_cfg_enable o_vdma_rise_hsize_bytes
        o_vdma_rise_vsize_lines o_vdma_rise_stride_bytes
        o_vdma_fall_cfg_valid i_vdma_fall_cfg_ready
        o_vdma_fall_cfg_enable o_vdma_fall_hsize_bytes
        o_vdma_fall_vsize_lines o_vdma_fall_stride_bytes} {
    l0_expect_count "internalized profile pin $removed_pin" \
        [get_bd_pins -quiet tdc_gpx_lidar_ctrl_v3_0/$removed_pin] 0
}

foreach port {i_pd_lvds_p i_pd_lvds_n o_tdc_stop} {
    l0_expect_count "disabled Echo Receiver external port $port" \
        [get_bd_ports -quiet $port] 0
}
foreach port {i_enc_a i_enc_b i_enc_z i_fire_done i_tdc_ef1 i_tdc_ef2 \
        i_tdc_irflag io_tdc_d o_tdc_adr o_tdc_csn o_tdc_rdn o_tdc_wrn \
        o_tdc_stopdis o_tdc_alutrigger o_tdc_puresn o_fire_pulse \
        o_start_tdc o_stop_tdc} {
    l0_expect_count "required board port $port" [get_bd_ports -quiet $port] 1
}

set assigned_addresses {}
foreach segment [get_bd_addr_segs -quiet] {
    set offset [get_property OFFSET $segment]
    if {$offset ne {}} {
        lappend assigned_addresses [string toupper $offset]
    }
}
foreach address {0x40000000 0x43000000 0x43010000} {
    if {[lsearch -exact $assigned_addresses [string toupper $address]] < 0} {
        error "Expected AXI-Lite address is absent: $address"
    }
}

validate_bd_design
puts "LIDAR_V3_L0_PARENT_VALIDATE_PASS output=$output_root width=$output_width"
close_project
exit 0
