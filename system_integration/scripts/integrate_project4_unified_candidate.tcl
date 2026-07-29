# Build the unified LiDAR candidate beside project_4/design_1.
#
# The source design is never saved by this script. A fresh design_1_unified is
# created on every run so the migration remains reviewable and repeatable.

set install_tcl_store [file normalize \
    [file join $::env(XILINX_VIVADO) data XilinxTclStore]]
lappend auto_path [file join $install_tcl_store support appinit]
foreach vendor_dir [glob -nocomplain -type d \
        [file join $install_tcl_store tclapp *]] {
    foreach app_dir [glob -nocomplain -type d [file join $vendor_dir *]] {
        lappend auto_path $app_dir
    }
}
catch {package require ::tclapp::support::appinit 1.2}

proc require_one {objects label} {
    if {[llength $objects] != 1} {
        error "Expected one $label, found [llength $objects]: $objects"
    }
    return [lindex $objects 0]
}

proc reconnect_scalar_net {source sink} {
    set source_net [get_bd_nets -quiet -of_objects $source]
    set sink_net [get_bd_nets -quiet -of_objects $sink]
    if {[llength $source_net] > 1 || [llength $sink_net] > 1} {
        error "Ambiguous scalar connection: $source -> $sink"
    }
    if {[llength $source_net] == 1 && [llength $sink_net] == 1 &&
        $source_net eq $sink_net} {
        return
    }
    if {[llength $sink_net] == 1} {
        disconnect_bd_net [lindex $sink_net 0] $sink
    }
    connect_bd_net $source $sink
}

proc reconnect_intf_net {source sink} {
    set source_net [get_bd_intf_nets -quiet -of_objects $source]
    set sink_net [get_bd_intf_nets -quiet -of_objects $sink]
    if {[llength $source_net] > 1 || [llength $sink_net] > 1} {
        error "Ambiguous interface connection: $source -> $sink"
    }
    if {[llength $source_net] == 1 && [llength $sink_net] == 1 &&
        $source_net eq $sink_net} {
        return
    }
    if {[llength $sink_net] == 1} {
        disconnect_bd_intf_net [lindex $sink_net 0] $sink
    }
    connect_bd_intf_net $source $sink
}

proc expose_pin {external_name internal_pin} {
    set existing [get_bd_ports -quiet $external_name]
    if {[llength $existing] == 1} {
        set net [get_bd_nets -quiet -of_objects $existing]
        if {[llength $net] == 0} {
            connect_bd_net $existing $internal_pin
            return $existing
        }
        if {[llength $net] != 1} {
            error "External port $external_name has ambiguous nets: $net"
        }
        if {[lsearch -exact [get_bd_pins -quiet -of_objects $net] \
                $internal_pin] < 0} {
            error "External port $external_name is not connected to $internal_pin"
        }
        return $existing
    }
    if {[llength $existing] != 0} {
        error "Expected at most one external port $external_name"
    }
    set direction [get_property DIR $internal_pin]
    set left [get_property LEFT $internal_pin]
    set right [get_property RIGHT $internal_pin]
    if {$left eq {} || $right eq {}} {
        set created [create_bd_port -dir $direction $external_name]
    } else {
        set created [create_bd_port -dir $direction \
            -from $left -to $right $external_name]
    }

    set pin_net [get_bd_nets -quiet -of_objects $internal_pin]
    if {[llength $pin_net] == 0} {
        connect_bd_net $created $internal_pin
    } elseif {[llength $pin_net] == 1} {
        connect_bd_net -net [lindex $pin_net 0] $created
    } else {
        error "Internal pin $internal_pin has ambiguous nets: $pin_net"
    }
    return $created
}

proc expose_interface {external_name internal_interface} {
    set existing [get_bd_intf_ports -quiet $external_name]
    if {[llength $existing] == 1} {
        return $existing
    }
    if {[llength $existing] != 0} {
        error "Expected at most one external interface $external_name"
    }
    set before [get_bd_intf_ports -quiet]
    make_bd_intf_pins_external $internal_interface
    set created {}
    foreach port [get_bd_intf_ports -quiet] {
        if {[lsearch -exact $before $port] < 0} {
            lappend created $port
        }
    }
    set created [require_one $created \
        "new external interface for $internal_interface"]
    set_property name $external_name $created
    return $created
}

proc append_repo_once {repo_paths_var repo} {
    upvar 1 $repo_paths_var repo_paths
    set normalized [file normalize $repo]
    foreach current $repo_paths {
        if {[file normalize $current] eq $normalized} {
            return
        }
    }
    lappend repo_paths $normalized
}

set script_dir [file normalize [file dirname [info script]]]
set hdl_dir [file normalize [file join $script_dir ../..]]
set ip_root [file normalize [file join $hdl_dir ../..]]
if {[llength $argv] > 0} {
    set project_dir [file normalize [lindex $argv 0]]
} else {
    set project_dir {C:/Projects/my_sp/ALINX/Logic/project_4}
}

set project_xpr [file join $project_dir project_4.xpr]
set source_bd [file join $project_dir \
    project_4.srcs sources_1 bd design_1 design_1.bd]
set candidate_name design_1_unified
set candidate_bd [file join $project_dir \
    project_4.srcs sources_1 bd $candidate_name $candidate_name.bd]

set motor_laser_repo [file join $ip_root motor_laser_ctrl ip_repo]
set echo_repo [file join $ip_root echo_receiver ip_repo]
set tdc_repo [file join $ip_root tdc_gpx_ctrl ip_repo]
set unified_repo [file join $ip_root lidar_unified_csr ip_repo]

foreach required [list $project_xpr $source_bd \
        [file join $motor_laser_repo component.xml] \
        [file join $echo_repo component.xml] \
        [file join $tdc_repo component.xml] \
        [file join $unified_repo component.xml]] {
    if {![file exists $required]} {
        error "Required integration artifact is missing: $required"
    }
}

open_project $project_xpr
set repo_paths [get_property IP_REPO_PATHS [current_project]]
foreach repo [list $motor_laser_repo $echo_repo $tdc_repo $unified_repo] {
    append_repo_once repo_paths $repo
}
set_property IP_REPO_PATHS $repo_paths [current_project]
update_ip_catalog -rebuild

foreach vlnv {
    victek.co.kr:my_ip:motor_laser_ctrl_top:1.0
    victek.co.kr:my_ip:echo_receiver_top:1.0
    victek.co.kr:my_ip:tdc_gpx_top:1.0
    victek.co.kr:my_ip:lidar_unified_csr:1.0
} {
    require_one [get_ipdefs -all -quiet $vlnv] "IP definition $vlnv"
}

open_bd_design $source_bd
save_bd_design_as -force $candidate_name
if {[llength [get_files -quiet $candidate_bd]] == 0} {
    add_files -norecurse $candidate_bd
}
close_bd_design [current_bd_design]
open_bd_design $candidate_bd
foreach ip [get_ips -quiet -of_objects [get_files $candidate_bd]] {
    if {[get_property IS_LOCKED $ip]} {
        puts "UPGRADING_CANDIDATE_IP=$ip"
        upgrade_ip $ip
    }
}

# Remove duplicated functionality. motor_laser_ctrl already owns its virtual
# encoder, Motor Decoder, and Laser Controller. system_ila_0 only observed the
# three retired standalone instances.
foreach cell_name {
    virtual_encoder_top_0 motor_decoder_top_0 laser_ctrl_top_0 system_ila_0
} {
    set cell [get_bd_cells -quiet $cell_name]
    if {[llength $cell] == 1} {
        delete_bd_objs $cell
    }
}

set ps [require_one [get_bd_cells processing_system7_0] {processing system}]
set interconnect [require_one [get_bd_cells axi_interconnect_0] \
    {AXI interconnect}]
set motor_laser [require_one [get_bd_cells motor_laser_ctrl_top_0] \
    {combined Motor/Laser controller}]
set echo [require_one [get_bd_cells echo_receiver_top_0] {Echo Receiver}]

set_property CONFIG.PCW_FPGA1_PERIPHERAL_FREQMHZ {100} $ps
set_property CONFIG.NUM_MI {3} $interconnect
set_property -dict [list \
    CONFIG.g_ENABLE_LOCAL_CSR {false} \
    CONFIG.g_PROC_CLK_MHZ {150}] $motor_laser
set_property -dict [list \
    CONFIG.g_ENABLE_LOCAL_CSR {false} \
    CONFIG.g_AXIS_CLK_MHZ {150} \
    CONFIG.g_ENABLE_SIM_PATH {false} \
    CONFIG.g_N_CHIPS {4} \
    CONFIG.g_STOPS_PER_CHIP {8}] $echo

# Remove nets whose source/interface disappeared with the retired standalone
# IPs, disabled local IRQs, and AXI master-port reduction. The two retained
# ILAs are rewired below only to live unified paths.
foreach net_name {
    echo_receiver_top_0_o_irq
    laser_ctrl_top_0_o_fire_pulse
    laser_ctrl_top_0_o_irq
    laser_ctrl_top_0_o_laser_active
    laser_ctrl_top_0_o_shot_face_index
    laser_ctrl_top_0_o_shot_start
    laser_ctrl_top_0_o_start_tdc
    laser_ctrl_top_0_o_stop_tdc
    laser_ctrl_top_0_o_warning
    laser_ctrl_top_0_o_warning_any
    motor_decoder_top_0_irq
    motor_decoder_top_0_o_n_faces
    motor_laser_ctrl_top_0_o_irq
} {
    set stale [get_bd_nets -quiet $net_name]
    if {[llength $stale] == 1} {
        delete_bd_objs $stale
    }
}
foreach net_name {
    axi_interconnect_0_M03_AXI
    axi_interconnect_0_M04_AXI
    axi_interconnect_0_M05_AXI
    axi_interconnect_0_M06_AXI
    axi_interconnect_0_M07_AXI
    laser_ctrl_top_0_lc_debug
    laser_ctrl_top_0_m_axis
    motor_decoder_top_0_m_axis
    motor_decoder_top_0_md_dbg
    virtual_encoder_top_0_debug
} {
    set stale [get_bd_intf_nets -quiet $net_name]
    if {[llength $stale] == 1} {
        delete_bd_objs $stale
    }
}

set central [create_bd_cell -type ip \
    -vlnv victek.co.kr:my_ip:lidar_unified_csr:1.0 \
    lidar_unified_csr_0]
set tdc [create_bd_cell -type ip \
    -vlnv victek.co.kr:my_ip:tdc_gpx_top:1.0 tdc_gpx_top_0]
set_property -dict [list \
    CONFIG.g_ENABLE_LOCAL_CSR {false} \
    CONFIG.g_AXIS_CLK_MHZ {150} \
    CONFIG.g_TDC_CLK_MHZ {200} \
    CONFIG.g_STREAM_CLK_MODE {ASYNC} \
    CONFIG.g_OUTPUT_WIDTH {32} \
    CONFIG.g_NUM_CHIPS {4} \
    CONFIG.g_PRESENT_CHIP_MASK {1111} \
    CONFIG.g_RISE_CHIP_MASK {0011} \
    CONFIG.g_FALL_CHIP_MASK {1100} \
    CONFIG.g_MAX_STOPS_PER_CHIP {8} \
    CONFIG.g_MAX_HITS_PER_STOP {7}] $tdc

foreach {master slave} {
    lidar_unified_csr_0/motor_unified_csr motor_laser_ctrl_top_0/motor_unified_csr
    lidar_unified_csr_0/laser_unified_csr motor_laser_ctrl_top_0/laser_unified_csr
    lidar_unified_csr_0/echo_unified_csr  echo_receiver_top_0/echo_unified_csr
    lidar_unified_csr_0/tdc_unified_csr   tdc_gpx_top_0/tdc_unified_csr
} {
    reconnect_intf_net [get_bd_intf_pins $master] [get_bd_intf_pins $slave]
}

reconnect_intf_net [get_bd_intf_pins axi_interconnect_0/M00_AXI] \
    [get_bd_intf_pins lidar_unified_csr_0/s_axi_csr]

set csr_clk [get_bd_pins processing_system7_0/FCLK_CLK0]
set csr_reset [get_bd_pins rst_ps7_0_50M/peripheral_aresetn]
set proc_clk_source [get_bd_pins processing_system7_0/FCLK_CLK1]
set proc_reset [get_bd_pins rst_ps7_0_100M/peripheral_aresetn]
set tdc_clk [get_bd_pins processing_system7_0/FCLK_CLK2]

# The Zynq PS IO PLL rounds a direct 150 MHz FCLK request to 142.857 MHz in
# this board preset. Derive an exact 150 MHz processing clock from the stable
# 100 MHz FCLK1 so the packaged real-time timebases keep their stated contract.
set proc_clk_wiz [create_bd_cell -type ip \
    -vlnv xilinx.com:ip:clk_wiz:6.0 proc_clk_wiz]
set_property -dict [list \
    CONFIG.PRIM_IN_FREQ {100.000} \
    CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {150.000} \
    CONFIG.USE_RESET {false} \
    CONFIG.USE_LOCKED {true}] $proc_clk_wiz
reconnect_scalar_net $proc_clk_source \
    [get_bd_pins proc_clk_wiz/clk_in1]
set proc_clk [get_bd_pins proc_clk_wiz/clk_out1]
reconnect_scalar_net [get_bd_pins proc_clk_wiz/locked] \
    [get_bd_pins rst_ps7_0_100M/dcm_locked]
reconnect_scalar_net $proc_clk \
    [get_bd_pins rst_ps7_0_100M/slowest_sync_clk]

foreach sink {
    axi_interconnect_0/M00_ACLK
    lidar_unified_csr_0/s_axi_csr_aclk
    motor_laser_ctrl_top_0/s_axi_aclk
    echo_receiver_top_0/i_unified_cfg_clk
    tdc_gpx_top_0/i_unified_cfg_clk
} {
    reconnect_scalar_net $csr_clk [get_bd_pins $sink]
}
foreach sink {
    axi_interconnect_0/M00_ARESETN
    lidar_unified_csr_0/s_axi_csr_aresetn
    motor_laser_ctrl_top_0/s_axi_aresetn
    echo_receiver_top_0/i_unified_cfg_rst_n
    tdc_gpx_top_0/i_unified_cfg_rst_n
} {
    reconnect_scalar_net $csr_reset [get_bd_pins $sink]
}
foreach sink {
    motor_laser_ctrl_top_0/proc_aclk
    echo_receiver_top_0/axis_aclk
    tdc_gpx_top_0/i_axis_aclk
    system_ila_2/clk
} {
    reconnect_scalar_net $proc_clk [get_bd_pins $sink]
}
foreach sink {
    motor_laser_ctrl_top_0/proc_aresetn
    echo_receiver_top_0/axis_aresetn
    tdc_gpx_top_0/i_axis_aresetn
} {
    reconnect_scalar_net $proc_reset [get_bd_pins $sink]
}
reconnect_scalar_net $tdc_clk [get_bd_pins tdc_gpx_top_0/i_tdc_clk]

# Hard-real-time control remains point-to-point and outside the CSR plane.
reconnect_intf_net [get_bd_intf_pins motor_laser_ctrl_top_0/m_axis] \
    [get_bd_intf_pins echo_receiver_top_0/s_laser_evt]
foreach {source sink} {
    motor_laser_ctrl_top_0/o_shot_start      echo_receiver_top_0/i_shot_start
    motor_laser_ctrl_top_0/o_stop_tdc        echo_receiver_top_0/i_stop_tdc
    motor_laser_ctrl_top_0/o_shot_start      tdc_gpx_top_0/i_shot_start
    motor_laser_ctrl_top_0/o_stop_tdc        tdc_gpx_top_0/i_stop_tdc
    motor_laser_ctrl_top_0/o_shot_face_index tdc_gpx_top_0/i_shot_face_index
    motor_laser_ctrl_top_0/o_n_faces         tdc_gpx_top_0/i_n_faces
} {
    reconnect_scalar_net [get_bd_pins $source] [get_bd_pins $sink]
}

# Calibration metadata is deterministic until a dedicated calibration owner
# is added. 81 ps/bin and Q16.16 unity match the maintained RTL/TB profile.
set bin_resolution [create_bd_cell -type inline_hdl \
    -vlnv xilinx.com:inline_hdl:ilconstant:1.0 \
    tdc_bin_ps_c]
set_property -dict [list CONFIG.CONST_WIDTH {16} CONFIG.CONST_VAL {81}] \
    $bin_resolution
set k_dist [create_bd_cell -type inline_hdl \
    -vlnv xilinx.com:inline_hdl:ilconstant:1.0 \
    tdc_kdist_c]
set_property -dict [list CONFIG.CONST_WIDTH {32} CONFIG.CONST_VAL {65536}] \
    $k_dist
reconnect_scalar_net [get_bd_pins $bin_resolution/dout] \
    [get_bd_pins tdc_gpx_top_0/i_bin_resolution_ps]
reconnect_scalar_net [get_bd_pins $k_dist/dout] \
    [get_bd_pins tdc_gpx_top_0/i_k_dist_fixed]

set ps_space [get_bd_addr_spaces processing_system7_0/Data]
assign_bd_address -offset 0x40000000 -range 0x00001000 \
    -target_address_space $ps_space \
    [get_bd_addr_segs lidar_unified_csr_0/s_axi_csr/reg0] -force

set irq_concat [require_one [get_bd_cells xlconcat_0] {IRQ concatenator}]
set_property CONFIG.NUM_PORTS {1} $irq_concat
foreach sink {xlconcat_0/In0 system_ila_1/probe3 system_ila_1/probe4} {
    reconnect_scalar_net [get_bd_pins lidar_unified_csr_0/o_irq] \
        [get_bd_pins $sink]
}

# Physical board boundary.
foreach {external pin} {
    i_enc_a                  motor_laser_ctrl_top_0/i_enc_a
    i_enc_b                  motor_laser_ctrl_top_0/i_enc_b
    i_enc_z                  motor_laser_ctrl_top_0/i_enc_z
    o_fire_pulse             motor_laser_ctrl_top_0/o_fire_pulse
    o_start_tdc              motor_laser_ctrl_top_0/o_start_tdc
    io_tdc_d                 tdc_gpx_top_0/io_tdc_d
    o_tdc_adr                tdc_gpx_top_0/o_tdc_adr
    o_tdc_csn                tdc_gpx_top_0/o_tdc_csn
    o_tdc_rdn                tdc_gpx_top_0/o_tdc_rdn
    o_tdc_wrn                tdc_gpx_top_0/o_tdc_wrn
    o_tdc_oen                tdc_gpx_top_0/o_tdc_oen
    o_tdc_stopdis            tdc_gpx_top_0/o_tdc_stopdis
    o_tdc_alutrigger         tdc_gpx_top_0/o_tdc_alutrigger
    o_tdc_puresn             tdc_gpx_top_0/o_tdc_puresn
    i_tdc_ef1                tdc_gpx_top_0/i_tdc_ef1
    i_tdc_ef2                tdc_gpx_top_0/i_tdc_ef2
    i_tdc_lf1                tdc_gpx_top_0/i_tdc_lf1
    i_tdc_lf2                tdc_gpx_top_0/i_tdc_lf2
    i_tdc_irflag             tdc_gpx_top_0/i_tdc_irflag
    i_tdc_errflag            tdc_gpx_top_0/i_tdc_errflag
    o_vdma_hsize_bytes_rise  tdc_gpx_top_0/o_vdma_hsize_bytes_rise
    o_vdma_hsize_bytes_fall  tdc_gpx_top_0/o_vdma_hsize_bytes_fall
    o_vdma_vsize_lines       tdc_gpx_top_0/o_vdma_vsize_lines
} {
    set internal_pin [require_one [get_bd_pins -quiet $pin] \
        "candidate pin $pin"]
    puts "EXPOSING_PIN=$pin"
    expose_pin $external $internal_pin
}
set rise_axis [require_one \
    [get_bd_intf_pins -quiet tdc_gpx_top_0/o_m_axis] \
    {candidate rising-result AXIS interface}]
set fall_axis [require_one \
    [get_bd_intf_pins -quiet tdc_gpx_top_0/o_m_axis_fall] \
    {candidate falling-result AXIS interface}]
expose_interface m_axis_tdc_rise $rise_axis
expose_interface m_axis_tdc_fall $fall_axis

validate_bd_design -force
save_bd_design
generate_target all [get_files $candidate_bd]

puts {PROJECT4_UNIFIED_CANDIDATE_PASS}
puts "PROJECT4_UNIFIED_CANDIDATE_BD=$candidate_bd"
puts {PROJECT4_UNIFIED_CSR_BASE=0x40000000}
puts {PROJECT4_UNIFIED_AXIS_MHZ=150}
puts {PROJECT4_UNIFIED_TDC_MHZ=200}
close_project
exit
