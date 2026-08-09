# Vivado 2025.2.1 L0 parent IP-property inventory.
# This creates only an in-memory project and does not modify a board project.

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

set script_dir [file normalize [file dirname [info script]]]
set v2_dir [file normalize [file join $script_dir ..]]

create_project -in_memory lidar_v2_l0_property_query -part xc7z020clg484-2
set_property ip_repo_paths [list [file join $v2_dir ip_repo]] [current_project]
update_ip_catalog -rebuild
create_bd_design l0_property_query

set vdma [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_vdma:6.3 vdma_q]
set gpio [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 gpio_q]
set protocol_converter [create_bd_cell -type ip \
    -vlnv xilinx.com:ip:axi_protocol_converter:2.1 protocol_converter_q]
set ps7 [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7_q]
set lidar [create_bd_cell -type ip \
    -vlnv victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v2:2.0 lidar_q]
set_property -dict [list \
    CONFIG.G_CSR_CLK_MHZ 100 \
    CONFIG.G_PROC_CLK_MHZ 150 \
    CONFIG.G_TDC_CLK_MHZ 200 \
    CONFIG.G_STREAM_CLK_MODE ASYNC \
    CONFIG.G_NUM_CHIPS 2 \
    CONFIG.G_STOPS_PER_CHIP 8 \
    CONFIG.G_MAX_RETURNS_PER_STOP 7 \
    CONFIG.G_RISE_CAPABILITY_MASK 0011 \
    CONFIG.G_FALL_CAPABILITY_MASK 0000 \
    CONFIG.G_OUTPUT_WIDTH 32 \
    CONFIG.G_NUM_FACES 5 \
    CONFIG.G_ENABLE_ECHO_RECEIVER false \
    CONFIG.G_ENABLE_ECHO_SIMULATION false \
    CONFIG.G_OEN_MODE PULLUP_OR_NOT_CONNECTED] $lidar

foreach {label cell patterns} [list \
        AXI_VDMA $vdma {CONFIG.*S2MM* CONFIG.*MM2S* CONFIG.*FSTORE* \
            CONFIG.*INCLUDE_SG* CONFIG.*DATA_WIDTH* CONFIG.*LINEBUFFER* \
            CONFIG.*s2mm* CONFIG.*mm2s* CONFIG.*fstore* \
            CONFIG.*include_sg* CONFIG.*data_width* CONFIG.*linebuffer*} \
        AXI_GPIO $gpio {CONFIG.*GPIO_WIDTH* CONFIG.*ALL_INPUTS* \
            CONFIG.*ALL_OUTPUTS* CONFIG.*DUAL* CONFIG.*DOUT_DEFAULT*} \
        AXI_PROTOCOL_CONVERTER $protocol_converter {CONFIG.*PROTOCOL* \
            CONFIG.*DATA_WIDTH* CONFIG.*ID_WIDTH* CONFIG.*THREAD*} \
        PS7 $ps7 {CONFIG.PCW_FPGA_FCLK* CONFIG.PCW_FPGA*_FREQMHZ \
            CONFIG.PCW_USE_S_AXI_HP* \
            CONFIG.PCW_S_AXI_HP*_DATA_WIDTH CONFIG.PCW_USE_M_AXI_GP*}] {
    puts "BEGIN_${label}_PROPERTIES"
    foreach property [lsort [list_property $cell]] {
        set matched false
        foreach pattern $patterns {
            if {[string match $pattern $property]} {
                set matched true
                break
            }
        }
        if {$matched} {
            puts "$property=[get_property $property $cell]"
        }
    }
    puts "END_${label}_PROPERTIES"
}

puts {BEGIN_LIDAR_INTERFACES}
foreach object [lsort [get_bd_intf_pins -of_objects $lidar]] {
    puts "[get_property NAME $object]:[get_property VLNV $object]:[get_property MODE $object]"
}
puts {END_LIDAR_INTERFACES}
puts {BEGIN_LIDAR_PINS}
foreach object [lsort [get_bd_pins -of_objects $lidar]] {
    puts "[get_property NAME $object]:[get_property DIR $object]:[get_property LEFT $object]:[get_property RIGHT $object]"
}
puts {END_LIDAR_PINS}

puts {LIDAR_V2_L0_IP_PROPERTY_QUERY_PASS}
close_project
exit 0
