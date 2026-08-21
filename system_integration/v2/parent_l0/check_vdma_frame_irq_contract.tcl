# Check the AXI VDMA synthesis contract required by the V3 PS adapter.
# The output IP is generated only below the caller-provided .work directory.

if {[llength $argv] != 1} {
    error {Usage: check_vdma_frame_irq_contract.tcl WORK_DIR}
}
set work_dir [file normalize [lindex $argv 0]]
file mkdir $work_dir
cd $work_dir

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

proc expect_equal {label actual expected} {
    if {$actual ne $expected} {
        error "$label mismatch: expected=$expected actual=$actual"
    }
}

proc expect_fixed_model_parameter {xml_text parameter expected} {
    set name_text "<spirit:name>${parameter}</spirit:name>"
    set name_index [string first $name_text $xml_text]
    if {$name_index < 0} {
        error "AXI VDMA model parameter is missing: $parameter"
    }
    set parameter_block [string range $xml_text $name_index \
        [expr {$name_index + 1000}]]
    set value_text ">${expected}</spirit:value>"
    if {[string first $value_text $parameter_block] < 0} {
        error "AXI VDMA $parameter is not fixed to $expected"
    }
}

proc expect_xci_value {xci_text parameter expected} {
    set name_text "\"${parameter}\":"
    set name_index [string first $name_text $xci_text]
    if {$name_index < 0} {
        error "Generated AXI VDMA XCI parameter is missing: $parameter"
    }
    set parameter_block [string range $xci_text $name_index \
        [expr {$name_index + 300}]]
    set value_text "\"value\": \"${expected}\""
    if {[string first $value_text $parameter_block] < 0} {
        error "Generated AXI VDMA XCI $parameter is not $expected"
    }
}

set component_xml [file normalize [file join $::env(XILINX_VIVADO) \
    data ip xilinx axi_vdma_v6_3 component.xml]]
if {![file exists $component_xml]} {
    error "AXI VDMA 6.3 component.xml is missing: $component_xml"
}
set channel [open $component_xml r]
set component_text [read $channel]
close $channel

# These are generated Model Parameters, not editable Block Design CONFIGs.
# INFO_14 keeps the S2MM delay field writable; INFO_15 implements the S2MM
# frame-completion counter used by XAxiVdma_SetFrameCounter().
expect_fixed_model_parameter $component_text C_ENABLE_DEBUG_INFO_14 1
expect_fixed_model_parameter $component_text C_ENABLE_DEBUG_INFO_15 1

create_project -in_memory lidar_vdma_frame_irq_contract \
    -part xc7z020clg484-2
create_ip -name axi_vdma -vendor xilinx.com -library ip -version 6.3 \
    -module_name vdma_frame_irq_contract -dir $work_dir
set vdma [get_ips vdma_frame_irq_contract]
set_property -dict [list \
    CONFIG.c_include_mm2s 0 \
    CONFIG.c_include_s2mm 1 \
    CONFIG.c_include_sg 0 \
    CONFIG.c_num_fstores 3 \
    CONFIG.c_m_axi_s2mm_data_width 64 \
    CONFIG.c_include_s2mm_sf 1 \
    CONFIG.c_s2mm_linebuffer_depth 512 \
    CONFIG.c_s2mm_max_burst_length 16 \
    CONFIG.c_s2mm_sof_enable 1 \
    CONFIG.c_use_s2mm_fsync 0] $vdma

expect_equal MM2S [get_property CONFIG.c_include_mm2s $vdma] 0
expect_equal S2MM [get_property CONFIG.c_include_s2mm $vdma] 1
expect_equal scatter_gather [get_property CONFIG.c_include_sg $vdma] 0
expect_equal frame_stores [get_property CONFIG.c_num_fstores $vdma] 3

# Do not stop at the editable CONFIG view.  Read the generated XCI because the
# frame/delay counter feature flags are fixed Model Parameters and are what the
# standalone driver finally receives through XAxiVdma_Config.
set xci_path [get_property IP_FILE $vdma]
if {![file exists $xci_path]} {
    error "Generated AXI VDMA XCI is missing: $xci_path"
}
set channel [open $xci_path r]
set xci_text [read $channel]
close $channel
expect_xci_value $xci_text C_INCLUDE_MM2S 0
expect_xci_value $xci_text C_INCLUDE_S2MM 1
expect_xci_value $xci_text C_NUM_FSTORES 3
expect_xci_value $xci_text C_ENABLE_DEBUG_INFO_14 1
expect_xci_value $xci_text C_ENABLE_DEBUG_INFO_15 1

puts {LIDAR_VDMA_FRAME_IRQ_CONTRACT_PASS stores=3 xci_info14=1 xci_info15=1}
close_project
exit 0
