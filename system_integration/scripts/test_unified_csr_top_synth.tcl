set script_dir [file normalize [file dirname [info script]]]
set hdl [file normalize [file join $script_dir ../..]]
set csr32 [file normalize [file join $hdl ../../my_axil_csr32/HDL]]

if {[llength $argv] > 0} {
    set work_dir [file normalize [lindex $argv 0]]
} else {
    set work_dir [file normalize [file join C:/tmp unified_csr_top_synth]]
}

file mkdir $work_dir
create_project -force unified_csr_top_synth $work_dir -part xc7z020clg484-2

set sources [list \
    [file join $csr32 my_axil_csr32_pkg.vhd] \
    [file join $csr32 axil_fsm_32.vhd] \
    [file join $csr32 axil_ctrl_regs_32.vhd] \
    [file join $csr32 axil_stat_regs_32.vhd] \
    [file join $csr32 axil_intr_32.vhd] \
    [file join $csr32 my_axil_csr32_top.vhd] \
    [file join $hdl system_integration/rtl/lidar_unified_csr_pkg.vhd] \
    [file join $hdl system_integration/rtl/lidar_unified_csr_top.vhd]]

foreach source $sources {
    read_vhdl -vhdl2008 $source
}

synth_design -top lidar_unified_csr_top -part xc7z020clg484-2 \
    -flatten_hierarchy none

set black_boxes [get_cells -hier -quiet -filter {IS_BLACKBOX == 1}]
if {[llength $black_boxes] != 0} {
    error "Unified CSR top contains black boxes: $black_boxes"
}

set csr_owners [get_cells -hier -quiet -filter \
    {REF_NAME == my_axil_csr32_top || ORIG_REF_NAME == my_axil_csr32_top}]
if {[llength $csr_owners] != 1} {
    error "Unified CSR top must contain exactly one CSR32 owner: $csr_owners"
}

set legacy_owners [get_cells -hier -quiet -filter \
    {REF_NAME == my_axil_csr_top || ORIG_REF_NAME == my_axil_csr_top}]
if {[llength $legacy_owners] != 0} {
    error "Unified CSR top retained a legacy local CSR owner: $legacy_owners"
}

set utilization [report_utilization -return_string]
set report_file [file join $work_dir utilization.rpt]
set channel [open $report_file w]
puts $channel $utilization
close $channel

puts "BLACK_BOX_COUNT=[llength $black_boxes]"
puts "CSR32_OWNER_COUNT=[llength $csr_owners]"
puts "LEGACY_CSR_OWNER_COUNT=[llength $legacy_owners]"
puts "LIDAR_UNIFIED_CSR_TOP_SYNTH_PASS"

close_project
exit
