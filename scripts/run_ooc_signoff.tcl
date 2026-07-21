# =============================================================================
# Reproducible out-of-context implementation sign-off for tdc_gpx_top.
#
# The source project is opened read-only. All generated reports/checkpoints are
# written below HDL/signoff_results/sessions by the PowerShell wrapper.
# =============================================================================

if {$argc != 12} {
    error "usage: run_ooc_signoff.tcl OUT_DIR WIDTH AXIS_MHZ TDC_MHZ PRESENT_MASK RISE_MASK FALL_MASK MAX_STOPS MAX_HITS STREAM_MODE DO_IMPL IMPL_STRATEGY"
}

set out_dir      [file normalize [lindex $argv 0]]
set width        [lindex $argv 1]
set axis_mhz     [lindex $argv 2]
set tdc_mhz      [lindex $argv 3]
set present_mask [lindex $argv 4]
set rise_mask    [lindex $argv 5]
set fall_mask    [lindex $argv 6]
set max_stops    [lindex $argv 7]
set max_hits     [lindex $argv 8]
set stream_mode  [lindex $argv 9]
set do_impl      [expr {[lindex $argv 10] eq "1"}]
set impl_strategy [lindex $argv 11]

if {$impl_strategy ni {DEFAULT TIMING_EXPLORE}} {
    error "unsupported implementation strategy: $impl_strategy"
}

set project_file "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/tdc_gpx_ctrl.xpr"
set part_name   "xc7z020clg484-2"
set project_dir [file dirname $project_file]
set gen_ip_dir  [file join $project_dir tdc_gpx_ctrl.gen sources_1 ip]
file mkdir $out_dir

puts "OOC_SIGNOFF_CONFIG width=$width axis_mhz=$axis_mhz tdc_mhz=$tdc_mhz present=$present_mask rise=$rise_mask fall=$fall_mask max_stops=$max_stops max_hits=$max_hits stream=$stream_mode impl=$do_impl strategy=$impl_strategy"

set axis_period [expr {1000.0 / double($axis_mhz)}]
set tdc_period  [expr {1000.0 / double($tdc_mhz)}]
set axi_period  10.0
set xdc_file [file join $out_dir ooc_timing.xdc]
set xdc [open $xdc_file w]
puts $xdc "create_clock -name axis_clk -period $axis_period \[get_ports i_axis_aclk\]"
puts $xdc "create_clock -name tdc_clk -period $tdc_period \[get_ports i_tdc_clk\]"
puts $xdc "create_clock -name axi_clk -period $axi_period \[get_ports s_axi_aclk\]"
if {$stream_mode eq "ASYNC"} {
    puts $xdc "set_clock_groups -asynchronous -group \[get_clocks axis_clk\] -group \[get_clocks tdc_clk\] -group \[get_clocks axi_clk\]"
} else {
    puts $xdc "set_clock_groups -asynchronous -group \[get_clocks axi_clk\] -group \[get_clocks {axis_clk tdc_clk}\]"
}
puts $xdc "set_false_path -from \[get_ports {i_axis_aresetn s_axi_aresetn}\]"
close $xdc

open_project -read_only $project_file
read_vhdl -vhdl2008 -library xil_defaultlib \
    [file join [file dirname [info script]] .. tdc_gpx_sync_fifo.vhd]
read_vhdl -vhdl2008 -library xil_defaultlib \
    [file join [file dirname [info script]] .. tdc_gpx_line_packer.vhd]
read_vhdl -vhdl2008 -library xil_defaultlib \
    [file join [file dirname [info script]] .. tdc_gpx_reg_rsp_cdc.vhd]

# The source .xpr has no OOC DCPs for the two custom CSR IPs. Read their
# generated implementation sources so a successful run cannot hide black boxes.
foreach source [list \
    [file join $gen_ip_dir tdc_gpx_axil_csr_pipeline src axil_fsm.vhd] \
    [file join $gen_ip_dir tdc_gpx_axil_csr_pipeline src axil_ctrl_regs.vhd] \
    [file join $gen_ip_dir tdc_gpx_axil_csr_pipeline src axil_stat_regs.vhd] \
    [file join $gen_ip_dir tdc_gpx_axil_csr_pipeline src axil_intr.vhd] \
    [file join $gen_ip_dir tdc_gpx_axil_csr_pipeline src my_axil_csr_top.vhd] \
    [file join $gen_ip_dir tdc_gpx_axil_csr_pipeline synth tdc_gpx_axil_csr_pipeline.vhd] \
    [file join $gen_ip_dir tdc_gpx_axil_csr32_chip src axil_fsm_32.vhd] \
    [file join $gen_ip_dir tdc_gpx_axil_csr32_chip src axil_ctrl_regs_32.vhd] \
    [file join $gen_ip_dir tdc_gpx_axil_csr32_chip src axil_stat_regs_32.vhd] \
    [file join $gen_ip_dir tdc_gpx_axil_csr32_chip src axil_intr_32.vhd] \
    [file join $gen_ip_dir tdc_gpx_axil_csr32_chip src my_axil_csr32_top.vhd] \
    [file join $gen_ip_dir tdc_gpx_axil_csr32_chip synth tdc_gpx_axil_csr32_chip.vhd]] {
    read_vhdl -vhdl2008 -library xil_defaultlib $source
}
update_compile_order -fileset sources_1
read_xdc -unmanaged $xdc_file

set generics [list \
    "g_OUTPUT_WIDTH=$width" \
    "g_PRESENT_CHIP_MASK=4'b$present_mask" \
    "g_RISE_CHIP_MASK=4'b$rise_mask" \
    "g_FALL_CHIP_MASK=4'b$fall_mask" \
    "g_MAX_STOPS_PER_CHIP=$max_stops" \
    "g_MAX_HITS_PER_STOP=$max_hits" \
    "g_AXIS_CLK_MHZ=$axis_mhz" \
    "g_TDC_CLK_MHZ=$tdc_mhz" \
    "g_STREAM_CLK_MODE=$stream_mode"]

synth_design \
    -top tdc_gpx_top \
    -part $part_name \
    -mode out_of_context \
    -flatten_hierarchy rebuilt \
    -generic $generics

set black_boxes [get_cells -hierarchical -filter {IS_BLACKBOX == 1}]
if {[llength $black_boxes] != 0} {
    set black_box_report [open [file join $out_dir post_synth_black_boxes.rpt] w]
    foreach cell $black_boxes {
        puts $black_box_report $cell
    }
    close $black_box_report
    error "OOC sign-off found [llength $black_boxes] black-box instance(s)"
}

# Reset deassertion is synchronized in RTL. Board-level input/output delays and
# pin constraints belong to the parent design; this OOC run closes reg-to-reg
# timing and reports external unconstrained endpoints explicitly.
check_timing -verbose -file [file join $out_dir post_synth_check_timing.rpt]
report_utilization -hierarchical -hierarchical_depth 6 \
    -file [file join $out_dir post_synth_utilization_hier.rpt]
report_timing_summary -delay_type min_max -report_unconstrained -max_paths 50 \
    -file [file join $out_dir post_synth_timing_summary.rpt]
report_cdc -details -file [file join $out_dir post_synth_cdc_full.rpt]
report_cdc -details \
    -from [get_clocks {axis_clk tdc_clk axi_clk}] \
    -to [get_clocks {axis_clk tdc_clk axi_clk}] \
    -file [file join $out_dir post_synth_cdc_data.rpt]
report_clock_interaction -file [file join $out_dir post_synth_clock_interaction.rpt]
report_methodology -file [file join $out_dir post_synth_methodology.rpt]
report_drc -file [file join $out_dir post_synth_drc.rpt]
report_control_sets -verbose -file [file join $out_dir post_synth_control_sets.rpt]
write_checkpoint -force [file join $out_dir post_synth.dcp]
puts "OOC_SIGNOFF_SYNTH_PASS"

if {$do_impl} {
    if {$impl_strategy eq "TIMING_EXPLORE"} {
        opt_design -directive ExploreWithRemap
        place_design -directive Explore
        phys_opt_design -directive AggressiveExplore
        route_design -directive AggressiveExplore -tns_cleanup

        # Post-route physical optimization uses actual routed delay rather
        # than pre-route estimates. Run it only for a remaining setup miss so
        # configurations already closed by the first route keep their result.
        set worst_setup_paths [get_timing_paths -setup -max_paths 1 -nworst 1]
        if {[llength $worst_setup_paths] != 0} {
            set first_route_wns [get_property SLACK [lindex $worst_setup_paths 0]]
            puts "OOC_SIGNOFF_FIRST_ROUTE_WNS=$first_route_wns"
            if {$first_route_wns < 0.0} {
                phys_opt_design -directive AggressiveExplore
                route_design -directive AggressiveExplore -tns_cleanup
                puts "OOC_SIGNOFF_POST_ROUTE_PHYS_OPT_DONE"
            }
        }
    } else {
        opt_design
        place_design
        phys_opt_design
        route_design
    }

    check_timing -verbose -file [file join $out_dir post_route_check_timing.rpt]
    report_utilization -hierarchical -hierarchical_depth 6 \
        -file [file join $out_dir post_route_utilization_hier.rpt]
    report_timing_summary -delay_type min_max -report_unconstrained -max_paths 50 \
        -file [file join $out_dir post_route_timing_summary.rpt]
    report_cdc -details -file [file join $out_dir post_route_cdc_full.rpt]
    report_cdc -details \
        -from [get_clocks {axis_clk tdc_clk axi_clk}] \
        -to [get_clocks {axis_clk tdc_clk axi_clk}] \
        -file [file join $out_dir post_route_cdc_data.rpt]
    report_clock_interaction -file [file join $out_dir post_route_clock_interaction.rpt]
    report_methodology -file [file join $out_dir post_route_methodology.rpt]
    report_drc -file [file join $out_dir post_route_drc.rpt]
    report_route_status -file [file join $out_dir post_route_status.rpt]
    write_checkpoint -force [file join $out_dir post_route.dcp]
    puts "OOC_SIGNOFF_IMPL_PASS"
}

close_project
