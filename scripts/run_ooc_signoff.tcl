# =============================================================================
# Reproducible out-of-context implementation sign-off for tdc_gpx_top.
#
# The flow builds an in-memory project from the canonical RTL manifest. All
# generated reports/checkpoints are written below HDL/signoff_results/sessions.
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
set num_chips 0
foreach bit [split $present_mask ""] {
    if {$bit eq "1"} {
        incr num_chips
    }
}

if {$impl_strategy ni {DEFAULT TIMING_EXPLORE}} {
    error "unsupported implementation strategy: $impl_strategy"
}

set part_name   "xc7z020clg484-2"
set script_dir  [file dirname [file normalize [info script]]]
set hdl_dir     [file normalize [file join $script_dir ..]]
set project_dir [file dirname $hdl_dir]
set ip_root     [file dirname $project_dir]
file mkdir $out_dir

puts "OOC_SIGNOFF_CONFIG width=$width axis_mhz=$axis_mhz tdc_mhz=$tdc_mhz chips=$num_chips present=$present_mask rise=$rise_mask fall=$fall_mask max_stops=$max_stops max_hits=$max_hits stream=$stream_mode impl=$do_impl strategy=$impl_strategy"

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

create_project -in_memory -part $part_name
set_property target_language VHDL [current_project]
set_property simulator_language Mixed [current_project]
set_property default_lib xil_defaultlib [current_project]

source [file join $script_dir tdc_gpx_csr_source_manifest.tcl]
foreach source [tdc_gpx_csr_source_manifest $ip_root] {
    if {![file exists $source]} {
        error "canonical CSR source is missing: $source"
    }
    read_vhdl -vhdl2008 -library xil_defaultlib $source
}

source [file join $script_dir tdc_gpx_rtl_manifest.tcl]
foreach file_name [tdc_gpx_rtl_manifest] {
    set source [file join $hdl_dir $file_name]
    if {![file exists $source]} {
        error "canonical RTL source is missing: $source"
    }
    read_vhdl -vhdl2008 -library xil_defaultlib $source
}

update_compile_order -fileset sources_1
read_xdc -unmanaged $xdc_file

set generics [list \
    "g_OUTPUT_WIDTH=$width" \
    "g_NUM_CHIPS=$num_chips" \
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

# The logical ABI remains four slots, while physical pins scale with the
# synthesis-time chip count. Count synthesized bit ports independently of the
# VHDL helper functions used for logical-lane compaction.
proc vector_port_width {base_name} {
    set pattern [format {^%s(\[[0-9]+\])?$} $base_name]
    return [llength [get_ports -quiet -regexp $pattern]]
}
set physical_port_contract [list \
    [list io_tdc_d [expr {$num_chips * 28}]] \
    [list o_tdc_adr [expr {$num_chips * 4}]] \
    [list o_tdc_csn $num_chips] \
    [list o_tdc_rdn $num_chips] \
    [list o_tdc_wrn $num_chips] \
    [list o_tdc_oen $num_chips] \
    [list o_tdc_stopdis $num_chips] \
    [list o_tdc_alutrigger $num_chips] \
    [list o_tdc_puresn $num_chips] \
    [list i_tdc_ef1 $num_chips] \
    [list i_tdc_ef2 $num_chips] \
    [list i_tdc_lf1 $num_chips] \
    [list i_tdc_lf2 $num_chips] \
    [list i_tdc_irflag $num_chips] \
    [list i_tdc_errflag $num_chips]]
set port_report [open [file join $out_dir physical_port_contract.txt] w]
foreach spec $physical_port_contract {
    lassign $spec port_name expected_width
    set actual_width [vector_port_width $port_name]
    if {$actual_width != $expected_width} {
        puts $port_report "FAIL port=$port_name expected=$expected_width actual=$actual_width"
        close $port_report
        error "physical port '$port_name' expected width $expected_width, got $actual_width"
    }
    puts $port_report "PASS port=$port_name width=$actual_width"
}
close $port_report
puts "OOC_PHYSICAL_PORT_CONTRACT_PASS chips=$num_chips"

# Prove the selected generated topology after synthesis. The fixed four-chip
# processing array retains four raw-stream CDC FIFO roots in ASYNC mode; SYNC
# must remove every u_raw_cdc instance.
# Match only the four XPM wrapper roots, not all descendants whose hierarchical
# path also contains u_raw_cdc.
set raw_cdc_cells [get_cells -hierarchical -quiet \
    -filter {NAME =~ *.gen_raw_async.u_raw_cdc}]
set raw_cdc_count [llength $raw_cdc_cells]
set topology_report [open [file join $out_dir raw_stream_topology.txt] w]
puts $topology_report "stream_mode=$stream_mode"
puts $topology_report "raw_cdc_cell_count=$raw_cdc_count"
foreach cell $raw_cdc_cells {
    puts $topology_report $cell
}
close $topology_report
if {$stream_mode eq "ASYNC" && $raw_cdc_count != 4} {
    error "ASYNC topology expected four raw-stream CDC FIFOs, got $raw_cdc_count"
}
if {$stream_mode eq "SYNC" && $raw_cdc_count != 0} {
    error "SYNC topology unexpectedly contains $raw_cdc_count raw-stream CDC cell(s)"
}
puts "OOC_RAW_STREAM_TOPOLOGY_PASS mode=$stream_mode raw_cdc_cells=$raw_cdc_count"

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
