# Verify the generated raw-stream clock topology in an existing synthesis DCP.
# Usage: vivado -mode batch -source check_raw_stream_checkpoint.tcl \
#        -tclargs <post_synth.dcp> <ASYNC|SYNC> <output.txt>

if {$argc != 3} {
    error {Usage: check_raw_stream_checkpoint.tcl <dcp> <ASYNC|SYNC> <output>}
}

set dcp_path [file normalize [lindex $argv 0]]
set stream_mode [string toupper [lindex $argv 1]]
set output_path [file normalize [lindex $argv 2]]

if {$stream_mode ni {ASYNC SYNC}} {
    error {Stream mode must be ASYNC or SYNC}
}
if {![file exists $dcp_path]} {
    error "Checkpoint does not exist: $dcp_path"
}

# Avoid the damaged per-user Tcl Store on this workstation. Opening a DCP can
# load XPM Tcl helpers before any explicit project command runs.
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

open_checkpoint $dcp_path
set raw_cdc_cells [get_cells -hierarchical -quiet \
    -filter {NAME =~ *.gen_raw_async.u_raw_cdc}]
set raw_cdc_count [llength $raw_cdc_cells]

if {$stream_mode eq {ASYNC} && $raw_cdc_count != 4} {
    error "ASYNC checkpoint expected four raw-stream CDC FIFOs, got $raw_cdc_count"
}
if {$stream_mode eq {SYNC} && $raw_cdc_count != 0} {
    error "SYNC checkpoint unexpectedly contains $raw_cdc_count raw-stream CDC FIFO(s)"
}

set report [open $output_path w]
puts $report "stream_mode=$stream_mode"
puts $report "raw_cdc_cell_count=$raw_cdc_count"
foreach cell $raw_cdc_cells {
    puts $report $cell
}
close $report

puts "RAW_STREAM_CHECKPOINT_PASS mode=$stream_mode raw_cdc_cells=$raw_cdc_count"
close_design
exit 0
