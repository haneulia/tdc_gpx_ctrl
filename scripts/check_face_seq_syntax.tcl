open_project {C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/tdc_gpx_ctrl.xpr}
set sync_fifo {C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/tdc_gpx_sync_fifo.vhd}
if {[llength [get_files -quiet $sync_fifo]] == 0} {
    puts "=== adding missing tdc_gpx_sync_fifo.vhd to sources_1 for this syntax session ==="
    add_files -quiet -fileset sources_1 $sync_fifo
    set_property file_type {VHDL 2008} [get_files $sync_fifo]
}
update_compile_order -fileset sources_1
puts "=== check_syntax on sources_1 ==="
if {[catch {check_syntax -fileset sources_1} err]} {
    puts "SYNTAX CHECK FAILED: $err"
    close_project
    exit 1
}
puts "=== check_syntax PASSED ==="
close_project
exit 0
