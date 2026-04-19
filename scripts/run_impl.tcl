open_project C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/tdc_gpx_ctrl.xpr
reset_run impl_1
launch_runs impl_1 -jobs 4
wait_on_runs impl_1
set status [get_property STATUS [get_runs impl_1]]
set progress [get_property PROGRESS [get_runs impl_1]]
puts "IMPL STATUS: $status"
puts "IMPL PROGRESS: $progress"
if {[string match "*Complete*" $status]} {
    puts "IMPLEMENTATION DONE"
    open_run impl_1
    report_utilization -file C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/impl_util.txt
    report_timing_summary -file C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/impl_timing.txt -max_paths 20
    report_clock_utilization -file C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/impl_clocks.txt
    report_cdc -file C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/impl_cdc.txt
    puts "Reports written"
} else {
    puts "IMPLEMENTATION FAILED"
}
close_project
