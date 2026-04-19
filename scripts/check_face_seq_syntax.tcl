open_project {C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/tdc_gpx_ctrl.xpr}
puts "=== check_syntax on sources_1 ==="
if {[catch {check_syntax -fileset sources_1} err]} {
    puts "SYNTAX CHECK FAILED: $err"
    close_project
    exit 1
}
puts "=== check_syntax PASSED ==="
close_project
exit 0
