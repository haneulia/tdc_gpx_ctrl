run 200 ns

set p /tb_tdc_gpx_top_int/u_dut

puts "C06_PROBE: force frame_done_faulted_rise"
add_force ${p}/s_frame_done_faulted_rise 1
run 5 ns
add_force ${p}/s_frame_done_faulted_rise 0
run 5 ns
set frame_sticky [get_value ${p}/s_frame_done_faulted_sticky_r]
puts "C06_PROBE: frame_done_faulted_sticky=$frame_sticky"
if {$frame_sticky ne "1"} {
    error "C06_PROBE_FAIL: frame_done_faulted_sticky did not set"
}

puts "C06_PROBE: force row_done_faulted_fall"
add_force ${p}/s_row_done_faulted_fall 1
run 5 ns
add_force ${p}/s_row_done_faulted_fall 0
run 5 ns
set row_sticky [get_value ${p}/s_row_done_faulted_sticky_r]
puts "C06_PROBE: row_done_faulted_sticky=$row_sticky"
if {$row_sticky ne "1"} {
    error "C06_PROBE_FAIL: row_done_faulted_sticky did not set"
}

puts "C06_PROBE: force run_timeout"
add_force ${p}/s_run_timeout 0001
run 5 ns
add_force ${p}/s_run_timeout 0000
run 5 ns
set run_mask [get_value -radix bin ${p}/s_run_timeout_sticky_r]
puts "C06_PROBE: run_timeout_sticky=$run_mask"
if {$run_mask ne "0001"} {
    error "C06_PROBE_FAIL: run_timeout_sticky did not set"
}

puts "C06_PROBE: force quarantine_escape"
add_force ${p}/s_quarantine_escape_rise 0010
run 5 ns
add_force ${p}/s_quarantine_escape_rise 0000
run 5 ns
set quarantine_mask [get_value -radix bin ${p}/s_quarantine_escape_mask_r]
puts "C06_PROBE: quarantine_escape_mask=$quarantine_mask"
if {$quarantine_mask ne "0010"} {
    error "C06_PROBE_FAIL: quarantine_escape_mask did not set"
}

puts "C06_PROBE: force err_soft_clear"
add_force ${p}/s_err_soft_clear 1
run 5 ns
add_force ${p}/s_err_soft_clear 0
run 5 ns

set frame_after [get_value ${p}/s_frame_done_faulted_sticky_r]
set row_after [get_value ${p}/s_row_done_faulted_sticky_r]
set run_after [get_value -radix bin ${p}/s_run_timeout_sticky_r]
set quarantine_after [get_value -radix bin ${p}/s_quarantine_escape_mask_r]
puts "C06_PROBE: after soft_clear frame=$frame_after row=$row_after run=$run_after quarantine=$quarantine_after"

if {$frame_after ne "0"} {
    error "C06_PROBE_FAIL: frame sticky was not soft-cleared"
}
if {$row_after ne "0"} {
    error "C06_PROBE_FAIL: row sticky was not soft-cleared"
}
if {$run_after ne "0000"} {
    error "C06_PROBE_FAIL: run_timeout sticky was not soft-cleared"
}
if {$quarantine_after ne "0010"} {
    error "C06_PROBE_FAIL: quarantine mask should remain reset-only"
}

puts "C06_PROBE_PASS: top sticky soft_clear policy verified"
quit
