# 테스트 자산 목적: 정상 운용에서 드물거나 도달 불가능한 GPX controller
# safety-net 사건을 직접 주입해 CLEAR_STATUS sticky 계약을 검증한다.
# 관련 RTL/TB: tdc_gpx_chip_ctrl, tb_tdc_gpx_chip_ctrl.
# 주의: sticky 저장 입력만 force하며 phase/FIFO/request 상태는 변경하지 않는다.

run 100 ns

set tb /tb_tdc_gpx_chip_ctrl
set dut ${tb}/u_chip_ctrl

proc expect_value {path expected message} {
    set actual [get_value $path]
    if {$actual ne $expected} {
        error "LIDAR_V2_K11_FAIL: $message expected=$expected actual=$actual"
    }
}

set fault_pairs [list \
    ${dut}/s_rsp_mismatch_event_c  ${tb}/s_err_rsp_mismatch  "lane bit 2 response mismatch" \
    ${dut}/s_raw_drop_event_c      ${tb}/s_err_raw_drop      "lane bit 3 raw drop" \
    ${dut}/s_drain_cap_event_c     ${tb}/s_err_drain_cap     "lane bit 4 drain cap" \
    ${dut}/s_cmd_collision_event_c ${tb}/s_err_cmd_collision "lane bit 11 command collision" \
    ${dut}/s_bus_fatal_event_c     ${tb}/s_err_bus_fatal     "lane bit 12 bus fatal"]

# 각 owner의 event 입력을 한 clock씩 주입해 sticky set을 확인한다.
foreach {event_path output_path label} $fault_pairs {
    add_force $event_path 1
    run 5 ns
    add_force $event_path 0
    run 5 ns
    expect_value $output_path 1 "$label did not set"
}

# CLEAR_STATUS 단독 동작: 기능 FSM과 무관하게 진단 이력만 0이 된다.
add_force ${tb}/s_soft_clear 1
run 5 ns
add_force ${tb}/s_soft_clear 0
run 5 ns
foreach {event_path output_path label} $fault_pairs {
    expect_value $output_path 0 "$label did not clear"
}

# 같은 TDC clock에 clear와 새 fault가 겹치면 새 fault가 우선한다.
foreach {event_path output_path label} $fault_pairs {
    add_force $event_path 1
}
add_force ${tb}/s_soft_clear 1
run 5 ns
foreach {event_path output_path label} $fault_pairs {
    add_force $event_path 0
}
add_force ${tb}/s_soft_clear 0
run 5 ns
foreach {event_path output_path label} $fault_pairs {
    expect_value $output_path 1 "$label lost same-cycle event"
}

# 한 번 더 clear해 다음 검사의 초기 조건을 만든다.
add_force ${tb}/s_soft_clear 1
run 5 ns
add_force ${tb}/s_soft_clear 0
run 5 ns

# live bus-fatal quarantine은 단순 이력이 아니다. 활성 상태가 남아 있으면
# CLEAR_STATUS가 sticky를 숨기지 못하고, 복구된 뒤에만 clear된다.
add_force ${dut}/s_bus_fatal_active_r 1
add_force ${tb}/s_soft_clear 1
run 5 ns
add_force ${tb}/s_soft_clear 0
run 5 ns
expect_value ${tb}/s_err_bus_fatal 1 \
    "live bus-fatal quarantine was hidden by CLEAR_STATUS"

add_force ${dut}/s_bus_fatal_active_r 0
add_force ${tb}/s_soft_clear 1
run 5 ns
add_force ${tb}/s_soft_clear 0
run 5 ns
expect_value ${tb}/s_err_bus_fatal 0 \
    "recovered bus-fatal history did not clear"

puts "LIDAR_V2_K11_TDC_STICKY_CLEAR_PASS"
quit
