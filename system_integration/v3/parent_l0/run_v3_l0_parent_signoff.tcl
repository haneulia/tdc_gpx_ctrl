# Synthesize and implement the generated Stage L0 parent project.
# The flow fails on missing board pins, timing failure, or routed DRC errors.

if {[llength $argv] != 3} {
    error {Usage: run_v3_l0_parent_signoff.tcl PROJECT_ROOT RESULT_DIR MODE}
}
lassign $argv project_root result_dir run_mode
set project_root [file normalize $project_root]
set result_dir [file normalize $result_dir]
set run_mode [string toupper $run_mode]
if {$run_mode ni {SYNTH IMPL}} {
    error "Unsupported L0 sign-off mode: $run_mode"
}

set project_path [file join $project_root project_4_lidar_v3_l0.xpr]
if {![file exists $project_path]} {
    error "Generated L0 project is missing: $project_path"
}
file mkdir $result_dir

source [file join [file dirname [info script]] verify_v3_l0_gpx_iob.tcl]

proc l0_expect_run_complete {run_name} {
    set status [get_property STATUS [get_runs $run_name]]
    if {![string match {*Complete*} $status]} {
        error "$run_name failed: status=$status"
    }
    return $status
}

proc l0_worst_slack {delay_type} {
    set path [lindex [get_timing_paths -quiet -delay_type $delay_type \
        -max_paths 1] 0]
    if {$path eq {}} {
        error "No $delay_type timing path was found"
    }
    return [get_property SLACK $path]
}

# PS FCLK2가 공급하는 TDC-GPX bus/acquisition 200 MHz 도메인을 전체 설계
# WNS/WHS와 분리해 관측한다.  0.100 ns는 timing PASS 기준이 아니라 5 ns
# 주기의 2% 미만 경로를 조기에 드러내는 유지보수용 민감도 기준이다.
proc l0_tdc_200mhz_metrics {} {
    set setup_paths [get_timing_paths -quiet -group clk_fpga_2 \
        -delay_type max -max_paths 100 -nworst 100]
    set hold_paths [get_timing_paths -quiet -group clk_fpga_2 \
        -delay_type min -max_paths 100 -nworst 100]
    if {[llength $setup_paths] == 0 || [llength $hold_paths] == 0} {
        error {TDC 200 MHz timing path group clk_fpga_2 is empty}
    }

    set setup_tight_count 0
    foreach path $setup_paths {
        if {[get_property SLACK $path] < 0.100} {
            incr setup_tight_count
        }
    }
    set hold_tight_count 0
    foreach path $hold_paths {
        if {[get_property SLACK $path] < 0.020} {
            incr hold_tight_count
        }
    }

    return [list \
        [get_property SLACK [lindex $setup_paths 0]] \
        [get_property SLACK [lindex $hold_paths 0]] \
        $setup_tight_count $hold_tight_count]
}

# TDC 200 MHz Setup 경로를 Reset 분배와 일반 상태/데이터 처리로 나눈다.
# proc_sys_reset의 peripheral_aresetn은 기능상 정상인 동기 Reset이지만,
# 4-Chip IOB 레지스터 전체로 퍼지는 고팬아웃 배선이므로 일반 처리 경로와
# 같은 WNS 하나로만 보면 병목의 물리 원인을 구분할 수 없다.
proc l0_tdc_setup_class_metrics {} {
    set paths [get_timing_paths -quiet -group clk_fpga_2 \
        -delay_type max -max_paths 1000 -nworst 1000]
    if {[llength $paths] == 0} {
        error {TDC 200 MHz Setup classification path set is empty}
    }

    set worst_reset {}
    set worst_regular {}
    set reset_tight_count 0
    set regular_tight_count 0
    foreach path $paths {
        set source [get_property STARTPOINT_PIN $path]
        set endpoint [get_property ENDPOINT_PIN $path]
        set slack [get_property SLACK $path]
        set is_reset [expr {
            [string match {*rst_tdc*} $source] ||
            [regexp {/(R|S|CLR|PRE)$} $endpoint]
        }]
        if {$is_reset} {
            if {$worst_reset eq {}} {
                set worst_reset $path
            }
            if {$slack < 0.100} {
                incr reset_tight_count
            }
        } else {
            if {$worst_regular eq {}} {
                set worst_regular $path
            }
            if {$slack < 0.100} {
                incr regular_tight_count
            }
        }
    }
    if {$worst_reset eq {} || $worst_regular eq {}} {
        error {TDC 200 MHz Setup classification did not find both path classes}
    }
    return [list \
        [get_property SLACK $worst_reset] \
        [get_property SLACK $worst_regular] \
        $reset_tight_count $regular_tight_count \
        [get_property MAX_FANOUT $worst_reset]]
}

proc l0_tdc_reset_replication_net {} {
    set reset_net [get_nets -quiet -hier -regexp \
        {^.*/rst_tdc/U0/peripheral_aresetn\[0\]$}]
    if {[llength $reset_net] != 1} {
        error "TDC Reset source net mismatch: expected=1 actual=[llength $reset_net]"
    }
    return $reset_net
}

proc l0_tdc_output_budget_metrics {} {
    set tdc_outputs [get_ports -quiet [list \
        {io_tdc_d[*]} {o_tdc_adr[*]} {o_tdc_csn[*]} {o_tdc_rdn[*]} \
        {o_tdc_wrn[*]} {o_tdc_stopdis[*]} {o_tdc_alutrigger[*]} \
        {o_tdc_puresn[*]}]]
    set paths [get_timing_paths -quiet -from [get_clocks -quiet clk_fpga_2] \
        -to $tdc_outputs -delay_type max -max_paths 152 -nworst 1]
    if {[llength $paths] != 152} {
        error "TDC output timing path-count mismatch: expected=152 actual=[llength $paths]"
    }

    set tight_count 0
    foreach path $paths {
        if {[get_property SLACK $path] < 0.100} {
            incr tight_count
        }
    }

    set worst_path [lindex $paths 0]
    set data_path_delay [get_property DATAPATH_DELAY $worst_path]
    # ENDPOINT_PIN은 Vivado 객체 handle이다. 합성 뒤 구현으로 design이 바뀌면
    # 이전 handle이 null로 해석될 수 있으므로 즉시 문자열 이름으로 고정한다.
    set worst_endpoint [get_property NAME \
        [get_property ENDPOINT_PIN $worst_path]]
    # Runtime TDC-GPX 버스 읽기 타이밍 (BUS_CLK_DIV/BUS_TICKS)은 최소
    # 25 ns의 핀 유지구간을 보장한다. 아래 값은 가장 느린 register-to-pad
    # 경로가 지난 뒤에도 외부 핀에 남는 보수적 안정시간이다.
    set protocol_stable_margin [format %.3f \
        [expr {25.000 - $data_path_delay}]]
    return [list \
        [get_property SLACK $worst_path] \
        $tight_count \
        $data_path_delay \
        $protocol_stable_margin \
        $worst_endpoint]
}

proc l0_report_tdc_200mhz {prefix result_dir} {
    report_timing -group clk_fpga_2 -delay_type max \
        -max_paths 100 -nworst 100 -input_pins \
        -file [file join $result_dir ${prefix}_tdc_200mhz_setup_top100.rpt]
    report_timing -group clk_fpga_2 -delay_type min \
        -max_paths 100 -nworst 100 -input_pins \
        -file [file join $result_dir ${prefix}_tdc_200mhz_hold_top100.rpt]
    report_timing -from [get_clocks -quiet clk_fpga_2] \
        -to [get_ports -quiet [list \
            {io_tdc_d[*]} {o_tdc_adr[*]} {o_tdc_csn[*]} {o_tdc_rdn[*]} \
            {o_tdc_wrn[*]} {o_tdc_stopdis[*]} {o_tdc_alutrigger[*]} \
            {o_tdc_puresn[*]}]] \
        -delay_type max -max_paths 152 -nworst 1 -input_pins \
        -file [file join $result_dir ${prefix}_tdc_output_budget_top152.rpt]
}

proc l0_require_count {label objects expected} {
    set actual [llength $objects]
    if {$actual != $expected} {
        error "$label endpoint-count mismatch: expected=$expected actual=$actual"
    }
    return $actual
}

proc l0_require_nonempty {label objects} {
    set actual [llength $objects]
    if {$actual == 0} {
        error "$label endpoint set is empty"
    }
    return $actual
}

proc l0_verify_timing_contract {result_dir} {
    set meta_d [get_pins -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/.*_meta_r_reg.*\/D$}]
    set encoder_meta_d [get_pins -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/.*phy_[abz]_sync_r_reg\[0\]\/D$}]
    set cfg_src [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_csr_config/u_config/u_manager/candidate_r_reg.*$}]
    set cfg_proc_dst [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_csr_config/u_config/u_proc_gateway/prepared_cfg_r_reg.*$}]
    set cfg_tdc_dst [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_csr_config/u_config/u_tdc_gateway/prepared_cfg_r_reg.*$}]
    set image_src [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_csr_config/u_gpx_image_transaction/candidate_image_r_reg.*$}]
    set image_dst [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_csr_config/gen_tdc_external_apply\.u_gpx_activation/image_r_reg.*$}]
    set diag_req_src [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_status_snapshot/u_(proc|tdc)_mailbox/request_payload_r_reg.*$}]
    set diag_req_dst [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_status_snapshot/u_(proc|tdc)_mailbox/domain_request_r_reg.*$}]
    set diag_rsp_src [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_status_snapshot/u_(proc|tdc)_mailbox/domain_response_r_reg.*$}]
    set diag_rsp_dst [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/u_status_snapshot/u_(proc|tdc)_mailbox/source_response_r_reg.*$}]
    set gpx_data_ports [get_ports -quiet {io_tdc_d[*]}]
    set gpx_capture_d [get_pins -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/.*u_proven_bus_phy/s_d_in_ff_r_reg\[[0-9]+\]\/D$}]
    set tdc_outputs [get_ports -quiet [list \
        {io_tdc_d[*]} {o_tdc_adr[*]} {o_tdc_csn[*]} {o_tdc_rdn[*]} \
        {o_tdc_wrn[*]} {o_tdc_stopdis[*]} {o_tdc_alutrigger[*]} \
        {o_tdc_puresn[*]}]]
    set proc_outputs [get_ports -quiet \
        {o_fire_pulse o_start_tdc o_stop_tdc}]
    set async_service_inputs [get_ports -quiet [list \
        i_enc_a i_enc_b i_enc_z i_fire_done \
        {i_tdc_ef1[*]} {i_tdc_ef2[*]} {i_tdc_irflag[*]}]]

    set counts [list \
        [list meta_first_stage \
            [l0_require_nonempty meta_first_stage $meta_d]] \
        [list encoder_first_stage \
            [l0_require_count encoder_first_stage $encoder_meta_d 3]] \
        [list config_source \
            [l0_require_nonempty config_source $cfg_src]] \
        [list config_proc_destination \
            [l0_require_nonempty config_proc_destination $cfg_proc_dst]] \
        [list config_tdc_destination \
            [l0_require_nonempty config_tdc_destination $cfg_tdc_dst]] \
        [list gpx_image_source \
            [l0_require_nonempty gpx_image_source $image_src]] \
        [list gpx_image_destination \
            [l0_require_nonempty gpx_image_destination $image_dst]] \
        [list diag_request_source \
            [l0_require_count diag_request_source $diag_req_src 16]] \
        [list diag_request_destination \
            [l0_require_count diag_request_destination $diag_req_dst 16]] \
        [list diag_response_source \
            [l0_require_count diag_response_source $diag_rsp_src 66]] \
        [list diag_response_destination \
            [l0_require_count diag_response_destination $diag_rsp_dst 66]] \
        [list gpx_data_ports \
            [l0_require_count gpx_data_ports $gpx_data_ports 112]] \
        [list gpx_capture_registers \
            [l0_require_count gpx_capture_registers $gpx_capture_d 112]] \
        [list tdc_output_ports \
            [l0_require_count tdc_output_ports $tdc_outputs 152]] \
        [list processing_output_ports \
            [l0_require_count processing_output_ports $proc_outputs 3]] \
        [list asynchronous_service_inputs \
            [l0_require_count asynchronous_service_inputs \
                $async_service_inputs 16]]]

    set channel [open [file join $result_dir timing_endpoint_contract.rpt] w]
    puts $channel {endpoint_group,count}
    foreach item $counts {
        puts $channel "[lindex $item 0],[lindex $item 1]"
    }
    close $channel
    return [list $gpx_data_ports $gpx_capture_d]
}

proc l0_apply_reviewed_waivers {gpx_data_ports gpx_capture_d} {
    # The GPX D[27:0] bus is captured only after the RDN-to-data-valid protocol
    # window. It is intentionally not a conventional free-running CDC and must
    # not be mislabeled as a 2-FF synchronizer. Keep a narrow endpoint waiver;
    # endpoint counts are checked separately before this waiver is created.
    create_waiver -quiet -type CDC -id CDC-1 -user {Victek LiDAR V3} \
        -description {Reviewed asynchronous TDC-GPX data bus captured by an IOB FF after the >=25 ns Runtime BUS_CLK_DIV/BUS_TICKS read window.} \
        -from $gpx_data_ports -to $gpx_capture_d

    # EF1/EF2/IRFLAG are asynchronous status levels synchronized in the TDC
    # clock domain. A synchronous input-delay number would be physically false;
    # retain the first-stage false path and waive only the matching TIMING-18
    # methodology entries.
    set gpx_async_status_ports [get_ports -quiet [list \
        {i_tdc_ef1[*]} {i_tdc_ef2[*]} {i_tdc_irflag[*]}]]
    l0_require_count gpx_async_status_ports $gpx_async_status_ports 12
    create_waiver -quiet -type METHODOLOGY -id TIMING-18 \
        -user {Victek LiDAR V3} \
        -description {Reviewed asynchronous TDC-GPX status levels terminate in explicit first-stage synchronizer registers.} \
        -objects $gpx_async_status_ports

    return [list [llength $gpx_capture_d] \
        [llength $gpx_async_status_ports]]
}

proc l0_read_report {path} {
    set channel [open $path r]
    set text [read $channel]
    close $channel
    return $text
}

proc l0_verify_report_gates {prefix result_dir} {
    set cdc_path [file join $result_dir ${prefix}_cdc.rpt]
    set skew_path [file join $result_dir ${prefix}_bus_skew.rpt]
    set methodology_path [file join $result_dir ${prefix}_methodology.rpt]
    set drc_path [file join $result_dir ${prefix}_drc.rpt]
    set cdc_text [l0_read_report $cdc_path]
    set skew_text [l0_read_report $skew_path]
    set methodology_text [l0_read_report $methodology_path]
    set drc_text [l0_read_report $drc_path]

    set active_cdc_critical_count [regexp -all -line \
        {^CDC-[0-9]+[[:space:]]+Critical[[:space:]]+[1-9][0-9]*} \
        $cdc_text]
    set bus_skew_violation_count [regexp -all {VIOLATED} $skew_text]
    set methodology_critical_count [regexp -all -line \
        {^\|[[:space:]]*[^|]+\|[[:space:]]*Critical Warning[[:space:]]*\|} \
        $methodology_text]
    set drc_blocking_count [regexp -all -line \
        {^\|[[:space:]]*[^|]+\|[[:space:]]*(Critical Warning|Error)[[:space:]]*\|} \
        $drc_text]

    if {$active_cdc_critical_count != 0} {
        error "$prefix CDC report has active Critical findings"
    }
    if {$bus_skew_violation_count != 0} {
        error "$prefix bus-skew report has violations"
    }
    if {$methodology_critical_count != 0} {
        error "$prefix methodology report has Critical Warning findings"
    }
    if {$drc_blocking_count != 0} {
        error "$prefix DRC report has Critical Warning or Error findings"
    }
    return [list $active_cdc_critical_count $bus_skew_violation_count \
        $methodology_critical_count $drc_blocking_count]
}

proc l0_report_design {prefix result_dir} {
    report_utilization -hierarchical -hierarchical_depth 7 \
        -file [file join $result_dir ${prefix}_utilization_hier.rpt]
    report_timing_summary -delay_type min_max -report_unconstrained \
        -check_timing_verbose -max_paths 100 \
        -file [file join $result_dir ${prefix}_timing_summary.rpt]
    report_cdc -details -file [file join $result_dir ${prefix}_cdc.rpt]
    report_clock_interaction \
        -file [file join $result_dir ${prefix}_clock_interaction.rpt]
    report_bus_skew -file [file join $result_dir ${prefix}_bus_skew.rpt]
    report_methodology -file [file join $result_dir ${prefix}_methodology.rpt]
    report_drc -file [file join $result_dir ${prefix}_drc.rpt]
    report_io -file [file join $result_dir ${prefix}_io.rpt]
    l0_report_tdc_200mhz $prefix $result_dir
}

proc l0_verify_service_pins {result_dir} {
    set patterns [list \
        {io_tdc_d[*]} {o_tdc_adr[*]} {o_tdc_csn[*]} {o_tdc_rdn[*]} \
        {o_tdc_wrn[*]} {o_tdc_stopdis[*]} {o_tdc_alutrigger[*]} \
        {o_tdc_puresn[*]} {i_tdc_ef1[*]} {i_tdc_ef2[*]} \
        {i_tdc_irflag[*]} i_enc_a i_enc_b i_enc_z i_fire_done \
        o_fire_pulse o_start_tdc o_stop_tdc]
    set service_ports {}
    foreach pattern $patterns {
        set service_ports [concat $service_ports [get_ports -quiet $pattern]]
    }
    set service_ports [lsort -unique $service_ports]
    if {[llength $service_ports] != 171} {
        error "Service-pin count mismatch: expected=171 actual=[llength $service_ports]"
    }

    set missing_loc {}
    set missing_iostandard {}
    set channel [open [file join $result_dir board_service_pin_contract.rpt] w]
    puts $channel {port,package_pin,iostandard}
    foreach port $service_ports {
        set package_pin [get_property PACKAGE_PIN $port]
        set iostandard [get_property IOSTANDARD $port]
        if {$package_pin eq {}} {
            lappend missing_loc $port
        }
        if {$iostandard eq {} || $iostandard eq {DEFAULT}} {
            lappend missing_iostandard $port
        }
        puts $channel "$port,$package_pin,$iostandard"
    }
    close $channel
    if {[llength $missing_loc] != 0} {
        error "Board service pins without PACKAGE_PIN: $missing_loc"
    }
    if {[llength $missing_iostandard] != 0} {
        error "Board service pins without IOSTANDARD: $missing_iostandard"
    }
    return [llength $service_ports]
}

open_project $project_path
set_param general.maxThreads 4
update_compile_order -fileset sources_1
# HLS-generated RTL has long file names.  Compiling every IP inside this one
# batch process avoids Windows child-run path/queue failures and also proves
# that the packaged IP is a complete source closure rather than a cached DCP.
set bd_file [get_files design_1_lidar_ctrl_v3.bd]
if {[llength $bd_file] != 1} {
    error {Expected exactly one V3 Parent block design}
}
set_property synth_checkpoint_mode None $bd_file
generate_target all $bd_file
synth_design -top design_1_lidar_ctrl_v3_wrapper \
    -part xc7z020clg484-2 -flatten_hierarchy rebuilt
set synth_status {COMPLETE_DIRECT_BATCH}
write_checkpoint -force [file join $result_dir post_synth.dcp]
set hierarchy_channel [open [file join $result_dir post_synth_v3_hierarchy.rpt] w]
foreach cell [lsort [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/.*$}]] {
    puts $hierarchy_channel $cell
}
close $hierarchy_channel
set black_boxes [get_cells -quiet -hier -filter {IS_BLACKBOX == 1}]
set latches [get_cells -quiet -hier \
    -filter {REF_NAME == LDCE || REF_NAME == LDPE}]
if {[llength $black_boxes] != 0} {
    error "V3 Parent synthesized with black boxes: $black_boxes"
}
if {[llength $latches] != 0} {
    error "V3 Parent inferred latches: $latches"
}
set service_pin_count [l0_verify_service_pins $result_dir]
set synth_gpx_iob_register_count [l0_verify_gpx_iob_contract \
    $result_dir post_synth false]
set timing_endpoints [l0_verify_timing_contract $result_dir]
set reviewed_waiver_counts [l0_apply_reviewed_waivers \
    [lindex $timing_endpoints 0] [lindex $timing_endpoints 1]]
l0_report_design post_synth $result_dir
set synth_wns [l0_worst_slack max]
set synth_whs [l0_worst_slack min]
set synth_tdc_metrics [l0_tdc_200mhz_metrics]
set synth_tdc_wns [lindex $synth_tdc_metrics 0]
set synth_tdc_whs [lindex $synth_tdc_metrics 1]
set synth_tdc_setup_lt_0p100_count [lindex $synth_tdc_metrics 2]
set synth_tdc_hold_lt_0p020_count [lindex $synth_tdc_metrics 3]
set synth_tdc_output_metrics [l0_tdc_output_budget_metrics]
set synth_tdc_output_budget_slack [lindex $synth_tdc_output_metrics 0]
set synth_tdc_output_lt_0p100_count [lindex $synth_tdc_output_metrics 1]
set synth_tdc_output_data_path_delay [lindex $synth_tdc_output_metrics 2]
set synth_tdc_output_protocol_margin [lindex $synth_tdc_output_metrics 3]
set synth_tdc_output_worst_port [lindex $synth_tdc_output_metrics 4]
if {$synth_wns < 0.0} {
    error "Synthesized setup timing failed: WNS=$synth_wns"
}
# 배치 전 hold는 실제 셀 위치와 배선 지연이 없는 추정값이다. 음수이면
# 결과에 명시하되 구현을 중단하지 않고, 최종 route WHS에서 엄격히 판정한다.
# setup은 구조적 장경로를 조기에 검출할 수 있으므로 기존 gate를 유지한다.
set synth_hold_advisory {NONNEGATIVE}
if {$synth_whs < 0.0} {
    set synth_hold_advisory {NEGATIVE_PRE_ROUTE_ESTIMATE}
    puts "WARNING: post-synth hold is advisory until placement: WHS=$synth_whs"
}
set synth_report_gate_counts [l0_verify_report_gates post_synth $result_dir]

set impl_status {NOT_RUN}
set route_wns {NA}
set route_whs {NA}
set route_tdc_wns {NA}
set route_tdc_whs {NA}
set route_tdc_setup_lt_0p100_count {NA}
set route_tdc_hold_lt_0p020_count {NA}
set route_tdc_setup_margin_advisory {NOT_RUN}
set route_initial_tdc_reset_wns {NA}
set route_initial_tdc_regular_wns {NA}
set route_reset_replication_applied {false}
set route_tdc_reset_wns {NA}
set route_tdc_regular_wns {NA}
set route_tdc_reset_lt_0p100_count {NA}
set route_tdc_regular_lt_0p100_count {NA}
set route_tdc_reset_max_fanout {NA}
set route_tdc_output_budget_slack {NA}
set route_tdc_output_lt_0p100_count {NA}
set route_tdc_output_data_path_delay {NA}
set route_tdc_output_protocol_margin {NA}
set route_tdc_output_worst_port {NA}
set route_tdc_output_budget_advisory {NOT_RUN}
set route_drc_error_count {NA}
set bitstream_path {NA}
set bitstream_size_bytes {NA}
set xsa_path {NA}
set xsa_size_bytes {NA}
set route_gpx_iob_register_count {NA}
if {$run_mode eq {IMPL}} {
    opt_design -directive ExploreWithRemap
    place_design -directive Explore
    phys_opt_design -directive AggressiveExplore
    # 1차 route 뒤 Reset 경로만 민감한 경우에 되돌아올 물리 기준점이다.
    # RTL 지연이나 Reset clock 수를 바꾸지 않고 배치된 Reset FF만 복제한다.
    set pre_route_checkpoint [file join $result_dir pre_route.dcp]
    write_checkpoint -force $pre_route_checkpoint
    route_design -directive AggressiveExplore -tns_cleanup

    set initial_class_metrics [l0_tdc_setup_class_metrics]
    set route_initial_tdc_reset_wns [lindex $initial_class_metrics 0]
    set route_initial_tdc_regular_wns [lindex $initial_class_metrics 1]
    if {$route_initial_tdc_reset_wns < 0.100} {
        puts "WARNING: TDC Reset distribution is sensitive after initial route: WNS=$route_initial_tdc_reset_wns ns"
        write_checkpoint -force [file join $result_dir post_initial_route.dcp]
        close_design
        open_checkpoint $pre_route_checkpoint
        set reset_net [l0_tdc_reset_replication_net]
        puts "TDC_RESET_REPLICATION_APPLY net=$reset_net fanout=[get_property FLAT_PIN_COUNT $reset_net]"
        phys_opt_design -force_replication_on_nets $reset_net
        route_design -directive AggressiveExplore -tns_cleanup
        set route_reset_replication_applied {true}
    }

    set impl_status {COMPLETE_DIRECT_BATCH}
    set timing_endpoints [l0_verify_timing_contract $result_dir]
    set reviewed_waiver_counts [l0_apply_reviewed_waivers \
        [lindex $timing_endpoints 0] [lindex $timing_endpoints 1]]
    set route_gpx_iob_register_count [l0_verify_gpx_iob_contract \
        $result_dir post_route true]
    l0_report_design post_route $result_dir
    report_route_status -file [file join $result_dir post_route_status.rpt]
    set route_wns [l0_worst_slack max]
    set route_whs [l0_worst_slack min]
    set route_tdc_metrics [l0_tdc_200mhz_metrics]
    set route_tdc_wns [lindex $route_tdc_metrics 0]
    set route_tdc_whs [lindex $route_tdc_metrics 1]
    set route_tdc_setup_lt_0p100_count [lindex $route_tdc_metrics 2]
    set route_tdc_hold_lt_0p020_count [lindex $route_tdc_metrics 3]
    set route_tdc_class_metrics [l0_tdc_setup_class_metrics]
    set route_tdc_reset_wns [lindex $route_tdc_class_metrics 0]
    set route_tdc_regular_wns [lindex $route_tdc_class_metrics 1]
    set route_tdc_reset_lt_0p100_count [lindex $route_tdc_class_metrics 2]
    set route_tdc_regular_lt_0p100_count [lindex $route_tdc_class_metrics 3]
    set route_tdc_reset_max_fanout [lindex $route_tdc_class_metrics 4]
    # 내부 5 ns 주기 경로는 단순 timing PASS(0 ns)보다 엄격하게 0.100 ns를
    # 최소 Sign-off 여유로 사용한다. 외부 8 ns register-to-pad 예산은 아래의
    # 최소 25 ns 프로토콜 핀 안정시간과 별도 판정한다.
    if {$route_tdc_reset_wns < 0.100 || $route_tdc_regular_wns < 0.100} {
        error "TDC 200 MHz internal margin gate failed: reset=$route_tdc_reset_wns regular=$route_tdc_regular_wns ns"
    }
    set route_tdc_output_metrics [l0_tdc_output_budget_metrics]
    set route_tdc_output_budget_slack [lindex $route_tdc_output_metrics 0]
    set route_tdc_output_lt_0p100_count [lindex $route_tdc_output_metrics 1]
    set route_tdc_output_data_path_delay [lindex $route_tdc_output_metrics 2]
    set route_tdc_output_protocol_margin [lindex $route_tdc_output_metrics 3]
    set route_tdc_output_worst_port [lindex $route_tdc_output_metrics 4]
    set route_tdc_setup_margin_advisory {NONNEGATIVE_GE_0P100_NS}
    if {$route_tdc_wns < 0.100} {
        set route_tdc_setup_margin_advisory {TIGHT_LT_0P100_NS}
        puts "WARNING: TDC 200 MHz setup margin is sensitive: WNS=$route_tdc_wns ns"
    }
    set route_tdc_output_budget_advisory {NONNEGATIVE_GE_0P100_NS}
    if {$route_tdc_output_budget_slack < 0.100} {
        set route_tdc_output_budget_advisory {TIGHT_8NS_BUDGET_BUT_PROTOCOL_MARGIN_REMAINS}
        puts "WARNING: TDC output 8 ns budget is sensitive: slack=$route_tdc_output_budget_slack ns, protocol_stable_margin=$route_tdc_output_protocol_margin ns, port=$route_tdc_output_worst_port"
    }
    set route_drc_errors [get_drc_violations -quiet -filter {SEVERITY == Error}]
    set route_drc_error_count [llength $route_drc_errors]
    if {$route_drc_error_count != 0} {
        error "Routed DRC has errors: $route_drc_errors"
    }
    if {$route_wns < 0.0 || $route_whs < 0.0} {
        error "Routed timing failed: WNS=$route_wns WHS=$route_whs"
    }
    set route_report_gate_counts [l0_verify_report_gates \
        post_route $result_dir]
    write_checkpoint -force [file join $result_dir post_route.dcp]
    set bitstream_path [file normalize [file join $result_dir \
        design_1_lidar_ctrl_v3_wrapper.bit]]
    write_bitstream -force $bitstream_path
    if {![file exists $bitstream_path]} {
        error "Implementation completed without bitstream: $bitstream_path"
    }
    set bitstream_size_bytes [file size $bitstream_path]
    if {$bitstream_size_bytes <= 0} {
        error "Generated bitstream is empty: $bitstream_path"
    }
    set xsa_path [file normalize [file join $result_dir \
        tdc_gpx_lidar_ctrl_v3_4chip.xsa]]
    # This sign-off flow implements the current design directly instead of
    # launching the project impl_1 run.  Vivado's -include_bit option searches
    # the run database and therefore cannot discover the valid bitstream just
    # written above.  Export the fixed hardware handoff as XSA and keep the
    # independently verified .bit beside it as the programming artifact.
    write_hw_platform -fixed -force -file $xsa_path
    if {![file exists $xsa_path]} {
        error "Implementation completed without XSA: $xsa_path"
    }
    set xsa_size_bytes [file size $xsa_path]
    if {$xsa_size_bytes <= 0} {
        error "Generated XSA is empty: $xsa_path"
    }
}

set summary_path [file join $result_dir l0_parent_signoff_summary.txt]
set summary [open $summary_path w]
puts $summary "mode=$run_mode"
puts $summary "project=$project_root"
puts $summary "synth_status=$synth_status"
puts $summary "synth_wns_ns=$synth_wns"
puts $summary "synth_whs_ns=$synth_whs"
puts $summary "synth_hold_advisory=$synth_hold_advisory"
puts $summary "synth_tdc_200mhz_wns_ns=$synth_tdc_wns"
puts $summary "synth_tdc_200mhz_whs_ns=$synth_tdc_whs"
puts $summary "synth_tdc_setup_lt_0p100_count=$synth_tdc_setup_lt_0p100_count"
puts $summary "synth_tdc_hold_lt_0p020_count=$synth_tdc_hold_lt_0p020_count"
puts $summary "synth_tdc_output_budget_slack_ns=$synth_tdc_output_budget_slack"
puts $summary "synth_tdc_output_lt_0p100_count=$synth_tdc_output_lt_0p100_count"
puts $summary "synth_tdc_output_data_path_delay_ns=$synth_tdc_output_data_path_delay"
puts $summary "synth_tdc_output_protocol_stable_margin_ns=$synth_tdc_output_protocol_margin"
puts $summary "synth_tdc_output_worst_port=$synth_tdc_output_worst_port"
puts $summary "service_pin_count=$service_pin_count"
puts $summary "synth_gpx_iob_register_count=$synth_gpx_iob_register_count"
puts $summary "synth_black_box_count=[llength $black_boxes]"
puts $summary "synth_latch_count=[llength $latches]"
puts $summary "route_gpx_iob_register_count=$route_gpx_iob_register_count"
puts $summary "reviewed_cdc_waiver_count=[lindex $reviewed_waiver_counts 0]"
puts $summary "reviewed_async_status_waiver_count=[lindex $reviewed_waiver_counts 1]"
puts $summary "synth_active_cdc_critical_count=[lindex $synth_report_gate_counts 0]"
puts $summary "synth_bus_skew_violation_count=[lindex $synth_report_gate_counts 1]"
puts $summary "synth_methodology_critical_count=[lindex $synth_report_gate_counts 2]"
puts $summary "synth_blocking_drc_count=[lindex $synth_report_gate_counts 3]"
puts $summary "impl_status=$impl_status"
puts $summary "route_wns_ns=$route_wns"
puts $summary "route_whs_ns=$route_whs"
puts $summary "route_tdc_200mhz_wns_ns=$route_tdc_wns"
puts $summary "route_tdc_200mhz_whs_ns=$route_tdc_whs"
puts $summary "route_tdc_setup_lt_0p100_count=$route_tdc_setup_lt_0p100_count"
puts $summary "route_tdc_hold_lt_0p020_count=$route_tdc_hold_lt_0p020_count"
puts $summary "route_tdc_setup_margin_advisory=$route_tdc_setup_margin_advisory"
puts $summary "route_initial_tdc_reset_wns_ns=$route_initial_tdc_reset_wns"
puts $summary "route_initial_tdc_regular_wns_ns=$route_initial_tdc_regular_wns"
puts $summary "route_reset_replication_applied=$route_reset_replication_applied"
puts $summary "route_tdc_reset_wns_ns=$route_tdc_reset_wns"
puts $summary "route_tdc_regular_wns_ns=$route_tdc_regular_wns"
puts $summary "route_tdc_reset_lt_0p100_count=$route_tdc_reset_lt_0p100_count"
puts $summary "route_tdc_regular_lt_0p100_count=$route_tdc_regular_lt_0p100_count"
puts $summary "route_tdc_reset_max_fanout=$route_tdc_reset_max_fanout"
puts $summary "route_tdc_output_budget_slack_ns=$route_tdc_output_budget_slack"
puts $summary "route_tdc_output_lt_0p100_count=$route_tdc_output_lt_0p100_count"
puts $summary "route_tdc_output_data_path_delay_ns=$route_tdc_output_data_path_delay"
puts $summary "route_tdc_output_protocol_stable_margin_ns=$route_tdc_output_protocol_margin"
puts $summary "route_tdc_output_worst_port=$route_tdc_output_worst_port"
puts $summary "route_tdc_output_budget_advisory=$route_tdc_output_budget_advisory"
puts $summary "route_drc_error_count=$route_drc_error_count"
puts $summary "bitstream_path=$bitstream_path"
puts $summary "bitstream_size_bytes=$bitstream_size_bytes"
puts $summary "xsa_path=$xsa_path"
puts $summary "xsa_size_bytes=$xsa_size_bytes"
puts $summary "xsa_contains_bit=false"
close $summary

puts "LIDAR_V3_L0_PARENT_${run_mode}_SIGNOFF_PASS results=$result_dir"
close_project
exit 0
