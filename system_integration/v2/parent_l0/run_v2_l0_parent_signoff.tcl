# Synthesize and implement the generated Stage L0 parent project.
# The flow fails on missing board pins, timing failure, or routed DRC errors.

if {[llength $argv] != 3} {
    error {Usage: run_v2_l0_parent_signoff.tcl PROJECT_ROOT RESULT_DIR MODE}
}
lassign $argv project_root result_dir run_mode
set project_root [file normalize $project_root]
set result_dir [file normalize $result_dir]
set run_mode [string toupper $run_mode]
if {$run_mode ni {SYNTH IMPL}} {
    error "Unsupported L0 sign-off mode: $run_mode"
}

set project_path [file join $project_root project_4_lidar_v2_l0.xpr]
if {![file exists $project_path]} {
    error "Generated L0 project is missing: $project_path"
}
file mkdir $result_dir

source [file join [file dirname [info script]] verify_v2_l0_gpx_iob.tcl]

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
        {^.*/tdc_gpx_lidar_ctrl_v2_0/U0/.*_meta_r_reg.*\/D$}]
    set encoder_meta_d [get_pins -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v2_0/U0/.*phy_[abz]_sync_r_reg\[0\]\/D$}]
    set cfg_src [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v2_0/U0/u_csr_config/u_config/u_manager/candidate_r_reg.*$}]
    set cfg_proc_dst [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v2_0/U0/u_csr_config/u_config/u_proc_gateway/prepared_cfg_r_reg.*$}]
    set cfg_tdc_dst [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v2_0/U0/u_csr_config/u_config/u_tdc_gateway/prepared_cfg_r_reg.*$}]
    set image_src [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v2_0/U0/u_csr_config/u_gpx_image_transaction/candidate_image_r_reg.*$}]
    set image_dst [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v2_0/U0/u_csr_config/gen_tdc_external_apply\.u_gpx_activation/image_r_reg.*$}]
    set diag_req_src [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v2_0/U0/u_status_snapshot/u_(proc|tdc)_mailbox/request_payload_r_reg.*$}]
    set diag_req_dst [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v2_0/U0/u_status_snapshot/u_(proc|tdc)_mailbox/domain_request_r_reg.*$}]
    set diag_rsp_src [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v2_0/U0/u_status_snapshot/u_(proc|tdc)_mailbox/domain_response_r_reg.*$}]
    set diag_rsp_dst [get_cells -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v2_0/U0/u_status_snapshot/u_(proc|tdc)_mailbox/source_response_r_reg.*$}]
    set gpx_data_ports [get_ports -quiet {io_tdc_d[*]}]
    set gpx_capture_d [get_pins -quiet -hier -regexp \
        {^.*/tdc_gpx_lidar_ctrl_v2_0/U0/.*u_proven_bus_phy/s_d_in_ff_r_reg\[[0-9]+\]\/D$}]
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
    create_waiver -quiet -type CDC -id CDC-1 -user {Victek LiDAR V2} \
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
        -user {Victek LiDAR V2} \
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
update_compile_order -fileset sources_1

reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
set synth_status [l0_expect_run_complete synth_1]
open_run synth_1
set service_pin_count [l0_verify_service_pins $result_dir]
set synth_gpx_iob_register_count [l0_verify_gpx_iob_contract \
    $result_dir post_synth false]
set timing_endpoints [l0_verify_timing_contract $result_dir]
set reviewed_waiver_counts [l0_apply_reviewed_waivers \
    [lindex $timing_endpoints 0] [lindex $timing_endpoints 1]]
l0_report_design post_synth $result_dir
set synth_wns [l0_worst_slack max]
set synth_whs [l0_worst_slack min]
if {$synth_wns < 0.0 || $synth_whs < 0.0} {
    error "Synthesized timing failed: WNS=$synth_wns WHS=$synth_whs"
}
set synth_report_gate_counts [l0_verify_report_gates post_synth $result_dir]
close_design

set impl_status {NOT_RUN}
set route_wns {NA}
set route_whs {NA}
set route_drc_error_count {NA}
set bitstream_path {NA}
set bitstream_size_bytes {NA}
set route_gpx_iob_register_count {NA}
if {$run_mode eq {IMPL}} {
    reset_run impl_1
    set_property strategy Performance_Explore [get_runs impl_1]
    launch_runs impl_1 -to_step write_bitstream -jobs 8
    wait_on_run impl_1
    set impl_status [l0_expect_run_complete impl_1]
    open_run impl_1
    set timing_endpoints [l0_verify_timing_contract $result_dir]
    set reviewed_waiver_counts [l0_apply_reviewed_waivers \
        [lindex $timing_endpoints 0] [lindex $timing_endpoints 1]]
    set route_gpx_iob_register_count [l0_verify_gpx_iob_contract \
        $result_dir post_route true]
    l0_report_design post_route $result_dir
    report_route_status -file [file join $result_dir post_route_status.rpt]
    set route_wns [l0_worst_slack max]
    set route_whs [l0_worst_slack min]
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
    set bitstream_path [file normalize [file join $project_root \
        project_4_lidar_v2_l0.runs impl_1 \
        design_1_lidar_ctrl_v2_wrapper.bit]]
    if {![file exists $bitstream_path]} {
        error "Implementation completed without bitstream: $bitstream_path"
    }
    set bitstream_size_bytes [file size $bitstream_path]
    if {$bitstream_size_bytes <= 0} {
        error "Generated bitstream is empty: $bitstream_path"
    }
    close_design
}

set summary_path [file join $result_dir l0_parent_signoff_summary.txt]
set summary [open $summary_path w]
puts $summary "mode=$run_mode"
puts $summary "project=$project_root"
puts $summary "synth_status=$synth_status"
puts $summary "synth_wns_ns=$synth_wns"
puts $summary "synth_whs_ns=$synth_whs"
puts $summary "service_pin_count=$service_pin_count"
puts $summary "synth_gpx_iob_register_count=$synth_gpx_iob_register_count"
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
puts $summary "route_drc_error_count=$route_drc_error_count"
puts $summary "bitstream_path=$bitstream_path"
puts $summary "bitstream_size_bytes=$bitstream_size_bytes"
close $summary

puts "LIDAR_V2_L0_PARENT_${run_mode}_SIGNOFF_PASS results=$result_dir"
close_project
exit 0
