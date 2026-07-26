# Canonical sibling RTL and testbench source lists used by the TDC-GPX
# integration project. Paths are returned as normalized absolute names.

proc tdc_gpx_integration_rtl_manifest {ip_root} {
    set ve_dir [file join $ip_root virtual_encoder HDL]
    set md_dir [file join $ip_root motor_decoder HDL]
    set lc_dir [file join $ip_root laser_ctrl HDL]
    set er_dir [file join $ip_root echo_receiver HDL]

    return [list \
        [file join $ve_dir enc_pkg.vhd] \
        [file join $ve_dir enc_param_apply_ctrl.vhd] \
        [file join $ve_dir enc_phase_counter.vhd] \
        [file join $ve_dir enc_position_tracker.vhd] \
        [file join $ve_dir enc_index_pulse.vhd] \
        [file join $ve_dir enc_timing_generator.vhd] \
        [file join $ve_dir enc_top.vhd] \
        [file join $md_dir quad_decoder.vhd] \
        [file join $md_dir mirror_active_detect.vhd] \
        [file join $md_dir motor_irq_bridge.vhd] \
        [file join $md_dir motor_decoder_cfg_pkg.vhd] \
        [file join $md_dir motor_cfg_commit_ctrl.vhd] \
        [file join $md_dir motor_axis_stream_out.vhd] \
        [file join $md_dir motor_decoder_csr.vhd] \
        [file join $md_dir motor_decoder_top.vhd] \
        [file join $lc_dir laser_ctrl_cfg_pkg.vhd] \
        [file join $lc_dir laser_ctrl_types_pkg.vhd] \
        [file join $lc_dir laser_ctrl_timebase.vhd] \
        [file join $lc_dir laser_ctrl_cdc_snapshot.vhd] \
        [file join $lc_dir laser_ctrl_event_counters.vhd] \
        [file join $lc_dir laser_ctrl_status.vhd] \
        [file join $lc_dir laser_ctrl_tdc.vhd] \
        [file join $lc_dir laser_ctrl_result.vhd] \
        [file join $lc_dir laser_ctrl_fire_done_bridge.vhd] \
        [file join $lc_dir laser_ctrl_scheduler.vhd] \
        [file join $lc_dir laser_ctrl_csr.vhd] \
        [file join $lc_dir laser_ctrl_metrics.vhd] \
        [file join $lc_dir laser_ctrl_axis_in.vhd] \
        [file join $lc_dir laser_ctrl_executor.vhd] \
        [file join $lc_dir laser_ctrl_top.vhd] \
        [file join $er_dir echo_receiver_pkg.vhd] \
        [file join $er_dir echo_receiver_timebase.vhd] \
        [file join $er_dir echo_receiver_stop_frontend.vhd] \
        [file join $er_dir echo_receiver_core.vhd] \
        [file join $er_dir echo_receiver_cdc_snapshot.vhd] \
        [file join $er_dir echo_receiver_csr.vhd] \
        [file join $er_dir echo_receiver_top.vhd] \
    ]
}

proc tdc_gpx_integration_tb_manifest {ip_root} {
    set md_dir [file join $ip_root motor_decoder HDL]
    set lc_dir [file join $ip_root laser_ctrl HDL]
    set er_dir [file join $ip_root echo_receiver HDL]

    return [list \
        [file join $md_dir tb_motor_decoder_pkg.vhd] \
        [file join $md_dir motor_decoder_top_tb.vhd] \
        [file join $md_dir enc_top_tb.vhd] \
        [file join $lc_dir tb_laser_ctrl_pkg.vhd] \
        [file join $lc_dir tb_laser_ctrl_tests_pkg.vhd] \
        [file join $lc_dir tb_laser_ctrl_monitors.vhd] \
        [file join $lc_dir tb_laser_ctrl_axis_in_unit.vhd] \
        [file join $lc_dir tb_laser_ctrl_scheduler_unit.vhd] \
        [file join $lc_dir tb_laser_ctrl_executor_unit.vhd] \
        [file join $lc_dir tb_laser_ctrl_fire_done_bridge_unit.vhd] \
        [file join $lc_dir tb_laser_ctrl_result_unit.vhd] \
        [file join $lc_dir tb_laser_ctrl_metrics_unit.vhd] \
        [file join $lc_dir tb_laser_ctrl.vhd] \
        [file join $er_dir tb_echo_receiver_pkg.vhd] \
        [file join $er_dir tb_echo_receiver_core_only.vhd] \
        [file join $er_dir tb_echo_receiver.vhd] \
    ]
}
