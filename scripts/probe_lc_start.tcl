# Use add_condition to fire a snap exactly when lc_start_tdc rises to '1'.
set p "/tb_tdc_gpx_full_int/u_td"
set pc "/tb_tdc_gpx_full_int"

proc snap_now {} {
    set p "/tb_tdc_gpx_full_int/u_td"
    set pc "/tb_tdc_gpx_full_int"
    set t    [current_time]
    set fst  [get_value -radix hex ${p}/u_face_seq/s_face_state_r]
    set hi   [get_value -radix hex ${p}/s_hdr_idle]
    set hfi  [get_value -radix hex ${p}/s_hdr_fall_idle]
    set abq  [get_value -radix hex ${p}/u_face_seq/s_abort_quiesce_r]
    set pabr [get_value -radix hex ${p}/s_pipeline_abort]
    set pkgs [get_value -radix hex ${p}/u_face_seq/s_packet_start_r]
    set cb   [get_value -radix hex ${p}/s_chip_busy]
    set ishot [get_value -radix hex ${p}/i_shot_start]
    set spr  [get_value -radix hex ${p}/u_face_seq/s_shot_raw_pulse]
    set srd  [get_value -radix hex ${p}/u_face_seq/s_shot_raw_d_r]
    puts "EDGE\@$t  face_st=$fst i_shot=$ishot shot_raw_pulse=$spr shot_raw_d=$srd packet=$pkgs hdr_i=$hi hdr_fi=$hfi abort=$pabr abq=$abq chip_busy=$cb"
}

add_condition -name c_lc_start_rise {/tb_tdc_gpx_full_int/lc_start_tdc == 1} {snap_now}

run all
quit
