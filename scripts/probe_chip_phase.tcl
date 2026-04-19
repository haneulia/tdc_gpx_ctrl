# chip_ctrl + FIFO per-chip state using escaped generate labels
proc snap_chip {tag} {
    set pc "/tb_tdc_gpx_full_int"
    puts "--- $tag ---"
    foreach i {0 1 2 3} {
        set cp "/tb_tdc_gpx_full_int/u_td/u_config_ctrl/\\gen_chip($i)\\/u_chip_ctrl"
        set phase [get_value ${cp}/s_phase_r]
        set f1    [get_value -radix unsigned ${pc}/fifo1_fill\[$i\]]
        set f2    [get_value -radix unsigned ${pc}/fifo2_fill\[$i\]]
        set irf   [get_value ${pc}/i_tdc_irflag\[$i\]]
        puts "  chip$i  phase=$phase  fifo1=$f1 fifo2=$f2 irflag=$irf"
    }
    set sspc  [get_value -radix hex /tb_tdc_gpx_full_int/u_td/s_shot_start_per_chip]
    set ishot [get_value -radix hex /tb_tdc_gpx_full_int/u_td/i_shot_start]
    set sraw  [get_value -radix hex /tb_tdc_gpx_full_int/u_td/s_raw_sk_tvalid]
    puts "  top   : i_shot_start=$ishot shot_per_chip=$sspc raw_sk=$sraw"
}

run 30us
snap_chip "t=30us (post enc start)"
run 30us
snap_chip "t=60us"
run 30us
snap_chip "t=90us"
run 30us
snap_chip "t=120us"
quit
