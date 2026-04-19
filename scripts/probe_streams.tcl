# Probe intermediate pipeline stream tvalid signals to find where data stops
set p "/tb_tdc_gpx_full_int/u_td"

proc snap_streams {tag} {
    set p "/tb_tdc_gpx_full_int/u_td"
    set rv   [get_value -radix hex ${p}/s_raw_sk_tvalid]
    set ev   [get_value -radix hex ${p}/s_evt_sk_tvalid]
    set crv  [get_value -radix hex ${p}/s_cell_rise_tvalid]
    set crvl [get_value -radix hex ${p}/s_cell_rise_tlast]
    set ft   [get_value -radix hex ${p}/s_face_tvalid]
    set fbt  [get_value -radix hex ${p}/s_face_buf_tvalid]
    set vd   [get_value -radix hex ${p}/o_m_axis_tvalid]
    puts "STR\[$tag\] raw_sk=$rv  evt_sk=$ev  cell_rise(v/l)=$crv/$crvl  face=$ft  face_buf=$fbt  vdma=$vd"
}

# Sticky "ever high" via add_condition (first match time)
set ever_raw  0
set ever_evt  0
set ever_cell 0
set ever_face 0
set ever_fbuf 0

add_condition -name c_raw  [subst -nocommands {${p}/s_raw_sk_tvalid != 0}] {
    global ever_raw;  if {$ever_raw == 0} {set ever_raw [current_time]; puts "EVER raw_sk_tvalid hi at $ever_raw"}
}
add_condition -name c_evt  [subst -nocommands {${p}/s_evt_sk_tvalid != 0}] {
    global ever_evt;  if {$ever_evt == 0} {set ever_evt [current_time]; puts "EVER evt_sk_tvalid hi at $ever_evt"}
}
add_condition -name c_cell [subst -nocommands {${p}/s_cell_rise_tvalid != 0}] {
    global ever_cell; if {$ever_cell == 0} {set ever_cell [current_time]; puts "EVER cell_rise_tvalid hi at $ever_cell"}
}
add_condition -name c_face [subst -nocommands {${p}/s_face_tvalid == 1}] {
    global ever_face; if {$ever_face == 0} {set ever_face [current_time]; puts "EVER face_tvalid hi at $ever_face"}
}
add_condition -name c_fbuf [subst -nocommands {${p}/s_face_buf_tvalid == 1}] {
    global ever_fbuf; if {$ever_fbuf == 0} {set ever_fbuf [current_time]; puts "EVER face_buf_tvalid hi at $ever_fbuf"}
}

# Periodic snaps every 10us
for {set t 10} {$t <= 140} {incr t 10} {
    run 10us
    snap_streams "t=${t}us"
}

puts "SUMMARY ever_raw=$ever_raw ever_evt=$ever_evt ever_cell=$ever_cell ever_face=$ever_face ever_fbuf=$ever_fbuf"
quit
