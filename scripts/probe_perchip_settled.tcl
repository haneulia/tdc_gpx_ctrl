# probe_perchip_settled.tcl
# R4a re-investigation probe: settled-value (clock-edge) sampling of the
# per-chip shot start / busy / drain chain inside tdc_gpx_top.
#
# Why settled sampling:
#   Our earlier probes (__probe_perchip.tcl / __probe_perchip_v3.tcl) used
#   `add_condition "sig(N) == 1"` per-bit counters and reported "chip 3 only",
#   which matched the TB's dbg_* counters -- both were wrong:
#     1. TB dbg_* signals are STUB signals (see tb_tdc_gpx_full_int.vhd L454)
#        because xsim 2025.2.1 rejects VHDL-2008 external names.
#     2. xsim add_condition on single-bit slices of a slv under-counts when
#        all 4 bits transition together in the same delta.
#
#   `run N ns ; get_value` reads the SETTLED value at an explicit simulation
#   time, which is immune to both issues.
#
# Usage:
#   Built against the existing tb_tdc_gpx_full_int_snap.
#     xsim tb_tdc_gpx_full_int_snap -tclbatch HDL/scripts/probe_perchip_settled.tcl

set p "/tb_tdc_gpx_full_int/u_td"

# Skip through TB init / startup; the first shot fires around ~89us
run 88 us

# Sample every 5 ns (= 1 axis clock @ 200 MHz) for a 6 us window.
# Only print rows where at least one observed signal is non-idle.
for {set k 0} {$k < 1200} {incr k} {
    run 5 ns
    set t    [current_time]
    set ssg  [get_value          ${p}/s_shot_start_gated]
    set ssp  [get_value -radix bin ${p}/s_shot_start_per_chip]
    set busy [get_value -radix bin ${p}/s_chip_busy]
    set raw  [get_value -radix bin ${p}/s_raw_sk_tvalid]
    if {$ssg == 1 || $ssp != "0000" || $busy != "0000" || $raw != "0000"} {
        puts "$t  ssg=$ssg  ssp=$ssp  busy=$busy  raw=$raw"
    }
}
quit
