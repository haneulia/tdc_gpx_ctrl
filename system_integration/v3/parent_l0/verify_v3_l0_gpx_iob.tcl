# Verify the physical TDC-GPX bus register contract.
#
# Fixed PACKAGE_PIN constraints determine the IOB site.  The IOB attributes in
# tdc_gpx_bus_phy then force the associated input, output, and tri-state
# registers into that pin's ILOGIC/OLOGIC.  This check makes a synthesis or
# implementation run fail if optimization moves, merges, or removes any of
# those boundary registers.  The VT parent uses
# G_OEN_MODE=PULLUP_OR_NOT_CONNECTED and intentionally does not externalize
# OEN, so OEN is not part of this board-specific count.

proc l0_iob_property_is_true {value} {
    set normalized [string toupper [string trim $value]]
    return [expr {$normalized in {1 TRUE YES}}]
}

proc l0_verify_gpx_iob_contract {result_dir stage require_placed} {
    set root {^.*/tdc_gpx_lidar_ctrl_v3_0/inst/.*u_proven_bus_phy/}
    set groups [list \
        [list gpx_d_input  ${root}s_d_in_ff_r_reg\[[0-9]+\]$ 112 ILOGIC IFF] \
        [list gpx_d_output ${root}s_d_out_r_reg\[[0-9]+\]$   112 OLOGIC OUTFF] \
        [list gpx_d_tri    ${root}s_d_tri_r_reg\[[0-9]+\]$   112 OLOGIC TFF] \
        [list gpx_address  ${root}s_adr_r_reg\[[0-9]+\]$      16 OLOGIC OUTFF] \
        [list gpx_csn      ${root}s_csn_r_reg$                  4 OLOGIC OUTFF] \
        [list gpx_rdn      ${root}s_rdn_r_reg$                  4 OLOGIC OUTFF] \
        [list gpx_wrn      ${root}s_wrn_r_reg$                  4 OLOGIC OUTFF]]

    set report_path [file join $result_dir ${stage}_gpx_iob_contract.rpt]
    set channel [open $report_path w]
    puts $channel {group,cell,iob,loc,bel}
    set total 0

    foreach group $groups {
        lassign $group label pattern expected loc_prefix bel_suffix
        set cells [lsort [get_cells -quiet -hier -regexp $pattern]]
        set actual [llength $cells]
        if {$actual != $expected} {
            close $channel
            error "$stage $label register-count mismatch: expected=$expected actual=$actual"
        }

        foreach cell $cells {
            set iob [get_property IOB $cell]
            set loc [get_property LOC $cell]
            set bel [get_property BEL $cell]
            puts $channel "$label,$cell,$iob,$loc,$bel"

            if {![l0_iob_property_is_true $iob]} {
                close $channel
                error "$stage $label is missing IOB=TRUE: cell=$cell value=$iob"
            }
            if {$require_placed} {
                if {![string match ${loc_prefix}* $loc]} {
                    close $channel
                    error "$stage $label is not in $loc_prefix: cell=$cell LOC=$loc"
                }
                if {![string match *.$bel_suffix $bel]} {
                    close $channel
                    error "$stage $label has wrong IOB register role: cell=$cell BEL=$bel expected=*$bel_suffix"
                }
            }
        }
        incr total $actual
    }

    close $channel
    return $total
}
