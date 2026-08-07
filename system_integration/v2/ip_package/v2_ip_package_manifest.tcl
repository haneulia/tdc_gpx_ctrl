# Canonical source-to-package mapping for tdc_gpx_lidar_ctrl_v2.

proc lidar_v2_ip_package_manifest {hdl_root} {
    set order_file [file join $hdl_root system_integration v2 scripts \
        v2_rtl_compile_order.txt]
    if {![file exists $order_file]} {
        error "Missing v2 compile order: $order_file"
    }

    set channel [open $order_file r]
    set lines [split [read $channel] "\n"]
    close $channel

    set entries {}
    set packaged_paths {}
    foreach line $lines {
        set source_entry [string trim $line]
        if {$source_entry eq {} || [string match "#*" $source_entry]} {
            continue
        }

        set source [file normalize [file join $hdl_root $source_entry]]
        if {![file exists $source]} {
            error "Canonical source is missing: $source"
        }

        if {[string match "../*" $source_entry]} {
            set packaged_path [file join external_csr [file tail $source]]
        } else {
            set packaged_path [string map {\\ /} $source_entry]
        }

        if {$packaged_path in $packaged_paths} {
            error "Duplicate package destination: $packaged_path"
        }
        lappend packaged_paths $packaged_path
        lappend entries [list $source $packaged_path]
    }

    return $entries
}
