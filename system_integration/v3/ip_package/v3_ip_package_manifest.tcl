# Canonical self-contained source manifest for tdc_gpx_lidar_ctrl_v3.
#
# V3 deliberately reuses the verified V2 control/CDC/physical acquisition
# RTL, but replaces the V2 integrated Top and the Processing data path with
# the V3 HLS adapters and generated H1-H4 RTL.  Every generated Verilog and
# LUTRAM initialization file is copied into the packaged IP so downstream
# Vivado projects never depend on this repository's .work directory.

proc lidar_v3_append_entry {entries_name destinations_name source relative type} {
    upvar 1 $entries_name entries
    upvar 1 $destinations_name destinations
    set normalized [file normalize $source]
    if {![file exists $normalized]} {
        error "V3 package source is missing: $normalized"
    }
    set packaged [string map {\\ /} $relative]
    if {$packaged in $destinations} {
        error "Duplicate V3 package destination: $packaged"
    }
    lappend destinations $packaged
    lappend entries [list $normalized $packaged $type]
}

proc lidar_v3_ip_package_manifest {hdl_root} {
    set hdl_root [file normalize $hdl_root]
    set v2_order [file join $hdl_root system_integration v2 scripts \
        v2_rtl_compile_order.txt]
    if {![file exists $v2_order]} {
        error "Missing V2 control-plane compile order: $v2_order"
    }

    set entries {}
    set destinations {}
    set channel [open $v2_order r]
    set lines [split [read $channel] "\n"]
    close $channel
    foreach line $lines {
        set source_entry [string trim $line]
        if {$source_entry eq {} || [string match "#*" $source_entry]} {
            continue
        }
        # The V2 Top is a Golden reference, not a child of the V3 package.
        if {$source_entry eq \
                {system_integration/v2/rtl/top/tdc_gpx_lidar_ctrl_v2_top.vhd}} {
            continue
        }
        # V3 owns these functions with its HLS adapters/status subsystem.
        # Keeping the unused V2 implementations in component.xml would make
        # ownership ambiguous and needlessly enlarge every downstream project.
        set replaced_v2_sources {
            system_integration/v2/rtl/status/lidar_processing_status_source.vhd
            system_integration/v2/rtl/status/lidar_status_snapshot_subsystem.vhd
            system_integration/v2/rtl/proc/lidar_gpx_hit_decoder.vhd
            system_integration/v2/rtl/proc/lidar_gpx_cell_collector.vhd
            system_integration/v2/rtl/proc/lidar_gpx_frame_lane_assembler.vhd
            system_integration/v2/rtl/proc/lidar_gpx_hit_cell_frame_pipeline.vhd
            system_integration/v2/rtl/proc/lidar_gpx_b5_b8_subsystem.vhd
            system_integration/v2/rtl/proc/lidar_gpx_cell_word_serializer.vhd
            system_integration/v2/rtl/proc/lidar_gpx_shot_line_builder.vhd
            system_integration/v2/rtl/proc/lidar_gpx_hole_line_expander.vhd
            system_integration/v2/rtl/proc/lidar_gpx_face_footer_builder.vhd
            system_integration/v2/rtl/proc/lidar_gpx_axis_lane_pipeline.vhd
            system_integration/v2/rtl/proc/lidar_gpx_axis_output_subsystem.vhd
        }
        if {$source_entry in $replaced_v2_sources} {
            continue
        }
        if {[string match "../*" $source_entry]} {
            set relative [file join external_csr [file tail $source_entry]]
        } else {
            set relative $source_entry
        }
        lidar_v3_append_entry entries destinations \
            [file join $hdl_root $source_entry] $relative vhdl2008
    }

    foreach root_source {tdc_gpx_sync_fifo.vhd} {
        lidar_v3_append_entry entries destinations \
            [file join $hdl_root $root_source] \
            [file join shared $root_source] vhdl2008
    }

    set v3_relatives {
        pkg/lidar_v3_hls_contract_pkg.vhd
        pkg/lidar_v3_status_pkg.vhd
        rtl/status/lidar_v3_processing_status_source.vhd
        rtl/status/lidar_v3_status_snapshot_subsystem.vhd
        rtl/bridges/lidar_gpx_hit_decoder_hls_adapter.vhd
        rtl/bridges/lidar_gpx_cell_collector_hls_adapter.vhd
        rtl/bridges/lidar_gpx_frame_lane_assembler_hls_adapter.vhd
        rtl/bridges/lidar_gpx_lane_word_formatter_hls_adapter.vhd
        rtl/top/lidar_gpx_hls_hit_cell_frame_pipeline.vhd
        rtl/top/lidar_gpx_hls_axis_output_subsystem.vhd
        rtl/top/lidar_gpx_hls_mixed_data_top.vhd
        rtl/top/lidar_gpx_hls_parent_data_subsystem.vhd
        rtl/top/tdc_gpx_lidar_ctrl_v3_top.vhd
    }
    foreach relative $v3_relatives {
        set type vhdl2008
        if {[file tail $relative] eq {tdc_gpx_lidar_ctrl_v3_top.vhd}} {
            # Keep the public Top classified as VHDL-93 for Vivado IP
            # Packager compatibility; its dependencies remain VHDL-2008.
            set type vhdl
        }
        lidar_v3_append_entry entries destinations \
            [file join $hdl_root system_integration v3 $relative] \
            [file join system_integration v3 $relative] $type
    }

    set generated_roots [list \
        [file join $hdl_root .work v3_hls_hit_decoder_component hls syn verilog] \
        [file join $hdl_root .work v3_hls_cell_collector_component hls syn verilog] \
        [file join $hdl_root .work v3_hls_frame_assembler_component hls syn verilog] \
        [file join $hdl_root .work v3_hls_lane_word_formatter_component hls syn verilog]]
    foreach generated_root $generated_roots {
        if {![file isdirectory $generated_root]} {
            error "Generated HLS RTL directory is missing: $generated_root"
        }
        set files [concat \
            [glob -nocomplain -type f [file join $generated_root *.v]] \
            [glob -nocomplain -type f [file join $generated_root *.dat]]]
        if {[llength $files] == 0} {
            error "Generated HLS RTL directory is empty: $generated_root"
        }
        foreach source [lsort $files] {
            set extension [string tolower [file extension $source]]
            set type [expr {$extension eq ".v" ? "verilog" : "data"}]
            set component [file tail [file dirname [file dirname \
                [file dirname [file dirname $generated_root]]]]]
            # Derive a stable folder from the generated top module prefix.
            set file_name [file tail $source]
            if {[string match {gpx_hit_decoder_hls*} $file_name]} {
                set component gpx_hit_decoder_hls
            } elseif {[string match {gpx_cell_collector_hls*} $file_name]} {
                set component gpx_cell_collector_hls
            } elseif {[string match {gpx_frame_assembler_hls*} $file_name]} {
                set component gpx_frame_assembler_hls
            } else {
                set component gpx_lane_word_formatter_hls
            }
            lidar_v3_append_entry entries destinations $source \
                [file join hls_generated $component $file_name] $type
        }
    }
    return $entries
}
