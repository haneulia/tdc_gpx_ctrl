set project_dir [file normalize [lindex $argv 0]]
set project_name tdc_gpx_lidar_ctrl_v2_gui
set project_path [file join $project_dir ${project_name}.xpr]

if {![file exists $project_path]} {
    error "Vivado GUI project is missing: $project_path"
}

set install_tcl_store [file normalize \
    [file join $::env(XILINX_VIVADO) data XilinxTclStore]]
lappend auto_path [file join $install_tcl_store support appinit]
foreach vendor_dir [glob -nocomplain -type d \
        [file join $install_tcl_store tclapp *]] {
    foreach app_dir [glob -nocomplain -type d [file join $vendor_dir *]] {
        lappend auto_path $app_dir
    }
}
package require ::tclapp::support::appinit 1.2

proc v2_require_equal {actual expected label} {
    if {$actual ne $expected} {
        error "$label mismatch: expected=$expected actual=$actual"
    }
}

set profiles [list \
    [list bd_async32_tdc_faster 150 200 ASYNC 32 true] \
    [list bd_async128_proc_faster 200 150 ASYNC 128 false] \
    [list bd_sync64_equal 150 150 SYNC 64 true]]

open_project $project_path
v2_require_equal [get_property PART [current_project]] \
    xc7z020clg484-2 PROJECT.PART

foreach profile $profiles {
    lassign $profile bd_name proc_mhz tdc_mhz mode width echo_enabled
    set bd_file [lindex [get_files -quiet */${bd_name}.bd] 0]
    if {$bd_file eq ""} {
        error "Block Design is missing: $bd_name"
    }

    open_bd_design $bd_file
    set cell [get_bd_cells -quiet u_lidar_ctrl]
    if {[llength $cell] != 1} {
        error "$bd_name must contain one u_lidar_ctrl instance"
    }
    foreach {property expected} [list \
            CONFIG.G_PROC_CLK_MHZ $proc_mhz \
            CONFIG.G_TDC_CLK_MHZ $tdc_mhz \
            CONFIG.G_STREAM_CLK_MODE $mode \
            CONFIG.G_OUTPUT_WIDTH $width \
            CONFIG.G_ENABLE_ECHO_RECEIVER $echo_enabled] {
        v2_require_equal [get_property $property $cell] $expected \
            ${bd_name}.${property}
    }
    validate_bd_design

    if {!$echo_enabled} {
        foreach disabled_port {i_pd_lvds_p i_pd_lvds_n o_tdc_stop} {
            if {[llength [get_bd_ports -quiet $disabled_port]] != 0} {
                error "$bd_name exposes disabled Echo port: $disabled_port"
            }
        }
    }
    puts "LIDAR_V2_K010_GUI_BD_VERIFY_PASS profile=$bd_name"
}

foreach profile $profiles {
    set bd_name [lindex $profile 0]
    set run_name ${bd_name}_u_lidar_ctrl_0_synth_1
    set run [get_runs -quiet $run_name]
    if {[llength $run] != 1} {
        error "Synthesis run is missing: $run_name"
    }
    v2_require_equal [get_property PROGRESS $run] 100% \
        ${run_name}.PROGRESS
    v2_require_equal [get_property STATUS $run] {synth_design Complete!} \
        ${run_name}.STATUS

    open_run $run_name
    set black_boxes [get_cells -quiet -hier -filter {IS_BLACKBOX == 1}]
    set latches [get_cells -quiet -hier \
        -filter {REF_NAME == LDCE || REF_NAME == LDPE}]
    if {[llength $black_boxes] != 0} {
        error "$run_name contains black boxes: $black_boxes"
    }
    if {[llength $latches] != 0} {
        error "$run_name contains inferred latches: $latches"
    }
    close_design
    puts "LIDAR_V2_K010_GUI_SYNTH_VERIFY_PASS run=$run_name"
}

close_project
puts "LIDAR_V2_K010_GUI_PROJECT_VERIFY_PASS project=$project_path"
exit 0
