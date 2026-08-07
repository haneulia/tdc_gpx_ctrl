# Create a persistent Vivado project containing the three K0-10 packaged-IP
# profiles as independent Block Designs and preserve each OOC synthesis run.

set script_dir [file normalize [file dirname [info script]]]
set v2_dir [file normalize [file join $script_dir ..]]
set hdl_root [file normalize [file join $v2_dir ../..]]
set project_dir [file join $hdl_root .work tdc_gpx_lidar_ctrl_v2_gui]
set jobs 4
if {[llength $argv] > 0} {
    set project_dir [file normalize [lindex $argv 0]]
}
if {[llength $argv] > 1} {
    set jobs [lindex $argv 1]
}

set project_name tdc_gpx_lidar_ctrl_v2_gui
set part_name xc7z020clg484-2
set v2_repo [file join $v2_dir ip_repo]
set v1_repo [file normalize [file join $hdl_root .. ip_repo]]
set v2_vlnv victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v2:2.0
set v1_vlnv victek.co.kr:my_ip:tdc_gpx_top:1.0
set report_dir [file join $project_dir gui_signoff_reports]

foreach path [list $v2_repo $v1_repo] {
    if {![file isdirectory $path]} {
        error "Required IP repository is missing: $path"
    }
}
file mkdir $project_dir
file mkdir $report_dir

# Avoid the damaged per-user Tcl Store on this workstation. Vivado still emits
# Common 17-1297 during startup, then uses this installed catalog successfully.
if {[info exists ::env(XILINX_VIVADO)]} {
    set install_tcl_store [file normalize \
        [file join $::env(XILINX_VIVADO) data XilinxTclStore]]
    if {[file isdirectory $install_tcl_store]} {
        lappend auto_path [file join $install_tcl_store support appinit]
        foreach vendor_dir [glob -nocomplain -type d \
                [file join $install_tcl_store tclapp *]] {
            foreach app_dir [glob -nocomplain -type d \
                    [file join $vendor_dir *]] {
                lappend auto_path $app_dir
            }
        }
        catch {package require ::tclapp::support::appinit 1.2}
    }
}

create_project -force $project_name $project_dir -part $part_name
set_property target_language VHDL [current_project]
set_property simulator_language Mixed [current_project]
set_property default_lib xil_defaultlib [current_project]
set_property ip_repo_paths [list $v2_repo $v1_repo] [current_project]
update_ip_catalog -rebuild

foreach vlnv [list $v1_vlnv $v2_vlnv] {
    if {[llength [get_ipdefs -all -quiet $vlnv]] != 1} {
        error "Expected exactly one catalog definition for $vlnv"
    }
}

proc v2_externalize_cell {cell} {
    foreach intf_pin [get_bd_intf_pins -quiet -of_objects $cell] {
        make_bd_intf_pins_external $intf_pin
    }
    foreach pin [get_bd_pins -quiet -of_objects $cell] {
        # Interface member pins were already connected by
        # make_bd_intf_pins_external. Externalizing them again would break the
        # AXI bundle into scalar nets (BD 41-1306).
        if {![get_property INTF $pin] &&
                [llength [get_bd_nets -quiet -of_objects $pin]] == 0} {
            make_bd_pins_external $pin
        }
    }
}

proc v2_require_config {cell property expected label} {
    set actual [get_property $property $cell]
    if {$actual ne $expected} {
        error "$label retained $property=$actual, expected $expected"
    }
}

set profiles {
    {bd_async32_tdc_faster 150 200 ASYNC 32 true}
    {bd_async128_proc_faster 200 150 ASYNC 128 false}
    {bd_sync64_equal 150 150 SYNC 64 true}
}

set bd_files {}
foreach profile $profiles {
    lassign $profile bd_name proc_mhz tdc_mhz mode width echo_enabled
    create_bd_design $bd_name
    current_bd_design $bd_name
    set cell [create_bd_cell -type ip -vlnv $v2_vlnv u_lidar_ctrl]
    set_property -dict [list \
        CONFIG.G_CSR_CLK_MHZ 100 \
        CONFIG.G_PROC_CLK_MHZ $proc_mhz \
        CONFIG.G_TDC_CLK_MHZ $tdc_mhz \
        CONFIG.G_STREAM_CLK_MODE $mode \
        CONFIG.G_NUM_CHIPS 4 \
        CONFIG.G_STOPS_PER_CHIP 8 \
        CONFIG.G_MAX_RETURNS_PER_STOP 7 \
        CONFIG.G_RISE_CAPABILITY_MASK 0011 \
        CONFIG.G_FALL_CAPABILITY_MASK 1100 \
        CONFIG.G_OUTPUT_WIDTH $width \
        CONFIG.G_NUM_FACES 5 \
        CONFIG.G_ENABLE_ECHO_RECEIVER $echo_enabled \
        CONFIG.G_ENABLE_ECHO_SIMULATION false] $cell

    foreach {property expected} [list \
        CONFIG.G_PROC_CLK_MHZ $proc_mhz \
        CONFIG.G_TDC_CLK_MHZ $tdc_mhz \
        CONFIG.G_STREAM_CLK_MODE $mode \
        CONFIG.G_OUTPUT_WIDTH $width \
        CONFIG.G_ENABLE_ECHO_RECEIVER $echo_enabled] {
        v2_require_config $cell $property $expected $bd_name
    }

    v2_externalize_cell $cell
    assign_bd_address
    regenerate_bd_layout
    validate_bd_design
    save_bd_design

    set bd_file [lindex [get_files -quiet */${bd_name}.bd] 0]
    if {$bd_file eq ""} {
        error "Block Design file was not created for $bd_name"
    }
    set_property synth_checkpoint_mode Hierarchical $bd_file
    generate_target synthesis $bd_file
    create_ip_run $bd_file
    lappend bd_files $bd_file
    puts "LIDAR_V2_K010_GUI_BD_PASS profile=$bd_name"
}

set summary_lines [list \
    {LIDAR_V2_K010_GUI_PROJECT_PASS} \
    "project=[file join $project_dir ${project_name}.xpr]" \
    "part=$part_name" \
    "v1=$v1_vlnv" \
    "v2=$v2_vlnv"]

foreach profile $profiles {
    lassign $profile bd_name proc_mhz tdc_mhz mode width echo_enabled
    set run_name ${bd_name}_u_lidar_ctrl_0_synth_1
    set run [get_runs -quiet $run_name]
    if {[llength $run] != 1} {
        error "Expected one synthesis run named $run_name"
    }
    launch_runs $run -jobs $jobs
    wait_on_run $run
    set status [get_property STATUS $run]
    set progress [get_property PROGRESS $run]
    if {$progress ne "100%" || $status ne "synth_design Complete!"} {
        error "$run_name failed: status=$status progress=$progress"
    }

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
    report_utilization -hierarchical \
        -file [file join $report_dir utilization_${bd_name}.rpt]
    close_design

    lappend summary_lines \
        "profile=$bd_name proc_mhz=$proc_mhz tdc_mhz=$tdc_mhz mode=$mode width=$width echo=$echo_enabled run=$run_name status=$status"
    puts "LIDAR_V2_K010_GUI_SYNTH_PASS profile=$bd_name run=$run_name"
}

set summary_file [file join $report_dir GUI_PROJECT_SUMMARY.txt]
close_project

set fp [open $summary_file w]
foreach line $summary_lines {
    puts $fp $line
}
close $fp

puts "LIDAR_V2_K010_GUI_PROJECT_PASS project=[file join $project_dir ${project_name}.xpr]"
exit 0
