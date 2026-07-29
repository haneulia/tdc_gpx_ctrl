# Synthesize design_1_unified in an isolated, disposable Vivado project.
#
# This flow never promotes the candidate to the parent project's active top.
# The temporary project is removed after a successful run to avoid retaining
# multi-gigabyte Vivado output. Failed runs remain under C:/tmp for diagnosis.

proc require_one {objects label} {
    if {[llength $objects] != 1} {
        error "Expected one $label, found [llength $objects]: $objects"
    }
    return [lindex $objects 0]
}

proc has_clock_period {clocks target tolerance} {
    foreach clock $clocks {
        set period [get_property PERIOD $clock]
        if {$period ne {} && abs($period - $target) <= $tolerance} {
            return true
        }
    }
    return false
}

set script_dir [file normalize [file dirname [info script]]]
set hdl_dir [file normalize [file join $script_dir ../..]]
set ip_root [file normalize [file join $hdl_dir ../..]]

if {[llength $argv] > 0} {
    set parent_dir [file normalize [lindex $argv 0]]
} else {
    set parent_dir {C:/Projects/my_sp/ALINX/Logic/project_4}
}

set candidate_bd [file join $parent_dir project_4.srcs sources_1 bd \
    design_1_unified design_1_unified.bd]
set clock_domain_xdc [file join $hdl_dir system_integration constraints \
    project4_unified_clock_domains.xdc]
if {![file exists $candidate_bd]} {
    error "Unified candidate BD is missing: $candidate_bd"
}
if {![file exists $clock_domain_xdc]} {
    error "Unified clock-domain constraint is missing: $clock_domain_xdc"
}

set work_root [file normalize {C:/tmp/project4_unified_ooc}]
if {[file tail $work_root] ne {project4_unified_ooc}} {
    error "Refusing unsafe temporary path: $work_root"
}
if {[file exists $work_root]} {
    file delete -force $work_root
}
file mkdir $work_root

set original_dir [pwd]
cd $work_root
create_project -force project4_unified_ooc \
    [file join $work_root project] -part xc7z020clg484-2
set_property target_language VHDL [current_project]
set_property simulator_language Mixed [current_project]

set repo_paths {}
foreach relative_repo {
    virtual_encoder/ip_repo
    motor_decoder/ip_repo
    laser_ctrl/ip_repo
    motor_laser_ctrl/ip_repo
    echo_receiver/ip_repo
    tdc_gpx_ctrl/ip_repo
    lidar_unified_csr/ip_repo
} {
    set repo [file normalize [file join $ip_root $relative_repo]]
    if {![file exists [file join $repo component.xml]]} {
        error "Required IP repository is missing component.xml: $repo"
    }
    lappend repo_paths $repo
}
set_property IP_REPO_PATHS $repo_paths [current_project]
update_ip_catalog -rebuild

set candidate_copy_dir [file join $work_root project_4.srcs sources_1 bd \
    design_1_unified]
file mkdir $candidate_copy_dir
set candidate_copy [file join $candidate_copy_dir design_1_unified.bd]
file copy -force $candidate_bd $candidate_copy
add_files -norecurse $candidate_copy
set bd_file [require_one [get_files -quiet $candidate_copy] \
    {unified candidate BD source}]
generate_target all $bd_file

set wrapper_files [make_wrapper -files $bd_file -top]
if {[llength $wrapper_files] == 0} {
    error {Vivado did not create the unified candidate wrapper}
}
add_files -norecurse $wrapper_files
add_files -fileset constrs_1 -norecurse $clock_domain_xdc
set clock_domain_file [require_one [get_files -quiet $clock_domain_xdc] \
    {unified clock-domain constraint}]
set_property PROCESSING_ORDER LATE $clock_domain_file
set_property top design_1_unified_wrapper [get_filesets sources_1]
update_compile_order -fileset sources_1

launch_runs synth_1 -jobs 8
wait_on_run synth_1
set synth_run [require_one [get_runs -quiet synth_1] {synthesis run}]
set synth_status [get_property STATUS $synth_run]
if {[string first {Complete!} $synth_status] < 0} {
    error "Unified candidate synthesis failed: $synth_status"
}

open_run $synth_run
set clocks [get_clocks -quiet]
if {![has_clock_period $clocks 5.000 0.010]} {
    error {Synthesized candidate lacks the required 200 MHz clock}
}
if {![has_clock_period $clocks 6.667 0.020]} {
    error {Synthesized candidate lacks the required 150 MHz clock}
}

report_utilization -file [file join $work_root utilization.rpt]
report_timing_summary -delay_type max -max_paths 20 \
    -file [file join $work_root timing_summary.rpt]
set cdc_report [report_cdc -details -return_string]
set cdc_channel [open [file join $work_root cdc.rpt] w]
puts $cdc_channel $cdc_report
close $cdc_channel

# Keep the console result compact while preserving the CDC rule/count rows
# needed to distinguish known synchronizers from unsafe or unknown crossings.
set cdc_summary_lines {}
set cdc_rejected_rules {}
set cdc15_summary_count 0
foreach line [split $cdc_report \n] {
    if {[regexp {^CDC-([0-9]+)[[:space:]]+([^[:space:]]+)[[:space:]]+([0-9]+)} \
            $line match rule_id severity rule_count]} {
        lappend cdc_summary_lines $line
        if {$rule_id eq {15} && $severity eq {Warning}} {
            set cdc15_summary_count $rule_count
        } elseif {$severity ne {Info}} {
            lappend cdc_rejected_rules $line
        }
    }
}
puts {PROJECT4_UNIFIED_OOC_CDC_SUMMARY_BEGIN}
foreach line $cdc_summary_lines {
    puts $line
}
puts {PROJECT4_UNIFIED_OOC_CDC_SUMMARY_END}

set cdc15_detail_count 0
set cdc15_xpm_fifo_count 0
foreach line [split $cdc_report \n] {
    if {[regexp {^[[:space:]]*[0-9]+[[:space:]]+CDC-15[[:space:]]+Warning} \
            $line]} {
        incr cdc15_detail_count
        if {[string first {xpm_fifo_base_inst} $line] >= 0} {
            incr cdc15_xpm_fifo_count
        }
    }
}
if {[llength $cdc_rejected_rules] != 0} {
    error "Rejected CDC rules: $cdc_rejected_rules"
}
if {$cdc15_detail_count != $cdc15_summary_count ||
    $cdc15_xpm_fifo_count != $cdc15_summary_count} {
    error "CDC-15 is not exclusively owned by XPM async FIFOs: summary=$cdc15_summary_count details=$cdc15_detail_count xpm=$cdc15_xpm_fifo_count"
}
puts "PROJECT4_UNIFIED_OOC_CDC15_XPM_FIFO_ACCEPTED=$cdc15_xpm_fifo_count"

set worst_paths [get_timing_paths -quiet -delay_type max -max_paths 1]
if {[llength $worst_paths] == 0} {
    set worst_slack {NA}
} else {
    set worst_path [lindex $worst_paths 0]
    set worst_slack [get_property SLACK $worst_path]
    puts {PROJECT4_UNIFIED_OOC_WORST_PATH_BEGIN}
    puts [report_property -return_string $worst_path]
    puts {PROJECT4_UNIFIED_OOC_WORST_PATH_END}
}

if {$worst_slack eq {NA} || $worst_slack < 0.0} {
    error "Unified candidate synthesized timing is not met: WNS=$worst_slack ns"
}

foreach clock $clocks {
    puts "PROJECT4_UNIFIED_OOC_CLOCK=[get_property NAME $clock]:[get_property PERIOD $clock]ns"
}
puts {PROJECT4_UNIFIED_OOC_SYNTH_PASS}
puts "PROJECT4_UNIFIED_OOC_STATUS=$synth_status"
puts "PROJECT4_UNIFIED_OOC_WNS_NS=$worst_slack"
puts {PROJECT4_UNIFIED_OOC_CLOCK_DOMAIN_CONTRACT=CSR50_PROC150_TDC200_ASYNC}

close_project
cd $original_dir
file delete -force $work_root
puts {PROJECT4_UNIFIED_OOC_TEMP_CLEAN_PASS}
exit
