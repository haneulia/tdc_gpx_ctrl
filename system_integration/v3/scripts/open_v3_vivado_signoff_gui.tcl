# Open the editable parent project and its immutable routed sign-off checkpoint.

if {$argc != 3} {
    error "usage: open_v3_vivado_signoff_gui.tcl <profile> <project.xpr> <post_route.dcp>"
}

set profile [lindex $argv 0]
set project_path [file normalize [lindex $argv 1]]
set checkpoint_path [file normalize [lindex $argv 2]]

if {![file exists $project_path]} {
    error "Vivado project does not exist: $project_path"
}
if {![file exists $checkpoint_path]} {
    error "Routed sign-off checkpoint does not exist: $checkpoint_path"
}

# A sign-off session is evidence inspection, not an IP upgrade workspace.
# Read-only mode prevents generated-target resets from partially modifying the
# project while the routed checkpoint is open.
open_project -read_only $project_path

# Keep the Block Design available as a GUI tab while the routed checkpoint is
# opened as the active physical design. A BD open failure must not hide the
# verified routed result.
set bd_files [get_files -quiet -all *design_1_lidar_ctrl_v3.bd]
if {[llength $bd_files] > 0} {
    if {[catch {open_bd_design [lindex $bd_files 0]} bd_error]} {
        puts "LIDAR_V3_GUI_BD_OPEN_WARNING profile=$profile detail=$bd_error"
    }
} else {
    puts "LIDAR_V3_GUI_BD_NOT_FOUND profile=$profile"
}

open_checkpoint $checkpoint_path

# Create an in-memory timing report that can be selected in the GUI. The
# archived report beside the checkpoint remains the sign-off evidence.
set timing_report_name "SIGNOFF_${profile}_TIMING"
if {[catch {
    report_timing_summary \
        -delay_type min_max \
        -max_paths 20 \
        -report_unconstrained \
        -name $timing_report_name
} timing_error]} {
    puts "LIDAR_V3_GUI_TIMING_REPORT_WARNING profile=$profile detail=$timing_error"
}

puts "LIDAR_V3_VIVADO_SIGNOFF_GUI_READY profile=$profile"
puts "LIDAR_V3_VIVADO_ACCESS_MODE=READ_ONLY"
puts "LIDAR_V3_VIVADO_PROJECT=$project_path"
puts "LIDAR_V3_VIVADO_CHECKPOINT=$checkpoint_path"
