# Verify coexistence and dependency closure for the V3 HLS child-IP profile.

set script_dir [file normalize [file dirname [info script]]]
set v3_dir [file normalize [file join $script_dir ..]]
if {[llength $argv] > 0} {
    set repo [file normalize [lindex $argv 0]]
} else {
    set repo [file join $v3_dir ip_repo]
}

set parent_vlnv victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3_hls_ip:3.0
set embedded_vlnv victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3:3.0
set child_vlnvs {
    victek.co.kr:hls_ip:gpx_hit_decoder_hls:3.0
    victek.co.kr:hls_ip:gpx_cell_collector_hls:3.0
    victek.co.kr:hls_ip:gpx_frame_assembler_hls:3.0
    victek.co.kr:hls_ip:gpx_lane_word_formatter_hls:3.0
}
set parent_component [file join $repo \
    tdc_gpx_lidar_ctrl_v3_hls_ip_3_0 component.xml]
set canonical_xgui [file join $v3_dir ip_package \
    tdc_gpx_lidar_ctrl_v3_xgui.tcl]
set packaged_xgui [file join $repo tdc_gpx_lidar_ctrl_v3_hls_ip_3_0 \
    xgui tdc_gpx_lidar_ctrl_v3_hls_ip_v3_0.tcl]
set canonical_dual_guide [file join $v3_dir docs \
    V3_DUAL_HLS_PACKAGING_GUIDE_KO.md]
set packaged_dual_guide [file join $repo \
    tdc_gpx_lidar_ctrl_v3_hls_ip_3_0 doc \
    V3_DUAL_HLS_PACKAGING_GUIDE_KO.md]
set canonical_xgui_regeneration_rules [file join $v3_dir docs \
    V3_IP_XACT_XGUI_REGENERATION_RULES_KO.md]
set packaged_xgui_regeneration_rules [file join $repo \
    tdc_gpx_lidar_ctrl_v3_hls_ip_3_0 doc \
    V3_IP_XACT_XGUI_REGENERATION_RULES_KO.md]
foreach required [list $parent_component $canonical_xgui $packaged_xgui \
        $canonical_dual_guide $packaged_dual_guide \
        $canonical_xgui_regeneration_rules \
        $packaged_xgui_regeneration_rules] {
    if {![file exists $required]} {
        error "Required V3 HLS-IP package artifact is missing: $required"
    }
}

proc v3_hls_ip_read_binary {path} {
    set channel [open $path r]
    fconfigure $channel -translation binary
    set data [read $channel]
    close $channel
    return $data
}
set canonical_xgui_text [string trimright [string map \
    [list "\r\n" "\n" "\r" "\n"] \
    [v3_hls_ip_read_binary $canonical_xgui]]]
set packaged_xgui_text [string trimright [string map \
    [list "\r\n" "\n" "\r" "\n"] \
    [v3_hls_ip_read_binary $packaged_xgui]]]
if {$canonical_xgui_text ne $packaged_xgui_text} {
    error {V3 HLS-IP XGUI copy is stale}
}
source [file join $script_dir check_v3_xgui_source_contract.tcl]
v3_check_xgui_source_contract $packaged_xgui
if {[v3_hls_ip_read_binary $canonical_dual_guide] ne \
        [v3_hls_ip_read_binary $packaged_dual_guide]} {
    error {V3 HLS-IP dual packaging guide copy is stale}
}
if {[v3_hls_ip_read_binary $canonical_xgui_regeneration_rules] ne \
        [v3_hls_ip_read_binary $packaged_xgui_regeneration_rules]} {
    error {V3 HLS-IP XGUI regeneration rules copy is stale}
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

create_project -in_memory v3_hls_ip_check -part xc7z020clg484-2
set_property ip_repo_paths [list $repo] [current_project]
update_ip_catalog
foreach vlnv [concat [list $embedded_vlnv $parent_vlnv] $child_vlnvs] {
    if {[llength [get_ipdefs -all -quiet $vlnv]] != 1} {
        error "Expected one packaged IP definition: $vlnv"
    }
}
puts {LIDAR_V3_DUAL_VARIANT_CATALOG_PASS}

set core [ipx::open_core $parent_component]
if {[get_property vlnv $core] ne $parent_vlnv} {
    error "Unexpected V3 HLS-IP parent VLNV: [get_property vlnv $core]"
}
set synth_group [ipx::get_file_groups -quiet \
    xilinx_anylanguagesynthesis -of_objects $core]
if {[llength $synth_group] != 1} {
    error {Expected one V3 HLS-IP synthesis file group}
}
set source_files [ipx::get_files -of_objects $synth_group]
if {[llength $source_files] < 80} {
    error "V3 HLS-IP parent source closure is too small: [llength $source_files]"
}
foreach packaged_file $source_files {
    set name [string map {\\ /} [get_property name $packaged_file]]
    if {[string match "*hls_generated*" $name]} {
        error "Embedded HLS RTL leaked into child-IP profile: $name"
    }
    if {[string match "../*" $name] || [string match "*/../*" $name]} {
        error "External source reference entered child-IP profile: $name"
    }
}
set found_subcores {}
foreach group [ipx::get_file_groups -of_objects $core] {
    foreach dependency [get_property component_subcores $group] {
        if {$dependency ni $found_subcores} {
            lappend found_subcores $dependency
        }
    }
}
foreach child_vlnv $child_vlnvs {
    if {[lsearch -exact $found_subcores $child_vlnv] < 0} {
        error "V3 HLS-IP dependency is missing: $child_vlnv; found=$found_subcores"
    }
}
set drc [ipx::check_integrity -verbose $core]
if {!$drc} {
    error {V3 HLS-IP parent integrity DRC failed}
}
ipx::unload_core $core
puts "LIDAR_V3_HLS_IP_DEPENDENCY_PASS subcores=[llength $found_subcores]"

foreach {label width} {hls_ip_w32 32 hls_ip_w64 64} {
    create_ip -vlnv $parent_vlnv -module_name $label
    if {$width == 64} {
        set rise_mask 1111
        set fall_mask 1111
    } else {
        set rise_mask 0011
        set fall_mask 1100
    }
    set_property -dict [list CONFIG.G_OUTPUT_WIDTH $width \
        CONFIG.G_NUM_CHIPS 4 CONFIG.G_STOPS_PER_CHIP 8 \
        CONFIG.G_MAX_RETURNS_PER_STOP 7 \
        CONFIG.G_RISE_CAPABILITY_MASK $rise_mask \
        CONFIG.G_FALL_CAPABILITY_MASK $fall_mask] [get_ips $label]
    generate_target instantiation_template [get_ips $label]
    if {[get_property IS_LOCKED [get_ips $label]]} {
        error "Generated V3 HLS-IP instance is locked: $label"
    }
    puts "LIDAR_V3_HLS_IP_CUSTOMIZATION_PASS profile=$label width=$width rise=$rise_mask fall=$fall_mask"
}

puts {TDC_GPX_LIDAR_CTRL_V3_HLS_IP_CHECK_PASS}
exit 0
