# =============================================================================
# set_all_vhdl_2008.tcl
#   xpr 의 모든 VHDL 소스 파일을 "VHDL 2008" 파일 타입으로 설정.
#   이미 등록된 파일뿐 아니라 IP 내부 VHDL 도 커버.
#   + sim fileset 의 runtime 을 200 us 로 유지.
# =============================================================================

set prj_dir "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl"
open_project "$prj_dir/tdc_gpx_ctrl.xpr"

# -----------------------------------------------------------------------------
# sources_1 : 모든 VHDL → VHDL 2008
# -----------------------------------------------------------------------------
set vhdl_src_files [get_files -of_objects [get_filesets sources_1] \
                      -filter "FILE_TYPE == \"VHDL\" || FILE_TYPE == \"VHDL 2008\""]
puts "INFO: sources_1 — [llength $vhdl_src_files] VHDL file(s)"
foreach f $vhdl_src_files {
    set_property file_type "VHDL 2008" $f
}

# -----------------------------------------------------------------------------
# sim_1 : 모든 VHDL → VHDL 2008
# -----------------------------------------------------------------------------
set vhdl_sim_files [get_files -of_objects [get_filesets sim_1] \
                      -filter "FILE_TYPE == \"VHDL\" || FILE_TYPE == \"VHDL 2008\""]
puts "INFO: sim_1     — [llength $vhdl_sim_files] VHDL file(s)"
foreach f $vhdl_sim_files {
    set_property file_type "VHDL 2008" $f
}

# IP simset 들도 포함 (OOC 시뮬레이션용)
foreach fs [get_filesets -quiet -filter "FILESET_TYPE == SimulationSrcs"] {
    if {$fs eq "sim_1"} { continue }
    set vhdl_f [get_files -of_objects $fs \
                  -filter "FILE_TYPE == \"VHDL\" || FILE_TYPE == \"VHDL 2008\""]
    puts "INFO: $fs — [llength $vhdl_f] VHDL file(s)"
    foreach f $vhdl_f {
        set_property file_type "VHDL 2008" $f
    }
}

# IP synth/impl fileset (BlockSrcs) 의 VHDL 도 일괄 변경
foreach fs [get_filesets -quiet -filter "FILESET_TYPE == BlockSrcs"] {
    set vhdl_f [get_files -of_objects $fs \
                  -filter "FILE_TYPE == \"VHDL\" || FILE_TYPE == \"VHDL 2008\""]
    puts "INFO: $fs — [llength $vhdl_f] VHDL file(s)"
    foreach f $vhdl_f {
        set_property file_type "VHDL 2008" $f
    }
}

# -----------------------------------------------------------------------------
# Sim runtime + elaborate debug 재설정
# -----------------------------------------------------------------------------
set_property -name {xsim.simulate.runtime}    -value {200us}   -objects [get_filesets sim_1]
set_property -name {xsim.elaborate.debug_level} -value {typical} -objects [get_filesets sim_1]

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

puts "========================================================"
puts "  모든 VHDL 소스: file_type = VHDL 2008"
puts "========================================================"

close_project
