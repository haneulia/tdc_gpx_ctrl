# Shared source-level checks for the Vivado-native V3 XGUI.
#
# Vivado IP Packager exposes Layout and Preview only while the XGUI callbacks
# remain in its generated form. Lists, ranges, labels and enablement belong to
# component.xml; cross-parameter build contracts belong to
# fn_validate_build_config and its regression test.

proc v3_check_xgui_source_contract {xgui_path} {
    set channel [open $xgui_path r]
    fconfigure $channel -translation binary
    set source_text [read $channel]
    close $channel

    if {![string match {# Definitional proc to organize widgets*} $source_text]} {
        error {XGUI is not in Vivado-native generated form}
    }
    if {[string first {namespace eval} $source_text] >= 0 ||
        [string first {set_property errmsg} $source_text] >= 0 ||
        [string first {value_validation_list} $source_text] >= 0} {
        error {XGUI contains hand-maintained callback logic; keep it in IP-XACT or RTL}
    }

    set init_position [string first {proc init_gui} $source_text]
    set callback_position [string first {proc update_PARAM_VALUE.} $source_text]
    if {$init_position < 0 || $callback_position <= $init_position} {
        error {XGUI must place explicit init_gui before generated callbacks}
    }
    set init_text [string range $source_text $init_position         [expr {$callback_position - 1}]]
    if {[string first {foreach } $init_text] >= 0 ||
        [string first {eval } $init_text] >= 0} {
        error {XGUI init_gui must not generate Layout items dynamically}
    }

    foreach required_layout {
        {ipgui::add_page $IPINST -name "Clock and Output"}
        {ipgui::add_page $IPINST -name "TDC-GPX Topology"}
        {ipgui::add_page $IPINST -name "Echo Frontend"}
        {ipgui::add_page $IPINST -name "Physical Timing"}
    } {
        if {[string first $required_layout $init_text] < 0} {
            error "XGUI visual Layout item is missing: $required_layout"
        }
    }

    set generics {
        G_CSR_CLK_MHZ G_PROC_CLK_MHZ G_TDC_CLK_MHZ
        G_STREAM_CLK_MODE G_NUM_CHIPS G_STOPS_PER_CHIP
        G_MAX_RETURNS_PER_STOP G_RISE_CAPABILITY_MASK
        G_FALL_CAPABILITY_MASK G_OUTPUT_WIDTH G_NUM_FACES
        G_ENABLE_ECHO_RECEIVER G_ENABLE_ECHO_SIMULATION G_OEN_MODE
        G_PHASE_TIMEOUT_US G_POWERUP_TIME_NS G_RECOVERY_TIME_NS
        G_ALU_PULSE_TIME_NS G_BUS_IDLE_STABLE_TIME_NS
        G_DRAIN_MARGIN_TIME_NS
    }
    foreach generic $generics {
        foreach prefix {
            update_PARAM_VALUE.
            validate_PARAM_VALUE.
            update_MODELPARAM_VALUE.
        } {
            set declaration "proc $prefix$generic "
            if {[string first $declaration $source_text] < 0} {
                error "XGUI callback is missing: $prefix$generic"
            }
        }
    }

    set xgui_interp [interp create -safe]
    $xgui_interp eval $source_text
    foreach generic $generics {
        set update_name "update_PARAM_VALUE.$generic"
        set validate_name "validate_PARAM_VALUE.$generic"
        set model_name "update_MODELPARAM_VALUE.$generic"
        set update_body [$xgui_interp eval [list info body $update_name]]
        set validate_body [$xgui_interp eval [list info body $validate_name]]
        set model_body [$xgui_interp eval [list info body $model_name]]
        if {[string first {set_property} $update_body] >= 0 ||
            [string first {set_property} $validate_body] >= 0} {
            error "XGUI parameter callback was hand edited: $generic"
        }
        if {[string first {return true} $validate_body] < 0} {
            error "XGUI generated validation stub is malformed: $generic"
        }
        set expected "set_property value \[get_property value "
        append expected "\$\{PARAM_VALUE.$generic\}\] "
        append expected "\$\{MODELPARAM_VALUE.$generic\}"
        if {[string first $expected $model_body] < 0} {
            error "XGUI model-parameter mapping is not direct: $generic"
        }
    }
    interp delete $xgui_interp
    puts {LIDAR_V3_XGUI_NATIVE_VISUAL_CONTRACT_PASS}
}

if {[file normalize [info script]] eq [file normalize $argv0]} {
    if {[llength $argv] != 1} {
        error {Usage: check_v3_xgui_source_contract.tcl <xgui.tcl>}
    }
    v3_check_xgui_source_contract [file normalize [lindex $argv 0]]
}
